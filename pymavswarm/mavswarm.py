# pymavswarm is an interface for swarm control and interaction
# Copyright (C) 2022  Evan Palmer

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from __future__ import annotations

import atexit
import logging
import math
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Any, Union

import monotonic
from pymavlink import mavutil

from pymavswarm import Connection
from pymavswarm._types import (
    AgentID,
    CommandExecutor,
    MessageCode,
    MessageHandler,
    PostExecutionHandler,
    StateVerifier,
)
from pymavswarm.agent import Agent
from pymavswarm.handlers import MessageReceivers
from pymavswarm.message import codes
from pymavswarm.message.response import Response
from pymavswarm.state import Parameter
from pymavswarm.utils import Event, FileLogger, NotifierDict, init_logger


class MavSwarm:
    """
    User-facing pymavswarm interface.

    Provides an interface for controlling robot swarms, obtaining swarm state, and
    configuring agents.
    """

    def __init__(
        self,
        max_workers: int = 5,
        log_level: int = logging.INFO,
        log_to_file: bool = False,
        log_filename: str | None = None,
    ) -> None:
        """
        Construct MavSwarm interface.

        :param max_workers: maximum number of workers available in the thread pool used
            to send messages, defaults to 5
        :type: max_workers: int, optional
        :param log_level: log level of the system, defaults to logging.INFO
        :type log_level: int, optional
        :param log_to_file: flag indicating whether to log the results to a file,
            defaults to False
        :type log_to_file: bool, optional
        :param log_filename: name of the file to log messages to, defaults to None
        :type log_filename: str | None, optional
        """
        super().__init__()

        self._logger = init_logger(__name__, log_level=log_level)
        self.__agent_list_changed = Event()
        self._agents = NotifierDict(self.__agent_list_changed)

        # Register a listener to update the system time publish rate
        self.__agent_list_changed.add_listener(self.__request_system_time)

        self._connection = Connection(log_level=log_level)
        self.__message_receivers = MessageReceivers(log_level=log_level)

        # Add callbacks to handle clock synchronization
        # These callbacks are implemented here, rather than in the handlers, because
        # of their one-off dependence on single properties
        self.__message_receivers.add_message_handler("TIMESYNC", self.__measure_ping)
        self.__message_receivers.add_message_handler("SYSTEM_TIME", self.__sync_clocks)

        self.__file_logger: FileLogger | None = None
        self.__boot_time: float | None = None

        if log_to_file:
            self.__file_logger = FileLogger(log_filename)

        # Mutexes
        self.__access_agents_mutex = threading.Lock()
        self.__send_message_mutex = threading.Lock()
        self.__read_message_mutex = threading.Lock()

        # Threads
        self.__incoming_message_thread = threading.Thread(target=self.__receive_message)
        self.__incoming_message_thread.setDaemon(True)
        self.__heartbeat_thread = threading.Thread(target=self.__send_heartbeat)
        self.__heartbeat_thread.setDaemon(True)
        self.__ping_thread = threading.Thread(target=self.__request_ping_measurement)
        self.__ping_thread.setDaemon(True)

        # Thread pool for sending messages
        self.__send_message_thread_pool_executor = ThreadPoolExecutor(
            max_workers=max_workers
        )

        # Register the exit callback
        atexit.register(self.disconnect)

        return

    @property
    def agents(self) -> list[Agent]:
        """
        List of agents in the swarm.

        :return: swarm agents
        :rtype: list[Agent]
        """
        return [*self._agents.values()]

    @property
    def agent_ids(self) -> list[AgentID]:
        """
        List of agent IDs in the swarm.

        :return: list of agent IDs
        :rtype: list[AgentID]
        """
        return [*self._agents]

    @property
    def agent_list_changed(self) -> Event:
        """
        Event signaling that the list of agents in the swarm has changed.

        :return: event
        :rtype: Event
        """
        return self.__agent_list_changed

    @property
    def connected(self) -> bool:
        """
        Flag indicating whether there is an active MAVLink connection.

        :return: flag
        :rtype: bool
        """
        return self._connection.connected

    @property
    def supported_modes(self) -> dict[str, int] | None:
        """
        List of supported flight modes.

        :return: supported flight modes
        :rtype: dict[str, int]
        """
        if self._connection.mavlink_connection is not None:
            return self._connection.mavlink_connection.mode_mapping()

        return None

    @property
    def time_since_boot(self) -> int | None:
        """
        Time since the connection was established.

        :return: time since connection was established.
        :rtype: int | None
        """
        time_since_boot = None

        if self.__boot_time is not None:
            time_since_boot = int((time.time() - self.__boot_time) * 1e3)

        return time_since_boot

    def connect(
        self,
        port: str,
        baudrate: int,
        source_system: int = 255,
        source_component: int = 0,
        connection_attempt_timeout: float = 2.0,
    ) -> bool:
        """
        Connect to the network.

        Attempt to establish a MAVLink connection using the provided configurations.

        :param port: port over which a connection should be established
        :type port: str
        :param baud: baudrate that a connection should be established with
        :type baud: int
        :param source_system: system ID of the connection, defaults to 255
        :type source_system: int, optional
        :param source_component: component ID of the connection, defaults to 0
        :type source_component: int, optional
        :param connection_attempt_timeout: maximum amount of time allowed to attempt
            to establish a connection, defaults to 2.0 [s]
        :type connection_attempt_timeout: float, optional
        :return: whether or not the connection attempt was successful
        :rtype: bool
        """
        self._logger.debug(
            f"Attempting to establish a new MAVLink connection over port {port} with "
            f"baudrate {baudrate}"
        )

        if not self._connection.connect(
            port, baudrate, source_system, source_component, connection_attempt_timeout
        ):
            return False

        # Set the boot time
        self.__boot_time = time.time()

        # Start threads
        self.__incoming_message_thread.start()
        self.__heartbeat_thread.start()
        self.__ping_thread.start()

        return True

    def disconnect(self) -> None:
        """Disconnect from the MAVLink network and shutdown all services."""
        self._logger.debug("Disconnecting the MAVLink connection")

        # Shutdown the thread pool executor
        self.__send_message_thread_pool_executor.shutdown()

        # Disconnect the MAVLink connection
        self._connection.disconnect()

        if (
            self.__incoming_message_thread is not None
            and self.__incoming_message_thread.is_alive()
        ):
            self.__incoming_message_thread.join()

        if self.__heartbeat_thread is not None and self.__heartbeat_thread.is_alive():
            self.__heartbeat_thread.join()

        if self.__ping_thread is not None and self.__ping_thread.is_alive():
            self.__ping_thread.join()

        # Clear the agents list
        self._agents.clear()
        self.__agent_list_changed.listeners.clear()

        return

    def add_agent(self, agent: Agent) -> None:
        """
        Manually add an agent to the swarm.

        This is useful if an agent isn't automatically recognized, but commands still
        need to be sent to this agent.

        :param agent: agent to add
        :type agent: Agent
        """
        # Add the agent
        self._agents[(agent.system_id, agent.component_id)] = agent

        return

    def remove_agent(self, agent: Agent) -> None:
        """
        Manually remove an agent from the swarm.

        :param agent: agent to remove
        :type agent: Agent
        """
        # Remove the agent
        del self._agents[(agent.system_id, agent.component_id)]

        return

    def arm(
        self,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
        verify_state: bool = False,
        verify_state_timeout: float = 1.0,
    ) -> Future:
        """
        Arm the desired agents.

        If the target agent IDs are not provided, the system will attempt to arm
        all agents in the swarm.

        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry arming an agent on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try arming an agent
            before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of an arming attempt, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :param verify_state: flag indicating whether or not the system should attempt
            to verify that the agent switched into the armed state, defaults to False
        :type verify_state: bool, optional
        :param verify_state_timeout: maximum amount of time allowed per attempt to
            verify that an agent is in the armed state, defaults to 1.0 [s]
        :type verify_state_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.command_long_send(
                    agent_id[0],
                    agent_id[1],
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            return

        # Construct a method to use for verifying state change
        def verify_state_changed(agent_id: AgentID) -> bool:
            ack = True
            start_time = time.time()

            while not self._agents[agent_id].armed.value:
                if time.time() - start_time >= verify_state_timeout:
                    ack = False
                    break

            return ack

        return self._send_command(
            agent_ids,
            executor,
            "ARM",
            retry,
            message_timeout,
            ack_timeout,
            state_verifier=verify_state_changed if verify_state else None,
        )

    def disarm(
        self,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
        verify_state: bool = False,
        verify_state_timeout: float = 1.0,
    ) -> Future:
        """
        Disarm the desired agents.

        If the target agent IDs are not provided, the system will attempt to disarm
        all agents in the swarm.

        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry disarming an agent on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try disarming an agent
            before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a disarming attempt, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :param verify_state: flag indicating whether or not the system should attempt
            to verify that the agent switched into the disarmed state, defaults to False
        :type verify_state: bool, optional
        :param verify_state_timeout: maximum amount of time allowed per attempt to
            verify that an agent is in the disarmed state, defaults to 1.0 [s]
        :type verify_state_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.command_long_send(
                    agent_id[0],
                    agent_id[1],
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            return

        # Construct a method to use for verifying state change
        def verify_state_changed(agent_id: AgentID) -> bool:
            ack = True
            start_time = time.time()

            while self._agents[agent_id].armed.value:
                if time.time() - start_time >= verify_state_timeout:
                    ack = False
                    break

            return ack

        return self._send_command(
            agent_ids,
            executor,
            "DISARM",
            retry,
            message_timeout,
            ack_timeout,
            state_verifier=verify_state_changed if verify_state else None,
        )

    def reboot(
        self,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Reboot the desired agents.

        If the target agent IDs are not provided, the system will attempt to reboot
        all agents in the swarm.

        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry rebooting an agent on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try rebooting an agent
            before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a reboot attempt, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.command_long_send(
                    agent_id[0],
                    agent_id[1],
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                    0,
                    1,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            return

        return self._send_command(
            agent_ids,
            executor,
            "REBOOT",
            retry,
            message_timeout,
            ack_timeout,
        )

    def shutdown(
        self,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Shutdown the desired agents.

        If the target agent IDs are not provided, the system will attempt to shutdown
        all agents in the swarm.

        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry shutting down an agent on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try shutting down an
            agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a shutdown attempt, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.command_long_send(
                    agent_id[0],
                    agent_id[1],
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                    0,
                    2,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            return

        return self._send_command(
            agent_ids,
            executor,
            "SHUTDOWN",
            retry,
            message_timeout,
            ack_timeout,
        )

    def set_mode(
        self,
        flight_mode: str,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
        verify_state: bool = False,
        verify_state_timeout: float = 1.0,
    ) -> Future:
        """
        Set the flight mode of the desired agents.

        If the target agent IDs are not provided, the system will attempt to set the
        flight mode of all agents in the swarm.

        :param flight_mode: flight mode to switch the agents into
        :type flight_mode: str
        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry changing the mode of an agent on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try changing the mode
            of an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a mode change attempt, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :param verify_state: flag indicating whether or not the system should attempt
            to verify that the agent switched into the target mode, defaults to False
        :type verify_state: bool, optional
        :param verify_state_timeout: maximum amount of time allowed per attempt to
            verify that an agent is in the desired mode, defaults to 1.0 [s]
        :type verify_state_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                # Reset target
                self._connection.mavlink_connection.target_system = agent_id[0]
                self._connection.mavlink_connection.target_component = agent_id[1]

                # Send flight mode
                self._connection.mavlink_connection.set_mode(
                    self._connection.mavlink_connection.mode_mapping()[flight_mode]
                )
            return

        # Construct a method to use for verifying the state change
        def verify_state_changed(agent_id: AgentID) -> bool:
            ack = True
            start_time = time.time()

            while self._agents[agent_id].mode.value != flight_mode:
                if time.time() - start_time >= verify_state_timeout:
                    ack = False
                    break

            return ack

        return self._send_command(
            agent_ids,
            executor,
            "SET_MODE",
            retry,
            message_timeout,
            ack_timeout,
            state_verifier=verify_state_changed if verify_state else None,
        )

    def set_airspeed(
        self,
        speed: float,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Set the airspeed of the desired agents.

        If the target agent IDs are not provided, the system will attempt to set the
        airspeed of all agents in the swarm.

        :param speed: target airspeed [m/s]
        :type speed: float
        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry setting the airspeed of an agent on failure, defaults to
            False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try setting the
            airspeed of an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a airspeed change attempt, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.command_long_send(
                    agent_id[0],
                    agent_id[1],
                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    0,
                    0,
                    speed,
                    -1,
                    0,
                    0,
                    0,
                    0,
                )
            return

        return self._send_command(
            agent_ids,
            executor,
            "SET_AIRSPEED",
            retry,
            message_timeout,
            ack_timeout,
        )

    def set_groundspeed(
        self,
        speed: float,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Set the groundspeed of the desired agents.

        If the target agent IDs are not provided, the system will attempt to set the
        groundspeed of all agents in the swarm.

        :param speed: target groundspeed [m/s]
        :type speed: float
        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry setting the groundspeed of an agent on failure, defaults to
            False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try setting the
            groundspeed of an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a groundspeed change attempt, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.command_long_send(
                    agent_id[0],
                    agent_id[1],
                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    0,
                    1,
                    speed,
                    -1,
                    0,
                    0,
                    0,
                    0,
                )
            return

        return self._send_command(
            agent_ids,
            executor,
            "SET_GROUNDSPEED",
            retry,
            message_timeout,
            ack_timeout,
        )

    def gyroscope_calibration(
        self,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Perform gyroscope calibration on the specified agents.

        If the target agent IDs are not provided, the system will attempt to perform
        gyroscope calibration on all swarm agents.

        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry gyroscope calibration on an agent on calibration failure,
            defaults to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try gyroscope
            calibration on an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a gyroscope calibration attempt, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.command_long_send(
                    agent_id[0],
                    agent_id[1],
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                    0,
                    1,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            return

        return self._send_command(
            agent_ids,
            executor,
            "GYROSCOPE_CALIBRATION",
            retry,
            message_timeout,
            ack_timeout,
        )

    def magnetometer_calibration(
        self,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Perform magnetometer calibration on the specified agents.

        If the target agent IDs are not provided, the system will attempt to perform
        magnetometer calibration on all swarm agents.

        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry magnetometer calibration on an agent on calibration failure,
            defaults to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try magnetometer
            calibration on an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a magnetometer calibration attempt, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.command_long_send(
                    agent_id[0],
                    agent_id[1],
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                    0,
                    0,
                    1,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            return

        return self._send_command(
            agent_ids,
            executor,
            "MAGNETOMETER_CALIBRATION",
            retry,
            message_timeout,
            ack_timeout,
        )

    def ground_pressure_calibration(
        self,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Perform ground pressure calibration on the specified agents.

        If the target agent IDs are not provided, the system will attempt to perform
        ground pressure calibration on all swarm agents.

        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry ground pressure calibration on an agent on calibration
            failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try ground pressure
            calibration on an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a ground pressure calibration attempt, defaults to
            0.5 [s]
        :type ack_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.command_long_send(
                    agent_id[0],
                    agent_id[1],
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                    0,
                    0,
                    0,
                    3,
                    0,
                    0,
                    0,
                    0,
                )
            return

        return self._send_command(
            agent_ids,
            executor,
            "GROUND_PRESSURE_CALIBRATION",
            retry,
            message_timeout,
            ack_timeout,
        )

    def airspeed_calibration(
        self,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Perform airspeed calibration on the specified agents.

        If the target agent IDs are not provided, the system will attempt to perform
        airspeed calibration on all swarm agents.

        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry airspeed calibration on an agent on calibration failure,
            defaults to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try airspeed
            calibration on an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a airspeed calibration attempt, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.command_long_send(
                    agent_id[0],
                    agent_id[1],
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    2,
                    0,
                )
            return

        return self._send_command(
            agent_ids,
            executor,
            "AIRSPEED_CALIBRATION",
            retry,
            message_timeout,
            ack_timeout,
        )

    def barometer_temperature_calibration(
        self,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Perform barometer temperature calibration on the specified agents.

        If the target agent IDs are not provided, the system will attempt to perform
        barometer calibration on all swarm agents.

        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry barometer temperature calibration on an agent on calibration
            failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try barometer
            temperature calibration on an agent before a timeout occurs, defaults to
            2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a barometer temperature calibration attempt, defaults to
            0.5 [s]
        :type ack_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.command_long_send(
                    agent_id[0],
                    agent_id[1],
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    3,
                )
            return

        return self._send_command(
            agent_ids,
            executor,
            "BAROMETER_TEMPERATURE_CALIBRATION",
            retry,
            message_timeout,
            ack_timeout,
        )

    def accelerometer_calibration(
        self,
        simple_calibration: bool = True,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Perform accelerometer calibration on the specified agents.

        If the target agent IDs are not provided, the system will attempt to perform
        accelerometer calibration on all swarm agents.

        :param simple_calibration: perform simple accelerometer calibration, defaults to
            True
        :type simple_calibration: bool, optional
        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry accelerometer calibration on an agent on calibration
            failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try accelerometer
            calibration on an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a accelerometer calibration attempt, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.command_long_send(
                    agent_id[0],
                    agent_id[1],
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                    0,
                    0,
                    0,
                    0,
                    0,
                    4 if simple_calibration else 1,
                    0,
                    0,
                )
            return

        return self._send_command(
            agent_ids,
            executor,
            "ACCELEROMETER_CALIBRATION",
            retry,
            message_timeout,
            ack_timeout,
        )

    def send_debug_message(
        self,
        name: str,
        value: Union[int, float],
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Send a debug message to the specified agents.

        If the target agent IDs are not provided, the system will attempt to send the
        debug message to all swarm agents.

        :param name: debug message name
        :type name: str
        :param value: debug message value
        :type value: Union[int, float]
        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry sending the debug message to an agent on failure, defaults
            to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try sending the debug
            message to an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of the debug message, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """
        if not isinstance(name, str):
            raise TypeError(f"Invalid name provided. Expected string, got {type(name)}")

        if not isinstance(value, int) and not isinstance(value, float):
            raise TypeError(
                f"Invalid value provided. Expected an int or a float, got {type(value)}"
            )

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                # Reset target
                self._connection.mavlink_connection.target_system = agent_id[0]
                self._connection.mavlink_connection.target_component = agent_id[1]

                # Send flight mode
                if isinstance(value, int):
                    self._connection.mavlink_connection.mav.named_value_int_send(
                        int(time.time()), str.encode(name), value
                    )
                else:
                    self._connection.mavlink_connection.mav.named_value_float_send(
                        int(time.time()), str.encode(name), value
                    )

            return

        return self._send_command(
            agent_ids,
            executor,
            "DEBUG",
            retry,
            message_timeout,
            ack_timeout,
        )

    def takeoff(
        self,
        altitude: float,
        latitude: float = 0,
        longitude: float = 0,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Switch the specified agents into takeoff mode.

        This does NOT perform a full takeoff sequence.

        If the target agent IDs are not provided, the system will attempt to switch all
        swarm agents into takeoff mode.

        State verification is not supported for this command.

        :param altitude: altitude to takeoff to
        :type altitude: float
        :param latitude: latitude to takeoff to, defaults to 0
        :type latitude: float, optional
        :param longitude: longitude to takeoff to, defaults to 0
        :type longitude: float, optional
        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry switching the agent into takeoff mode on failure, defaults
            to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try switching an
            agent into takeoff mode before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of the takeoff message, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.command_long_send(
                    agent_id[0],
                    agent_id[1],
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0,
                    0,
                    0,
                    0,
                    0,
                    latitude,
                    longitude,
                    altitude,
                )
            return

        return self._send_command(
            agent_ids,
            executor,
            "TAKEOFF",
            retry,
            message_timeout,
            ack_timeout,
        )

    def takeoff_sequence(
        self,
        altitude: float,
        latitude: float = 0,
        longitude: float = 0,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
        verify_state: bool = False,
        verify_state_timeout: float = 1.0,
        stage_delay: float = 3.0,
    ) -> Union[list[Response], Response]:
        """
        Command the desired agents to perform a full takeoff sequence.

        The full takeoff sequence includes the following stages:

        1. Arm the agent
        2. Command takeoff

        Prior to executing this command, ensure that all agents are in the correct
        flight mode. In the case of ArduPilot, this should be GUIDED mode. If a failure
        occurs at stage 1, the system will attempt to command all agents to disarm. If
        a failure occurs at stage 2, the system will attempt to
        command all agents to land.

        This command is a blocking command and does not run asynchronously (i.e., no
        future will be returned). This command is multi-staged as well; therefore, it
        may take a while to complete. Furthermore, there is NO altitude verification to
        ensure that an agent has reached the desired altitude.

        :param altitude: altitude to takeoff to
        :type altitude: float
        :param latitude: latitude to takeoff to, defaults to 0
        :type latitude: float, optional
        :param longitude: longitude to takeoff to, defaults to 0
        :type longitude: float, optional
        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry performing a stage on stage failure, defaults
            to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try complete a stage
            on an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a stage, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :param verify_state: verify that the agents successfully changed flight modes
            and armed; if state verification fails, failure response will be initiaed,
            defaults to False
        :type verify_state: bool, optional
        :param verify_state_timeout: maximum amount of time allowed to verify a state
            change on an agent, defaults to 1.0
        :type verify_state_timeout: float, optional
        :param stage_delay: amount of time to delay between takeoff sequence stages,
            defaults to 3.0
        :type stage_delay: float, optional
        :return: message response; if a failure occurs, the response(s) will be the
            stage's responses (e.g., if any message fails at stage 1, the returned
            value will be the message responses for that particular stage)
        :rtype: Union[list[Response], Response]
        """

        def failure_occured(responses: Union[list[Response], Response]) -> bool:
            """
            Check the responses to determine if a message failure occurred.

            :param responses: message responses
            :type responses: Union[list[Response], Response]
            :return: true if failure occurred, false otherwise
            :rtype: bool
            """
            if isinstance(responses, list):
                return all(not response.result for response in responses)

            return not responses.result

        # Arming stage
        future = self.arm(
            agent_ids=agent_ids,
            retry=retry,
            message_timeout=message_timeout,
            ack_timeout=ack_timeout,
            verify_state=verify_state,
            verify_state_timeout=verify_state_timeout,
        )

        # Wait for the arming stage to complete
        while not future.done():
            pass

        # Attempt to disarm all agents on stage 1 failure
        if failure_occured(future.result()):
            self._logger.warning("Takeoff sequence command failed at stage 1")
            self.disarm(agent_ids=agent_ids, retry=True, verify_state=True)
            return future.result()

        time.sleep(stage_delay)

        # Takeoff stage
        future = self.takeoff(
            altitude,
            latitude=latitude,
            longitude=longitude,
            agent_ids=agent_ids,
            retry=retry,
            message_timeout=message_timeout,
            ack_timeout=ack_timeout,
        )

        # Wait for the takeoff stage to complete
        while not future.done():
            pass

        # Attempt to disarm all agents on stage 2 failure
        if failure_occured(future.result()):
            self._logger.warning("Takeoff sequence command failed at stage 2")
            self.set_mode("LAND", agent_ids=agent_ids, retry=True, verify_state=True)
            return future.result()

        self._logger.info("Successfully completed all stages of the takeoff sequence")

        if agent_ids is None:
            agent_ids = self.__get_expected_agent_ids()

        if isinstance(agent_ids, int):
            responses = Response(agent_ids, "TAKEOFF_SEQUENCE", True, codes.SUCCESS)
        else:
            responses = [
                Response(
                    agent_id,  # type: ignore
                    "TAKEOFF_SEQUENCE",
                    True,
                    codes.SUCCESS,
                )
                for agent_id in agent_ids
            ]

        return responses

    def read_parameter(
        self,
        parameter_id: str,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Read the parameter value from the desired agents.

        :param parameter_id: ID of the parameter to read
        :type parameter_id: str
        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry reading the parameter on an agent when parameter read
            fails, defaults to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try reading the value
            of the parameter on an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of the read parameter message, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.param_request_read_send(
                    agent_id[0], agent_id[1], str.encode(parameter_id), -1
                )
            return

        def post_execution_handler(
            agent_id: AgentID,
            result: bool,
            code: tuple[int, str],
            ack_msg: dict | None,
        ) -> None:
            try:
                if ack_msg is not None:
                    read_param = Parameter(
                        ack_msg["param_id"],
                        ack_msg["param_value"],
                        ack_msg["param_type"],
                        ack_msg["param_index"],
                        ack_msg["param_count"],
                    )
            except KeyError:
                self._logger.debug(
                    "Received an invalid acknowledgement message when reading "
                    f"parameter {parameter_id}"
                )
                return

            self._agents[agent_id].last_params_read.append(read_param)
            return

        return self._send_command(
            agent_ids,
            executor,
            "READ_PARAMETER",
            retry,
            message_timeout,
            ack_timeout,
            ack_packet_type="PARAM_VALUE",
            post_execution_handler=post_execution_handler,
        )

    def set_parameter(
        self,
        parameter_id: str,
        parameter_value: Any,
        parameter_type: int = 9,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Set a parameter on the specified agents.

        If the target agent IDs are not provided, the system will attempt to set the
        specified parameter on all swarm agents.

        :param parameter_id: ID of the parameter to set
        :type parameter_id: str
        :param parameter_value: value to set the parameter to
        :type parameter_value: Any
        :param parameter_type: parameter value type, defaults to 9
        :type parameter_type: int, optional
        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry setting the parameter on an agent when parameter setting
            fails, defaults to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try setting the value
            of the parameter on an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of the set parameter message, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.param_set_send(
                    agent_id[0],
                    agent_id[1],
                    str.encode(parameter_id),
                    parameter_value,
                    parameter_type,
                )
            return

        return self._send_command(
            agent_ids,
            executor,
            "SET_PARAMETER",
            retry,
            message_timeout,
            ack_timeout,
            ack_packet_type="PARAM_VALUE",
        )

    def get_home_position(
        self,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Get the current home position of the swarm agents.

        If the target agent IDs are not provided, the system will attempt to get the
        home position of all swarm agents.

        The home position will be updated in the agent's home position property.

        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry getting the home position of an agent on failure, defaults
            to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try getting the home
            position of an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of the get home position request message, defaults to
            0.5 [s]
        :type ack_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.command_long_send(
                    agent_id[0],
                    agent_id[1],
                    mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            return

        return self._send_command(
            agent_ids,
            executor,
            "GET_HOME_POSITION",
            retry,
            message_timeout,
            ack_timeout,
        )

    def set_home_position(
        self,
        use_current_position: bool = True,
        latitude: float = 0,
        longitude: float = 0,
        altitude: float = 0,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
        verify_state: bool = False,
        lat_lon_deviation_tolerance: float = 0.001,
        altitude_deviation_tolerance: float = 1.0,
        verify_state_timeout: float = 1.0,
    ) -> Future:
        """
        Set the home position of the swarm agents.

        If the target agent IDs are not provided, the system will attempt to set the
        home position of all swarm agents.

        :param use_current_position: use the current position of an agent as its home
            position, defaults to True
        :type use_current_position: bool, optional
        :param latitude: latitude of the home position, defaults to 0
        :type latitude: float, optional
        :param longitude: longitude of the home position, defaults to 0
        :type longitude: float, optional
        :param altitude: altitude of the home position, defaults to 0
        :type altitude: float, optional
        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry setting the home position of an agent on failure, defaults
            to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try setting the home
            position of an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of the set home position request message, defaults to
            0.5 [s]
        :type ack_timeout: float, optional
        :param verify_state: flag indicating whether or not the system should attempt
            to verify that the agent's home position was properly set, defaults to False
        :type verify_state: bool, optional

        :param lat_lon_deviation_tolerance: maximum lat/lon deviation allowed when
            verifying that an agent's home position was set to its current position,
            defaults to 0.001
        :type lat_lon_deviation_tolerance: float, optional
        :param altitude_deviation_tolerance: maximum altitude deviation allowed when
            verifying that an agent's home position was set to its current position,
            defaults to 1.0
        :type altitude_deviation_tolerance: float, optional
        :param verify_state_timeout: maximum amount of time allowed per attempt to
            verify that an agent's home position has been set properly, defaults to
            1.0 [s]
        :type verify_state_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.command_long_send(
                    agent_id[0],
                    agent_id[1],
                    mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                    0,
                    1 if use_current_position else 0,
                    0,
                    0,
                    0,
                    0 if use_current_position else latitude,
                    0 if use_current_position else longitude,
                    0 if use_current_position else altitude,
                )
            return

        if use_current_position:

            def verify_state_changed(agent_id: AgentID):
                ack = True
                start_time = time.time()

                current_location = self._agents[agent_id].location

                while (
                    not math.isclose(
                        current_location.latitude,
                        self._agents[agent_id].home_position.latitude,
                        abs_tol=lat_lon_deviation_tolerance,
                    )
                    or not math.isclose(
                        current_location.latitude,
                        self._agents[agent_id].home_position.latitude,
                        abs_tol=lat_lon_deviation_tolerance,
                    )
                    or not math.isclose(
                        current_location.altitude,
                        self._agents[agent_id].home_position.altitude,
                        abs_tol=altitude_deviation_tolerance,
                    )
                ):
                    if self.__send_message_mutex.acquire(timeout=0.1):
                        try:
                            if self._connection.mavlink_connection is not None:
                                self._connection.mavlink_connection.mav.command_long_send(  # noqa
                                    agent_id[0],
                                    agent_id[1],
                                    mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                )
                        except Exception:
                            self._logger.debug(
                                "An error occurred while verifying state change in the "
                                "set home position command"
                            )
                        finally:
                            self.__send_message_mutex.release()

                    if time.time() - start_time >= verify_state_timeout:
                        ack = False
                        break

                return ack

        else:

            def verify_state_changed(agent_id: AgentID):
                ack = True
                start_time = time.time()

                while (
                    self._agents[agent_id].home_position.latitude != latitude
                    and self._agents[agent_id].home_position.longitude != longitude
                    and self._agents[agent_id].home_position.altitude != altitude
                ):
                    if self.__send_message_mutex.acquire(timeout=0.1):
                        try:
                            if self._connection.mavlink_connection is not None:
                                self._connection.mavlink_connection.mav.command_long_send(  # noqa
                                    agent_id[0],
                                    agent_id[1],
                                    mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                )
                        except Exception:
                            self._logger.debug(
                                "An error occurred while verifying state change in the "
                                "set home position command"
                            )
                        finally:
                            self.__send_message_mutex.release()

                    if time.time() - start_time >= verify_state_timeout:
                        ack = False
                        break

                return ack

        return self._send_command(
            agent_ids,
            executor,
            "SET_HOME_POSITION",
            retry,
            message_timeout,
            ack_timeout,
            state_verifier=verify_state_changed if verify_state else None,
        )

    def goto(
        self,
        latitude: float = 0,
        longitude: float = 0,
        altitude: float = 0,
        hold: float = 0,
        agent_ids: Union[AgentID, list[AgentID]] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Command the agents to go to the desired location.

        Use this cautiously. If multiple agents fly to the same location, there may be
        a collision at this location.

        :param latitude: target latitude, defaults to 0
        :type latitude: float, optional
        :param longitude: target longitude, defaults to 0
        :type longitude: float, optional
        :param altitude: target altitude, defaults to 0
        :type altitude: float, optional
        :param hold: time to stay at waypoint for rotary wing (ignored by fixed wing),
            defaults to 0
        :type hold: float, optional
        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]],
            optional
        :param retry: retry commanding the desired agents to go to the desired location
            on acknowledgement failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try commanding an
            agent to fly to the specified location before a timeout occurs, defaults to
            2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of the goto message, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :return: future message response, if any
        :rtype: Future
        """
        if agent_ids is None or len(agent_ids) > 1:
            self._logger.warning(
                "More than one agent has received a command to fly to the same "
                "location. This may result in a collision."
            )

        def executor(agent_id: AgentID) -> None:
            if self._connection.mavlink_connection is not None:
                self._connection.mavlink_connection.mav.mission_item_send(
                    agent_id[0],
                    agent_id[1],
                    0,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    2,
                    0,
                    hold,
                    0,
                    0,
                    0,
                    latitude,
                    longitude,
                    altitude,
                )
            return

        return self._send_command(
            agent_ids,
            executor,
            "GOTO",
            retry,
            message_timeout,
            ack_timeout,
        )

    def get_agent_by_id(self, agent_id: AgentID) -> Agent | None:
        """
        Get the agent with the specified ID (system ID, component ID).

        :param agent_id: (system ID, component ID)
        :type agent_id: AgentID
        :return: agent, if found
        :rtype: Optional[Agent]
        """
        if agent_id in self._agents:
            return self._agents[agent_id]

        return None

    def add_custom_message_handler(
        self, message: str, callback: MessageHandler
    ) -> None:
        """
        Add a custom message handler for the specified message.

        :param message: message type to call the handler on
        :type message: str
        :param callback: function to call when the message is received
        :type callback: MessageHandler
        """
        self.__message_receivers.add_message_handler(message, callback)

        return

    def __get_expected_agent_ids(self) -> list[AgentID]:
        """
        Get the expected agent IDs in the swarm.

        This is used when the target agents aren't provided and the system needs
        to guess which agents to send the command to.

        :return: filtered agent IDs
        :rtype: list[AgentID]
        """
        # Get the agents that aren't the connection and have a component ID of 1
        # (typically used by the flight controllers)
        def filter_fn(agent: Agent):
            if self._connection.mavlink_connection is not None:
                if (
                    agent.system_id != self._connection.mavlink_connection.source_system
                    and agent.component_id
                    != self._connection.mavlink_connection.source_component
                    and agent.component_id == 1
                ):
                    return True

            return False

        return [
            (agent.system_id, agent.component_id)
            for agent in filter(filter_fn, self.agents)
        ]

    def _send_command(
        self,
        agent_ids: Union[AgentID, list[AgentID]] | None,
        executor: CommandExecutor,
        command_type: str,
        retry: bool,
        message_timeout: float,
        ack_timeout: float,
        ack_packet_type: str = "COMMAND_ACK",
        state_verifier: StateVerifier | None = None,
        post_execution_handler: PostExecutionHandler | None = None,
    ) -> Future:
        """
        Send a command to the desired agents.

        :param agent_ids: agents to send the command to
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]]
        :param executor: function used to execute the command
        :type executor: CommandExecutor
        :param command_type: type of command being executed; used for debugging
        :type command_type: str
        :param retry: retry sending the command on failure
        :type retry: bool
        :param message_timeout: maximum amount of time allowed to try sending the
            command to an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a command, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :param ack_packet_type: packet used to indicate message acknowledgment, defaults
            to "COMMAND_ACK"
        :type ack_packet_type: str, optional
        :param state_verifier: function called to verify that the command resulted in
            the desired changes on an agent, defaults to None
        :type state_verifier: StateVerifier | None, optional
        :param post_execution_handler: function called after successful execution, can
            be used to perform post-processing or handle the acknowledgment message,
            defaults to None
        :type post_execution_handler: PostExecutionHandler | None, optional
        :raises RuntimeError: Attempted to send a message without an active MAVLink
            connection
        :return: future message response, if any
        :rtype: Future
        """
        if not self._connection.connected:
            raise RuntimeError(
                "Attempted to send a message without an active MAVLink connection."
            )

        if agent_ids is None:
            agent_ids = self.__get_expected_agent_ids()

        if self._connection.connected:
            if isinstance(agent_ids, list):
                future = self.__send_message_thread_pool_executor.submit(
                    self.__send_command_list_handler,
                    agent_ids,
                    executor,
                    command_type,
                    retry,
                    ack_packet_type,
                    message_timeout,
                    ack_timeout,
                    state_verifier,
                    post_execution_handler,
                )
            else:
                future = self.__send_message_thread_pool_executor.submit(
                    self.__send_command_handler,  # type: ignore
                    agent_ids,
                    executor,
                    command_type,
                    retry,
                    ack_packet_type,
                    message_timeout,
                    ack_timeout,
                    state_verifier,
                    post_execution_handler,
                )

        return future

    def __send_command_list_handler(
        self,
        agent_ids: list[AgentID],
        executor: CommandExecutor,
        command_type: str,
        retry: bool,
        ack_packet_type: str,
        message_timeout: float,
        ack_timeout: float,
        state_verifier: StateVerifier | None,
        post_execution_handler: PostExecutionHandler | None,
    ) -> list[Response]:
        """
        Handle sending a list of commands.

        :param agent_ids: agents to send the command to
        :type agent_ids: Optional[Union[AgentID, list[AgentID]]]
        :param executor: function used to execute the command
        :type executor: CommandExecutor
        :param command_type: type of command being executed; used for debugging
        :type command_type: str
        :param retry: retry sending the command on failure
        :type retry: bool
        :param message_timeout: maximum amount of time allowed to try sending the
            command to an agent before a timeout occurs
        :type message_timeout: float
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a command
        :type ack_timeout: float
        :param ack_packet_type: packet used to indicate message acknowledgment
        :type ack_packet_type: str
        :param state_verifier: function called to verify that the command resulted in
            the desired changes on an agent
        :type state_verifier: StateVerifier | None
        :param post_execution_handler: function called after successful execution, can
            be used to perform post-processing or handle the acknowledgment message
        :type post_execution_handler: PostExecutionHandler | None
        :return: response of each message sent
        :rtype: list[Response]
        """
        responses: list[Response] = []

        for agent_id in agent_ids:
            responses.append(
                self.__send_command_handler(
                    agent_id,
                    executor,
                    command_type,
                    retry,
                    ack_packet_type,
                    message_timeout,
                    ack_timeout,
                    state_verifier,
                    post_execution_handler,
                )
            )

        return responses

    def __send_command_handler(
        self,
        agent_id: AgentID,
        executor: CommandExecutor,
        command_type: str,
        retry: bool,
        ack_packet_type: str,
        message_timeout: float,
        ack_timeout: float,
        state_verifier: StateVerifier | None,
        post_execution_handler: PostExecutionHandler | None,
    ) -> Response:
        """
        Handle sending a single command.

        :param agent_id: agent to send the command to
        :type agent_id: Optional[Union[AgentID, list[AgentID]]]
        :param executor: function used to execute the command
        :type executor: CommandExecutor
        :param command_type: type of command being executed; used for debugging
        :type command_type: str
        :param retry: retry sending the command on failure
        :type retry: bool
        :param message_timeout: maximum amount of time allowed to try sending the
            command to an agent before a timeout occurs
        :type message_timeout: float
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a command
        :type ack_timeout: float
        :param ack_packet_type: packet used to indicate message acknowledgment
        :type ack_packet_type: str
        :param state_verifier: function called to verify that the command resulted in
            the desired changes on an agent
        :type state_verifier: StateVerifier | None
        :param post_execution_handler: function called after successful execution, can
            be used to perform post-processing or handle the acknowledgment message
        :type post_execution_handler: PostExecutionHandler | None
        :return: message response
        :rtype: Response
        """
        # Determine whether the agent has been recognized
        if agent_id not in self._agents:
            self._logger.info(
                "The current set of registered agents does not include Agent "
                f"({agent_id[0]}, {agent_id[1]}). The provided message will "
                "still be sent; however, the system may not be able to confirm "
                "reception of the message."
            )

        try:
            # Execute the command
            if self.__send_message_mutex.acquire(timeout=0.1):
                try:
                    executor(agent_id)
                except Exception:
                    self._logger.debug(
                        "An error occurred while executing a command", exc_info=True
                    )
                finally:
                    self.__send_message_mutex.release()

            # Get the result of the message and retry if desired
            result, code, ack_msg = self.__get_message_result(
                agent_id,
                executor,
                retry,
                ack_packet_type=ack_packet_type,
                state_verifier=state_verifier,
                message_timeout=message_timeout,
                ack_timeout=ack_timeout,
            )

        except Exception:
            self._logger.exception(
                f"Exception occurred while sending message: {command_type}",
                exc_info=True,
            )
            return Response(
                agent_id,
                command_type,
                False,
                codes.EXCEPTION,
            )

        # Run the post-execution handler
        if result and post_execution_handler is not None:
            post_execution_handler(agent_id, result, code, ack_msg)

        return Response(
            agent_id,
            command_type,
            result,
            code,
        )

    def __get_message_result(
        self,
        agent_id: AgentID,
        executor: CommandExecutor,
        retry: bool,
        ack_packet_type: str,
        state_verifier: StateVerifier | None,
        message_timeout: float,
        ack_timeout: float,
    ) -> tuple[bool, MessageCode, dict | None]:
        """
        Get the result of the command.

        Attempt to acknowledge the message and verify the state of an agent if a state
        verifier is provided. If the retry flag has been set, retry sending the message.

        :param agent_id: agent to send the command to
        :type agent_id: Optional[Union[AgentID, list[AgentID]]]
        :param executor: function used to execute the command
        :type executor: CommandExecutor
        :param retry: retry sending the command on failure
        :type retry: bool
        :param ack_packet_type: packet used to indicate message acknowledgment, defaults
            to "COMMAND_ACK"
        :type ack_packet_type: str, optional
        :param state_verifier: function called to verify that the command resulted in
            the desired changes on an agent, defaults to None
        :type state_verifier: StateVerifier | None, optional
        :param message_timeout: maximum amount of time allowed to try sending the
            command to an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of a command, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :return: message send was successful, message response code, acknowledgement
            message received
        :rtype: Tuple[bool, MessageCode, dict | None]
        """
        ack = False
        code = codes.ACK_FAILURE

        ack, ack_msg = self.__ack_message(ack_packet_type, timeout=ack_timeout)

        if ack:
            code = codes.SUCCESS

            if (
                agent_id[0],
                agent_id[1],
            ) in self._agents and state_verifier is not None:
                ack = state_verifier(agent_id)

                if not ack:
                    code = codes.STATE_VALIDATION_FAILURE

        else:
            code = codes.ACK_FAILURE

        if retry and not ack:
            start_time = time.time()

            while time.time() - start_time <= message_timeout:
                if self.__send_message_mutex.acquire(timeout=0.1):
                    try:
                        executor(agent_id)
                    except Exception:
                        self._logger.debug(
                            "An error occurred while executing a command", exc_info=True
                        )
                    finally:
                        self.__send_message_mutex.release()

                ack, code, ack_msg = self.__get_message_result(
                    agent_id,
                    executor,
                    False,
                    ack_packet_type=ack_packet_type,
                    state_verifier=state_verifier,
                    message_timeout=message_timeout,
                    ack_timeout=ack_timeout,
                )

                if ack:
                    break

        return ack, code, ack_msg

    def __ack_message(
        self, packet_type: str, timeout: float = 0.5
    ) -> tuple[bool, dict | None]:
        """
        Ensure that a distributed message is acknowledged.

        :param packet_type: The type of message that should be acknowledged
        :type packet_type: str
        :param timeout: The acceptable time period before the acknowledgement is
            considered timed out, defaults to 1.0
        :type timeout: float, optional
        :return: acknowledgement success, message received indicating success (if any)
        :rtype: Tuple[bool, Any]
        """
        # Flag indicating whether the message was acknowledged
        ack_success = False

        # Message received converted to a dictionary
        message = None

        # Start acknowledgement timer
        start_t = time.time()

        while time.time() - start_t < timeout:
            if self.__read_message_mutex.acquire(timeout=0.1):
                try:
                    if self._connection.mavlink_connection is not None:
                        message = self._connection.mavlink_connection.recv_match(
                            type=packet_type, blocking=False
                        )
                except Exception:
                    self._logger.debug(
                        "An exception was raised while attempting to acknowledge a "
                        "message.",
                        exc_info=True,
                    )
                finally:
                    self.__read_message_mutex.release()
            else:
                continue

            if message is not None:
                message = message.to_dict()

                if message["mavpackettype"] == packet_type:
                    ack_success = True
                    break

        return ack_success, message

    def __update_agent_timeout_states(self) -> None:
        """Update the timeout status of each agent in the network."""
        for agent in self._agents.values():
            if agent.last_heartbeat.value is not None:
                agent.timeout.value = (
                    monotonic.monotonic() - agent.last_heartbeat.value
                ) >= agent.timeout_period.value

        return

    def __receive_message(self) -> None:
        """Handle incoming messages and distribute them to their respective handlers."""
        while (
            self._connection.connected
            and self._connection.mavlink_connection is not None
        ):
            self.__update_agent_timeout_states()

            message = None

            # Attempt to read the message
            # Note that a timeout has been integrated. Consequently not ALL messages
            # may be received from an agent
            if self.__read_message_mutex.acquire(timeout=0.1):
                try:
                    message = self._connection.mavlink_connection.recv_msg()
                except Exception:
                    self._logger.debug(
                        "An error occurred on MAVLink message reception", exc_info=True
                    )
                    self._logger.debug(
                        "This exception may be raised if a message is being processed "
                        "and a disconnect is attempted"
                    )
                finally:
                    self.__read_message_mutex.release()

            # Continue if the message read was not read properly
            if message is None:
                continue

            # Log the message to the logfile
            if self.__file_logger is not None and message.get_type() != "BAD_DATA":
                self.__file_logger(
                    int(time.time() * 1.0e6),
                    message.get_srcSystem(),
                    message.get_srcComponent(),
                    message.get_type(),
                    message.to_dict(),
                )

            # Execute the respective message handler(s)
            if message.get_type() in self.__message_receivers.receivers:
                for function in self.__message_receivers.receivers[message.get_type()]:
                    with self.__access_agents_mutex:
                        try:
                            function(message, self._agents)
                        except Exception:
                            self._logger.exception(
                                "Exception in message handler for "
                                f"{message.get_type()}",
                                exc_info=True,
                            )

        return

    def __send_heartbeat(self) -> None:
        """Send a GCS heartbeat to the network."""
        HEARTBEAT_FREQ = 1  # Hz

        last_send = time.time()

        while (
            self._connection.connected
            and self._connection.mavlink_connection is not None
        ):
            # Sleep until it's time to resend the message
            if time.time() - last_send < 1 / HEARTBEAT_FREQ:
                time.sleep(1 / HEARTBEAT_FREQ - (time.time() - last_send))

            if self.__send_message_mutex.acquire(timeout=0.1):
                try:
                    self._connection.mavlink_connection.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0,
                        0,
                        0,
                    )
                    last_send = time.time()
                except Exception:
                    self._logger.debug(
                        "An error occurred when sending a MAVLink heartbeat",
                        exc_info=True,
                    )
                finally:
                    self.__send_message_mutex.release()

        return

    def __request_ping_measurement(self) -> None:
        """Send a TIMESYNC to signal ping measurement."""
        TIMESYNC_FREQ = 0.5  # Hz

        last_send = time.time()

        while (
            self._connection.connected
            and self._connection.mavlink_connection is not None
        ):
            # Sleep until it's time to resend the message
            if time.time() - last_send < 1 / TIMESYNC_FREQ:
                time.sleep(1 / TIMESYNC_FREQ - (time.time() - last_send))

            if self.__send_message_mutex.acquire(timeout=0.1):
                try:
                    # Construct ts1
                    # We use the top 10 bytes for the timestamp and the bottom 6
                    # for the source system and source component stamps
                    sys_time = str(int(time.time() * 1e6))[:10]
                    src_sys = str(
                        self._connection.mavlink_connection.source_system
                    ).zfill(3)
                    src_comp = str(
                        self._connection.mavlink_connection.source_component
                    ).zfill(3)
                    ts1 = int(f"{sys_time}{src_sys}{src_comp}")

                    self._connection.mavlink_connection.mav.timesync_send(0, ts1)
                    last_send = time.time()
                except Exception:
                    self._logger.debug(
                        "An error occurred when sending a timesync request to the "
                        "swarm agents",
                        exc_info=True,
                    )
                finally:
                    self.__send_message_mutex.release()

        return

    def __request_system_time(self, operation: str, key: AgentID, value: Agent) -> None:
        """
        Request the system time from the agent at the desired rate.

        :param operation: _description_
        :type operation: str
        :param agent_id: _description_
        :type agent_id: AgentID
        :param agent: _description_
        :type agent: Agent
        """
        REQUEST_FREQ = 0.5  # Hz

        if operation == "set" and self._connection.mavlink_connection is not None:
            # Request that the agent broadcast its system time at the target interval
            # Note that we request that it broadcast to reduce reduncancy in the
            # case where multiple agents create a request for this message

            def executor(agent_id: AgentID) -> None:
                if self._connection.mavlink_connection is not None:
                    self._connection.mavlink_connection.mav.command_long_send(
                        agent_id[0],
                        agent_id[1],
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        0,
                        mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME,
                        1e6 / REQUEST_FREQ,
                        0,
                        0,
                        0,
                        0,
                        2,
                    )
                return

            future = self._send_command(
                key,
                executor,
                "SET_MESSAGE_INTERVAL",
                True,
                2.5,
                0.5,
            )

            while not future.done():
                pass

            response = future.result()

            if not response.result:
                self._logger.warning(
                    "Unable to set the message interval of the SYSTEM_TIME message on "
                    f"agent {response.target_agent_id}. This warning can be safely "
                    "ignored if the target ID is not an actual agent in the swarm or "
                    "if state estimation is not required."
                )
        return

    def __measure_ping(self, message: Any, agents: dict[AgentID, Agent]) -> None:
        """
        Measure the message latency between the source and the agent.

        This has been implemented externally to the receivers because of its dependence
        on the MavSwarm ID.

        :param message: Incoming MAVLink message
        :type message: Any
        :param agents: agents in the swarm
        :type agents: dict[AgentID, Agent]
        """
        receive_time = time.time()

        try:
            if (
                self._connection.mavlink_connection is not None
                and int(str(message.ts1)[-6:-3])
                == self._connection.mavlink_connection.source_system
                and int(str(message.ts1)[-2:])
                == self._connection.mavlink_connection.source_component
            ):
                agent_id = (message.get_srcSystem(), message.get_srcComponent())

                ping = int((receive_time - float(str(message.ts1)[:-6])) * 1000)
                agents[agent_id].ping.value = ping

                # Log the ping to the log file
                if self.__file_logger is not None:
                    self.__file_logger(
                        int(time.time() * 1.0e6),
                        message.get_srcSystem(),
                        message.get_srcComponent(),
                        "PING",
                        {"ping": ping},
                    )
        except Exception:
            self._logger.debug(
                "An error occurred while attempting to handle the time sync message"
            )

        return

    def __sync_clocks(self, message: Any, agents: dict[AgentID, Agent]) -> None:
        """
        Measure the offset between the agent and source clocks.

        This has been implemented externally from the message receivers because of its
        dependence on the time since boot.

        :param message: Incoming MAVLink message
        :type message: Any
        :param agents: agents in the swarm
        :type agents: dict[AgentID, Agent]
        """
        agent_id = (message.get_srcSystem(), message.get_srcComponent())

        if self.time_since_boot is not None and agent_id in agents:
            # We make the assumption that the latency is equivalent in both directions
            agents[agent_id].update_clock_offset(
                message.time_boot_ms
                - int(agents[agent_id].ping.value / 2)
                - self.time_since_boot
            )

        return
