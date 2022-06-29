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

import atexit
import logging
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Callable, Dict, List, Optional, Tuple, Union

import monotonic
from pymavlink import mavutil

import pymavswarm.utils as swarm_utils
from pymavswarm import Connection
from pymavswarm.agent import Agent
from pymavswarm.handlers import MessageReceivers
from pymavswarm.messages import results
from pymavswarm.messages.response import Response
from pymavswarm.utils import Event


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
    ) -> None:
        """
        Construct MavSwarm interface.

        :param max_workers: maximum number of workers available in the thread pool used
            to send messages, defaults to 5
        :type: max_workers: int, optional
        :param log_level: log level of the system, defaults to logging.INFO
        :type log_level: int, optional
        """
        super().__init__()

        self.__logger = swarm_utils.init_logger(__name__, log_level=log_level)
        self.__connection = Connection(log_level=log_level)
        self.__agents: Dict[Tuple[int, int], Agent] = {}
        self.__agent_list_changed = Event()

        self.__message_receivers = MessageReceivers(log_level=log_level)

        # Mutexes
        self.__read_message_mutex = threading.RLock()
        self.__send_message_mutex = threading.RLock()

        self.__incoming_message_thread = threading.Thread(target=self.__receive_message)
        self.__incoming_message_thread.daemon = True

        # Thread pool for sending messages
        self.__send_message_thread_pool_executor = ThreadPoolExecutor(
            max_workers=max_workers
        )

        # Register the exit callback
        atexit.register(self.disconnect)

        return

    @property
    def agents(self) -> List[Agent]:
        """
        List of agents in the swarm.

        :return: swarm agents
        :rtype: List[Agent]
        """
        return [*self.__agents.values()]

    @property
    def agent_list_changed(self) -> Event:
        """
        Event signalling that the list of agents in the swarm has changed.

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
        return self.__connection.connected

    @property
    def supported_modes(self) -> Dict[str, int]:
        """
        List of supported flight modes.

        :return: supported flight modes
        :rtype: Dict[str, int]
        """
        return self.__connection.mavlink_connection.mode_mapping()

    def connect(
        self,
        port: str,
        baudrate: int,
        source_system: int,
        source_component: int,
        connection_attempt_timeout: float,
    ) -> bool:
        """
        Connect to the network.

        Attempt to establish a MAVLink connection using the provided configurations.

        :param port: serial port to attempt connection on
        :type port: str
        :param baudrate: serial connection baudrate
        :type baudrate: int
        :param source_system: system ID for the source system, defaults to 255
        :type source_system: int, optional
        :param source_component: component ID for the source system, defaults to 0
        :type source_component: int, optional
        :param connection_attempt_timeout: maximum time taken to establish a connection,
            defaults to 2.0 [s]
        :type connection_attempt_timeout: float, optional
        :return: flag indicating whether connection was successful
        :rtype: bool
        """
        if not self.__connection.connect(
            port, baudrate, source_system, source_component, connection_attempt_timeout
        ):
            return False

        # Start threads
        self.__incoming_message_thread.start()

        return True

    def disconnect(self) -> None:
        """Disconnect from the MAVLink network and shutdown all services."""
        self.__connection.disconnect()

        if self.__incoming_message_thread is not None:
            self.__incoming_message_thread.join()

        # Shutdown the thread pool executor
        self.__send_message_thread_pool_executor.shutdown(cancel_futures=True)

        # Clear the agents list
        self.__agents.clear()
        self.__agent_list_changed.listeners.clear()

        return

    def arm(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
        validate_state_timeout: float = 1.0,
    ) -> Optional[Future]:
        def executor(agent_id: Tuple[int, int]) -> None:
            self.__connection.mavlink_connection.mav.command_long_send(
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
        def verify_state_changed(agent_id: Tuple[int, int]) -> bool:
            ack = True
            start_time = time.time()

            while not self.__agents[agent_id].armed.value:
                if time.time() - start_time >= validate_state_timeout:
                    ack = False
                    break

            return ack

        return self.__send_command(
            agent_ids,
            executor,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            retry,
            message_timeout,
            ack_timeout,
            state_verifier=verify_state_changed,
        )

    def disarm(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
        validate_state_timeout: float = 1.0,
    ) -> Optional[Future]:
        def executor(agent_id: Tuple[int, int]) -> None:
            self.__connection.mavlink_connection.mav.command_long_send(
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
        def verify_state_changed(agent_id: Tuple[int, int]) -> bool:
            ack = True
            start_time = time.time()

            while self.__agents[agent_id].armed.value:
                if time.time() - start_time >= validate_state_timeout:
                    ack = False
                    break

            return ack

        return self.__send_command(
            agent_ids,
            executor,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            retry,
            message_timeout,
            ack_timeout,
            state_verifier=verify_state_changed,
        )

    def reboot(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Optional[Future]:
        def executor(agent_id: Tuple[int, int]) -> None:
            self.__connection.mavlink_connection.mav.command_long_send(
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

        return self.__send_command(
            agent_ids,
            executor,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            retry,
            message_timeout,
            ack_timeout,
        )

    def shutdown(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Optional[Future]:
        def executor(agent_id: Tuple[int, int]) -> None:
            self.__connection.mavlink_connection.mav.command_long_send(
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

        return self.__send_command(
            agent_ids,
            executor,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            retry,
            message_timeout,
            ack_timeout,
        )

    def set_mode(
        self,
        flight_mode: str,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
        validate_state_timeout: float = 1.0,
    ) -> Optional[Future]:
        def executor(agent_id: Tuple[int, int]) -> None:
            # Reset target
            self.__connection.mavlink_connection.target_system = agent_id[0]
            self.__connection.mavlink_connection.target_component = agent_id[1]

            # Send flight mode
            self.__connection.mavlink_connection.set_mode(
                self.__connection.mavlink_connection.mode_mapping()[flight_mode]
            )
            return

        # Construct a method to use for verifying the state change
        def verify_state_changed(agent_id: Tuple[int, int]) -> bool:
            ack = True
            start_time = time.time()

            while self.__agents[agent_id].mode.value != flight_mode:
                if time.time() - start_time >= validate_state_timeout:
                    ack = False
                    break

            return ack

        return self.__send_command(
            agent_ids,
            executor,
            "FLIGHT_MODE",
            retry,
            message_timeout,
            ack_timeout,
            state_verifier=verify_state_changed,
        )

    def set_airspeed(
        self,
        speed: float,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Optional[Future]:
        def executor(agent_id: Tuple[int, int]) -> None:
            self.__connection.mavlink_connection.mav.command_long_send(
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

        return self.__send_command(
            agent_ids,
            executor,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            retry,
            message_timeout,
            ack_timeout,
        )

    def set_groundspeed(
        self,
        speed: float,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Optional[Future]:
        def executor(agent_id: Tuple[int, int]) -> None:
            self.__connection.mavlink_connection.mav.command_long_send(
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

        return self.__send_command(
            agent_ids,
            executor,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            retry,
            message_timeout,
            ack_timeout,
        )

    def gyroscope_calibration(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Optional[Future]:
        def executor(agent_id: Tuple[int, int]) -> None:
            self.__connection.mavlink_connection.mav.command_long_send(
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

        return self.__send_command(
            agent_ids,
            executor,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            retry,
            message_timeout,
            ack_timeout,
        )

    def magnetometer_calibration(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Optional[Future]:
        def executor(agent_id: Tuple[int, int]) -> None:
            self.__connection.mavlink_connection.mav.command_long_send(
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

        return self.__send_command(
            agent_ids,
            executor,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            retry,
            message_timeout,
            ack_timeout,
        )

    def ground_pressure_calibration(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Optional[Future]:
        def executor(agent_id: Tuple[int, int]) -> None:
            self.__connection.mavlink_connection.mav.command_long_send(
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

        return self.__send_command(
            agent_ids,
            executor,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            retry,
            message_timeout,
            ack_timeout,
        )

    def airspeed_calibration(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Optional[Future]:
        def executor(agent_id: Tuple[int, int]) -> None:
            self.__connection.mavlink_connection.mav.command_long_send(
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

        return self.__send_command(
            agent_ids,
            executor,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            retry,
            message_timeout,
            ack_timeout,
        )

    def barometer_temperature_calibration(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Optional[Future]:
        def executor(agent_id: Tuple[int, int]) -> None:
            self.__connection.mavlink_connection.mav.command_long_send(
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

        return self.__send_command(
            agent_ids,
            executor,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            retry,
            message_timeout,
            ack_timeout,
        )

    def accelerometer_calibration(
        self,
        simple_calibration: bool = True,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Optional[Future]:
        def executor(agent_id: Tuple[int, int]) -> None:
            self.__connection.mavlink_connection.mav.command_long_send(
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

        return self.__send_command(
            agent_ids,
            executor,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            retry,
            message_timeout,
            ack_timeout,
        )

    def ahrs_trim(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Optional[Future]:
        def executor(agent_id: Tuple[int, int]) -> None:
            self.__connection.mavlink_connection.mav.command_long_send(
                agent_id[0],
                agent_id[1],
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0,
                0,
                0,
                0,
                0,
                2,
                0,
                0,
            )
            return

        return self.__send_command(
            agent_ids,
            executor,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            retry,
            message_timeout,
            ack_timeout,
        )

    def takeoff(self) -> Future:
        pass

    def takeoff_sequence(self) -> Future:
        pass

    def read_parameter(self) -> Future:
        pass

    def set_parameter(self) -> Future:
        pass

    def get_home_position(self) -> Future:
        pass

    def set_home_position(self) -> Future:
        pass

    def goto(self) -> Future:
        pass

    def send_debug_message(self) -> Future:
        pass

    def create_cluster(self):
        pass

    def get_agent_by_id(self, agent_id: Tuple[int, int]) -> Optional[Agent]:
        """
        Get the agent with the specified ID (system ID, component ID).

        :param agent_id: (system ID, component ID)
        :type agent_id: Tuple[int, int]
        :return: agent, if found
        :rtype: Optional[Agent]
        """
        if agent_id in self.__agents:
            return self.__agents[agent_id]

        return None

    def get_agent_by_name(self, name: str) -> Optional[Agent]:
        """
        Get an agent by its name.

        Get the first agent in the swarm with the specified name.

        :param name: name of the agent to access
        :type name: str
        :return: first agent identified with the given name
        :rtype: Optional[Agent]
        """
        for agent in self.__agents.values():
            if agent.name == name:
                return agent

        return None

    def get_filtered_agents(
        self,
        ignore_gcs: bool = True,
        ignore_agent_ids: Optional[List[Tuple[int, int]]] = None,
        ignore_system_ids: Optional[List[int]] = None,
        ignore_comp_ids: Optional[List[int]] = None,
    ) -> List[Agent]:
        """
        Get a filtered list of agents.

        :param ignore_gcs: ignore the ground control station agent, defaults to True
        :type ignore_gcs: bool, optional
        :param ignore_agent_ids: list of agent IDs to ignore, defaults to None
        :type ignore_agent_ids: Optional[List[Tuple[int, int]]], optional
        :param ignore_system_ids: list of system IDs to ignore, defaults to None
        :type ignore_system_ids: Optional[List[int]], optional
        :param ignore_comp_ids: list of component IDs to ignore, defaults to None
        :type ignore_comp_ids: Optional[List[int]], optional
        :return: list of filtered agents
        :rtype: List[Agent]
        """
        agents: List[Agent] = []

        for agent in self.__agents.values():
            # Check if the agent is the GCS
            if ignore_gcs:
                if (
                    agent.system_id == self.__connection.source_system
                    and agent.component_id == self.__connection.source_component
                ):
                    continue

            # Check if the agent is in the list of agent IDs to ignore
            if ignore_agent_ids is not None:
                if (agent.system_id, agent.component_id) in ignore_agent_ids:
                    continue

            # Check if the agent's system ID is in the list of system IDs to ignore
            if ignore_system_ids is not None:
                if agent.system_id in ignore_system_ids:
                    continue

            # Check if the agent's component ID is in the list of component IDs to
            # ignore
            if ignore_comp_ids is not None:
                if agent.component_id in ignore_comp_ids:
                    continue

            agents.append(agent)

        return agents

    def get_filtered_agent_ids(
        self,
        ignore_gcs: bool = True,
        ignore_agent_ids: Optional[List[Tuple[int, int]]] = None,
        ignore_system_ids: Optional[List[int]] = None,
        ignore_comp_ids: Optional[List[int]] = None,
    ) -> List[Tuple[int, int]]:
        """
        Get a filtered list of agent IDs.

        :param ignore_gcs: ignore the ground control station agent, defaults to True
        :type ignore_gcs: bool, optional
        :param ignore_agent_ids: list of agent IDs to ignore, defaults to None
        :type ignore_agent_ids: Optional[List[Tuple[int, int]]], optional
        :param ignore_system_ids: list of system IDs to ignore, defaults to None
        :type ignore_system_ids: Optional[List[int]], optional
        :param ignore_comp_ids: list of component IDs to ignore, defaults to None
        :type ignore_comp_ids: Optional[List[int]], optional
        :return: list of filtered agents
        :rtype: List[Tuple[int, int]]
        """
        agents: List[Tuple[int, int]] = []

        for agent in self.__agents.values():
            # Check if the agent is the GCS
            if ignore_gcs:
                if (
                    agent.system_id == self.__connection.source_system
                    and agent.component_id == self.__connection.source_component
                ):
                    continue

            # Check if the agent is in the list of agent IDs to ignore
            if ignore_agent_ids is not None:
                if (agent.system_id, agent.component_id) in ignore_agent_ids:
                    continue

            # Check if the agent's system ID is in the list of system IDs to ignore
            if ignore_system_ids is not None:
                if agent.system_id in ignore_system_ids:
                    continue

            # Check if the agent's component ID is in the list of component IDs to
            # ignore
            if ignore_comp_ids is not None:
                if agent.component_id in ignore_comp_ids:
                    continue

            agents.append((agent.system_id, agent.component_id))

        return agents

    def __send_command(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
        executor: Callable,
        command_type: str,
        retry: bool,
        message_timeout: float,
        ack_timeout: float,
        ack_packet_type: str = "COMMAND_ACK",
        state_verifier: Optional[Callable] = None,
    ) -> Optional[Future]:
        if not self.__connection.connected:
            self.__logger.error(
                "Attempted to send a message without an active MAVLink connection."
            )
            return None

        if agent_ids is None:
            agent_ids = self.get_filtered_agent_ids(ignore_gcs=True)

        if self.__connection.connected:
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
                )
            else:
                future = self.__send_message_thread_pool_executor.submit(
                    self.__send_message_handler,  # type: ignore
                    agent_ids,
                    executor,
                    command_type,
                    retry,
                    ack_packet_type,
                    message_timeout,
                    ack_timeout,
                    state_verifier,
                )

        return future

    def __send_command_list_handler(
        self,
        agent_ids: List[Tuple[int, int]],
        executor: Callable,
        command_type: str,
        retry: bool,
        ack_packet_type: str,
        message_timeout: float,
        ack_timeout: float,
        state_verifier: Optional[Callable],
    ) -> List[Response]:
        responses: List[Response] = []

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
                )
            )

        return responses

    def __send_command_handler(
        self,
        agent_id: Tuple[int, int],
        executor: Callable,
        command_type: str,
        retry: bool,
        ack_packet_type: str,
        message_timeout: float,
        ack_timeout: float,
        state_verifier: Optional[Callable],
    ) -> Response:
        target_system = agent_id[0]
        target_component = agent_id[1]

        # Determine whether the agent has been recognized
        if agent_id not in self.__agents:
            self.__logger.info(
                "The current set of registered agents does not include Agent "
                f"({target_system}, {target_component}). The provided message will "
                "still be sent; however, the system may not be able to confirm "
                "reception of the message."
            )

        with self.__send_message_mutex:
            try:
                executor(agent_id)

                result, code, _ = self.__get_message_result(
                    agent_id,
                    executor,
                    retry,
                    ack_packet_type=ack_packet_type,
                    state_verifier=state_verifier,
                    message_timeout=message_timeout,
                    ack_timeout=ack_timeout,
                )
            except Exception:
                self.__logger.exception(
                    f"Exception occurred while sending message: {command_type}",
                    exc_info=True,
                )
                return Response(
                    target_system,
                    target_component,
                    command_type,
                    False,
                    results.EXCEPTION,
                )

        return Response(
            target_system,
            target_component,
            command_type,
            result,
            code,
        )

    def __get_message_result(
        self,
        agent_id: Tuple[int, int],
        executor: Callable,
        retry: bool,
        ack_packet_type: str,
        state_verifier: Optional[Callable],
        message_timeout: float,
        ack_timeout: float,
    ) -> Tuple[bool, Tuple[int, str], Optional[dict]]:
        ack = False
        code = results.ACK_FAILURE

        ack, ack_msg = self.__ack_message(ack_packet_type, timeout=ack_timeout)

        if ack:
            code = results.SUCCESS

            if (
                agent_id[0],
                agent_id[1],
            ) in self.__agents and state_verifier is not None:
                ack = state_verifier(agent_id, self.__agents)

                if not ack:
                    code = results.STATE_VALIDATION_FAILURE

        else:
            code = results.ACK_FAILURE

        if retry and not ack:
            start_time = time.time()

            while time.time() - start_time <= message_timeout:
                executor(agent_id)

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
    ) -> Tuple[bool, Optional[dict]]:
        """
        Ensure that a distributed message is acknowledged.

        :param message_type: The type of message that should be acknowledged
        :type message_type: str
        :param timeout: The acceptable time period before the acknowledgement is
            considered timed out, defaults to 1.0
        :type timeout: float, optional
        :return: acknowledgement success, message received indicating success (if any)
        :rtype: Tuple[bool, Any]
        """
        # Attempt to acquire the mutex
        if not self.__read_message_mutex.acquire(timeout=1.0):
            return False, None

        # Flag indicating whether the message was acknowledged
        ack_success = False

        # Message received converted to a dictionary
        message = None

        # Start acknowledgement timer
        start_t = time.time()

        while time.time() - start_t < timeout:
            try:
                message = self.__connection.mavlink_connection.recv_match(
                    type=packet_type, blocking=False
                )
                message = message.to_dict()

                if message["mavpackettype"] == packet_type:
                    ack_success = True
                    break
            except Exception:
                # This is a catch-all to ensure that the system continues attempting
                # acknowledgement
                continue

        # Continue reading status messages
        self.__read_message_mutex.release()

        return ack_success, message

    def __update_agent_timeout_states(self) -> None:
        """Update the timeout status of each agent in the network."""
        for agent in self.__agents.values():
            if agent.last_heartbeat.value is not None:
                agent.timeout.value = (
                    monotonic.monotonic() - agent.last_heartbeat.value
                ) >= agent.timeout_period.value

        return

    def __receive_message(self) -> None:
        """Handle incoming messages and distribute them to their respective handlers."""
        while (
            self.__connection.connected
            and self.__connection.mavlink_connection is not None
        ):
            self.__update_agent_timeout_states()

            message = None

            # Attempt to read the message
            # Note that a timeout has been integrated. Consequently not ALL messages
            # may be received from an agent
            if self.__read_message_mutex.acquire(timeout=0.1):
                try:
                    message = self.__connection.mavlink_connection.recv_msg()
                except Exception:
                    self.__logger.debug(
                        "An error occurred on MAVLink message reception", exc_info=True
                    )
                finally:
                    self.__read_message_mutex.release()

            # Continue if the message read was not read properly
            if message is None:
                continue

            # Execute the respective message handler(s)
            if message.get_type() in self.__message_receivers.receivers:
                for function in self.__message_receivers.receivers[message.get_type()]:
                    try:
                        function(message, self.__agents)
                    except Exception:
                        self.__logger.exception(
                            f"Exception in message handler for {message.get_type()}",
                            exc_info=True,
                        )

        return

    def __timer(self) -> Callable:
        """
        Log the time that a sender takes to complete. Used for debugging purposes.

        :return: decorator
        :rtype: Callable
        """

        def decorator(function: Callable) -> Callable:
            def wrapper(*args):
                start_t = time.time()
                response = function(*args)
                self.__logger.debug(
                    f"Time taken to execute function: {time.time() - start_t}s"
                )
                return response

            return wrapper

        return decorator
