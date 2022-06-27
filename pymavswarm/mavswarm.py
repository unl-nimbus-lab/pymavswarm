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
from pymavswarm.messages.message_wrapper import MessageWrapper
from pymavswarm.messages.response import Response, message_results
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
    ) -> Future:
        """
        Arm the desired agent(s).

        Arm each agent in the swarm. If specific agent IDs are provided, arm only the
        agents with the specified IDs.

        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry sending message on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        # Construct a method to use for verifying state change
        def verify_state_changed(
            message: MessageWrapper,
            agents: Dict[Tuple[int, int], Agent],
        ):
            ack = True
            start_time = time.time()

            while not agents[
                (message.target_system, message.target_component)
            ].armed.value:
                if time.time() - start_time >= message.state_timeout:
                    ack = False
                    break

            return ack

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
            state_verification_function=verify_state_changed,
        )

    def disarm(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Disarm the desired agent(s).

        Disarm each agent in the swarm. If specific agent IDs are provided, disarm only
        the agents with the specified IDs.

        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry sending message on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        # Construct a method to use for verifying state change
        def verify_state_changed(
            message: MessageWrapper,
            agents: Dict[Tuple[int, int], Agent],
        ):
            ack = True
            start_time = time.time()

            while agents[(message.target_system, message.target_component)].armed.value:
                if time.time() - start_time >= message.state_timeout:
                    ack = False
                    break

            return ack

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
            state_verification_function=verify_state_changed,
        )

    def reboot(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Reboot the desired agent(s).

        Reboot each agent in the swarm. If specific agent IDs are provided, reboot only
        the agents with the specified IDs.

        State verification is not provided for this command.

        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry sending message on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
        )

    def shutdown(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Shutdown the desired agent(s).

        Shutdown each agent in the swarm. If specific agent IDs are provided, reboot
        only the agents with the specified IDs.

        State verification is not provided for this command.

        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry sending message on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
        )

    def set_mode(
        self,
        mode: Union[str, int],
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Optional[Future]:
        """
        Set the flight mode on the desired agent(s).

        If specific agent IDs are provided, set the flight mode of only the agents with
        the specified IDs.

        :param mode: desired flight mode to switch into
        :type mode: Union[str, int]
        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry sending message on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Optional[Future]
        """
        if isinstance(mode, str):
            if mode not in self.__connection.mavlink_connection.mode_mapping():
                self.__logger.exception(
                    f"Attempted to set an invalid flight mode: {mode}"
                )
                return None

            mode = self.__connection.mavlink_connection.mode_mapping()[mode]

        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
                    agent_id[0],
                    agent_id[1],
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            )

            return mavlink_message

        # Construct a method to use for verifying the state change
        def verify_state_changed(
            message: MessageWrapper,
            agents: Dict[Tuple[int, int], Agent],
        ):
            ack = True
            start_time = time.time()

            while (
                agents[(message.target_system, message.target_component)].mode.value
                != mode
            ):
                if time.time() - start_time >= message.state_timeout:
                    ack = False
                    break

            return ack

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
            state_verification_function=verify_state_changed,
        )

    def set_airspeed(
        self,
        speed: float,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Change airspeed.

        Change the speed of each agent in the swarm. If specific agent IDs are provided,
        change the speed of only the agents with the specified IDs.

        State verification is not provided for this command.

        :param speed: desired airspeed [m/s]
        :type speed: float
        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry sending message on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
        )

    def set_groundspeed(
        self,
        speed: float,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Change groundspeed.

        Change the speed of each agent in the swarm. If specific agent IDs are provided,
        change the speed of only the agents with the specified IDs.

        State verification is not provided for this command.

        :param speed: desired groundspeed [m/s]
        :type speed: float
        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry sending message on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
        )

    def takeoff(
        self,
        altitude: float,
        latitude: Optional[float] = None,
        longitude: Optional[float] = None,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Execute a takeoff command (not a full sequence).

        Note that acknowledgement of this command does not indicate that the
        altitude was reached, but rather that the system will attempt to reach
        the specified altitude.

        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry sending message on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        if latitude is None or longitude is None:
            self.__logger.info(
                "Latitude and longitude were not provided for the takeoff command. "
                "Attempting to takeoff to the desired altitude."
            )
            latitude = 0
            longitude = 0

        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
        )

    def takeoff_sequence(
        self,
        altitude: float,
        latitude: Optional[float] = None,
        longitude: Optional[float] = None,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
        state_transition_delay: float = 5.0,
    ) -> Future:
        """
        Execute a full takeoff sequence.

        Command used to signal execution of a full takeoff sequence:
            1. Switch to GUIDED mode
            2. Arm
            3. Takeoff

        The result of this command does not indicate whether the agent successfully
        took of to the desired altitude, but rather indicates whether or not the message
        acknowledged.

        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry sending message on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        if latitude is None or longitude is None:
            self.__logger.info(
                "Latitude and longitude were not provided for the takeoff command. "
                "Attempting to takeoff to the desired altitude."
            )
            latitude = 0
            longitude = 0

        guided_future = self.set_mode("GUIDED")

        if guided_future is None:
            pass

        while not guided_future.done():
            pass

        if not guided_future.result().result:
            pass

    def read_parameter(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Arm the desired agent(s).

        Arm each agent in the swarm. If specific agent IDs are provided, arm only the
        agents with the specified IDs.

        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry arming an agent on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
        )

    def set_parameter(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Arm the desired agent(s).

        Arm each agent in the swarm. If specific agent IDs are provided, arm only the
        agents with the specified IDs.

        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry arming an agent on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
        )

    def get_home_position(self):
        pass

    def set_home_position(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Arm the desired agent(s).

        Arm each agent in the swarm. If specific agent IDs are provided, arm only the
        agents with the specified IDs.

        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry arming an agent on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
        )

    def gyroscope_calibration(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Arm the desired agent(s).

        Arm each agent in the swarm. If specific agent IDs are provided, arm only the
        agents with the specified IDs.

        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry arming an agent on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
        )

    def magnetometer_calibration(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Arm the desired agent(s).

        Arm each agent in the swarm. If specific agent IDs are provided, arm only the
        agents with the specified IDs.

        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry arming an agent on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
        )

    def ground_pressure_calibration(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Arm the desired agent(s).

        Arm each agent in the swarm. If specific agent IDs are provided, arm only the
        agents with the specified IDs.

        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry arming an agent on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
        )

    def airspeed_calibration(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Arm the desired agent(s).

        Arm each agent in the swarm. If specific agent IDs are provided, arm only the
        agents with the specified IDs.

        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry arming an agent on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
        )

    def barometer_temperature_calibration(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Arm the desired agent(s).

        Arm each agent in the swarm. If specific agent IDs are provided, arm only the
        agents with the specified IDs.

        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry arming an agent on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
        )

    def accelerometer_calibration(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Arm the desired agent(s).

        Arm each agent in the swarm. If specific agent IDs are provided, arm only the
        agents with the specified IDs.

        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry arming an agent on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
        )

    def ahrs_trim(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Arm the desired agent(s).

        Arm each agent in the swarm. If specific agent IDs are provided, arm only the
        agents with the specified IDs.

        :param agent_ids: (system ID, component ID) of each agent that should be armed,
            this may be a single agent ID or a list of agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry arming an agent on failure, defaults to False
        :type retry: bool, optional
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 2.5
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 0.5
        :type ack_timeout: float, optional
        :return: future to be populated with a message response
        :rtype: Future
        """
        # Create a helper method to get a message wrapper
        def get_mavlink_message(agent_id: Tuple[int, int]):
            mavlink_message = (
                self.__connection.mavlink_connection.mav.command_long_encode(
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
            )

            return mavlink_message

        return self.__construct_and_send_message_wrappers(
            agent_ids,
            retry,
            message_timeout,
            ack_timeout,
            get_mavlink_message,
        )

    def goto(self):
        pass

    def send_debug_message(self):
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

    def __construct_and_send_message_wrappers(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
        retry: bool,
        message_timeout: float,
        ack_timeout: float,
        mavlink_message_generator: Callable,
        state_verification_function: Optional[Callable] = None,
    ) -> Future:
        """
        Construct a message wrapper for each of the target agents and send the messages.

        :param agent_ids: _description_
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]]
        :param retry: _description_
        :type retry: bool
        :param message_timeout: _description_
        :type message_timeout: float
        :param ack_timeout: _description_
        :type ack_timeout: float
        :param mavlink_message_generator: _description_
        :type mavlink_message_generator: Callable
        :param state_verification_function: _description_, defaults to None
        :type state_verification_function: Optional[Callable], optional
        :return: _description_
        :rtype: Future
        """
        target_agents: List[Tuple[int, int]] = []

        if agent_ids is None:
            target_agents = self.get_filtered_agent_ids()
        else:
            if isinstance(agent_ids, list):
                target_agents = agent_ids
            else:
                target_agents = [agent_ids]

        messages: List[MessageWrapper] = []

        for agent_id in target_agents:
            messages.append(
                MessageWrapper(
                    agent_id[0],
                    agent_id[1],
                    mavlink_message_generator(agent_id),
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    retry,
                    message_timeout=message_timeout,
                    ack_timeout=ack_timeout,
                )
            )
        return self.__send_command(messages, state_verification_function)

    def __send_command(
        self,
        message: Union[MessageWrapper, List[MessageWrapper]],
        state_verification_function: Optional[Callable] = None,
    ) -> Future:
        """
        Send a message.

        :param message: message to send
        :type message: Union[Any, List[Any]]
        :param state_verification_function: function used to verify that the target
            state was properly modified, defaults to None
        :type state_verification_function: Optional[Callable], optional
        :return: future message response
        :rtype: Future
        """
        if self.__connection.connected:
            if isinstance(message, list):
                future = self.__send_message_thread_pool_executor.submit(
                    self.__send_command_list_handler,
                    message,
                    state_verification_function,
                )
            else:
                future = self.__send_message_thread_pool_executor.submit(
                    self.__send_message_handler,  # type: ignore
                    message,
                    state_verification_function,
                )

        return future

    def __send_command_list_handler(
        self,
        messages: List[MessageWrapper],
        state_verification_function: Optional[Callable] = None,
    ) -> List[Response]:
        """
        Send a list of commands.

        :param messages: list of messages to send
        :type messages: List[Message]
        :param state_verification_function: function used to verify that the target
            state was properly modified, defaults to None
        :type state_verification_function: Optional[Callable], optional
        :return: list of message responses
        :rtype: List[Response]
        """
        responses: List[Response] = []

        for message in messages:
            responses.append(
                self.__send_command_handler(message, state_verification_function)
            )

        return responses

    def __send_command_handler(
        self,
        message: MessageWrapper,
        state_verification_function: Optional[Callable] = None,
    ) -> Response:
        """
        Send a MAVLink command.

        :param message: MAVLink message wrapper
        :type message: Message
        :param state_verification_function: function used to verify that the target
            state was properly modified, defaults to None
        :type state_verification_function: Optional[Callable], optional
        :return: message response
        :rtype: Response
        """
        # Determine whether the agent has been recognized
        if (message.target_system, message.target_component) not in self.__agents:
            self.__logger.info(
                "The current set of registered agents does not include Agent "
                f"({message.target_system}, {message.target_component}). The provided "
                "message will still be sent; however, the system may not be able to "
                "confirm reception of the message.",
            )

        with self.__send_message_mutex:
            try:
                self.__connection.mavlink_connection.mav.send(message.mavlink_message)

                result, code, _ = self.__get_message_result(
                    message, state_verification_function=state_verification_function
                )
            except Exception:
                self.__logger.exception(
                    f"Exception occurred while sending message: {message.message_type}",
                    exc_info=True,
                )
                return Response(
                    message.target_system,
                    message.target_component,
                    message.message_type,
                    False,
                    message_results.EXCEPTION,
                )

        return Response(
            message.target_system,
            message.target_component,
            message.message_type,
            result,
            code,
        )

    def __get_message_result(
        self,
        message: MessageWrapper,
        ack_packet_type: str = "COMMAND_ACK",
        state_verification_function: Optional[Callable] = None,
    ) -> Tuple[bool, Tuple[int, str], Optional[dict]]:
        """
        Verify the result of a message and retry sending the message, if desired.

        :param message: message whose response should be captured
        :type message: Any
        :param state_verification_function: function used to verify that the target
            state was properly modified, defaults to None
        :type state_verification_function: Optional[Callable], optional
        :return: message acknowledgement, message code, acknowledgement message
        :rtype: Tuple[bool, Tuple[int, str], Optional[dict]]
        """
        ack = False
        code = message_results.ACK_FAILURE

        ack, ack_msg = self.__ack_message(ack_packet_type, timeout=message.ack_timeout)

        if ack:
            code = message_results.SUCCESS

            if (
                message.target_system,
                message.target_component,
            ) in self.__agents and state_verification_function is not None:
                ack = state_verification_function(message)

                if not ack:
                    code = message_results.STATE_VALIDATION_FAILURE
        else:
            code = message_results.ACK_FAILURE

        if message.retry and not ack:
            # Disable retry to prevent creating an infinite loop
            message.retry = False

            start_time = time.time()

            while time.time() - start_time <= message.message_timeout:
                self.__connection.mavlink_connection.mav.send(message.mavlink_message)

                ack, code, ack_msg = self.__get_message_result(
                    message, ack_packet_type, state_verification_function
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
