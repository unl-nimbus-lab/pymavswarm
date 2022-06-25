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

import logging
import time
from copy import deepcopy
from typing import Any, Callable, Dict, List, Optional, Tuple

from pymavswarm.messages.response import message_results
from pymavswarm.utils import init_logger


class Senders:
    """Base class used to implement message senders."""

    def __init__(
        self, logger_name: str = __name__, log_level: int = logging.INFO
    ) -> None:
        """
        Create a new senders object.

        :param logger_name: logger name, defaults to __name__
        :type logger_name: str, optional
        :param log_level: logging level, defaults to logging.INFO
        :type log_level: int, optional
        """
        self.__logger = init_logger(logger_name, log_level=log_level)
        self.__senders: Dict[str, List[Callable]] = {}

        return

    @property
    def senders(self) -> Dict[str, List[Callable]]:
        """
        Get the methods responsible for sending messages.

        :return: list of senders
        :rtype: dict
        """
        return self.__senders

    def _ack_message(
        self, packet_type: str, connection, timeout: float = 1.0
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
        if not connection.read_message_mutex.acquire(timeout=1.0):
            return False, None

        # Flag indicating whether the message was acknowledged
        ack_success = False

        # Message received converted to a dictionary
        message = None

        # Start acknowledgement timer
        start_t = time.time()

        while time.time() - start_t < timeout:
            try:
                message = connection.mavlink_connection.recv_match(
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
        connection.read_message_mutex.release()

        return ack_success, message

    def _get_message_response(
        self,
        message: Any,
        connection,
        function_idx: int,
        ack_packet_type: str = "COMMAND_ACK",
        state_verification_function: Optional[Callable] = None,
    ) -> Tuple[bool, Tuple[int, str], Optional[dict]]:
        """
        Verify the result of a message and retry sending the message, if desired.

        :param message: message whose response should be captured
        :type message: Any

        :param connection: MAVLink connection
        :type connection: Connection

        :param function_idx: index of the function to use when sending the message
        :type function_idx: int

        :param state_verification_function: function used to verify that the target
            state was properly modified, defaults to None
        :type state_verification_function: Optional[Callable], optional

        :return: message acknowledgement, message code, acknowledgement message
        :rtype: Tuple[bool, Tuple[int, str], Optional[dict]]
        """
        ack = False
        code = message_results.ACK_FAILURE

        ack, ack_msg = self._ack_message(
            ack_packet_type, connection, timeout=message.ack_timeout
        )

        if ack:
            code = message_results.SUCCESS

            if (
                message.target_system,
                message.target_comp,
            ) in connection.agents and state_verification_function is not None:
                ack = state_verification_function(message, connection)

                if not ack:
                    code = message_results.STATE_VALIDATION_FAILURE
        else:
            code = message_results.ACK_FAILURE

        if message.retry and not ack:
            ack, code, ack_msg = self._retry_message_send(
                deepcopy(message),
                connection,
                self.__senders[message.message_type][function_idx],
            )

        return ack, code, ack_msg

    def _retry_message_send(
        self,
        message: Any,
        connection,
        function: Callable,
    ) -> Tuple[bool, Tuple[int, str], Optional[dict]]:
        """
        Retry a message send until success/timeout.

        :param message: message to retry sending
        :type message: pymavswarm message

        :param function: function to call using the message
        :type function: Callable

        :return: message acknowledged, message response, acknowledgement message
        :rtype: Tuple[bool, Tuple[int, str], Optional[dict]]
        """
        ack = False
        start_time = time.time()

        # Don't let the message come back here and create an infinite loop
        message.retry = False

        while time.time() - start_time <= message.message_timeout:
            ack, response, ack_msg = function(self, message, connection)

            if ack:
                break

        return ack, response, ack_msg

    def _send_sequence_message(self, message: Any, connection) -> bool:
        """
        Send a sequence message.

        Helper function used to handle calling all of the handlers for a message.
        This method is used by sequence commands (such as the full takeoff
        command) to provide indication of a function execution result.

        :param message: The message to send
        :type message: Any

        :return: Indicates whether all of the message senders for a given message
            successfully sent their respective message
        :rtype: bool
        """
        if message.message_type not in self.__senders:
            return False

        for function_idx, function in enumerate(self.__senders[message.message_type]):
            try:
                success, _, _ = function(
                    self,
                    message,
                    connection,
                    function_idx=function_idx,
                )

                if not success:
                    return False

            except Exception:
                return False

        return True

    def _send_message(self, message: str) -> Callable:
        """
        Create a sender for a mavlink message.

        :param message: The message type to connect to the sender
        :type message: Union[list, str]

        :return: decorator
        :rtype: Callable
        """

        def decorator(function: Callable):
            if message not in self.__senders:
                self.__senders[message] = []

            if function not in self.__senders[message]:
                self.__senders[message].append(function)

        return decorator

    def _timer(self) -> Callable:
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
