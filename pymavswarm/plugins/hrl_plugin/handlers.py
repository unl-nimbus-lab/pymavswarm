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

import pymavswarm.messages as swarm_msgs
from pymavswarm import Connection
from pymavswarm.messages import responses
from pymavswarm.plugins import PluginHandlers


class HrlHandlers(PluginHandlers):
    """Handlers for HRL messages."""

    def __init__(
        self, logger_name: str = "HRL Plugin", log_level: int = logging.INFO
    ) -> None:
        """
        Create a new HRL handler.

        :param logger_name: name of logger, defaults to "HRL Plugin"
        :type logger_name: str, optional

        :param log_level: logging level, defaults to logging.INFO
        :type log_level: int, optional
        """
        super().__init__(logger_name, log_level)

        @self._send_message("HRL_COMMAND")
        def sender(
            self,
            msg: swarm_msgs.HRLMessage,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Send an HRL command to the swarm.

            :param msg: HRL message
            :type msg: HRLMsg

            :param function_id: The index of the method in the message type function
                handler list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            # Reset target
            connection.mavlink_connection.target_system = msg.target_system
            self.master.target_component = msg.target_comp

            # Send flight mode
            connection.mavlink_connection.mav.named_value_int_send(
                int(time.time()), str.encode("hrl-state-arg"), msg.hrl_command
            )

            ack = False
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the HRL command "
                    f"{msg.hrl_command} sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    start_time = time.time()

                    while (
                        not self.__devices[
                            (msg.target_system, msg.target_comp)
                        ].hrl_state.value
                        != msg.hrl_command
                    ):
                        if time.time() - start_time >= msg.state_timeout:
                            ack = False
                            break
                    if ack:
                        self.logger.info(
                            f"Successfully verified that Agent ({msg.target_system}, "
                            f"{msg.target_comp}) executed HRL command {msg.hrl_command}"
                        )
                    else:
                        self.logger.error(
                            f"Failed to verify that Agent ({msg.target_system}, "
                            f"{msg.target_comp}) executed HRL command {msg.hrl_command}"
                        )
                        msg_code = responses.STATE_VALIDATION_FAILURE
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the HRL command "
                    f"{msg.hrl_command} sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        return
