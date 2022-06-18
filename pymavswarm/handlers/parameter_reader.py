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

import pymavswarm.utils as swarm_utils
from pymavswarm import Connection
from pymavswarm.messages.param import Parameter


class ParameterReader:
    """Handlers for reading parameters."""

    def __init__(
        self, logger_name: str = "parameter-reader", log_level: int = logging.INFO
    ) -> None:
        """
        Construct a ParameterReaders interface.

        :param logger_name: name of the class logger, defaults to "parameter-reader"
        :type logger_name: str, optional

        :param log_level: log level, defaults to logging.INFO
        :type log_level: int, optional
        """
        self.__logger = swarm_utils.init_logger(logger_name, log_level=log_level)

        return

    def read_parameter(self, param: Parameter, connection: Connection) -> bool:
        """
        Read a parameter value from the target agent.

        :param param: The parameter to read
        :type param: Parameter

        :param connection: MAVLink connection
        :type connection: Connection

        :return: indicate whether or not the parameter was successfully read
        :rtype: bool
        """
        try:
            connection.mavlink_connection.mav.param_request_read_send(
                param.system_id, param.component_id, str.encode(param.parameter_id), -1
            )
        except Exception:
            self.__logger.exception(
                "An exception occurred while attempting to read %d "
                "from Agent (%d, %d)",
                param.parameter_id,
                param.system_id,
                param.component_id,
                exc_info=True,
            )
            return False

        ack = False

        ack, msg = swarm_utils.ack_message(
            "PARAM_VALUE", connection, timeout=param.ack_timeout
        )

        if ack:
            read_param = swarm_state.ReadParameter(
                msg["param_id"],
                msg["param_value"],
                msg["param_type"],
                msg["param_index"],
                msg["param_count"],
            )

            self.__agents[
                (param.system_id, param.component_id)
            ].last_params_read.append(read_param)
        else:
            if param.retry:
                if self.__retry_param_send(param, self.__read_param):
                    ack = True

        if ack:
            self.__logger.info(
                f"Successfully read {param.parameter_id} from Agent ({param.system_id}, "
                f"{param.component_id}). Value: {msg}"
            )
        else:
            self.__logger.error(
                f"Failed to read {param.parameter_id} from Agent ({param.system_id}, "
                f"{param.component_id})"
            )

        return ack

    def read_param_handler(self, param: Parameter) -> None:
        """
        Read the value of a parameter

        :param param: The parameter to read
        :type param: Parameter
        """
        # Make sure that a connection is established before attempting to set a param
        if self.__connected:
            handler_t = threading.Thread(
                target=self.__read_param_handler, args=(param,)
            )

            # Send the message
            handler_t.start()

        return

    def __read_param_handler(self, param: Parameter) -> None:
        """
        Handler responsible for reading requested parameters.

        NOTE: This thread is primarily responsible for handling read requests and
        verifying that a read was accomplished on the message listener thread. The
        agent state itself is updated on the message listener thread

        :param param: The parameter to read
        :type param: Parameter
        """
        # Prevent multiple reads from occurring at once
        self.__send_message_mutex.acquire()

        try:
            self.__read_param(param)
        except Exception:
            self.__logger.exception(
                "An error occurred while attempting to send the provided message"
            )
        finally:
            self.__send_message_mutex.release()

        return
