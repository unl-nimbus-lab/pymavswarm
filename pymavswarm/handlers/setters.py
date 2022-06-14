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

    def __retry_param_send(self, param: Any, function: Callable) -> bool:
        """
        Retry a parameter send until the an acknowledgement is received or a timeout
        occurs

        :param param: The parameter to retry sending
        :type msg: Any

        :param function: The function to call using the message
        :type function: function

        :return: Indicate whether the retry was successful
        :rtype: bool
        """
        ack = False
        start_time = time.time()

        # Don't let the message come back here and create an infinite loop
        param.retry = False

        while time.time() - start_time <= param.msg_timeout:
            # Reattempt the message send
            if function(param):
                ack = True
                break

        return ack


    def set_param_handler(self, param: Parameter) -> None:
        """
        Set the value of a parameter on a given agent

        :param param: The parameter to set
        :type param: Parameter
        """
        # Make sure that a connection is established before attempting to set a param
        if self.__connected:
            handler_t = threading.Thread(target=self.__set_param_handler, args=(param,))

            # Set the parameter
            handler_t.start()

        return

    def __set_param_handler(self, param: Parameter) -> None:
        """
        Handle setting parameters on an agent in the network

        :param param: The parameter to set
        :type param: Parameter
        """
        # Prevent multiple sends from occurring at once
        self.__send_message_mutex.acquire()

        try:
            self.__set_param(param)
        except Exception:
            self.__logger.exception(
                "An error occurred while attempting to send the provided message",
                exc_info=True,
            )
        finally:
            self.__send_message_mutex.release()

        return

    def __set_param(self, param: Parameter) -> bool:
        """
        Set the value of a parameter.

        NOTE: This sets the parameter value in RAM and not to EEPROM. Therefore, on
        reboot, the parameters will be reset to their default values

        :param param: The parameter to set
        :type param: Parameter

        :return: Indicates whether the parameter was successfully set
        :rtype: bool
        """
        try:
            # NOTE: In the current state, we only support float parameter value types
            #       Additional types may be added in the future
            self.__mavlink_connection.mav.param_set_send(
                param.sys_id,
                param.comp_id,
                str.encode(param.param_id),
                param.param_value,
                9,
            )
        except Exception:
            self.__logger.error(
                f"An error occurred while attempting to set {param.param_id} to "
                f"{param.param_value}",
                exc_info=True,
            )
            return False

        ack = False

        if self.__ack_msg("PARAM_VALUE", timeout=param.ack_timeout)[0]:
            ack = True
        else:
            if param.retry:
                if self.__retry_param_send(param, self.__set_param):
                    ack = True

        if ack:
            self.__logger.info(
                f"Successfully set {param.param_id} to {param.param_value} on "
                f"Agent ({param.sys_id}, {param.comp_id})"
            )
        else:
            self.__logger.error(
                f"Failed to set {param.param_id} to {param.param_value} on Agent "
                f"({param.sys_id}, {param.comp_id})"
            )

        return ack
