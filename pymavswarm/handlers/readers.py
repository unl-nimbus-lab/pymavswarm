
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

    def __read_param(self, param: Parameter) -> bool:
        """
        Read a desired parameter value

        :param param: The parameter to read
        :type param: Parameter

        :return: Indicates whether the parameter was successfully read
        :rtype: bool
        """
        try:
            self.__mavlink_connection.mav.param_request_read_send(
                param.sys_id, param.comp_id, str.encode(param.param_id), -1
            )
        except Exception:
            self.__logger.exception(
                f"An exception occurred while attempting to read {param.param_id} "
                f"from Agent ({param.sys_id}, {param.comp_id})",
                exc_info=True,
            )
            return False

        ack = False

        ack, msg = self.__ack_msg("PARAM_VALUE", timeout=param.ack_timeout)

        if ack:
            read_param = swarm_state.ReadParameter(
                msg["param_id"],
                msg["param_value"],
                msg["param_type"],
                msg["param_index"],
                msg["param_count"],
            )

            self.__agents[(param.sys_id, param.comp_id)].last_params_read.append(
                read_param
            )
        else:
            if param.retry:
                if self.__retry_param_send(param, self.__read_param):
                    ack = True

        if ack:
            self.__logger.info(
                f"Successfully read {param.param_id} from Agent ({param.sys_id}, "
                f"{param.comp_id}). Value: {msg}"
            )
        else:
            self.__logger.error(
                f"Failed to read {param.param_id} from Agent ({param.sys_id}, "
                f"{param.comp_id})"
            )

        return ack
