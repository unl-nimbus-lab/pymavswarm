import logging
from .Agent import Agent
from typing import Optional
from .Connection import Connection


class MavSwarm:
    """
    This object is the primary interface for mavswarm and enables users to
    send commands to the swarm and read the swarm state
    """
    def __init__(self, log: bool=False, debug: bool=False) -> None:
        super().__init__()

        # Initialize loggers
        self.__log = log
        self.__debug = debug
        self.logger = self.__init_logger('mavswarm', debug=debug, log=log)

        # Class variables
        self.connection = None

    
    def __init_logger(self, name, debug: bool=False, log: bool=False) -> logging.Logger:
        """
        Initialize the logger with the desired debug levels
        """
        logging.basicConfig()

        # Set the desired debug level
        if debug or (debug and log):
            logger = logging.getLogger(name)
            logger.setLevel(logging.DEBUG)
            return logger
        elif log:
            logger = logging.getLogger(name)
            logger.setLevel(logging.INFO)
            return logger
        else:
            return logging.getLogger(name)


    def connect(self, port: str, baudrate: int, source_system: int=255, source_component: int=0) -> bool:
        """
        Create a new connection using the specified serial port
        """
        if self.connection is None:            
            try:
                self.connection = Connection(port, baudrate, source_system, source_component, log=self.__log, debug=self.__debug)
                self.connection.start_connection()
            except Exception:
                # Handle the error message
                self.logger.debug('MavSwarm was unable to establish a connection with the specified device')

                return False

        return True

    
    def disconnect(self) -> bool:
        """
        Disconnect the connection
        """
        if self.connection is not None:
            self.connection.disconnect()

        return True


    def send_msg(self, msgs: list) -> None:
        """
        Add the message to the connection's outgoing messages queue
        """
        for msg in msgs:
            # Ensure that the intended agent is in the network
            if (msg.target_system, msg.target_comp) in self.connection.devices:
                self.connection.send_msg_handler(msg)
        
        return


    def set_param(self, params: list) -> None:
        """
        Add the params to the connection's outgoing parameter queue
        """
        for param in params:
            # Ensure that the intended agent is in the network
            if (param.sys_id, param.comp_id) in self.connection.devices:
                self.connection.outgoing_params.put(param)

        return


    def read_param(self, params: list) -> None:
        """
        Read a desired parameter value. Note that, in the current configuration
        each agent stores the most recent 5 parameter values read in a circular
        buffer. Therefore, when performing a bulk parameter read, ensure that
        a maximum of five parameters are read at once on the specific agent
        """
        for param in params:
            # Ensure that the intended agent is in the network
            if (param.sys_id, param.comp_id) in self.connection.devices:
                self.connection.read_params.put(param)

        return

    
    def get_agents(self) -> list:
        """
        Get the list of agents in the network
        """
        if self.connection is not None:
            return [*self.connection.devices.values()]
        else:
            return []


    def get_agent_by_id(self, sys_id: int, comp_id: int) -> Optional[Agent]:
        """
        Get a specific agent by its system ID and component ID
        """
        for agent in self.connection.devices.values():
            if agent.sys_id == sys_id and agent.comp_id == comp_id:
                return agent

        return None
        

    def get_agent_by_name(self, name: str) -> Optional[Agent]:
        """
        Get the first agent in the swarm with the specified name
        """
        for agent in self.connection.devices.values():
            if agent.name == name:
                return agent

        return None
