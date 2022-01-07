import logging
from .Connection import Connection
from .msg import AgentMsg


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
            except Exception as e:
                # Handle the error message
                self.logger.debug('MavSwarm was unable to establish a connection with the specified device', e)

                return False

        return True

    
    def disconnect(self) -> bool:
        """
        Disconnect the connection
        """
        if self.connection is not None:
            self.connection.disconnect()

        return True


    def send_msg(self, msgs: list[AgentMsg]) -> None:
        """
        Add the message to the connection's outgoing messages queue
        """
        for msg in msgs:
            # Ensure that the intended agent is in the network
            if (msg.sys_id, msg.comp_id) in self.connection.devices:
                self.connection.outgoing_msgs.put(msg)
        
        return

    
    def get_agents(self) -> list:
        """
        Get the list of agents in the network
        """
        if self.connection is not None:
            return [*self.connection.devices.values()]
        else:
            return []