class AgentMsg:
    """
    Parent class used to construct MAVLink commands
    Params:
        - msg_type       [str]   : The sub-message type for a message
        - target_system  [int]   : The target system ID
        - target_comp    [int]   : The target component ID
        - retry          [bool]  : Indicate whether pymavswarm should retry sending the message until acknowledgement
        - msg_timeout    [float] : The amount of time that pymavswarm should attempt to resend a message if
                                   acknowledgement is not received. This is only used when check_ack is set
                                   to true
        - ack_timeout    [float] : The amount of time that pymavswarm should wait to check for an acknowledgement
                                   from an agent. This is only used when check_ack is set to true. This should be kept
                                   as short as possible to keep agent state information up-to-date
        - state_timeout  [float] : The amount of time that pymavswarm should wait for a given agent's state to change 
                                   after receiving a mavlink message
        - state_delay    [float] : The amount of time that pymavswarm should wait after sending a command prior to sending
                                   another command. This parameter is used for sequence-driven commands such as the full
                                   takeoff command sequence.
        - validate_state [bool]  : Flag indicating that pymavswarm should check to ensure that the message caused the desired
                                   state change in the system
                                   
    """
    def __init__(self, msg_type: str, 
                 target_system: int, 
                 target_comp: int, 
                 retry: bool,
                 msg_timeout: float=5.0,
                 ack_timeout: float=1.0,
                 state_timeout: float=5.0,
                 state_delay: float=3.0,
                 validate_state: bool=False,
                 callbacks: list=[]) -> None:
        self.msg_type = msg_type
        self.target_system = target_system
        self.target_comp = target_comp
        self.retry = retry
        self.msg_timeout = msg_timeout
        self.ack_timeout = ack_timeout
        self.state_timeout = state_timeout
        self.state_delay = state_delay
        self.validate_state = validate_state
        self.__callbacks = callbacks

        return
        

    def get_type(self) -> str:
        return self.msg_type

    
    def add_message_callback(self, fn) -> None:
        self.__callbacks.append(fn)
        return

    
    def remove_message_callback(self, fn) -> None:
        if fn in self.__callbacks:
            self.__callbacks.remove(fn)
        return