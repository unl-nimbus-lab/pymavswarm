class AgentMsg:
    """
    Parent class used to construct MAVLink commands
    Params:
        - msg_type    [Enum]  : The sub-message type for a message
        - target_id   [int]   : The target system ID
        - target_comp [int]   : The target component ID
        - retry       [bool]  : Indicate whether pymavswarm should retry sending the message until acknowledgement
        - msg_timeout [float] : The amount of time that pymavswarm should attempt to resend a message if
                                acknowledgement is not received. This is only used when check_ack is set
                                to true
        - ack_timeout [float] : The amount of time that pymavswarm should way to check for an acknowledgement
                                from an agent. This is only used when check_ack is set to true. This should be kept
                                as short as possible to keep agent state information up-to-date
    """
    def __init__(self, msg_type: str, 
                 target_id: int, 
                 target_comp: int, 
                 retry: bool,
                 msg_timeout: float=5.0,
                 ack_timeout: float=1.0) -> None:
        self.msg_type = msg_type
        self.target_id = target_id
        self.target_comp = target_comp
        self.retry = retry
        self.msg_timeout = msg_timeout
        self.ack_timeout = ack_timeout


    def get_type(self) -> str:
        return self.msg_type