class AgentMsg:
    """
    Helper class provided by mavswarm to enable sending messages easily
    Params:
        - msg_type    [Enum]  : The sub-message type for a message
        - sys_id      [int]   : The target system ID
        - comp_id     [int]   : The target component ID
        - check_ack   [bool]  : Indicate whether or not pymavswarm should check for message acknowledgement
        - msg_timeout [float] : The amount of time that pymavswarm should attempt to resend a message if
                                acknowledgement is not received. This is only used when check_ack is set
                                to true
        - ack_timeout [float] : The amount of time that pymavswarm should way to check for an acknowledgement
                                from an agent. This is only used when check_ack is set to true
    """
    def __init__(self, msg_type: str, 
                 sys_id: int, 
                 comp_id: int, 
                 check_ack: bool,
                 msg_timeout: float=5.0,
                 ack_timeout: float=1.0) -> None:
        self.msg_type = msg_type
        self.sys_id = sys_id
        self.comp_id = comp_id
        self.check_ack = check_ack
        self.msg_timeout = msg_timeout
        self.ack_timeout = ack_timeout

    def get_type(self) -> str:
        return self.msg_type