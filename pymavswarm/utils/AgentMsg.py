class AgentMsg:
    """
    Helper class provided by mavswarm to enable sending messages easily
    """
    def __init__(self, msg_type: str, sys_id: int, comp_id: int, require_ack: bool) -> None:
        self.msg_type = msg_type
        self.sys_id = sys_id
        self.comp_id = comp_id
        self.require_ack = require_ack

    def get_type(self) -> str:
        return self.msg_type