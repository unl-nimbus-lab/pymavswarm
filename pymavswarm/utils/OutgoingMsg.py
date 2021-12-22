class OutgoingMsg:
    """
    Helper class provided by mavswarm to enable sending messages easily
    """
    def __init__(self, msg_type: str, sys_id: int, comp_id: int) -> None:
        self.msg_type = msg_type
        self.sys_id = sys_id
        self.comp_id = comp_id

    def get_type(self) -> str:
        return self.msg_type