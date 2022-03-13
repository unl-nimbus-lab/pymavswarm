from .AgentMsg import AgentMsg
from utils import SupportedCommands



class SystemCommandMsg(AgentMsg):
    """
    SystemCommandMsg class represents messages that perform some system-level operation on
    an agent such as arming or reboot.
    """
    def __init__(self, command: str, target_system: int, target_comp: int, retry: bool, msg_timeout: float=5.0) -> None:
        supported_system_commands = SupportedCommands().system_commands.get_supported_system_commands()

        if command not in supported_system_commands:
            raise TypeError(f'{command} is not a supported system command. Supported system commands include: {supported_system_commands}')

        super().__init__(command, target_system, target_comp, retry, msg_timeout=msg_timeout)

        return