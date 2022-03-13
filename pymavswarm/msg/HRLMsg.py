from .AgentMsg import AgentMsg
from utils import SupportedCommands



class HRLMsg(AgentMsg):
    """
    HRLMsg class represents messages that indicate that an HRL command should be executed.
    The message types that should be used with this type include:
        - start_path_execution
        - stop_path_execution
        - reset_path_execution
        - start_live_execution
    """
    def __init__(self, hrl_command: int, target_system: int, target_comp: int, retry: bool, msg_timeout: float=5.0) -> None:
        supported_hrl_commands = SupportedCommands().hrl_commands.get_supported_hrl_commands()

        if hrl_command not in supported_hrl_commands:
            raise TypeError(f'{hrl_command} is not a supported HRL command. Supported HRL commands include: {supported_hrl_commands}')

        super().__init__('HRL_COMMAND', target_system, target_comp, retry, msg_timeout=msg_timeout)
        self.hrl_command = hrl_command

        return