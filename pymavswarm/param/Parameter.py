from typing import Union


class Parameter:
    """
    Parameter represents a key/value pair that may be sent to an agent to 
    set the respective parameter value
    Params:
        - target_system : int          : The system ID of the agent whose param value should be set
        - target_comp   : int          : The component ID of the agent whose param value should be set
        - param_name    : str          : The name of the parameter that should be set on an agent
        - param_value   : float or int : The value that the parameter should be set to  
        - retry         : bool         : Flag indicating whether the system should re-attempt parameter setting
                                         if the system fails to acknowledge it
        - msg_timeout   : float        : The period of time that pymavswarm should attempt to re-set a parameter
                                         if acknowledgement fails
        - ack_timeout   : float        : The amount of time that pymavswarm should wait for acknowledgement before
                                         considering that the system failed to acknowledge the parameter setting
    """
    def __init__(self, target_system: int,
                 target_comp: int,
                 param_name: str, 
                 param_value: Union[float, int],
                 retry: bool,
                 msg_timeout: float,
                 ack_timeout: float=1.0) -> None:
        self.target_system = target_system
        self.target_comp = target_comp
        self.param_name = param_name
        self.param_value = param_value
        self.retry = retry
        self.param_timeout = msg_timeout
        self.ack_timeout = ack_timeout

        return