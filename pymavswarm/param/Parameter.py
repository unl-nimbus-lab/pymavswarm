from typing import Union, Optional


class Parameter:
    """
    Parameter represents a key/value pair that may be sent to an agent to 
    set the respective parameter value
    Params:
        - sys_id      : int          : The system ID of the agent whose param value should be set/read
        - comp_id     : int          : The component ID of the agent whose param value should be set/read
        - param_id    : str          : The ID of the parameter that should be set/read on an agent
        - param_value : float or int : The value that the parameter should be set to  
        - retry       : bool         : Flag indicating whether the system should re-attempt parameter setting/reading
                                         if the system fails to acknowledge it
        - msg_timeout : float        : The period of time that pymavswarm should attempt to re-set a parameter
                                         if acknowledgement fails (parameter setting only)
        - ack_timeout : float        : The amount of time that pymavswarm should wait for acknowledgement before
                                         considering that the system failed to acknowledge the parameter setting/reading
    """
    def __init__(self, sys_id: int,
                 comp_id: int,
                 param_id: str,
                 retry: bool,
                 msg_timeout: float=3.0,
                 param_value: Optional[Union[float, int]]=None,
                 ack_timeout: float=1.0) -> None:
        self.sys_id = sys_id
        self.comp_id = comp_id
        self.param_id = param_id
        self.param_value = param_value
        self.retry = retry
        self.msg_timeout = msg_timeout
        self.ack_timeout = ack_timeout

        return