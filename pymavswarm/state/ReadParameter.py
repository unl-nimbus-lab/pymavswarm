from ctypes import Union


from typing import Union



class ReadParameter:
    """
    ReadParameter represents a parameter that has been read from the respective agent
    Params:
        - param_id     : str        : The parameter ID
        - param_value  : float, int : The parameter value
        - param_type   : int        : The type of value that the parameter may contain (Should be 6: float or int)
        - param_index  : int        : The index of the parameter
        - param_count  : int        : The parameter count
    """
    def __init__(self, param_id: str,
                 param_value: Union[float, int],
                 param_type: int,
                 param_index: int,
                 param_count: int) -> None:
        self.param_id = param_id
        self.param_value = param_value
        self.param_type = param_type
        self.param_index = param_index
        self.param_count = param_count

        return