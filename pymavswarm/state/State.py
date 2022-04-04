class State:
    """
    Parent class used to provide relationship between state objects
    """
    def __init__(self, callbacks: list=[]) -> None:
        self.callbacks = callbacks


    def add_callback(self, fn) -> None:
        self.callbacks.append(fn)
        return

    
    def remove_callback(self, fn) -> None:
        if fn in self.callbacks:
            self.callbacks.remove(fn)
        return

    
    def get_current_state(self) -> dict:
        raise NotImplementedError