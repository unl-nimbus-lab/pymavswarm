from typing import Any
from .State import State


class Common(State):
    def __init__(self, value: Any, name: str, callbacks: list=[]) -> None:
        super().__init__(callbacks)

        self.__value = value
        self.__name = name

        return


    def get_current_state(self) -> dict:
        return {
            self.__name: self.id
        }


    @property
    def value(self) -> int:
        return self.__value


    @value.setter
    def value(self, value: int) -> None:
        self.__value = value

        for cb in self.callbacks:
            cb(self.get_current_state())

        return