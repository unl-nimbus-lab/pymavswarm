from .State import State



class DockerInfo(State):
    """
    version     : The current version of the Docker image deployed by the agent
    last_update : The last date that the agent was updated
    """
    def __init__(self, version: str='None', 
                 last_update: str='None',
                 callbacks: list=[]) -> None:
        super().__init__(callbacks)

        self.__version = version
        self.__last_update = last_update

        return


    def get_current_state(self) -> dict:
        return {
            'version': self.version,
            'last_update': self.last_update
        }


    @property
    def version(self) -> str:
        return self.__version


    @version.setter
    def version(self, version) -> None:
        self.__version = version

        for cb in self.callbacks:
            cb(self.get_current_state())

        return


    @property
    def last_update(self) -> str:
        return self.__last_update

    
    @last_update.setter
    def last_update(self, date: str) -> None:
        self.__last_update = date

        for cb in self.callbacks:
            cb(self.get_current_state())

        return