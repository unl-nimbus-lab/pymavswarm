from .State import State


class DockerInfo(State):
    """
    State of the Docker image deployed on an agent
    """

    def __init__(
        self, version: str = "None", last_update: str = "None", callbacks: list = []
    ) -> None:
        """
        :param version: Current version of the Docker image deployed on an agent,
            defaults to "None"
        :type version: str, optional

        :param last_update: The date on which the Docker image deployed on the agent
            was last updated, defaults to "None"
        :type last_update: str, optional

        :param callbacks: Observers to call on state change, defaults to []
        :type callbacks: list, optional
        """
        super().__init__(callbacks)

        self.__version = version
        self.__last_update = last_update

        return

    def get_current_state(self) -> dict:
        """
        Get the current state as a dictionary for callbacks

        :return: Properties of interested associated with the docker information
        :rtype: dict
        """
        return {"version": self.version, "last_update": self.last_update}

    @property
    def version(self) -> str:
        """
        The current version of the Docker image deployed on the agent

        :rtype: str
        """
        return self.__version

    @version.setter
    def version(self, version: str) -> None:
        """
        version setter

        :param version: Docker image version
        :type version: str
        """
        self.__version = version

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    @property
    def last_update(self) -> str:
        """
        The date that the image was last updated

        :rtype: str
        """
        return self.__last_update

    @last_update.setter
    def last_update(self, date: str) -> None:
        """
        last_update setter

        :param date: Date that the agent's image was last updated
        :type date: str
        """
        self.__last_update = date

        for cb in self.callbacks:
            cb(self.get_current_state())

        return
