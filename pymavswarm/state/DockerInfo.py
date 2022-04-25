from .State import State
import datetime


class DockerInfo(State):
    """
    State of the Docker image deployed on an agent
    """

    def __init__(self, version: str = "0.0.0", last_update: datetime.datetime = datetime.datetime(datetime.MINYEAR, 1, 1)) -> None:
        """
        :param version: Current version of the Docker image deployed on an agent,
            defaults to "0.0.0"
        :type version: str, optional

        :param last_update: The date on which the Docker image deployed on the agent
            was last updated, defaults to 1/1/1
        :type last_update: datetime.datetime, optional
        """
        super().__init__()

        self.__version = version
        self.__last_update = last_update

        return

    @property
    def context(self) -> dict:
        """
        Get the current state as a dictionary for callbacks

        :return: Properties of interested associated with the docker information
        :rtype: dict
        """
        return {"version": self.__version, "last_update": self.__last_update}

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

        # Signal state change event
        self.__state_changed_event.notify(context=self.context)

        return
