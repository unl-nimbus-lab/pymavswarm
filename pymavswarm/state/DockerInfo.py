import datetime
from pymavswarm.state.State import State


class DockerInfo(State):
    """
    State of the Docker image deployed on an agent
    """

    def __init__(
        self,
        version: str = "0.0.0",
        last_update: datetime.datetime = datetime.datetime(datetime.MINYEAR, 1, 1),
        optional_context_props: dict = {},
    ) -> None:
        """
        :param version: Current version of the Docker image deployed on an agent,
            defaults to "0.0.0"
        :type version: str, optional

        :param last_update: The date on which the Docker image deployed on the agent
            was last updated, defaults to 1/1/1
        :type last_update: datetime.datetime, optional

        :param optional_context_props: Optional properties to add to the context
        :type optional_context_props: dict, optional
        """
        super().__init__()

        self.__version = version
        self.__last_update = last_update
        self.__optional_context_props = optional_context_props

        return

    @property
    def context(self) -> dict:
        """
        Get the current state as a dictionary for callbacks

        :return: Properties of interested associated with the docker information
        :rtype: dict
        """
        context = {"version": self.__version, "last_update": self.__last_update}
        context.update(self.__optional_context_props)

        return context

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
        prev_version = self.__version
        self.__version = version

        # Signal state change event
        if self.__version != prev_version:
            self.state_changed_event.notify(context=self.context)

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
        prev_last_update = self.__last_update
        self.__last_update = date

        # Signal state change event
        if self.__last_update != prev_last_update:
            self.state_changed_event.notify(context=self.context)

        return
