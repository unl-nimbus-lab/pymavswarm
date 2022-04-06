import logging
from typing import Optional
from .Waypoint import Waypoint


class Mission:
    """
    A set of waypoints that should be executed sequentially by an agent
    """

    def __init__(self, waypoints: list = [], log_level: int = logging.INFO) -> None:
        """
        :param waypoints: The set of waypoints to be executed, defaults to []
        :type waypoints: list, optional

        :param log_level: The desired debugging level, defaults to logging.INFO
        :type debug: int, optional
        """
        self.__logger = self.__init_logger("mission", log_level=log_level)
        self.__waypoints = waypoints

        return

    @property
    def waypoints(self) -> list:
        """
        The set of waypoints included in the mission

        :rtype: list
        """
        return self.__waypoints

    def __init_logger(self, name: str, log_level: int = logging.INFO) -> logging.Logger:
        """
        Initialize the system logger

        :param name: The name of the logger
        :type name: str

        :param log_level: The log level to use, defaults to logging.INFO
        :type log_level: int, optional

        :return: A new logger
        :rtype: logging.Logger
        """
        logging.basicConfig()
        logger = logging.getLogger(name)
        logger.setLevel(log_level)

        return logger

    def add_waypoint(self, waypoint: Waypoint, index: Optional[int] = None) -> bool:
        """
        Add a waypoint to the mission

        :param waypoint: The new waypoint to add
        :type waypoint: Waypoint

        :param index: The index that the waypoint should be placed at in the mission
            sequence, defaults to None
        :type index: Optional[int], optional

        :return: Indication of whether the waypoint was inserted successfully
        :rtype: bool
        """
        if index is not None:
            try:
                self.__waypoints.insert(index)
            except IndexError:
                self.__logger.exception(
                    "An attempt was made to insert a waypoint at "
                    f"an invalid index. Mission size: {len(self.__waypoints)}. "
                    f"Index: {index}"
                )
                return False
        else:
            self.__waypoints.append(waypoint)

        return True

    def remove_waypoint_by_index(self, index: int) -> bool:
        """
        Remove the waypoint at the given index

        :param index: The index of the waypoint in the mission that should be removed
        :type index: int

        :return: Indication of whether the waypoint was removed successfully
        :rtype: bool
        """
        try:
            self.__waypoints.pop(index)
        except IndexError as e:
            self.__logger.exception(
                f"An attempt was made to remove a waypoint at an invalid index. "
                f"Mission size: {len(self.__waypoints)}. Index: {index}"
            )
            return False

        return True

    def remove_waypoint_by_value(
        self, latitude: float, longitude: float, altitude: float
    ) -> bool:
        """
        Remove all waypoints that have the specified latitude, longitude, and altitude
        values

        :param latitude: Waypoint's latitude
        :type latitude: float

        :param longitude: Waypoint's longitude
        :type longitude: float

        :param altitude: Waypoint's altitude
        :type altitude: float

        :return: Indication of whether the waypoint was removed properly
        :rtype: bool
        """
        for waypoint in self.__waypoints:
            if (
                waypoint.latitude == latitude
                and waypoint.longitude == longitude
                and waypoint.altitude == altitude
            ):
                self.__waypoints.remove(waypoint)

        return True
