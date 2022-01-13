import logging
from typing import Optional
from .Waypoint import Waypoint



class Mission:
    """
    Mission defines a set of waypoints that should be executed sequentially by an agent
    """
    def __init__(self, waypoints: list=[], log: bool=False, debug: bool=False) -> None:
        self.logger = self.__init_logger('mission', log=log, debug=debug)
        self.waypoints = waypoints


    def __init_logger(self, name, debug: bool=False, log: bool=False) -> logging.Logger:
        """
        Initialize the logger with the desired debug levels
        """
        logging.basicConfig()

        # Set the desired debug level
        if debug or (debug and log):
            logger = logging.getLogger(name)
            logger.setLevel(logging.DEBUG)
            return logger
        elif log:
            logger = logging.getLogger(name)
            logger.setLevel(logging.INFO)
            return logger
        else:
            return logging.getLogger(name)


    def add_waypoint(self, waypoint: Waypoint, index: Optional[int]=None) -> bool:
        """
        Add a waypoint to the mission
        The index that the waypoint should be set to in the mission may optionally be set
        """
        if index is not None:
            try:
                self.waypoints.insert(index)
            except IndexError as e:
                self.logger.exception(f'An attempt was made to insert a waypoint at an invalid index. Mission size: {len(self.waypoints)}. Index: {index}')
                return False
        else:
            self.waypoints.append(waypoint)

        return True


    def remove_waypoint_by_index(self, index: int) -> bool:
        """
        Remove a waypoint by index
        """
        try:
            self.waypoints.pop(index)
        except IndexError as e:
            self.logger.exception(f'An attempt was made to remove a waypoint at an invalid index. Mission size: {len(self.waypoints)}. Index: {index}')
            return False

        return True

    
    def remove_waypoint_by_value(self, latitude: float, longitude: float, altitude: float):
        """
        Remove all waypoints that have the specified latitude, longitude, altitude values
        """
        for waypoint in self.waypoints:
            if waypoint.latitude == latitude and waypoint.longitude == longitude and waypoint.altitude == altitude:
                self.waypoints.remove(waypoint)

        return