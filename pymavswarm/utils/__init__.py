# pymavswarm is an interface for swarm control and interaction
# Copyright (C) 2022  Evan Palmer

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import math

from .event import Event
from .logging import FileLogger, init_logger, parse_log_file
from .notifier_dict import NotifierDict

# Specify Earth's radius to use for calculating positions
RADIUS_EARTH = 6378137


def latitude_conversion(latitude: float, offset: float) -> float:
    """
    Compute a new latitude given a reference position and a desired offset.

    :param latitude: latitude to update
    :type latitude: float
    :param offset: distance from the previous latitude and the new latitude [m]
    :type offset: float
    :return: updated latitude
    :rtype: float
    """
    return round(latitude + (offset / RADIUS_EARTH) * (180 / math.pi), 7)


def longitude_conversion(latitude: float, longitude: float, offset: float) -> float:
    """
    Compute a new longitude given a reference position and a desired offset.

    :param latitude: latitude used to compute the updated longitude
    :type latitude: float
    :param longitude: longitude to reference for the updated longitude
    :param offset: distance from the previous longitude and the new longitude [m]
    :type offset: float
    :return: updated longitude
    :rtype: float
    """
    return round(
        longitude
        + (offset / RADIUS_EARTH)
        * (180 / math.pi)
        / math.cos(latitude * math.pi / 180),
        7,
    )
