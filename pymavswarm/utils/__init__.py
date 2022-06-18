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

import logging


def init_logger(name: str, log_level: int = logging.INFO) -> logging.Logger:
    """
    Initialize the logger with the desired debug levels.

    :param name: The name of the logger
    :type name: str

    :param log_level: The log level to display, defaults to logging.INFO
    :type log_level: int, optional

    :return: A newly configured logger
    :rtype: logging.Logger
    """
    logging.basicConfig()
    logger = logging.getLogger(name)
    logger.setLevel(log_level)

    return logger
