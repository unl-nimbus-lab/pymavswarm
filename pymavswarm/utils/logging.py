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

import csv
import logging
import os
from datetime import datetime

import pandas as pd


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


class FileLogger:
    """File logging handler."""

    def __init__(self, filename: str | None = None) -> None:
        """
        Create a new file logger.

        :param filename: name of the file to write to, defaults to None
        :type filename: str | None, optional
        """
        log_dir = os.path.join(os.getcwd(), "logs")

        if not os.path.isdir(log_dir):
            os.mkdir(log_dir)

        if filename is None:
            filename = os.path.join(
                log_dir, f"{datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.log"
            )
        else:
            filename = os.path.join(log_dir, filename)

        self.__log_file = open(filename, "w")  # type: ignore

        self.__log_file.write("timestamp,system_id,component_id,message_type,message\n")

        return

    def __call__(
        self,
        timestamp: int,
        system_id: int,
        component_id: int,
        message_type: str,
        message: dict,
    ) -> None:
        """
        Add a log to the log file.

        :param timestamp: timestamp that the message was received
        :type timestamp: int
        :param system_id: system ID of the agent that sent the message
        :type system_id: int
        :param component_id: component ID of the agent that sent the message
        :type component_id: int
        :param message_type: MAVLink message type
        :type message_type: str
        :param message: MAVLink message
        :type message: dict
        """
        if "mavpackettype" in message:
            message.pop("mavpackettype")

        message_log = ""
        for key, value in message.items():
            if isinstance(value, list):
                value_log = ""
                for elm in value:
                    value_log += f"{elm}/"

                message_log += f"{key}:{value_log};"
            else:
                message_log += f"{key}:{value};"

        self.__log_file.write(
            f"{timestamp},{system_id},{component_id},{message_type},{message_log}\n"
        )

        return


def parse_log_file(
    logfile: str, message_type: str | None = None
) -> pd.DataFrame | dict[str, pd.DataFrame]:
    """
    Parse a pymavswarm log file.

    :param logfile: logfile to parse (including its path)
    :type logfile: str
    :param message_type: specific message type to get, defaults to None
    :type message_type: str | None, optional
    :return: messages received of the specified type or a dictionary with all messages
        with their respective types as keys
    :rtype: pd.DataFrame | dict[str, pd.DataFrame]
    """
    log_dict: dict[str, pd.DataFrame] = {}

    # Specify the indexes of each of the different properties
    TIMESTAMP_IDX = 0
    SYS_ID_IDX = 1
    COMP_ID_IDX = 2
    MSG_TYPE_IDX = 3
    MSG_IDX = 4
    PROP_NAME_IDX = 0
    PROP_VALUE_IDX = 1

    with open(logfile, "r") as log:
        reader = csv.reader(log)

        # Skip file headers
        next(reader, None)

        for row in reader:
            msg_dict: dict[str, str | list[str]] = {
                "timestamp": row[TIMESTAMP_IDX],
                "system_id": row[SYS_ID_IDX],
                "component_id": row[COMP_ID_IDX],
            }

            # Split the message
            msg = row[MSG_IDX].split(";")

            # Remove any empty strings
            msg = [prop for prop in msg if prop]

            # Split each message property into a key value pair
            for prop in msg:
                prop_split = prop.split(":")

                # Parse lists into lists when necessary
                if "/" in prop_split[PROP_VALUE_IDX]:
                    msg_dict[prop_split[PROP_NAME_IDX]] = prop_split[
                        PROP_VALUE_IDX
                    ].split("/")
                else:
                    msg_dict[prop_split[PROP_NAME_IDX]] = prop_split[PROP_VALUE_IDX]

            # Create a dataframe from the dictionary
            msg_df = pd.DataFrame([msg_dict])

            # Append the dataframe to the dictionary
            if row[MSG_TYPE_IDX] not in log_dict:
                log_dict[row[MSG_TYPE_IDX]] = []

            log_dict[row[MSG_TYPE_IDX]].append(msg_df)

        # Concatenate each of the dataframes
        for msg_type in log_dict.keys():
            log_dict[msg_type] = pd.concat(log_dict[msg_type])

    # If a specific message type was desired, return only that dataframe
    if message_type is not None:
        return log_dict[message_type]

    return log_dict
