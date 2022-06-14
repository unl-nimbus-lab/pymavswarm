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

"""
The status codes and their respective meanings used for messages.
"""

# The message was successfully sent
SUCCESS = (1, "success")

# The message was not acknowledged
ACK_FAILURE = (2, "acknowledgement failure")

# The state change that the message was attempting to accomplish was not verified
STATE_VALIDATION_FAILURE = (3, "state validation failure")

# A stage within a sequence command failed
SEQUENCE_STAGE_FAILURE = (4, "sequence state failure")

# The message is configured wrong
INVALID_PROPERTIES = (5, "invalid message properties")

# Package failed
PACKAGE_FAILURE = (6, "package failure")
