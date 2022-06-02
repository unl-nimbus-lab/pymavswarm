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
