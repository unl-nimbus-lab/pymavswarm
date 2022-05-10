from dataclasses import dataclass


@dataclass
class MsgStatusCodes:
    """
    The status codes and their respective meanings used for messages
    """
    
    # The message was successfully sent
    SUCCESS = 1

    # The message was not acknowledged
    ACK_FAILURE = 2

    # The state change that the message was attempting to accomplish was not verified
    STATE_VALIDATION_FAILURE = 3

    # A stage within a sequence command failed
    SEQUENCE_STAGE_FAILURE = 4

    # The message is configured wrong
    INVALID_PROPERTIES = 5
