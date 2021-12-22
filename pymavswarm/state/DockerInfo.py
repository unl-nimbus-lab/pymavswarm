class DockerInfo:
    def __init__(self, 
                 version: str='None', 
                 last_update: str='None') -> None:
        """
        version     : The current version of the Docker image deployed by the agent
        last_update : The last date that the agent was updated
        """
        self.version: str = version
        self.last_update: str = last_update