import json


class Configuration:
    def __init__(self, dimension):
        self.dimension = dimension

class ConfigJsonSerializer:
    @staticmethod
    def deserialize(filename: str) -> Configuration:
        """
        Deserializes the json object from filename and returns the
        Configuration object.
        """
        jsonFile = open(filename, "r")
        config = json.loads(jsonFile.read())
        return Configuration(**config)

    @staticmethod
    def serialize(config: Configuration, filename: str) -> bool:
        """
        Serializes the Configuration object into json file with the filename
        and returns true on success or false otherwise.
        """
        configData = json.dumps(config.__dict__, indent=4)
        jsonFile = open(filename, "w")
        jsonFile.write(configData)
        jsonFile.close()
        
