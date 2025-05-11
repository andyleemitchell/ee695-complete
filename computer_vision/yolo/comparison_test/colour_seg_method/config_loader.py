import json

# this class is from https://stackoverflow.com/a/68244012


class Dict(dict):
    """dot.notation access to dictionary attributes"""

    __getattr__ = dict.__getitem__
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__


class ConfigLoader:
    @staticmethod
    def __load__(data):
        if type(data) is dict:
            return ConfigLoader.load_dict(data)
        if type(data) is list:
            return ConfigLoader.load_list(data)
        return data

    @staticmethod
    def load_dict(data: dict):
        result = Dict()
        for key, value in data.items():
            result[key] = ConfigLoader.__load__(value)
        return result

    @staticmethod
    def load_list(data: list):
        return [ConfigLoader.__load__(item) for item in data]

    @staticmethod
    def load_json(path: str):
        with open(path) as f:
            return ConfigLoader.__load__(json.loads(f.read()))

    @staticmethod
    def write_json(path: str, config: dict):
        with open(path, "w") as f:
            json.dump(config, f, sort_keys=True, indent=4)
