import yaml


class GenericConfig(object):

    def __init__(self, cfg):
        self.cfg = cfg

    def apply(self):
        for key in self.cfg:
            val = self.cfg[key]
            setattr(self, key, val)


def load_yaml_config(yaml_filename, expect=None, version=None):
    with open(yaml_filename, "r", encoding="utf-8") as infile:
        data = yaml.load(infile, Loader=yaml.FullLoader)
        if expect is not None:
            if "$type" not in data or data["$type"] != expect:
                raise Exception(f"Invalid YAML file, expecting $type=={expect}!")
        if version is not None:
            if "$version" not in data or str(data["$version"]) != version:
                raise Exception(f"Invalid YAML file version, expecting $version=={version}!")

        return data

