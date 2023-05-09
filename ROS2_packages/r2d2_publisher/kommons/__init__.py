import os, json, itertools, argparse, yaml, sys
import unicodedata
import datetime
import random 
import shutil



# a pure python shingling function that will be used in comparing
# LSH to true Jaccard similarities
def get_shingles(text, char_ngram=5):
    """
    Create a set of overlapping character n-grams.
    
    Only full length character n-grams are created, that is the first character
    n-gram is the first `char_ngram` characters from text, no padding is applied.

    Each n-gram is spaced exactly one character apart.

    Parameters
    ----------

    text: str
        The string from which the character n-grams are created.

    char_ngram: int (default 5)
        Length of each character n-gram.
    """
    return set(text[head:head + char_ngram] for head in range(0, len(text) - char_ngram))


def jaccard(set_a, set_b):
    """
    Jaccard similarity of two sets.
    
    The Jaccard similarity is defined as the size of the intersection divided by
    the size of the union of the two sets.

    Parameters
    ---------
    set_a: set
        Set of arbitrary objects.

    set_b: set
        Set of arbitrary objects.
    """
    intersection = set_a & set_b
    union = set_a | set_b
    return len(intersection) / len(union)


def flatten(lines):
    """
    Takes a list of lists as input. 
    Returns a flattened list.
    """
    return [item for sublist in lines for item in sublist]


def pick_n(list, n):
    """
    Returns a list of n random items from the input list.
    """
    return [random.choice(list) for i in range(n)]


def get_file_mod_datetime(file):
    """
    Returns the last modification datetime of the file.
    """
    return datetime.datetime.fromtimestamp(os.path.getmtime(file))
    

def ensure_dir(d):
    """
    Creates the directory if it doesn't exist.
    """
    if not os.path.exists(d):
        os.makedirs(d)


def empty_dir(folder):
    """
    Deletes all files and folders in the given folder.
    """
    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print('Failed to delete %s. Reason: %s' % (file_path, e))


def save_text(path, txt):
    """
    Saves the text to the specified path.
    """
    with open(path, "w", encoding="utf-8") as outfile:
        outfile.write(txt)


def load_text(path):
    """
    Returns the text loaded from the specified file path.
    """
    with open(path, "r", encoding="utf-8") as infile:
        txt = infile.read()
    return txt


def load_yaml(yaml_path):
    """
    Returns a dictionary loaded from the specified yaml file path.
    """
    with open(yaml_path, "r", encoding="utf-8") as infile:
        return yaml.load(infile, Loader=yaml.FullLoader)


def load_json(path):
    """
    Returns a dictionary loaded from the specified json file path.
    """
    with open(path, "r", encoding="utf-8") as infile:
        data = json.load(infile)
    return data


def save_json(path, data, ensure_ascii=False, cls=None):
    """
    Saves data to the specified json file path.
    """
    with open(path, "w", encoding="utf-8") as outfile:
        json.dump(data, outfile, indent=4, ensure_ascii=ensure_ascii, cls=cls)


def save_yaml(path, data):
    """
    Saves data to the specified yaml file path.
    """
    with open(path, 'w', encoding="utf-8") as file:
        yaml.dump(data, file, allow_unicode=True)


def load_cfg(path):
    """
    Loads a config file in either yaml or json format.
    """
    if path.endswith(".json"):
        return load_json(path)
    elif path.endswith(".yaml"):
        return load_yaml(path)
    else:
        raise Exception(f"Unsupported config format: {path}")


def save_cfg(path, data):
    """
    Saves data to a config file in either yaml or json format.
    """
    if path.endswith(".json"):
        save_json(path, data)
    elif path.endswith(".yaml"):
        save_yaml(path, data)
    else:
        raise Exception(f"Unsupported config format: {path}")


def environ_or_required(key):
    """    
    Helper function for argparsers.
    Returns dictionary stating that the input var has a default value (from the environment): {"default":...} 
    or that the variable is missing (required): {"required": True}.
    """
    return (
        {'default': os.environ.get(key)} if os.environ.get(key)
        else {'required': True}
    )


def remove_accents(input_str):
    """
    Removes accent characters from the input string by replacing accents by non-accented variants.
    """
    nkfd_form = unicodedata.normalize('NFKD', str(input_str))
    return u"".join([c for c in nkfd_form if not unicodedata.combining(c)])

