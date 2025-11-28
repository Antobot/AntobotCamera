import os
import math
import yaml
from typing import Final


REMOTE_USER: Final = os.getenv("HOST_USER", "").strip()
SSH_PREFIX: Final = [] if not REMOTE_USER else ["ssh", f"{REMOTE_USER}@localhost"]

def is_valid_file(item):
    """
    return true/false depending whether item is a file and should be transferred
    item : as string
    """
    # file extensions to transfer
    valid_extensions = ('.svo', '.svo2', '.yaml', '.json', '.h264', '.txt')

    if os.path.isfile(item):
        # check if file extension is in defined set
        if os.path.splitext(item)[1] in valid_extensions:
            return True
        else:
            return False
    else:
        return False


def bytes2str_pretty(b):
    """
    return bytes as a 3 significant figure number with appropriate unit prefix
    """
    units = {0: 'B', 1: 'kB', 2: 'MB', 3: 'GB', 4: 'TB'}

    # thousand power
    tp = math.floor(math.log(b, 1000))

    return f"{b / (1000 ** tp):.3g} {units[tp]}"


def safe_yaml_write(path, data):
    tmp = path.with_suffix(".tmp")
    with tmp.open("w") as fh:
        yaml.safe_dump(data, fh)
        fh.flush()
        os.fsync(fh.fileno())
    tmp.replace(path)
