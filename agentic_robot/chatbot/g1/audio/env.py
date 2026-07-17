import os
import json

SETTINGS_PATH = os.getenv("G1_SETTINGS_PATH", "~/.config/g1/g1.json")
SETTINGS = json.load(open(os.path.expanduser(SETTINGS_PATH), "r"))
