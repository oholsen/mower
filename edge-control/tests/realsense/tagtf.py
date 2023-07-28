import json

import yaml

from edge_control.config import tag_config

# Site coordinates when in dock (origin for realsense)
dock_x = 2.51
dock_y = -1.70


def tags():
    for tag in tag_config.tags:
        tag.position.x += dock_x
        tag.position.y += dock_y

    # print(tag_config.tags)
    print(yaml.dump(tag_config, indent=2))


def site():
    s = json.load(open("../gui/sites/office.json"))
    for p in s["limit"]:
        p["x"] += dock_x
        p["y"] += dock_y
    print(json.dumps(s, indent=2))


def main():
    # tags()
    site()


if __name__ == "__main__":
    main()
