# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import yaml
import sys
import os

if __name__ == "__main__":

    if len(sys.argv) < 3:
        print("use: yamlConvertor.py path channel")
        print("file name and channel needed!")
        sys.exit(0)

    source = sys.argv[1]
    channel = int(sys.argv[2])

    if not os.path.exists(source):
        print("can't find source file: " + source)
        sys.exit(0)

    with open(source, 'r') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)

    allCrazyFlies = data['files']

    for item in allCrazyFlies:
        item['channel'] = channel
        item['type'] = "CF21SingleMarker"
        item['initialPosition'] = item["Position"]+[0.0]
        del item['Position']

    with open("output.yaml", 'w') as f:
        f.write(yaml.dump({"crazyflies": allCrazyFlies}))