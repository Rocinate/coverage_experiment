# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import json
import pickle
from sendJson import SendJson

if __name__ == "__main__":
    f = open("sim.txt", "rb")
    allWaypoints = pickle.load(f, encoding='latin1')
    f.close()

    for waypoints in allWaypoints:
        SendJson(json.dumps(waypoints))