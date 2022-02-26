import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--foo', help='foo help', action="store_true")
args = parser.parse_args()
print(args)


import os
import sys
# 添加路径
currentUrl = os.path.dirname(__file__)
parentUrl = os.path.abspath(os.path.join(currentUrl, os.pardir))
sys.path.append(parentUrl)
from pycrazyswarm import *