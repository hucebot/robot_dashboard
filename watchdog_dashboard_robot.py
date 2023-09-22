#!/usr/bin/python3
import sys, os
import subprocess

while True:
    ret = subprocess.run(["python3", "dashboard_robot.py"] + sys.argv[1:])
    if ret.returncode == 0:
        sys.exit(0)
