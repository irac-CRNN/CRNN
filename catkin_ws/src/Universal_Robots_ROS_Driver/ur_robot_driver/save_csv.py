#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 22 15:22:01 2021

@author: iracpc
"""
import csv
from UR3_control import base_control, base_robot, shoulder_control, shoulder_robot, elbow_control, elbow_robot, wrist3_control, wrist3_robot, time_check

with open('data.csb', 'w') as f:
    writer = csv.writer(f)
    writer.writerow(base_control)
    writer.writerow(base_robot)
    writer.writerow(shoulder_control)
    writer.writerow(shoulder_robot)
    writer.writerow(elbow_control)
    writer.writerow(elbow_robot)
    writer.writerow(wrist3_control)
    writer.writerow(wrist3_robot)
    writer.writerow(time_check)