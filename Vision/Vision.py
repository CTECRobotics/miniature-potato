import cv2
from networktables import NetworkTables
NetworkTables.initialize(server='roborio-6445-frc.local')
sd = NetworkTables.getTable('SmartDashboard')
Jetson = NetworkTables.getTable('Jetson')
from enum import Enum
while True:
    color = sd.getvalue("Alliance_Color")
    position = sd.getvalue("Auto_Mode")
    location = ""
    if position is 1:
    if position is 2:
    if position is 3:
