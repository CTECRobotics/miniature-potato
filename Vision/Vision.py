from blue import blue
from red import red
import cv2
from networktables import NetworkTables
NetworkTables.initialize(server='roborio-6445-frc.local')
sd = NetworkTables.getTable('SmartDashboard')
Jetson = NetworkTables.getTable('Jetson')
while True:
    color = sd.getvalue("Alliance_Color")
    position = sd.getvalue("Auto_Mode")
    if "blue" in color:
        blue.process(blue, cv2.VideoCapture(0))
    if "red" in color:
        red.process(red, cv2.VideoCapture(0))
    if position is 1:

    if position is 2:

    if position is 3:
