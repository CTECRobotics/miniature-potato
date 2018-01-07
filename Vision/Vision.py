from blue import blue
from red import red
import pyfrc
from networktables import NetworkTables
NetworkTables.initialize(server='roborio-6445-frc.local')
sd = NetworkTables.getTable('SmartDashboard')
jetson = NetworkTables.getTable('jetson')
while True:
