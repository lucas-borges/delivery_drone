from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import dronekit
from pymavlink import mavutil
import time
import frame_conversion

class Drone():
    """docstring for ClassName"""
    cruise_altitude = None
    address = None
    vehicle = None
    cmds = None
    target = None
    def __init__(self, address, target, altitude=10):
        self.target = target
        self.cruise_altitude = altitude
        self.address = address
        self.connect()
        self.cmds = self.vehicle.commands
    
    def connect(self):
        print 'Connecting to vehicle on: %s' % self.address
        self.vehicle = connect(self.address, wait_ready=True, baud=57600)

    def clear_mission(self):
        self.cmds.clear()
        self.cmds.upload()

    def download_mission(self):
        self.cmds.download()
        self.cmds.wait_ready()

    def begin_mission(self):
        self.vehicle.mode = VehicleMode("AUTO")

    def prepare_mission(self):
        # Takeoff (dummy for now, one because first isnt added)
        self.cmds.add(command_takeoff(self.cruise_altitude))
        # Go to destination
        self.cmds.add(command_waypoint(self.target[0], self.target[1], self.cruise_altitude))
        # Land
        self.cmds.add(command_land())
        # Release package
        # self.cmds.add(command_unlock())
        # Takeoff
        self.cmds.add(command_takeoff(self.cruise_altitude))
        # Undo release
        # self.cmds.add(command_lock())
        # MAV_CMD_NAV_RETURN_TO_LAUNCH
        self.cmds.add(command_rtl(self.cruise_altitude))
        # Dummy
        self.cmds.add(command_dummy())

    def upload_mission(self):
        self.cmds.upload()

    def distance_to_current_waypoint(self):
        # TODO: error safe it
        """
        Gets distance in metres to the current waypoint.
        It returns None for the first waypoint (Home location).
        """
        nextwaypoint=self.cmds.next
        if nextwaypoint ==0:
            return None
        missionitem=self.cmds[nextwaypoint-1] #commands are zero indexed
        lat=missionitem.x
        lon=missionitem.y
        alt=missionitem.z
        targetWaypointLocation=LocationGlobalRelative(lat,lon,alt)
        distancetopoint = frame_conversion.get_distance_metres(self.vehicle.location.global_frame, targetWaypointLocation)
        return distancetopoint

    def arm(self):
        print "Basic pre-arm checks"
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print " Waiting for vehicle to initialise..."
            time.sleep(1)

            
        print "Arming motors"
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True    

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:      
            print " Waiting for arming..."
            time.sleep(1)

    def takeoff(self):
        print "Taking off!"
        self.vehicle.simple_takeoff(self.cruise_altitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print " Altitude: ", self.vehicle.location.global_relative_frame.alt 
            #Break and return from function just below target altitude.        
            if self.vehicle.location.global_relative_frame.alt>=self.cruise_altitude*0.95: 
                print "Reached target altitude"
                break
            time.sleep(1)

    def simple_goto(self, waypoint):
        self.vehicle.simple_goto(waypoint)

    def run(self):
        print "Clearing mission"
        self.clear_mission()
        print "Preparing mission"
        self.prepare_mission()
        print "Uploading mission"
        self.upload_mission()
        time.sleep(2)
        raw_input("Press enter to begin arming and taking off")
        self.arm()
        self.takeoff()
        self.begin_mission()

def command_takeoff(alt):
    return dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, alt)

def command_waypoint(lat, lon, alt):
    return dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt)

def command_land():
    return dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0, 0)

def command_rtl(alt):
    return dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, alt)

def command_lock():
    return dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 0, 0, 0, 0, 0, 0, 0, self.cruise_altitude)

def command_unlock():
    return dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 0, 0, 0, 0, 0, 0, 0, self.cruise_altitude)

def command_dummy():
    return dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0, 0)