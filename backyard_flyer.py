import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

TOLERANCE = 0.5

class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)    
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = self.calculate_box()
        self.in_mission = True
        self.check_state = {}
        self.nextWaypoint = 0
        self.loops = 0

        # initial state
        self.flight_state = States.MANUAL

        # Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state == States.TAKEOFF:
            alt = self.local_position[2] * -1.0
            target_alt = self.target_position[2]
            if abs(target_alt-alt) < TOLERANCE:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            x = self.local_position[0]
            y = self.local_position[1]
            z = self.local_position[2] * -1.0
            target_x = self.target_position[0]
            target_y = self.target_position[1]
            target_z = self.target_position[2]

            if abs(target_x-x) < TOLERANCE and abs(target_y-y) < TOLERANCE and abs(target_z-z) < TOLERANCE:
                self.waypoint_transition()


    def velocity_callback(self):
        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.LANDING:
            z_vel = self.local_velocity[2]
            expected_distance_from_ground = self.local_position[2]
            if abs(z_vel) < .01 and abs(self.local_position[2]) < TOLERANCE:
                self.disarming_transition()
                self.manual_transition()

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if self.flight_state == States.MANUAL and self.armed:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()

    def calculate_box(self):
        """
        1. Return waypoints to fly a box
        """
        waypoint1 = np.array([0.0, 0.0, 10.0])
        waypoint2 = np.array([10.0, 0.0, 10.0])
        waypoint3 = np.array([10.0, 10.0, 10.0])
        waypoint4 = np.array([0.0, 10.0, 10.0])

        return [waypoint1, waypoint2, waypoint3, waypoint4]

    def arming_transition(self):
        """        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0], self.global_position[1], self.global_position[2])
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        self.target_position = np.array([0.0, 0.0, 3.0])
        self.takeoff(3.0)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")

        time.sleep(.5)

        if self.loops > 0:
            self.landing_transition()
        elif self.nextWaypoint < len(self.all_waypoints):
            self.target_position = self.all_waypoints[self.nextWaypoint]
            self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0)
            self.nextWaypoint += 1
            self.flight_state = States.WAYPOINT
        else:
            self.target_position = self.all_waypoints[0]
            self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0)
            self.nextWaypoint = 1
            self.flight_state = States.WAYPOINT
            self.loops += 1

    def landing_transition(self):
        """
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        self.land()
        print("landing transition")

        self.flight_state = States.LANDING

    def disarming_transition(self):
        """        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
