import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus
from offboard_control.d_star_lite import d_star_lite, graph, notes
from math import pi

class OffboardControl(Node):
    """Python package for providing offboard control"""

    ## INITITALISATION FUNCTION

    def __init__(self):
        super().__init__('UAV_Control')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability = 2,
            durability = 2,
            liveliness = 1,
            history = 1,
            depth = 1,
        )

        # Declare a parameter to hold the UAV ID
        self.declare_parameter('UAV_ID', '0')
        self.uav_id = self.get_parameter('UAV_ID').get_parameter_value().string_value
        # self.get_logger().info(f'UAV ID: {self.uav_id}')

        # Add a prefix to all ROS2 publishers and subscribers if multiple UAVs are being utilised
        if self.uav_id != '0':
            uav_prefix = '/px4_' + self.uav_id
        else:
            uav_prefix = ''

        # Create ROS2 Publisher Nodes
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, uav_prefix + '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, uav_prefix + '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, uav_prefix + '/fmu/in/vehicle_command', qos_profile)

        # Create ROS2 Subscribers Nodes
        self.position_subscriber = self.create_subscription(
            VehicleOdometry, uav_prefix + '/fmu/out/vehicle_odometry', self.position_callback, qos_profile)
        self.status_subscriber = self.create_subscription(
            VehicleStatus, uav_prefix + '/fmu/out/vehicle_status', self.status_callback, qos_profile)

        ## Initialise Mission Variables

        # UAV Status indicators
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        # Initial UAV Position
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        # Check if the UAV has been commanded to land after completing it's mission
        self.landed = False
        # The tolerance for reaching each waypoint
        self.waypoint_tol = 0.1 # [m]
        # The current waypoint being targeted
        self.waypoint_index = 0
        # The list of all waypoints for the mission in order
        if self.uav_id == '1':
            self.waypoints = [[1.0, 1.0, -2.5, pi],
                              [-1.0, -1.0, -2.5, 0.0],
                              [-1.0, 1.0, -5.0, -pi]]
        elif self.uav_id == '2':
            self.waypoints = [[-1.0, -1.0, -2.5, pi],
                              [-10.0, -10.0, -2.5, 0.0],
                              [-2.0, 5.0, -10.0, -pi]]
        else:
            self.waypoints = []
        # Whether the UAV has been armed
        self.armed = False
        # Whether the UAV is currently performing D* Lite searching
        self.d_star_lite_mode = False
        self.next_waypoint = self.waypoints[0] # [x, y, z, yaw]
        
        # Create a callback timer to periodically publish control commands
        self.timer_period = 0.1 # [s]
        self.timer = self.create_timer(self.timer_period, self.check_mission_status)

    ## CALLBACK FUNCTIONS

    def position_callback(self, msg_in):
        """Callback function for vehicle_local_position topic subscriber."""
        self.x = msg_in.position[0]
        self.y = msg_in.position[1]
        self.z = msg_in.position[2]

    def status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
        #self.get_logger().info(str(vehicle_status.nav_state))
        #self.get_logger().info(str(vehicle_status.arming_state))

    ## UAV STATUS FUNCTIONS

    def arm(self):
        """Send an arm command to the vehicle."""
        # It is necessary to first ensure a stable connection before transitioning to offboard control.
        # This involves sending 10 heartbeat signals before arming the UAV
        if self.offboard_setpoint_counter < 10:
            self.offboard_setpoint_counter += 1
        elif self.offboard_setpoint_counter == 10:
            # Change the UAV control mode to offboard control
            self.engage_offboard_mode()
            # Arm the UAV
            self.get_logger().info(f"Arming UAV {self.uav_id}")
            self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info('Arm command sent')
            # Note that the UAV has been armed
            self.armed = True   
        
    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Land at the current location"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    ## PUBLISH COMMAND FUNCTIONS

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = self.next_waypoint[:3] # [x, y, z]
        msg.yaw = self.next_waypoint[3] # Yaw angle [rads]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = int(self.uav_id) + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    ## MISSION HELPER FUNCTIONS

    def distance_to_waypoint(self, waypoint):
        """Calculates the Euclidean distance between the UAV and the current waypoint"""
        # Calculate the distance in each cardinal direction
        x_dis = waypoint[0] - self.x
        y_dis = waypoint[1] - self.y
        z_dis = waypoint[2] - self.z

        # Calculate the total distance using the pythagorean theorem
        waypoint_distance = (x_dis**2 + y_dis**2 + z_dis**2) ** (1/2)
        
        #self.get_logger().info(f'Distance to Waypoint: {waypoint_distance}')

        # Return the total distance
        return waypoint_distance
    
    def reached_waypoint(self, waypoint):
        """Determines if the UAV has reached the current waypoint"""
        return self.distance_to_waypoint(waypoint) <= self.waypoint_tol

    ## MAIN MISSION STATUS FUNCTION

    def check_mission_status(self):
        """Callback function to publish control commands"""

        # Always publish a heartbeat signal to ensure connectivity has not been lost
        self.publish_offboard_control_heartbeat_signal()
        # Begin the control sequence by arming the UAV
        if not self.armed:
            self.arm()

        # Start path planning once the UAV is armed
        else:
            # If the UAV is not currently performing a D* Lite Search
            if not self.d_star_lite_mode:
                # Initialise the D* Lite Algorithm between adjacent waypoints [[self.x, self.y, self.z], self.next_waypoint]
                # Set the next waypoint to be the first node within the D* Lite search

                # Note that the UAV is performing a D* Lite Search between waypoints
                self.d_star_lite_mode = True
            # If the UAV is currently performing a D* Lite search
            else:
                # If the next high-level waypoint has been reached, the D* Lite Search has been completed
                if self.reached_waypoint(self.waypoints[self.waypoint_index]):
                        self.get_logger().info(f'UAV {self.uav_id} has reached Waypoint {self.waypoint_index + 1}')
                        self.waypoint_index += 1
                        self.next_waypoint = self.waypoints[self.waypoint_index]
                        self.d_star_lite_mode = False
                        # If all waypoints have been reached, land the UAV and disarm the system
                        if self.waypoint_index == len(self.waypoints):
                            self.land()
                            self.landed = True
                elif self.reached_waypoint(self.next_waypoint):
                    pass
                    # Move and Rescan
            # If the UAV has not been landed, continuously publish the location of the next waypoint
            if not self.landed:
                self.publish_position_setpoint()

        # If the UAV has landed, shut down communications with the UAV
        if self.landed and self.vehicle_status.arming_state == 1:
            self.get_logger().info(f"UAV {self.uav_id} has completed the mission")
            raise SystemExit
                


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    try:
        rclpy.spin(offboard_control)
    except:
        rclpy.logging.get_logger('Shutdown').info('Shutting Down')

    offboard_control.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)