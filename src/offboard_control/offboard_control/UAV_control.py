import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus
from offboard_control.d_star_lite.d_star_lite import initDStarLite, computeShortestPath, moveAndRescan
from offboard_control.d_star_lite.graph import Grid
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
            self_uav_prefix = '/px4_' + self.uav_id
            if self.uav_id == '1':
                self.other_position_subscriber = self.create_subscription(VehicleOdometry, 
                                '/px4_2/fmu/out/vehicle_odometry', self.other_position_callback, qos_profile)
            else:
                self.other_position_subscriber = self.create_subscription(VehicleOdometry, 
                                '/px4_1/fmu/out/vehicle_odometry', self.other_position_callback, qos_profile)
        else:
            self_uav_prefix = ''

        # Create ROS2 Publisher Nodes
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, self_uav_prefix + '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, self_uav_prefix + '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, self_uav_prefix + '/fmu/in/vehicle_command', qos_profile)

        # Create ROS2 Subscribers Nodes
        self.position_subscriber = self.create_subscription(
            VehicleOdometry, self_uav_prefix + '/fmu/out/vehicle_odometry', self.position_callback, qos_profile)
        self.status_subscriber = self.create_subscription(
            VehicleStatus, self_uav_prefix + '/fmu/out/vehicle_status', self.status_callback, qos_profile)

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
            self.waypoints = [[1.0, 1.0, -2.5, 0.0],
                              [1.0, 8.0, -2.5, 0.0],
                              [8.0, 8.0, -2.5, 0.0]]
        elif self.uav_id == '2':
            self.waypoints = [[0.0, 0.0, -2.5, 0.0],
                              [10.0, 10.0, -2.5, 0.0],
                              [2.0, 5.0, -10.0, 0.0]]
        else:
            self.waypoints = []
        # Whether the UAV has been armed
        self.armed = False
        # Whether the UAV is currently performing D* Lite searching
        self.d_star_lite_mode = False
        self.next_waypoint = self.waypoints[0] # [x, y, z, yaw]
        
        ## D* Lite Functionality
        self.world_x_offset = 5 # [m]
        self.world_y_offset = 5 # [m]
        self.min_flight_height = 2.0 # [m]

        self.reached_min_flight_height = False

        self.env_length = 20 # [m]
        self.env_width = 20 # [m]
        self.env_height = 10 # [m]

        self.node_distance = 0.5 # [m / node]
        self.view_distance = 5 # View distance of the UAV for D* Lite rescanning  [nodes]

        self.node_length = int(self.env_length / self.node_distance) + 1
        self.node_width = int(self.env_width / self.node_distance) + 1
        self.node_height = int(self.env_height / self.node_distance) + 1

        # Nodal location of the other UAV in the mission
        self.other_node_location = []

        

        self.graph = Grid((self.node_length, self.node_width, self.node_height))

        print(f'Created Grid of size: {self.node_length} x {self.node_width} x {self.node_height}')
        # print(f'Node Location: {(0,0,0)}, World Location:{self.node_to_world((0,0,0))}')
        # print(f'World Location: {[0,3,-2]}, Node Location:{self.world_to_node([0,3,-2])}')

        # print(f'Node Location: {(10,10,10)}, World Location:{self.node_to_world((10,10,10))}')
        # print(f'World Location: {[2,4,-7]}, Node Location:{self.world_to_node([2,4,-7])}')

        # print(f'Node Location: {(20,20,20)}, World Location:{self.node_to_world((20,20,20))}')
        # print(f'World Location: {[5,5,-12]}, Node Location:{self.world_to_node([5,5,-12])}')

        # print(f'Node Location: {(self.node_length, self.node_width, self.node_height)}, World Location:{self.node_to_world((self.node_length, self.node_width, self.node_height))}')
        # print(f'World Location: {[15,15,-10]}, Node Location:{self.world_to_node([15,15,-10])}')


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

    def other_position_callback(self, msg_in):
        """Callback function to track the nodal location of other mission UAVs"""
        self.other_node_location = self.world_to_node([msg_in.position[0], 
                                                       msg_in.position[1], 
                                                       msg_in.position[2]])

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

    def world_to_node(self, world_location):
        """Function for translating between the world frame and the D* Lite node locations"""
        # self.get_logger().info(f'{world_location}')
        node_x = int(round((world_location[0] + self.world_x_offset) / self.node_distance))
        node_y = int(round((world_location[1] + self.world_y_offset) / self.node_distance))
        node_z = -int(round((world_location[2] + self.min_flight_height) / self.node_distance))
        node_location = (node_x, node_y, node_z)
        return node_location
    
    def node_to_world(self, node_location):
        """Function for translating between the D* Lite node locations and the world frame"""
        world_x = (node_location[0] * self.node_distance) - self.world_x_offset
        world_y = (node_location[1] * self.node_distance) - self.world_y_offset
        world_z = -(node_location[2] * self.node_distance) - self.min_flight_height
        
        yaw_angle = 0.0
        world_location = [world_x, world_y, world_z, yaw_angle]
        
        return world_location

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
            if not self.d_star_lite_mode and self.reached_min_flight_height and self.waypoint_index != len(self.waypoints):
                self.get_logger().info('Starting D* Lite Planning')
                self.graph = Grid((self.node_length, self.node_width, self.node_height))
                # Initialise the D* Lite Algorithm between adjacent waypoints [[self.x, self.y, self.z], self.next_waypoint]
                self.start_node = self.world_to_node([self.x, self.y, self.z])
                self.get_logger().info(f'Start Node: {self.start_node}')
                self.goal_node = self.world_to_node(self.waypoints[self.waypoint_index][:3])
                self.get_logger().info(f'Goal Node: {self.goal_node}')
                self.graph.setStart(self.start_node)
                self.graph.setGoal(self.goal_node)
                self.graph, self.queue, self.k_m = initDStarLite(self.graph, self.start_node, self.goal_node)
                computeShortestPath(self.graph, self.queue, self.start_node, self.k_m)

                
                
                # Set the next waypoint to be the first node within the D* Lite search
                self.target_node, self.k_m = moveAndRescan(self.graph, self.queue, self.start_node, self.k_m, self.view_distance, self.other_node_location)
                self.current_node = self.target_node
                self.next_waypoint = self.node_to_world(self.target_node)
                self.get_logger().info(f'The first D* Node to visit is {self.target_node} at: {self.next_waypoint}')
                
                # Hold the nodal locations of the path generated for logging purposes
                self.d_star_path = [self.start_node, self.target_node]

                # Note that the UAV is performing a D* Lite Search between waypoints
                self.d_star_lite_mode = True

            # If the UAV is currently performing a D* Lite search
            elif not self.landed:
                # If the next high-level waypoint has been reached, the D* Lite Search has been completed
                if not self.reached_min_flight_height:
                    self.next_waypoint = [self.x, self.y, -self.min_flight_height * 1.5, 0.0]
                if self.reached_waypoint(self.waypoints[self.waypoint_index]):
                        if self.reached_min_flight_height:
                            self.get_logger().info(f'UAV {self.uav_id} has reached Waypoint {self.waypoint_index + 1} at: {self.waypoints[self.waypoint_index][:3]}')
                            self.waypoint_index += 1
                            self.d_star_lite_mode = False
                            self.get_logger().info(f'D* Lite Nodal Path: {self.d_star_path}')
                        else:
                            self.reached_min_flight_height = True
                        # If all waypoints have been reached, land the UAV and disarm the system
                        if self.waypoint_index == len(self.waypoints):
                            self.land()
                            self.landed = True
                elif self.reached_waypoint(self.next_waypoint):
                    if self.reached_min_flight_height:
                        # If the last D* Lite node has been reached, rescan the area and determine the next suitable node position for the UAV
                        self.get_logger().info('Reached the desired D* Lite Waypoint, calculating next')
                        self.target_node, self.k_m = moveAndRescan(self.graph, self.queue, self.current_node, self.k_m, self.view_distance, self.other_node_location)
                        self.current_node = self.target_node
                        self.next_waypoint = self.node_to_world(self.target_node)
                        self.get_logger().info(f'The next D* Node to visit is {self.target_node} at: {self.next_waypoint}')
                    
                        # Add the next node to the stored D* Lite path
                        self.d_star_path.append(self.target_node)
                    
                    else:
                        self.reached_min_flight_height = True
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
    rclpy.spin(offboard_control)
    # try:
    #     rclpy.spin(offboard_control)
    # except:
    #     rclpy.logging.get_logger('Shutdown').info('Shutting Down')

    offboard_control.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)