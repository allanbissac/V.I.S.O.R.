#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ================= 0325 Arm-related Additions =================
from std_msgs.msg import Bool
# ====================================================

# ================= 0325 Added: Import Math, ActionClient, and Navigation Interfaces =================
import math
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
# ==================================================================================

# Import your custom message type (replace my_robot_interfaces with your actual package name)
from my_robot_interfaces.msg import ObjectCentroid

class NucController(Node):
    def __init__(self):
        super().__init__('nuc_controller')
        
        # ================= Core State Storage =================
        self.held_colour = None      # Color code of the object currently held by the arm
        self.is_holding = False      # Whether the arm is currently holding an object
        # ================================================
        
        # ================= 0325 Added: Create Navigation Action Client =================
        # As a client, only to detect if the "navigate_to_pose" server exists
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # ==============================================================
        
        # 1. Subscribe to the object_centroids topic published by the vision node
        self.sub = self.create_subscription(
            ObjectCentroid, 
            'object_centroids', 
            self.vision_callback, 
            10
        )
        
        # 2. Publish control commands to the arm_command topic
        self.pub = self.create_publisher(String, 'arm_command', 10)

        # ================= 0325 Arm-related Additions =================
        self.last_holding_state = None  # Used to record the last received arm state to avoid repeated printing
        
        # Subscribe to the arm's physical holding state
        self.holding_sub = self.create_subscription(
            Bool,
            'arm_holding_state',
            self.holding_state_callback,
            10
        )
        
        # Subscribe to arm out-of-bounds or error feedback
        self.feedback_sub = self.create_subscription(
            String,
            'arm_feedback',
            self.feedback_callback,
            10
        )
        # ====================================================
        
        self.get_logger().info("NUC central control node started, listening for vision data...")

    # ================= 0325 Arm-related Additions: State Change and Exception Handling Callbacks =================
    def holding_state_callback(self, msg):
        current_state = msg.data
        
        # Record initial state
        if self.last_holding_state is None:
            self.last_holding_state = current_state
            state_str = "Holding" if current_state else "Empty"
            self.get_logger().info(f"Arm physical state initial: {state_str}")
            return
            
        # Only display in terminal and process when the state changes
        if current_state != self.last_holding_state:
            state_str = "Holding" if current_state else "Empty"
            self.get_logger().info(f"Arm physical state : {state_str}")
            
            # [Important State Handling]: If the arm changes from "holding" to "empty"
            if current_state == False and self.last_holding_state == True:
                # Check if the central control still thinks it's holding something (handling unexpected drops)
                if self.is_holding:
                    self.get_logger().warn("Detected unexpected empty arm! Synchronizing and resetting central control internal state...")
                    self.is_holding = False
                    self.held_colour = None
                    
            # Update last state
            self.last_holding_state = current_state

    def feedback_callback(self, msg):
        self.get_logger().error(f"Received arm feedback alarm: {msg.data}")
        # When receiving out-of-bounds or other errors, it means the previously issued command was rejected by the arm.
        # If the central control just issued a pick command and assumes it holds something, rollback the state immediately.
        if self.is_holding:
            self.get_logger().warn("Command intercepted by physical layer, rolling back central control holding state to empty!")
            self.is_holding = False
            self.held_colour = None
    # ==============================================================================

    def vision_callback(self, msg):
        
        # ================= Testing object_centroids communication: =================
        # Used to test if topic communication is clear, print all raw data received. You can safely delete this block after testing.
        self.get_logger().info(
            f"[Communication Test] Received vision data -> "
            f"detected: {msg.detected}, "
            f"type: {'Cube' if msg.object_type else 'Bin'}, "
            f"colour: {msg.colour}, "
            f"xyz: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})"
        )
        # ==============================================================


        # If vision detects no objects, drop the data directly
        if not msg.detected:
            return

        # ================= 0325 Added: Generate Reached Target Position flag =================
        # Calculate the absolute 3D spatial distance to the target (Unit: mm)
        distance = math.sqrt(msg.x**2 + msg.y**2 + msg.z**2)
        
        # Use server_is_ready() to detect if the server exists (Exists=Moving, Does not exist=Stationary)
        is_navigating = self.nav_client.server_is_ready()
        
        # Core flag: If not navigating and distance < 1000.0 (1 meter), consider the target position reached
        robot_arrived = (not is_navigating) and (distance < 1000.0)
        # ================================================================================

        # ================= Action Logic 1: Pick =================
        # Condition: Saw a Cube (object_type == True) and the arm is currently empty
        # ================= 0325 Added: Integrate robot_arrived into pick command decision =================
        if msg.object_type == True and not self.is_holding and robot_arrived:
        # =======================================================================================
            
            # Record the color and state of the currently grabbed object
            self.held_colour = msg.colour
            self.is_holding = True
            
            self.get_logger().info(f"Found target Cube (Color: {msg.colour}), and reached target position (Distance: {distance:.1f}mm), locking target!")
            
            # Construct and send pick command to the arm 
            # (Since bool cannot distinguish size, output 'big' temporarily, you can modify as needed)
            cmd_msg = String()
            cmd_msg.data = f"pick big {msg.x:.1f} {msg.y:.1f} {msg.z:.1f}"
            self.pub.publish(cmd_msg)
            
            self.get_logger().info(f"Sent command: {cmd_msg.data}")

        # ================= Action Logic 2: Place =================
        # Condition: Saw a Bin (object_type == False) and the arm is currently holding an object
        # ================= 0325 Added: Integrate robot_arrived into place command decision =================
        elif msg.object_type == False and self.is_holding and robot_arrived:
        # ========================================================================================
            
            # Key: Color must match!
            if msg.colour == self.held_colour:
                self.get_logger().info(f"Found matching Bin (Color: {msg.colour}), and reached target position (Distance: {distance:.1f}mm), preparing to place!")
                
                # Construct and send place command to the arm
                cmd_msg = String()
                cmd_msg.data = f"place {msg.x:.1f} {msg.y:.1f} {msg.z:.1f}"
                self.pub.publish(cmd_msg)
                
                self.get_logger().info(f"Sent command: {cmd_msg.data}")
                
                # Action completed, reset state to empty, prepare for the next Cube
                self.is_holding = False
                self.held_colour = None
                
            else:
                # Color mismatch, belongs to another bin.
                # Keep silent, wait for the camera to scan the bin with the correct color.
                pass

def main(args=None):
    rclpy.init(args=args)
    node = NucController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()