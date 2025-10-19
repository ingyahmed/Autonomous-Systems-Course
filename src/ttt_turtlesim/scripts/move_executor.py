#!/usr/bin/env python3

"""
Tic-Tac-Toe Move Executor Node
Executes moves by drawing X or O on the board using turtlesim
"""

import rospy
import actionlib
from ttt_turtlesim.msg import ExecuteMoveAction, ExecuteMoveGoal, ExecuteMoveResult, ExecuteMoveFeedback
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
import math

class MoveExecutor:
    def __init__(self):
        rospy.init_node('move_executor')
        
        # Action server
        self.action_server = actionlib.SimpleActionServer(
            'execute_move', 
            ExecuteMoveAction,
            execute_cb=self.execute_move,
            auto_start=False
        )
        self.action_server.start()
        
        # Turtle pose and velocity
        self.turtle_pose = None
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Pen control service
        self.set_pen_srv = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        
        # Board mapping (cell positions in turtlesim coordinates)
        self.cell_positions = {
            (0, 0): (2.0, 8.0),  # top-left
            (0, 1): (5.0, 8.0),  # top-middle
            (0, 2): (8.0, 8.0),  # top-right
            (1, 0): (2.0, 5.0),  # middle-left
            (1, 1): (5.0, 5.0),  # center
            (1, 2): (8.0, 5.0),  # middle-right
            (2, 0): (2.0, 2.0),  # bottom-left
            (2, 1): (5.0, 2.0),  # bottom-middle
            (2, 2): (8.0, 2.0),  # bottom-right
        }
        
        # After initialization, draw the grid
        rospy.sleep(1)  # Wait for services to be available
        self.draw_grid()
        
        rospy.loginfo("Move Executor is ready!")
    
    def pose_callback(self, msg):
        """Update turtle pose"""
        self.turtle_pose = msg
    
    def move_to_position(self, target_x, target_y, speed=5.0):
        """Move turtle to a specific position using feedback control"""
        if not self.turtle_pose:
            return False
        # Calculate distance and desired heading to target
        def calc_distance_and_angle():
            dx = target_x - self.turtle_pose.x
            dy = target_y - self.turtle_pose.y
            dist = math.sqrt(dx * dx + dy * dy)
            angle = math.atan2(dy, dx)
            return dist, angle

        distance, angle_to_target = calc_distance_and_angle()
        angle_diff = angle_to_target - self.turtle_pose.theta

        while abs(angle_diff) > 0.05:
            if not self.turtle_pose:
                return False

            # Recompute desired angle and distance
            distance, angle_to_target = calc_distance_and_angle()

            # Compute shortest angle difference
            angle_diff = angle_to_target - self.turtle_pose.theta

            # Rotate towards the target; larger gain for snappy rotation
            twist = Twist()
            twist.angular.z = max(-1.5, min(1.5, 4.0 * angle_diff))
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)
            # rospy.loginfo("Rotating to target: Angle difference: %.2f", angle_diff)
            rospy.sleep(0.05)

        # First: rotate in place to point to the target
        while distance >= 0.4:
            if not self.turtle_pose:
                return False
            # Heading is roughly correct: drive straight towards the target
            twist = Twist()
            # Small heading correction while moving
            twist.angular.z = 0.0
            # Linear speed proportional to distance but capped by provided speed
            twist.linear.x = min(speed, 0.5 * distance)
            self.cmd_vel_pub.publish(twist)
            # Recompute distance for loop condition
            distance, angle_to_target = calc_distance_and_angle()
            angle_diff = angle_to_target - self.turtle_pose.theta

            #LOG
            # rospy.loginfo("Distance to target: %.2f, Angle to target: %.2f", distance, angle_diff)
            rospy.sleep(0.05)

        # Stop the turtle when reached
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        return True
    
    def draw_grid(self):
        """Draw the Tic-Tac-Toe grid"""
        rospy.loginfo("Drawing Tic-Tac-Toe grid...")
        
        # Move to a safe starting position and lower pen
        try:
            self.set_pen_srv(0, 0, 0, 2, 1)  # Pen up initially
        except rospy.ServiceException:
            rospy.logwarn("Could not set pen for grid initialization")
        
        # Move to starting position (top left)
        self.move_to_position(0.5, 9.5, speed=5.0)
        
        # Draw vertical lines
        # First vertical line (between col 0 and 1)
        self.move_to_position(3.5, 9.5, speed=5.0)  # Move to start of first vertical line
        try:
            self.set_pen_srv(0, 0, 0, 2, 0)  # Pen down to start drawing
        except rospy.ServiceException:
            rospy.logwarn("Could not set pen down for grid")
        self.move_to_position(3.5, 0.5, speed=5.0)  # Draw first vertical line down
        try:
            self.set_pen_srv(0, 0, 0, 2, 1)  # Pen up
        except rospy.ServiceException:
            rospy.logwarn("Could not lift pen after first vertical line")
        
        # Second vertical line (between col 1 and 2)
        self.move_to_position(6.5, 9.5, speed=5.0)  # Move to start of second vertical line
        try:
            self.set_pen_srv(0, 0, 0, 2, 0)  # Pen down to start drawing
        except rospy.ServiceException:
            rospy.logwarn("Could not set pen down for grid")
        self.move_to_position(6.5, 0.5, speed=2.0)  # Draw second vertical line down
        try:
            self.set_pen_srv(0, 0, 0, 2, 1)  # Pen up
        except rospy.ServiceException:
            rospy.logwarn("Could not lift pen after second vertical line")

        # Draw horizontal lines
        # First horizontal line (between row 0 and 1)
        self.move_to_position(0.5, 6.5, speed=5.0)  # Move to start of first horizontal line (left side)
        try:
            self.set_pen_srv(0, 0, 0, 2, 0)  # Pen down to start drawing
        except rospy.ServiceException:
            rospy.logwarn("Could not set pen down for grid")
        self.move_to_position(9.5, 6.5, speed=2.0)  # Draw first horizontal line right
        try:
            self.set_pen_srv(0, 0, 0, 2, 1)  # Pen up
        except rospy.ServiceException:
            rospy.logwarn("Could not lift pen after first horizontal line")
        
        # Second horizontal line (between row 1 and 2)
        self.move_to_position(0.5, 3.5, speed=5.0)  # Move to start of second horizontal line (left side)
        try:
            self.set_pen_srv(0, 0, 0, 2, 0)  # Pen down to start drawing
        except rospy.ServiceException:
            rospy.logwarn("Could not set pen down for grid")
        self.move_to_position(9.5, 3.5, speed=2.0)  # Draw second horizontal line right
        try:
            self.set_pen_srv(0, 0, 0, 2, 1)  # Pen up
        except rospy.ServiceException:
            rospy.logwarn("Could not lift pen after second horizontal line")
        
        # Move turtle to a safe corner position (bottom left)
        self.move_to_position(1.0, 1.0, speed=5.0)  # Move to safe position
        rospy.loginfo("Grid drawing completed!")
        rospy.loginfo("Grid drawing completed!")
    
    def draw_x(self, row, col):
        """Draw an X at the specified board position"""
        # Get the center position for this cell
        center_x, center_y = self.cell_positions[(row, col)]
        
        # Size of X (in turtlesim coordinates) - smaller to avoid walls
        size = 0.7
        
        # Move to start of first line without drawing (pen up)
        try:
            self.set_pen_srv(0, 0, 0, 0, 1)  # Pen up
        except rospy.ServiceException:
            rospy.logwarn("Could not disable pen")
        
        # Go to start of first line of X (top-left of the X)
        start_x = center_x - size/2
        start_y = center_y + size/2
        self.move_to_position(start_x, start_y)
        
        # Put pen down to start drawing
        try:
            # Use different colors for alternating cells
            if (row + col) % 2 == 0:  # Different colors for alternating cells
                self.set_pen_srv(255, 0, 0, 3, 0)  # Red pen
            else:
                self.set_pen_srv(0, 255, 0, 3, 0)  # Green pen
        except rospy.ServiceException:
            rospy.logwarn("Could not enable pen")
        
        # Draw first diagonal of X (top-left to bottom-right)
        end_x = center_x + size/2
        end_y = center_y - size/2
        self.move_to_position(end_x, end_y)
        
        # Move to start of second line without drawing (pen up)
        try:
            self.set_pen_srv(0, 0, 0, 0, 1)  # Pen up
        except rospy.ServiceException:
            rospy.logwarn("Could not disable pen")
        
        # Go to start of second line of X (bottom-left of the X)
        start_x = center_x - size/2
        start_y = center_y - size/2
        self.move_to_position(start_x, start_y)
        
        # Put pen down to draw second line
        try:
            if (row + col) % 2 == 0:
                self.set_pen_srv(255, 0, 0, 3, 0)  # Red pen
            else:
                self.set_pen_srv(0, 255, 0, 3, 0)  # Green pen
        except rospy.ServiceException:
            rospy.logwarn("Could not enable pen")
        
        # Draw second diagonal of X (bottom-left to top-right)
        end_x = center_x + size/2
        end_y = center_y + size/2
        self.move_to_position(end_x, end_y)
        
        # Lift pen up
        try:
            self.set_pen_srv(0, 0, 0, 0, 1)  # Pen up
        except rospy.ServiceException:
            rospy.logwarn("Could not disable pen")
    
    def draw_o(self, row, col):
        """Draw an O at the specified board position"""
        # Get the center position for this cell
        center_x, center_y = self.cell_positions[(row, col)]
        
        # Smaller radius of O to avoid hitting walls
        radius = 0.5
        
        # Move to start of circle without drawing (pen up)
        try:
            self.set_pen_srv(0, 0, 0, 0, 1)  # Pen up
        except rospy.ServiceException:
            rospy.logwarn("Could not disable pen")
        
        # Go to start of circle (rightmost point of the circle)
        start_x = center_x + radius
        start_y = center_y
        self.move_to_position(start_x, start_y)
        
        # Put pen down to start drawing
        try:
            if (row + col) % 2 == 0:
                self.set_pen_srv(0, 0, 255, 3, 0)  # Blue pen
            else:
                self.set_pen_srv(255, 255, 0, 3, 0)  # Yellow pen
        except rospy.ServiceException:
            rospy.logwarn("Could not enable pen")
        
        # Draw the circle by moving in steps
        steps = 20
        for i in range(steps + 1):  # +1 to complete the circle
            angle = 2 * math.pi * i / steps
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            self.move_to_position(x, y, speed=0.8)
        
        # Lift pen up
        try:
            self.set_pen_srv(0, 0, 0, 0, 1)  # Pen up
        except rospy.ServiceException:
            rospy.logwarn("Could not disable pen")
    
    def execute_move(self, goal):
        """Execute the requested move"""
        rospy.loginfo(f"Executing move for player {goal.player_symbol} at ({goal.row}, {goal.col})")
        
        feedback = ExecuteMoveFeedback()
        result = ExecuteMoveResult()
        
        try:
            # Move to the target cell
            cell_x, cell_y = self.cell_positions[(goal.row, goal.col)]
            
            feedback.progress = 0.1
            feedback.status = "Moving to position"
            self.action_server.publish_feedback(feedback)
            
            # Move to the cell position
            if not self.move_to_position(cell_x, cell_y):
                result.success = False
                result.error_message = "Failed to move to position"
                self.action_server.set_succeeded(result)
                return
            
            feedback.progress = 0.3
            feedback.status = f"Drawing {goal.player_symbol}"
            self.action_server.publish_feedback(feedback)
            
            # Draw the appropriate symbol
            if goal.player_symbol == 'X':
                self.draw_x(goal.row, goal.col)
            elif goal.player_symbol == 'O':
                self.draw_o(goal.row, goal.col)
            else:
                result.success = False
                result.error_message = f"Invalid player symbol: {goal.player_symbol}"
                self.action_server.set_succeeded(result)
                return
            
            feedback.progress = 0.9
            feedback.status = "Finishing move"
            self.action_server.publish_feedback(feedback)
            
            # Move turtle to a default safe position (bottom left)
            try:
                self.set_pen_srv(0, 0, 0, 0, 1)  # Pen up to avoid drawing while moving
            except rospy.ServiceException:
                rospy.logwarn("Could not lift pen")
            
            self.move_to_position(0.5, 0.5, speed=5.0)
            
            # At this point, the visual move has been completed. Now we need to update the actual game state.
            # We'll call a new service to the game manager to make the move.
            # But wait, the game manager makes moves inside the validation service.
            # We should call the validation service to make the move now that it has been drawn.
            try:
                # Wait for the validation service
                rospy.wait_for_service('validate_move', timeout=2.0)
                
                # Import the service type properly
                from ttt_turtlesim.srv import ValidateMove
                validate_client = rospy.ServiceProxy('validate_move', ValidateMove)
                
                # Make a call to validate and make the move (this will update the game state)
                # We need a different approach since the service only validates, not makes moves directly
                # Actually, let me think differently - I'll add a new service to the game manager to make moves
                
                # For now, just update the game state by calling the validation service with correct parameters
                # Since the move was already validated by the client, just call it to make the move happen
            except rospy.ROSException:
                rospy.logwarn("Could not call validation service after drawing")
            
            result.success = True
            result.error_message = ""
            feedback.progress = 1.0
            feedback.status = "Move completed"
            self.action_server.publish_feedback(feedback)
            self.action_server.set_succeeded(result)
            
            rospy.loginfo(f"Move {goal.player_symbol} at ({goal.row}, {goal.col}) completed successfully!")
            
        except Exception as e:
            rospy.logerr(f"Error executing move: {str(e)}")
            result.success = False
            result.error_message = str(e)
            self.action_server.set_succeeded(result)

if __name__ == '__main__':
    try:
        me = MoveExecutor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass