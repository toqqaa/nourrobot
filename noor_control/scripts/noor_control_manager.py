#!/usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus

class RobotControlManager:
    def __init__(self):
        rospy.init_node('robot_control_manager')
        
        # Action client for move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        
        # Subscriber for control commands
        self.control_sub = rospy.Subscriber('/ai/robot_control', String, self.control_callback)
        
        # Current goal storage
        self.current_goal = None
        self.last_goal_before_stop = None
        self.is_stopped = False
        
        # For receiving new navigation goals
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)
        
    def goal_callback(self, msg):
        """Store and send new goals that come in"""
        rospy.loginfo("Received new goal")
        self.current_goal = msg
        if not self.is_stopped:
            self.send_goal(msg)
        
    def control_callback(self, msg):
        """Handle stop/continue commands"""
        command = msg.data.lower().strip()
        
        if command == "stop":
            rospy.loginfo("Received STOP command")
            if self.current_goal and not self.is_stopped:
                self.last_goal_before_stop = self.current_goal
                self.safe_cancel_goal()
                self.is_stopped = True
                rospy.loginfo("Goal cancelled and robot stopped")
                
        elif command == "continue":
            rospy.loginfo("Received CONTINUE command")
            if self.is_stopped and self.last_goal_before_stop:
                self.current_goal = self.last_goal_before_stop
                self.send_goal(self.current_goal)
                self.is_stopped = False
                rospy.loginfo("Resuming previous goal")
                
    def safe_cancel_goal(self):
        """Safely cancel the current goal with state checking"""
        try:
            state = self.move_base_client.get_state()
            if state in [GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING]:
                self.move_base_client.cancel_goal()
                rospy.loginfo("Goal cancellation sent")
                # Wait for confirmation of cancellation
                rospy.sleep(0.5)
            else:
                rospy.logwarn("Cannot cancel goal in current state: %s", GoalStatus.to_string(state))
        except Exception as e:
            rospy.logerr("Error during goal cancellation: %s", str(e))
        
    def send_goal(self, goal_msg):
        """Send a goal to move_base"""
        goal = MoveBaseGoal()
        goal.target_pose = goal_msg
        self.move_base_client.send_goal(goal)
        rospy.loginfo(f"New goal sent to move_base: {goal_msg.pose.position}")
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        manager = RobotControlManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass