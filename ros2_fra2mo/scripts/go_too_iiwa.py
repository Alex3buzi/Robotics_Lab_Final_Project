#! /usr/bin/env python3

import os
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration


def main():
    rclpy.init()

    handover_node = rclpy.create_node('handover_publisher')
    pub_handover = handover_node.create_publisher(Bool, '/fra2mo/ready_for_handover', 10)
    
    navigator = BasicNavigator()

    config_path = os.path.join(os.getenv('HOME'), 'ros2_ws/src/ros2_fra2mo/config/iiwa_goal.yaml')
    
    target_x = 0.0
    target_y = 0.0
    
    if os.path.exists(config_path):
        print(f"Found configuration file: {config_path}")
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
            target_x = data['iiwa_goal']['x']
            target_y = data['iiwa_goal']['y']
            print(f"Target loaded -> X: {target_x}, Y: {target_y}")
    else:
        print(f"ERROR: File {config_path} not found! Using default coordinates (0,0).")


    print("waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active() 

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    #Offset
    offset_x = -0.8
    offset_y = 0.1

    final_x = target_x + offset_x
    final_y = target_y + offset_y

    print(f"\n[INFO] Applying offset (x={offset_x}, y={offset_y}) to approach iiwa safely for package pickup, ensuring no collision occurs.")
    print(f"[INFO] Original Target: ({target_x}, {target_y}) -> Adjusted Goal: ({final_x}, {final_y})\n")
    

    goal_pose.pose.position.x = final_x
    goal_pose.pose.position.y = final_y

    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0


    print(f"Sending robot to: {target_x}, {target_y}...")
    navigator.goToPose(goal_pose)

    i = 0
    goal_reached = False
    GOAL_TOLERANCE = 0.15
    while not navigator.isTaskComplete():
        
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback:
            distance_remaining = feedback.distance_remaining
            
            if distance_remaining < GOAL_TOLERANCE:
                print(f'\n[INFO] Target close enough ({distance_remaining:.2f}m < {GOAL_TOLERANCE}m). Stopping manually.')
                navigator.cancelTask()
                goal_reached = True
                break

            if i % 5 == 0:
                print(f'Distance remaining: {distance_remaining:.2f}m | ETA: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.0f}s')

            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600):
                navigator.cancelTask()
                
    if goal_reached:
        print("\n[INFO] Preparing to send handover signal to iiwa...")
        msg = Bool()
        msg.data = True
        
        # Pubblichiamo in un piccolo loop per assicurarci che il messaggio venga inviato e processato
        for _ in range(5):
            pub_handover.publish(msg)
            rclpy.spin_once(handover_node, timeout_sec=0.1)
        
        print("[INFO] SIGNAL SENT: /fra2mo/ready_for_handover = True")
    
    result = navigator.getResult()
    
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled (likely reached tolerance)! SUCCESS.')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    handover_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
