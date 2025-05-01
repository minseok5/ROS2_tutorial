# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from crazyflie_py import Crazyswarm

# import numpy as np
# import time

# # Global variables to store goal data
# current_goal = {
#     'x': 0.0,
#     'y': 0.0,
#     'z': 0.0,
#     'yaw': 0.0
# }

# class GoalSubscriber(Node):
#     def __init__(self):
#         super().__init__('goal_subscriber')
#         self.subscription = self.create_subscription(PoseStamped, 'goal', self.goal_callback, 10)
#         self.get_logger().info('Goal subscriber initialized')
#         self.goal_received = False  # Flag to track if we've received a goal
        
#         # Initialize Crazyflie without reinitializing ROS2
#         try:
#             # Create a new process for Crazyswarm
#             import subprocess
#             self.cf_process = subprocess.Popen(['python3', '-c', '''
# import sys
# sys.path.append('/home/minseok/ros2_ws/src/cf_move/cf_move')
# from crazyflie_py import Crazyswarm
# swarm = Crazyswarm()
# cf = swarm.allcfs.crazyflies[0]
# timeHelper = swarm.timeHelper
#             '''])
#             self.get_logger().info('Crazyswarm initialized in separate process')
#         except Exception as e:
#             self.get_logger().error(f'Failed to initialize Crazyswarm: {str(e)}')
#             self.cf_process = None

#     def goal_callback(self, msg):
#         if not self.goal_received:  # Only process the first goal
#             self.x = msg.pose.position.x
#             self.y = msg.pose.position.y
#             self.z = msg.pose.position.z
#             self.yaw = msg.pose.orientation.w

#             self.get_logger().info(f'Goal received: x={self.x}, y={self.y}, z={self.z}, yaw={self.yaw}')
            
#             # Execute move_to_goal
#             self.move_to_goal(self.x, self.y, self.z, self.yaw)
#             self.goal_received = True  # Mark that we've processed a goal

#     def move_to_goal(self, x, y, z, yaw):
#         try:
#             if self.cf_process is None:
#                 self.get_logger().error('Crazyflie not initialized')
#                 return

#             duration = 1.5
#             # Create a new process for the movement sequence
#             import subprocess
#             movement_script = f'''
# import sys
# sys.path.append('/home/minseok/ros2_ws/src/cf_move/cf_move')
# from crazyflie_py import Crazyswarm
# import time

# swarm = Crazyswarm()
# cf = swarm.allcfs.crazyflies[0]
# timeHelper = swarm.timeHelper

# setpoint = 0

# try:

#     if setpoint == 0:
#         # Takeoff
#         print("takeoff")
#         cf.takeoff({z}, {duration})
#         timeHelper.sleep({duration})
#         print("takeoff complete")
#         setpoint = 2

#     # if setpoint == 1:
#     #     # Hover
#     #     print("hover")
#     #     cf.hover({duration})
#     #     timeHelper.sleep({duration})
#     #     print("hover complete")
#     #     setpoint = 2

#     if setpoint == 2:
#         # Go to goal
#         print("go to")
#         cf.goTo({x}, {y}, {z}, {yaw}, {duration}, groupMask=1)
#         timeHelper.sleep({duration})
#         print("go to complete")
#         setpoint = 3

#     if setpoint == 3:
#         # Land
#         print("land")
#         cf.land(0.04,{duration})
#         timeHelper.sleep({duration})
#         print("land complete")
#         setpoint = 0

# finally:
#     # Ensure we stop the drone
#     print("stopping drone")
#     cf.stop()
# '''
#             subprocess.run(['python3', '-c', movement_script])
            
#         except Exception as e:
#             self.get_logger().error(f'Error in move_to_goal: {str(e)}')

#     def __del__(self):
#         # Cleanup when the node is destroyed
#         if self.cf_process is not None:
#             self.cf_process.terminate()

# def main(args=None):
#     rclpy.init()
#     node = GoalSubscriber()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_py import Crazyswarm
import multiprocessing
import time

class ROS2Listener(Node):
    """📡 C++에서 목표 고도(PoseStamped)를 받아오는 ROS2 노드"""
    def __init__(self, target_z_shared):
        super().__init__('goal_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped, '/crazyflie/pose', self.pose_callback, 10)
        self.target_z = target_z_shared  # ✅ 공유 메모리 사용
        self.target_z.value = -99.0  # ✅ 초기값을 -99.0으로 설정 (아직 명령 없음)
        self.get_logger().info("🚀 ROS2 Listener Initialized - Waiting for PoseStamped")

    def pose_callback(self, msg):
        """📡 C++에서 목표 고도를 받아서 공유 메모리에 저장"""
        self.target_z.value = msg.pose.position.z
        self.get_logger().info(f"📡 Received Target Altitude: {self.target_z.value:.2f}m")

def run_ros2(target_z_shared):
    """🔥 ROS 2 노드를 실행하는 프로세스"""
    rclpy.init()
    node = ROS2Listener(target_z_shared)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def run_crazyswarm(target_z_shared):
    """🔥 CrazySwarm을 실행하는 프로세스"""
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]  # ✅ 첫 번째 드론 선택

    print("🚀 Crazyflie Ready for Commands!")

    prev_target_z = -999  # ✅ 초기값 설정

    while True:
        target_z = target_z_shared.value  # ✅ 공유 메모리에서 목표 고도 가져오기

        # ✅ ROS 2에서 처음 명령을 받을 때까지 대기
        if target_z == -99.0:
            print("⏳ Waiting for first PoseStamped message...")
            timeHelper.sleep(0.5)
            continue

        # ✅ 목표값이 변할 때만 동작하도록 함
        if target_z != prev_target_z:
            prev_target_z = target_z  # ✅ 이전 값 업데이트
            print(f"📡 New Target Altitude: {target_z:.2f}m")

            if target_z > 0.9:
                print("🛸 Takeoff Initiated...")
                cf.takeoff(targetHeight=target_z, duration=2.5)
                timeHelper.sleep(3.0)
                
                print("✅ Hovering at Target Altitude for 5 seconds...")
                timeHelper.sleep(5.0)  # ✅ 5초간 호버링

                cf.goTo([1.0,1.0,1.0],0.0,2.0)
                timeHelper.sleep(2.5)

                print("🛬 Auto Landing Initiated...")
                cf.land(targetHeight=0.05, duration=2.5)
                timeHelper.sleep(3.0)
                print("✅ Landed Successfully!")

            elif target_z <= 0.1 and target_z >= 0.0:
                print("🛬 Manual Landing Initiated...")
                cf.land(targetHeight=0.05, duration=2.5)
                timeHelper.sleep(3.0)
                print("✅ Landed Successfully!")

            elif target_z == -1.0:  # ✅ 명확한 착륙 명령
                print("❗ Emergency Landing Requested...")
                cf.land(targetHeight=0.05, duration=2.5)
                timeHelper.sleep(3.0)
                print("🛑 Emergency Landing Complete. Exiting...")
                break  # ✅ 프로세스 종료

        timeHelper.sleep(0.5)  # ✅ 0.5초마다 목표값 확인

def main():
    """🔥 `multiprocessing`으로 ROS 2와 CrazySwarm을 동시에 실행"""
    target_z_shared = multiprocessing.Value('d', -99.0)  # ✅ 처음에는 명령이 없음을 의미하는 값 사용

    ros2_process = multiprocessing.Process(target=run_ros2, args=(target_z_shared,))
    crazyswarm_process = multiprocessing.Process(target=run_crazyswarm, args=(target_z_shared,))

    ros2_process.start()
    crazyswarm_process.start()

    ros2_process.join()
    crazyswarm_process.join()

if __name__ == '__main__':
    main()
