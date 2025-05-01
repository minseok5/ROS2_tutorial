import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_py import Crazyswarm
import multiprocessing
import time
import math



class ROS2Listener(Node):
    """📡 C++에서 목표 고도(PoseStamped)를 받아오는 ROS2 노드"""
    def __init__(self, target_shared):
        super().__init__('ros2_listener')
        self.subscription = self.create_subscription(
            PoseStamped, 'goal', self.pose_callback, 10)
        self.target = target_shared  # ✅ 공유 메모리 사용
        # self.target[2] = -99.0  # ✅ 초기값을 -99.0으로 설정 (아직 명령 없음)
        self.get_logger().info("🚀 ROS2 Listener Initialized - Waiting for PoseStamped")

    def pose_callback(self, msg):
        """📡 C++에서 목표 고도를 받아서 공유 메모리에 저장"""
        self.target[0] = msg.pose.position.x
        self.target[1] = msg.pose.position.y
        self.target[2] = msg.pose.position.z
        self.target[3] = msg.pose.orientation.x
        self.target[4] = msg.pose.orientation.y
        self.target[5] = msg.pose.orientation.z
        self.target[6] = msg.pose.orientation.w
        self.get_logger().info(f"📡 Received Target: {self.target[0]:.2f}m, {self.target[1]:.2f}m, {self.target[2]:.2f}m")

def run_ros2(target_shared):
    """🔥 ROS 2 노드를 실행하는 프로세스"""
    rclpy.init()
    node = ROS2Listener(target_shared)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def run_crazyswarm(target_shared):
    """🔥 CrazySwarm을 실행하는 프로세스"""
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]  # ✅ 첫 번째 드론 선택

    print("🚀 Crazyflie Ready for Commands!")

    # Initial takeoff to 1m
    print("🛸 Initial Takeoff to 1.0m...")
    cf.takeoff(targetHeight=1.0, duration=2.5)
    timeHelper.sleep(3.0)
    print("✅ Hovering at 1.0m...")

    # Wait for first command
    while target_shared[2] == -99.0:
        print("⏳ Waiting for first PoseStamped message...")
        timeHelper.sleep(0.5)
    print("✅ Received first command!")

    # Start time for mission duration
    mission_start_time = time.time()
    mission_duration = 30.0  # Mission duration in seconds (adjust as needed)
    is_moving = False

    while True:
        target_x = target_shared[0] 
        target_y = target_shared[1]
        target_z = target_shared[2]  # ✅ 공유 메모리에서 목표 고도 가져오기
        target_qx = target_shared[3]
        target_qy = target_shared[4]
        target_qz = target_shared[5]
        target_qw = target_shared[6]
        yaw = math.atan2(2 * (target_qw * target_qz + target_qx * target_qy), 1 - 2 * (target_qy * target_qy + target_qz * target_qz))
        
        
        # Continuous movement to target position
        if not is_moving:
            print("🔄 Starting continuous movement...")
            is_moving = True
    

        # Move to target position with yaw
        cf.goTo([target_x, target_y, target_z], yaw, 1.0)
        timeHelper.sleep(0.1)  # Small delay to prevent overwhelming the drone
        print(f"📡 New Target: {target_x:.2f}m, {target_y:.2f}m, {target_z:.2f}m, {yaw:.2f}rad")

        # Check for landing conditions
        if target_z <= 0.1 and target_z >= 0.0:
            print("🛬 Manual Landing Initiated...")
            cf.land(targetHeight=0.05, duration=2.5)
            timeHelper.sleep(3.0)
            print("✅ Landed Successfully!")
            break

        elif target_z == -1.0:  # Emergency landing
            print("❗ Emergency Landing Requested...")
            cf.land(targetHeight=0.05, duration=2.5)
            timeHelper.sleep(3.0)
            print("🛑 Emergency Landing Complete. Exiting...")
            break

        # Check mission duration
        current_time = time.time()
        if current_time - mission_start_time > mission_duration:
            print("⏰ Mission duration reached. Initiating landing...")
            cf.land(targetHeight=0.05, duration=2.5)
            timeHelper.sleep(3.0)
            print("✅ Mission completed. Landed successfully!")
            break

       
    print("Mission completed!")

def main():
    """🔥 `multiprocessing`으로 ROS 2와 CrazySwarm을 동시에 실행"""
    target_shared = multiprocessing.Array('d', [0.0, 0.0, -99.0, 0.0, 0.0, 0.0, 0.0])  # ✅ 처음에는 명령이 없음을 의미하는 값 사용

    ros2_process = multiprocessing.Process(target=run_ros2, args=(target_shared,))
    crazyswarm_process = multiprocessing.Process(target=run_crazyswarm, args=(target_shared,))

    ros2_process.start()
    crazyswarm_process.start()

    ros2_process.join()
    crazyswarm_process.join()

if __name__ == '__main__':
    main()
