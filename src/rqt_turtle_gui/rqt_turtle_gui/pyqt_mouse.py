import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QPainter, QPen, QMouseEvent
from PyQt5.QtCore import Qt, QPoint

class MousePublisher(Node):
    """ROS2 Node to publish mouse position""" 
    def __init__(self):
        super.__init__(self, "mouse_goal_publisher")
        self.publisher = self.create_publisher(Point, "/turtle1/goal_position", 10)
    
    def publish_goal(self, x_m, y_m):
        msg = Point()
        msg.x = x_m
        msg.y = y_m
        msg.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: ({x_m:.2f}, {y_m:.2f})")


class DrawingBoard(QWidget):
    """PyQt Window to capture mouse movements"""
    def __init__(self, ros_node):

        super().__init__()
        self.ros_node = ros_node

        # super().__init__()
        self.setWindowTitle("PyQt 그림판 - 마우스 위치 출력")
        self.W_px = 550
        self.H_px = 550
        self.W_m = 11
        self.H_m = 11
        self.setGeometry(100, 100, self.W_px, self.H_px)
        self.setMouseTracking(True)  # 클릭 없이도 마우스 이동 감지
        self.drawing = False
        self.last_point = None
        # self.pen_color = Qt.black  # 기본 펜 색상

    def mouseMoveEvent(self, event: QMouseEvent):
        """마우스가 움직일 때 좌표 출력"""

        x_px = event.x()
        y_px = event.y()

        x_m = (x_px / self.W_px) * self.W_m
        y_m = (1 - (y_px / self.H_px)) * self.H_m

        print(f"마우스 위치: ({x_m:.2f}, {y_m:.2f})")
        if self.drawing and self.last_point:
            self.update()
        self.last_point = event.pos()

        self.ros_node.publish_goal(x_m, y_m)


    # def mousePressEvent(self, event: QMouseEvent):
    #     """마우스를 클릭하면 그림 그리기 시작"""
    #     if event.button() == Qt.LeftButton:
    #         self.drawing = True
    #         self.last_point = event.pos()

    # def mouseReleaseEvent(self, event: QMouseEvent):
    #     """마우스 클릭 해제 시 그림 그리기 종료"""
    #     if event.button() == Qt.LeftButton:
    #         self.drawing = False
    #         self.last_point = None

    # def paintEvent(self, event):
    #     """그림을 그리는 이벤트"""
    #     painter = QPainter(self)
    #     pen = QPen(self.pen_color, 2, Qt.SolidLine)
    #     painter.setPen(pen)

    #     if self.last_point:
    #         painter.drawPoint(self.last_point)

def main():
    rclpy.init()

    ros_node = MousePublisher()
    app = QApplication(sys.argv)
    window = DrawingBoard(ros_node)
    window.show()

    try:
        rclpy.spin(window)
    except KeyboardInterrupt:    
        window.destroy()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
        sys.exit(app.exec_())



if __name__ == "__main__":
    main()