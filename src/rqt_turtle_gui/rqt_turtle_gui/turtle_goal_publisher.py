import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel, QApplication
from geometry_msgs.msg import Point

class TurtleGoalPublisher(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'turtle_goal_publisher')
        QWidget.__init__(self)

        self.publisher_ = self.create_publisher(Point, '/goal_position', 10)

        # Create the GUI layout
        self.setWindowTitle("Turtle Goal Position")
        layout = QVBoxLayout()

        self.label = QLabel("Click the button to set a new goal")
        layout.addWidget(self.label)

        self.button = QPushButton("Set New Goal")
        self.button.clicked.connect(self.publish_goal)
        layout.addWidget(self.button)

        self.setLayout(layout)

    def publish_goal(self):
        # Hardcoded example goal (this should later be set dynamically)
        goal_msg = Point()
        goal_msg.x = 5.0
        goal_msg.y = 5.0

        self.publisher_.publish(goal_msg)
        self.label.setText(f"Goal Set: ({goal_msg.x}, {goal_msg.y})")

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    gui = TurtleGoalPublisher()
    gui.show()
    sys.exit(app.exec_())

