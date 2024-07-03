import sys
import math

import rospy
import sensor_msgs.msg._Joy
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout, QPushButton
from PyQt5.QtWidgets import QLabel
from PyQt5.QtGui import QPainter, QBrush, QPen
from PyQt5.QtCore import Qt


class Joystick(QWidget):
    def __init__(self, name = 'Left'):
        super().__init__()

        self.window_title = 'Joystick'
        self.window_min_size = [200, 200]
        self.wnd_fit_size = 400
        self.window_size = [self.wnd_fit_size, self.wnd_fit_size]

        self.circle_margin_ratio = 0.1
        self.circle_diameter = int(self.window_size[0] * (1 - self.circle_margin_ratio * 2))

        self.stick_diameter_ratio = 0.1
        self.stick_diameter = int(self.circle_diameter * self.stick_diameter_ratio)
        self.is_mouse_down = False
        self.stick_pos = [0, 0]
        self.strength = 0

        self.stat_label_margin = 10
        self.stat_label = QLabel(self)

        self.control_name = name
        self.joy_pub = rospy.Publisher('joy', sensor_msgs.msg.Joy, queue_size=1)
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle(self.window_title)

        self.setMinimumSize(self.window_min_size[0], self.window_min_size[1])
        self.resize(self.window_size[0], self.window_size[1])

        self.stat_label.setAlignment(Qt.AlignLeft)
        self.stat_label.setGeometry(self.stat_label_margin, self.stat_label_margin,
                                    self.window_min_size[0] - self.stat_label_margin * 2,
                                    self.window_min_size[0] - self.stat_label_margin * 2)
        font = self.stat_label.font()
        font.setPointSize(10)

        self.setMouseTracking(True)

    def resizeEvent(self, event):
        self.wnd_fit_size = min(self.width(), self.height())

        self.circle_diameter = int(self.wnd_fit_size * (1 - self.circle_margin_ratio * 2))
        self.stick_diameter = int(self.circle_diameter * self.stick_diameter_ratio)

    def _draw_outer_circle(self, painter):
        painter.setPen(QPen(Qt.black, 2, Qt.SolidLine))

        circle_margin = int(self.wnd_fit_size * self.circle_margin_ratio)
        painter.drawEllipse(circle_margin, circle_margin,
                            self.circle_diameter, self.circle_diameter)

    def _draw_sub_lines(self, painter):
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QPen(Qt.lightGray, 1, Qt.DashLine))

        num_sub_line = 6
        for i in range(num_sub_line):
            theta = math.pi / 2 - math.pi * i / num_sub_line
            x0 = int(self.wnd_fit_size / 2 - self.circle_diameter / 2 * math.cos(theta))
            y0 = int(self.wnd_fit_size / 2 - self.circle_diameter / 2 * math.sin(theta))
            x1 = int(self.wnd_fit_size / 2 - self.circle_diameter / 2 * math.cos(theta + math.pi))
            y1 = int(self.wnd_fit_size / 2 - self.circle_diameter / 2 * math.sin(theta + math.pi))
            painter.drawLine(x0, y0, x1, y1)

    def _draw_sub_circles(self, painter):
        painter.setPen(QPen(Qt.lightGray, 1, Qt.DashLine))

        num_sub_circle = 4
        for i in range(num_sub_circle):
            sub_radius = int(self.circle_diameter / 2 * (i + 1) / (num_sub_circle + 1))
            sub_margin = int(self.wnd_fit_size / 2 - sub_radius)
            painter.drawEllipse(sub_margin, sub_margin, sub_radius * 2, sub_radius * 2)

        # Draw Inner(Joystick) Circle
        painter.setBrush(QBrush(Qt.black, Qt.SolidPattern))
        stick_margin = [int(self.wnd_fit_size / 2 + self.stick_pos[0] - self.stick_diameter / 2),
                        int(self.wnd_fit_size / 2 - self.stick_pos[1] - self.stick_diameter / 2)]
        painter.drawEllipse(stick_margin[0], stick_margin[1], self.stick_diameter, self.stick_diameter)

    def paintEvent(self, event):
        painter = QPainter(self)

        # Draw Outer(Main) Circle
        self._draw_outer_circle(painter)

        # Draw Sub Lines
        self._draw_sub_lines(painter)

        # Draw Sub Circles
        self._draw_sub_circles(painter)

        # Change Status Label Text (Angle In Degree)
        strength = self.get_strength()
        angle = self.get_angle(in_deg=True)
        if angle < 0:
            angle += 360
        self.stat_label.setText('Strength : {:.2f} \nDirection : {:.2f}°'.format(strength, angle))

    def mouseMoveEvent(self, event):
        # Move Stick Only When Mouse Left Button Pressed
        if self.is_mouse_down is False:
            return

        # Window Coordinate To Cartesian Coordinate
        pos = event.pos()
        stick_pos_buf = [pos.x() - self.wnd_fit_size / 2, self.wnd_fit_size / 2 - pos.y()]

        # If Cursor Is Not In Available Range, Correct It
        if self._get_strength(stick_pos_buf) > 1.0:
            theta = math.atan2(stick_pos_buf[1], stick_pos_buf[0])
            radius = (self.circle_diameter - self.stick_diameter) / 2
            stick_pos_buf[0] = radius * math.cos(theta)
            stick_pos_buf[1] = radius * math.sin(theta)

        self.stick_pos = stick_pos_buf
        self.repaint()

        max_distance = (self.circle_diameter - self.stick_diameter) / 2
        self.strength = self.get_strength()
        self.angle = self.get_angle(in_deg=False)

        if self.control_name == 'Left':
            joy_msg = sensor_msgs.msg.Joy()
            joy_msg.header.stamp = rospy.Time.now()
            joy_msg.header.frame_id = 'joy'
            joy_msg.axes = [self.stick_pos[0]/ max_distance, self.stick_pos[1]/max_distance, 0, 0, 0, 0, 0, 0]
            joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        else:
            joy_msg = sensor_msgs.msg.Joy()
            joy_msg.header.stamp = rospy.Time.now()
            joy_msg.header.frame_id = 'joy'
            joy_msg.axes = [0, 0, 0, self.stick_pos[0]/ max_distance, self.stick_pos[1]/max_distance, 0, 0, 0]
            joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.joy_pub.publish(joy_msg)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_mouse_down = True

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_mouse_down = False
            self.stick_pos = [0, 0]
            if self.control_name == 'Left':
                joy_msg = sensor_msgs.msg.Joy()
                joy_msg.header.stamp = rospy.Time.now()
                joy_msg.header.frame_id = 'joy'
                joy_msg.axes = [0, 0, 0, 0, 0, 0, 0, 0]
                joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                self.joy_pub.publish(joy_msg)
                print('Left Stick Released')

            else:
                joy_msg = sensor_msgs.msg.Joy()
                joy_msg.header.stamp = rospy.Time.now()
                joy_msg.header.frame_id = 'joy'
                joy_msg.axes = [0, 0, 0, 0, 0, 0, 0, 0]
                joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                self.joy_pub.publish(joy_msg)
                print('Right Stick Released')
            self.repaint()

    # Get Strength With Argument
    def _get_strength(self, stick_pos):
        max_distance = (self.circle_diameter - self.stick_diameter) / 2
        distance = math.sqrt(stick_pos[0] * stick_pos[0] + stick_pos[1] * stick_pos[1])

        return distance / max_distance

    # Get Strength With Current Stick Position
    def get_strength(self):
        max_distance = (self.circle_diameter - self.stick_diameter) / 2
        distance = math.sqrt(self.stick_pos[0] * self.stick_pos[0] + self.stick_pos[1] * self.stick_pos[1])

        return distance / max_distance

    def get_angle(self, in_deg=False):
        angle = math.atan2(self.stick_pos[1], self.stick_pos[0])
        if in_deg is True:
            angle = angle * 180 / math.pi

        return angle


class JoyButton(QWidget):
    def __init__(self):
        super().__init__()

        self.button_layout = QVBoxLayout()

        self.button1 = QPushButton('A', self)
        self.button2 = QPushButton('B', self)
        self.button3 = QPushButton('X', self)
        self.button4 = QPushButton('Y', self)

        self.button_msg = None
        self.botton_pub = rospy.Publisher('joy', sensor_msgs.msg.Joy, queue_size=1)

        self.button1.clicked.connect(self.on_button1_clicked)
        self.button2.clicked.connect(self.on_button2_clicked)
        self.button3.clicked.connect(self.on_button3_clicked)
        self.button4.clicked.connect(self.on_button4_clicked)

        self.button_layout.addWidget(self.button1)
        self.button_layout.addWidget(self.button2)
        self.button_layout.addWidget(self.button3)
        self.button_layout.addWidget(self.button4)

        self.setLayout(self.button_layout)
    
    def on_button1_clicked(self):
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.header.stamp = rospy.Time.now()
        joy_msg.header.frame_id = 'joy'
        joy_msg.axes = [0, 0, 0, 0, 0, 0, 0, 0]
        joy_msg.buttons = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.botton_pub.publish(joy_msg)
        print('Button A Clicked')
    
    def on_button2_clicked(self):
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.header.stamp = rospy.Time.now()
        joy_msg.header.frame_id = 'joy'
        joy_msg.axes = [0, 0, 0, 0, 0, 0, 0, 0]
        joy_msg.buttons = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.botton_pub.publish(joy_msg)
        print('Button B Clicked')
    
    def on_button3_clicked(self):
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.header.stamp = rospy.Time.now()
        joy_msg.header.frame_id = 'joy'
        joy_msg.axes = [0, 0, 0, 0, 0, 0, 0, 0]
        joy_msg.buttons = [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]

        self.botton_pub.publish(joy_msg)
        print('Button X Clicked')
    
    def on_button4_clicked(self):
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.header.stamp = rospy.Time.now()
        joy_msg.header.frame_id = 'joy'
        joy_msg.axes = [0, 0, 0, 0, 0, 0, 0, 0]
        joy_msg.buttons = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]

        self.botton_pub.publish(joy_msg)
        print('Button Y Clicked')

    def get_button_msg(self):
        return self.button_msg


class JoyPad(QWidget):
    def __init__(self):
        super().__init__()

        self.joystick1 = Joystick(name='Left')
        self.joystick2 = Joystick(name='Right')
        self.joybutton = JoyButton()

        self.layout = QHBoxLayout()
        self.layout.addWidget(self.joystick1)
        self.layout.addWidget(self.joystick2)
        self.layout.addWidget(self.joybutton)

        self.setLayout(self.layout)
        self.setWindowTitle('JoyPad')
        self.show()

        rospy.init_node('joy_pad')
        self.joy_pub = rospy.Publisher('joy', sensor_msgs.msg.Joy, queue_size=1)
