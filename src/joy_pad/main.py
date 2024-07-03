import sys
import time
import rospy

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QThread

from .joy_pad import Joystick, JoyPad
VERBOSE = False

class Thread(QThread):
    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent

    def run(self):
        while(rospy.is_shutdown() == False):

            # time to sleep
            time.sleep(2)
            strength_l = self.parent.joystick1.get_strength()
            angle_l = self.parent.joystick1.get_angle(in_deg=True)
            strength_r = self.parent.joystick2.get_strength()
            angle_r = self.parent.joystick2.get_angle(in_deg=True)
            if(VERBOSE == True):
                print("left: ",'Strength : {:.2f} | Angle : {:.2f}º'.format(strength_l, angle_l))
                print("right: ",'Strength : {:.2f} | Angle : {:.2f}º'.format(strength_r, angle_r))


def main():
    app = QApplication(sys.argv)
    joystick = JoyPad()

    th = Thread(joystick)
    th.start()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()