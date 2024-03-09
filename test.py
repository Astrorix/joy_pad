import sys
import time

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QThread

from joy import Joystick, JoyPad


class Thread(QThread):
    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent

    def run(self):
        for i in range(100):
            time.sleep(0.5)
            strength = joystick.joystick1.get_strength()
            angle = joystick.joystick1.get_angle(in_deg=True)
            print('Strength : {:.2f} | Angle : {:.2f}º'.format(strength, angle))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    joystick = JoyPad()

    th = Thread(joystick)
    th.start()

    sys.exit(app.exec_())