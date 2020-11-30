#!/usr/bin/python

import rospy
import sys
import signal
from ezgripper_libs.ezgripper_interface import EZGripper
from python_qt_binding.QtWidgets import QMainWindow, QPushButton, QApplication
from python_qt_binding.QtCore import QTimer

rospy.init_node("hello_ezgripper")
gripper = EZGripper("ezgripper/main")


class GripperGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):

        calibrateButton = QPushButton("Calibrate", self)
        calibrateButton.resize(100, 30)
        calibrateButton.clicked.connect(gripper.calibrate)
        calibrateButton.move(50, 10)
        calibrateButton.show()

        releaseButton = QPushButton("Release", self)
        releaseButton.resize(200, 200)
        releaseButton.clicked.connect(gripper.release)
        releaseButton.move(50, 50)

        hard_closeButton = QPushButton("Hard\nClose", self)
        hard_closeButton.resize(100, 200)
        hard_closeButton.clicked.connect(gripper.hard_close)
        hard_closeButton.move(250, 50)

        hard_closeButton = QPushButton("Soft\nClose", self)
        hard_closeButton.resize(100, 200)
        hard_closeButton.clicked.connect(gripper.soft_close)
        hard_closeButton.move(350, 50)

        openButton = QPushButton("Open", self)
        openButton.clicked.connect(gripper.open)
        openButton.resize(200, 200)
        openButton.move(450, 50)

        gotoButton = QPushButton("0%\nTorque Mode", self)
        gotoButton.resize(100, 200)
        gotoButton.clicked.connect(self.submit_goto1)
        gotoButton.move(50, 250)

        gotoButton = QPushButton("10%", self)
        gotoButton.resize(100, 200)
        gotoButton.clicked.connect(self.submit_goto2)
        gotoButton.move(150, 250)

        gotoButton = QPushButton("20%", self)
        gotoButton.resize(100, 200)
        gotoButton.clicked.connect(self.submit_goto3)
        gotoButton.move(250, 250)

        gotoButton = QPushButton("30%", self)
        gotoButton.resize(100, 200)
        gotoButton.clicked.connect(self.submit_goto4)
        gotoButton.move(350, 250)

        gotoButton = QPushButton("40%", self)
        gotoButton.resize(100, 200)
        gotoButton.clicked.connect(self.submit_goto5)
        gotoButton.move(450, 250)

        gotoButton = QPushButton("50%", self)
        gotoButton.resize(100, 200)
        gotoButton.clicked.connect(self.submit_goto6)
        gotoButton.move(550, 250)

        gotoButton = QPushButton("60%", self)
        gotoButton.resize(100, 200)
        gotoButton.clicked.connect(self.submit_goto7)
        gotoButton.move(150, 450)

        gotoButton = QPushButton("70%", self)
        gotoButton.resize(100, 200)
        gotoButton.clicked.connect(self.submit_goto8)
        gotoButton.move(250, 450)

        gotoButton = QPushButton("80%", self)
        gotoButton.resize(100, 200)
        gotoButton.clicked.connect(self.submit_goto9)
        gotoButton.move(350, 450)

        gotoButton = QPushButton("90%", self)
        gotoButton.resize(100, 200)
        gotoButton.clicked.connect(self.submit_goto10)
        gotoButton.move(450, 450)

        gotoButton = QPushButton("100%", self)
        gotoButton.resize(100, 200)
        gotoButton.clicked.connect(self.submit_goto11)
        gotoButton.move(550, 450)

        self.statusBar()

        self.setGeometry(300, 200, 800, 850)
        self.setWindowTitle("EZGripper GUI")
        self.show()

    def submit_goto1(self):

        gripper.goto_position(0.0, 100)

    def submit_goto2(self):

        gripper.goto_position(10.0, 100)

    def submit_goto3(self):

        gripper.goto_position(20.0, 100)

    def submit_goto4(self):

        gripper.goto_position(30.0, 100)

    def submit_goto5(self):

        gripper.goto_position(40.0, 100)

    def submit_goto6(self):

        gripper.goto_position(50.0, 100)

    def submit_goto7(self):

        gripper.goto_position(60.0, 100)

    def submit_goto8(self):

        gripper.goto_position(70.0, 100)

    def submit_goto9(self):

        gripper.goto_position(80.0, 100)

    def submit_goto10(self):

        gripper.goto_position(90.0, 100)

    def submit_goto11(self):

        gripper.goto_position(100.0, 100)


def main():
    signal.signal(signal.SIGINT, lambda *args: QApplication.quit())
    ezgripper_app = QApplication(sys.argv)
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)
    _ = GripperGUI()
    sys.exit(ezgripper_app.exec_())


if __name__ == "__main__":
    main()
