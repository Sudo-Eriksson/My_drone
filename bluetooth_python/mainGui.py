import sys

from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QWidget, QPushButton, QGridLayout, QPlainTextEdit
from PyQt5.QtGui import QTextCursor, QIcon
from PyQt5.QtCore import Qt
from QLed import QLed
from osLib import OsLib
from Arduino_connecter import ArduinoBluetoothConnector
from datetime import datetime
from joystickWidget import Joystick


class Window(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle('Drone Ctrl Gui')
        self.setGeometry(100, 100, 450, 80)
        self.move(200, 150)

        # Init external libraries
        self.os_lib = OsLib()
        self.arduino_connection = ArduinoBluetoothConnector()

        # Initial Gui variables
        self.isBluetoothOn = self.os_lib.is_bt_On()
        self.isConnectedToDrone = False
        self.isMpuStarted = False
        self.isArmed = False
        self.isAllowedToFly = False

        # Do stuff with the central widget
        #TODO: Google this.
        self.centralWidget = QWidget(self)
        self.setCentralWidget(self.centralWidget)
        self.mainGrid = QGridLayout(self.centralWidget)

        self._create_init_buttons()
        self._create_reset_button()
        self._create_status_leds()
        self._create_message_box()
        self._create_joystick()

        self.setLayout(self.mainGrid)

        if not self.os_lib.is_bt_On():
            self.write_to_msg_box("Bluetooth is off. Turn it on and restart application.")

    def _create_init_buttons(self):

        self.connectButton = QPushButton('Connect to drone')
        self.connectButton.clicked.connect(self._click_connect_to_drone)
        self.connectButton.setEnabled(self.isBluetoothOn)

        self.startMpuButton = QPushButton('Start MPU')
        self.startMpuButton.clicked.connect(self._click_start_mpu_setup)
        self.startMpuButton.setEnabled(False)

        self.armButton = QPushButton('Arm Motors')
        self.armButton.clicked.connect(self._click_arm_motors)
        self.armButton.setEnabled(False)

        self.flyButton = QPushButton('Allow flight')
        self.flyButton.clicked.connect(self._click_start_flying)
        self.flyButton.setEnabled(False)

        self.mainGrid.addWidget(self.connectButton, 0, 0)
        self.mainGrid.addWidget(self.startMpuButton, 1, 0)
        self.mainGrid.addWidget(self.armButton,     2, 0)
        self.mainGrid.addWidget(self.flyButton,     3, 0)

    def _create_reset_button(self):
        self.resetButton = QPushButton('Reset')
        self.resetButton.clicked.connect(self._click_reset)
        self.resetButton.setEnabled(True)

        self.mainGrid.addWidget(self.resetButton, 4, 1)

    def _create_status_leds(self):

        ledwidth = 20

        self.connectionLed = QLed()
        self.connectionLed.m_onColour = QLed.Green
        self.connectionLed.m_offColour = QLed.Red
        self.connectionLed.value = self.isConnectedToDrone
        self.connectionLed.setFixedWidth(ledwidth)

        self.mpuLed = QLed()
        self.mpuLed.m_onColour = QLed.Green
        self.mpuLed.m_offColour = QLed.Red
        self.mpuLed.value = self.isMpuStarted
        self.mpuLed.setFixedWidth(ledwidth)

        self.armLed = QLed()
        self.armLed.m_onColour = QLed.Green
        self.armLed.m_offColour = QLed.Red
        self.armLed.value = self.isArmed
        self.armLed.setFixedWidth(ledwidth)

        self.flyingLed = QLed()
        self.flyingLed.m_onColour = QLed.Green
        self.flyingLed.m_offColour = QLed.Red
        self.flyingLed.value = self.isAllowedToFly
        self.flyingLed.setFixedWidth(ledwidth)

        self.conn_label = QLabel('Connection Status:')
        self.mpu_label = QLabel('MPU status:')
        self.arm_label = QLabel('Arm Status:')
        self.fly_label = QLabel('Allowed to fly:')

        self.conn_label.setAlignment(Qt.AlignRight)
        self.mpu_label.setAlignment(Qt.AlignRight)
        self.arm_label.setAlignment(Qt.AlignRight)
        self.fly_label.setAlignment(Qt.AlignRight)

        self.mainGrid.addWidget(self.conn_label,    0, 1)
        self.mainGrid.addWidget(self.connectionLed, 0, 2)

        self.mainGrid.addWidget(self.mpu_label,     1, 1)
        self.mainGrid.addWidget(self.mpuLed,        1, 2)

        self.mainGrid.addWidget(self.arm_label,     2, 1)
        self.mainGrid.addWidget(self.armLed,        2, 2)

        self.mainGrid.addWidget(self.fly_label,     3, 1)
        self.mainGrid.addWidget(self.flyingLed,     3, 2)

    def _create_joystick(self):
        self.joyStick = Joystick();
        self.mainGrid.addWidget(self.joyStick, 4, 0)

    def _create_message_box(self):
        self.msg_box = QPlainTextEdit()
        self.msg_box.setReadOnly(True)

        self.mainGrid.addWidget(self.msg_box, 5, 0, 1, 3)

    def _click_connect_to_drone(self):
        conn_devs = self.os_lib.get_connected_devices()

        if len(conn_devs) >= 1:
            self.write_to_msg_box("One or more devices connected to computer. Please disconnect {}".format(conn_devs[0]))
        else:
            self.write_to_msg_box("Connecting to drone...")

            self.connect_to_drone()

            if self.isConnectedToDrone:
                conn_devs = self.os_lib.get_connected_devices()
                self.write_to_msg_box("Connected to {}".format(conn_devs[0]))
            else:
                self.write_to_msg_box("Could not connect to drone. Check drone power.")

            self.update_gui()

    def _click_start_mpu_setup(self):
        self.isMpuStarted = self.arduino_connection.send_setup_msg(2)

        if self.isMpuStarted:
            self.write_to_msg_box("MPU6050 started. Performing calibration.")
        else:
            self.write_to_msg_box("Setup of MPU6050 failed. Please restart drone and gui.")

        self.update_gui()

    def _click_arm_motors(self):
        self.isArmed = self.arduino_connection.send_setup_msg(3)

        if self.isArmed:
            self.write_to_msg_box("Motors are getting armed.")
        else:
            self.write_to_msg_box("Could not arm motors. Please restart drone and gui.")

        self.update_gui()

    def _click_start_flying(self):
        self.isAllowedToFly = self.arduino_connection.send_setup_msg(4)

        if self.isAllowedToFly:
            self.write_to_msg_box("Starting to fly!.")
        else:
            self.write_to_msg_box("Could not start flying. Please restart drone and gui.")

        self.update_gui()

    def _click_reset(self):

        successReset = self.arduino_connection.send_reset_msg()

        if successReset:
            self.arduino_connection.close_connection()

            self.isConnectedToDrone = False
            self.isConnectedToDrone = False
            self.isArmed = False
            self.isAllowedToFly = False
        else:
            print("No reset acc from drone.")

        self.update_gui()

    def update_gui(self):
        # Update leds
        self.connectionLed.value = self.isConnectedToDrone
        self.mpuLed.value = self.isConnectedToDrone
        self.armLed.value = self.isArmed
        self.flyingLed.value = self.isAllowedToFly

        # Update buttons
        self.connectButton.setEnabled(self.isBluetoothOn)
        self.startMpuButton.setEnabled(self.isConnectedToDrone)
        self.armButton.setEnabled(self.isMpuStarted)
        self.flyButton.setEnabled(self.isMpuStarted and self.isConnectedToDrone and self.isArmed)

        self.repaint()

    def write_to_msg_box(self, text):
        # Cursor stuff is for adding text to top of textbox
        cursor = QTextCursor(self.msg_box.document())
        cursor.setPosition(0)
        self.msg_box.setTextCursor(cursor)

        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")

        self.msg_box.insertPlainText("{0} {1}\n".format(current_time, text))

    def connect_to_drone(self):
        #TODO: Detta bör göras i en tråd.
        connection_result = self.arduino_connection.connect_to_arduino(port="/dev/tty.HC-06-DevB", nr=9600)
        if connection_result:
            self.isConnectedToDrone = self.arduino_connection.send_setup_msg(1)
        else:
            self.isConnectedToDrone = False


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setWindowIcon(QIcon('./icon.png'))
    win = Window()
    win.show()
    sys.exit(app.exec_())
