import sys

from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QLabel, QWidget, QPushButton, QGridLayout, QPlainTextEdit
from PyQt5.QtGui import QTextCursor
from PyQt5.Qt import Qt
from PyQt5 import QtCore
from QLed import QLed
from osLib import OsLib
from Arduino_connecter import ArduinoBluetoothConnector


class Window(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle('Drone Ctrl Gui')
        self.setGeometry(100, 100, 350, 80)
        self.move(200, 150)

        # Init external libraries
        self.os_lib = OsLib()
        self.arduino_connection = ArduinoBluetoothConnector()


        # Initial Gui variables
        self.isBluetoothOn = self.os_lib.is_bt_On()
        self.isConnectedToDrone = False
        self.isArmed = False
        self.isAllowedToFly = False

        # Do stuff with the central widget
        #TODO: Google this.
        self.centralWidget = QWidget(self)
        self.setCentralWidget(self.centralWidget)
        self.mainGrid = QGridLayout(self.centralWidget)

        self._create_init_buttons()
        self._create_status_leds()
        self._create_message_box()

        self.setLayout(self.mainGrid)

        if not self.os_lib.is_bt_On():
            self.write_to_msg_box("Bluetooth is off. Turn it on and restart application.")

    def _create_init_buttons(self):

        self.connectButton = QPushButton('Connect to drone')
        self.connectButton.clicked.connect(self._click_connect_to_drone)
        self.connectButton.setEnabled(self.isBluetoothOn)

        self.armButton = QPushButton('Arm Motors')
        self.armButton.clicked.connect(self._click_arm_motors)
        self.armButton.setEnabled(False)

        self.flyButton = QPushButton('Allow flight')
        self.flyButton.clicked.connect(self._click_start_flying)
        self.flyButton.setEnabled(False)

        self.mainGrid.addWidget(self.connectButton, 0, 0)
        self.mainGrid.addWidget(self.armButton,     1, 0)
        self.mainGrid.addWidget(self.flyButton,     2, 0)

    def _create_status_leds(self):

        ledwidth = 20

        self.connectionLed = QLed()
        self.connectionLed.m_onColour = QLed.Green
        self.connectionLed.m_offColour = QLed.Red
        self.connectionLed.value = self.isConnectedToDrone
        self.connectionLed.setFixedWidth(ledwidth)

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

        self.mainGrid.addWidget(QLabel('Con. Status:'),     0, 1)
        self.mainGrid.addWidget(self.connectionLed,         0, 2)

        self.mainGrid.addWidget(QLabel('Arm Status:'),      1, 1)
        self.mainGrid.addWidget(self.armLed,                1, 2)

        self.mainGrid.addWidget(QLabel('Allowed to fly:'),   2, 1)
        self.mainGrid.addWidget(self.flyingLed,              2, 2)

    def _create_message_box(self):
        self.msg_box = QPlainTextEdit()
        self.msg_box.setReadOnly(True)

        self.mainGrid.addWidget(self.msg_box, 3, 0, 1, 3)

    def _click_connect_to_drone(self):
        conn_devs = self.os_lib.get_connected_devices()

        if len(conn_devs) > 1:
            self.write_to_msg_box("One or more devices connected to computer. Please disconnect:")
            for dev in conn_devs:
                self.write_to_msg_box(dev)
        else:
            self.connect_to_arduino()

            conn_devs = self.os_lib.get_connected_devices()
            self.write_to_msg_box("Connected to {}".format(conn_devs[0]))

            self.update_gui()

    def _click_arm_motors(self):
        self.isArmed = True
        self.update_gui()

    def _click_start_flying(self):
        self.isAllowedToFly = True
        self.update_gui()

    def update_gui(self):
        # Update leds
        self.connectionLed.value = self.isConnectedToDrone
        self.armLed.value = self.isArmed
        self.flyingLed.value = self.isAllowedToFly

        # Update buttons
        self.connectButton.setEnabled(self.isBluetoothOn)
        self.armButton.setEnabled(self.isConnectedToDrone)
        self.flyButton.setEnabled(self.isConnectedToDrone and self.isArmed)

        self.repaint()

    def write_to_msg_box(self, text):
        # Cursor stuff is for adding text to top of textbox
        cursor = QTextCursor(self.msg_box.document())
        cursor.setPosition(0)
        self.msg_box.setTextCursor(cursor)

        self.msg_box.insertPlainText("{}\n".format(text))

        self.repaint()

    def connect_to_arduino(self):
        self.arduino_connection.connect_to_arduino(port="/dev/tty.HC-06-DevB", nr=9600)
        self.isConnectedToDrone = True

if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = Window()
    win.show()
    sys.exit(app.exec_())
