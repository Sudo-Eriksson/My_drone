import time
import serial


class ArduinoBluetoothConnector():

    def __init__(self):
        self.bt_con=-1

    def connect_to_arduino(self, port, nr):
        """
        Method for connecting to an Arduino bluetooth module.
        :param port: The port where the module is located on the computer
        :param nr: The baud. Should be the same as in the Arduino program
        """

        self.bt_con = serial.Serial(port, nr)
        print("Connected to Arduino!")
        self.bt_con.flushInput()

    def send_string(self, string):
        """
        Method for sending a string to the Arduino.
        :param string: The string to send.
        """

        self.bt_con.write(str.encode(string))
        time.sleep(0.1)

    def read_string(self):
        message = self.bt_con.readline()
        return message.decode()

    def close_connection(self):
        """
        Method for closing the connection to the Arduino.
        """

        self.bt_con.close()

"""

# Example of usage

print("Start")

arduino_connection = ArduinoBluetoothConnector()

arduino_connection.connect_to_arduino(port="/dev/tty.HC-06-DevB", nr=9600)

arduino_connection.send_string("100")

arduino_connection.close_connection()

print("done")
"""