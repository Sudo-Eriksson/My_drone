import time
import serial
import struct


class ArduinoBluetoothConnector():

    def __init__(self):
        self.bt_con=-1

    def connect_to_arduino(self, port, nr):
        """
        Method for connecting to an Arduino bluetooth module.
        :param port: The port where the module is located on the computer
        :param nr: The baud. Should be the same as in the Arduino program
        """
        try:
            # Connect to HC-module
            self.bt_con = serial.Serial(port, nr, timeout=10)
            self.bt_con.flushInput()

            # Send message to drone to continue the setup
            self.send_connect_msg()

            # Wait for acc from drone
            print("Waiting for acc...")
            drone_acc = self.bt_con.read(10)
            if drone_acc == b'1':
                return True
            else:
                #TODO: Skicka abort meddelande till drönare här.
                return False

        except serial.serialutil.SerialException:
            return False

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

    def send_connect_msg(self):
        msg = 1

        msg_struct = struct.pack("b", msg)

        self.bt_con.write(msg_struct)


"""

# Example of usage

print("Start")

arduino_connection = ArduinoBluetoothConnector()

arduino_connection.connect_to_arduino(port="/dev/tty.HC-06-DevB", nr=9600)

arduino_connection.send_string("100")

arduino_connection.close_connection()

print("done")
"""