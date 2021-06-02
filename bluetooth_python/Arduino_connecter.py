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
            self.bt_con = serial.Serial(port, nr, timeout=20)
            self.bt_con.flushInput()
            return True

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

    def send_setup_msg(self, byte_to_send):

        # Send message to drone to continue the setup
        msg_struct = struct.pack("b", byte_to_send)
        self.bt_con.write(msg_struct)

        # Wait for acc from drone
        print("Waiting for acc...")
        drone_acc = self.bt_con.read(1)
        print("Got acc: {}".format(drone_acc))
        if int(drone_acc) == byte_to_send:
            print("acc ok!")
            return True
        else:
            # TODO: Skicka abort meddelande till drönare här.
            return False


"""

# Example of usage

print("Start")

arduino_connection = ArduinoBluetoothConnector()

arduino_connection.connect_to_arduino(port="/dev/tty.HC-06-DevB", nr=9600)

arduino_connection.send_string("100")

arduino_connection.close_connection()

print("done")
"""