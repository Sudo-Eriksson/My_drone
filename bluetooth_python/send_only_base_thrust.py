import pygame.locals
import sys
import time
import struct
from Arduino_connecter import ArduinoBluetoothConnector

pygame.init()

BLACK = (0, 0, 0)

WIDTH = 300
HEIGHT = 300

windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)

windowSurface.fill(BLACK)

# Set limit to the control signal, this might be scaled later
ctrl_max = 1900
ctrl_min = 1000

# Initial value of bese thrust.
msg_base_thrust = ctrl_min

"""
# Start the bluethooth connection
# TODO: Lägg detta som en knapp i spelet
arduino_connection = ArduinoBluetoothConnector()
arduino_connection.connect_to_arduino(port="/dev/tty.HC-06-DevB", nr=9600)

print("Sending python-ready-signal")
arduino_connection.send_string("0001")

ready = False
print("Waiting on drone to setup...")
# Wait for the drone to setup. It will send the string "start" when done.
while not ready:
    line = arduino_connection.read_string()
    if "start" in line:
        ready = True
    time.sleep(1)
"""
print("Drone setup done, start sending.")

while True:

    t = time.time()

    # Handle if the keys are still held down
    keys = pygame.key.get_pressed()
    if keys[pygame.K_UP]:
        msg_base_thrust += 5
    elif keys[pygame.K_DOWN]:
        msg_base_thrust -= 5

    for event in pygame.event.get():

        # Handle the exit button in the program
        if event.type == pygame.locals.QUIT:
            #arduino_connection.close_connection()
            pygame.quit()
            sys.exit()

        # Let's handle the key presses
        elif event.type == pygame.locals.KEYDOWN:

            if event.key == pygame.K_UP:
                msg_base_thrust += 5

            elif event.key == pygame.K_DOWN:
                msg_base_thrust -= 5

            elif event.key == pygame.K_SPACE:
                msg_base_thrust = 0


    # Avoid to small and large numbers 
    if msg_base_thrust > ctrl_max:
        msg_base_thrust = ctrl_max

    elif msg_base_thrust < ctrl_min:
        msg_base_thrust = ctrl_min

    # TODO: Visa värden i spelet istället
    #print(msg_base_thrust)
    # Skicka värden
    # TODO: Här måste man komma på något smart sätt att skicka värden

    #r_vec_packed = struct.pack("i", msg_base_thrust) # r_vec[1])
    print(msg_base_thrust)

    #arduino_connection.send_string(str(msg_base_thrust))

    time.sleep(0.5)#0.004)