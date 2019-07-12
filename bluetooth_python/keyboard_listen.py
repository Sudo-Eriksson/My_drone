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
ctrl_lim = 2000;

# Define the reference vector that will be transferred
# [ PITCH, ROLL ]
r_vec = [ctrl_lim, ctrl_lim]

# Start the bluethooth connection
# TODO: Lägg detta som en knapp i spelet
#arduino_connection = ArduinoBluetoothConnector()
#arduino_connection.connect_to_arduino(port="/dev/tty.HC-06-DevB", nr=9600)

while True:

    # Key is still down and we will keep counting
    # This will be ignored in the first loop
    if r_vec[0] != ctrl_lim and r_vec[0] < ctrl_lim*2 and r_vec[0] > 0:
        if r_vec[0] > ctrl_lim:
            r_vec[0] += 1
        else:
            r_vec[0] -= 1

    if r_vec[1] != ctrl_lim and r_vec[1] < ctrl_lim*2 and r_vec[1] > 0:
        if r_vec[1] > ctrl_lim:
            r_vec[1] += 1
        else:
            r_vec[1] -= 1

    for event in pygame.event.get():

        # Handle the exit button in the program
        if event.type == pygame.locals.QUIT:
            #arduino_connection.close_connection()
            pygame.quit()
            sys.exit()

        # Let's handle the key presses
        elif event.type == pygame.locals.KEYDOWN:

            if event.key == pygame.K_UP:
                r_vec[0] = ctrl_lim-1

            elif event.key == pygame.K_DOWN:
                r_vec[0] = ctrl_lim+1

            if event.key == pygame.K_RIGHT:
                r_vec[1] = ctrl_lim+1

            elif event.key == pygame.K_LEFT:
                r_vec[1] = ctrl_lim-1

        # We must handle when we release the buttons. The reference should then turn to zero.
        elif event.type == pygame.locals.KEYUP:

            if event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                r_vec[0] = ctrl_lim

            if event.key == pygame.K_RIGHT or event.key == pygame.K_LEFT:
                r_vec[1] = ctrl_lim

    # TODO: Visa värden i spelet istället
    print(r_vec[0])
    # Skicka värden
    # TODO: Här måste man komma på något smart sätt att skicka värden

    r_vec_packed = struct.pack("i", r_vec[0]) # r_vec[1])
    print(r_vec_packed)
    print(r_vec)

    #arduino_connection.send_string(str(r_vec))

    time.sleep(1)
