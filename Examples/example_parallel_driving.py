import emir.core as emir
import time
import math

def main():
    delay = 0.2

    red = emir.Emir("eMIR-Yellow")
    red.connect()
    red.startReceivingRobotStatus(False)

    red.setMaxSpeed(20)
    time.sleep(delay)

    red.setMaxRotation(10)
    time.sleep(delay)

    red.startSendingMoveCommands()
    time.sleep(delay)

    red.setSpeed = 20
    red.setRotation = 0

    k_P_parallelism = 1
    k_D_distance = 4

    wantedDistanceFromWall = 25

    while red.proximitySensors[0] > 15:
        red.setRotation = int(k_D_distance * (wantedDistanceFromWall - red.proximitySensors[4]) -\
                          k_P_parallelism * ((red.proximitySensors[2] * math.cos(math.radians(56) + 4.39)) - red.proximitySensors[4]))

    red.setSpeed = 0
    red.setRotation = 0
    time.sleep(0.2)

    red.stopReceivingRobotStatus()
    red.stopSendingMoveCommands()

    red.beep(1)
    time.sleep(delay)

    red.stop()
    time.sleep(delay)
    red.turnOff()

def sign(number):
    return number and (1, -1)[number < 0]


if __name__ == '__main__':
    main()
