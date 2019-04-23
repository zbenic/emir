import emir.core as emir
import time

def main():
    delay = 0.05

    red = emir.Emir("eMIR-Red")
    red.connect()
    red.startReceivingRobotStatus(False)

    red.setMaxSpeed(20)
    time.sleep(delay)

    red.setMaxRotation(10)
    time.sleep(delay)

    red.startSendingMoveCommands()

    red.setSpeed = 20
    red.setRotation = 0

    k_P_distance = 5
    k_P_parallel = 2.5
    wantedDistanceFromWall = 20

    while red.proximitySensors[0] > 15:
        distanceError = wantedDistanceFromWall - red.proximitySensors[4]
        parallelError = (9**2 + (red.proximitySensors[4] + 3.8)**2)**0.5 - red.proximitySensors[2]
        red.setRotation = sign(distanceError) * k_P_distance * distanceError + sign(parallelError) * k_P_parallel * parallelError

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
