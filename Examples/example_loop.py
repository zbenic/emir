import emir.core as emir
import time

def main():
    delay = 0.25

    red = emir.Emir("eMIR-Red")
    red.connect()
    red.startReceivingRobotStatus(True)

    red.setMaxSpeed(50)
    time.sleep(delay)

    red.setMaxRotation(50)
    time.sleep(delay)

    red.startSendingMoveCommands()

    for counter in range(0, 100):
        red.setSpeed = 100
        red.setRotation = 100

    red.stopReceivingRobotStatus()

    red.setSpeed = 0
    red.setRotation = 0

    red.stopSendingMoveCommands()

    red.beep(1)
    time.sleep(delay)

    red.stop()
    time.sleep(delay)
    red.turnOff()

if __name__ == '__main__':
    main()
