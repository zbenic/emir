import emir.core as emir
import time

def main():
    delay = 0.05

    red = emir.Emir("eMIR-Red")
    red.connect()
    red.startReceivingRobotStatus(False)

    red.setMaxSpeed(50)
    time.sleep(delay)

    red.setMaxRotation(50)
    time.sleep(delay)

    red.startSendingMoveCommands()

    red.setSpeed = 25
    red.setRotation = 0

    while True:
        if red.proximitySensors[0] < 25:
            red.setSpeed = 0
            break

    red.stopReceivingRobotStatus()
    red.stopSendingMoveCommands()

    red.beep(1)
    time.sleep(delay)

    red.stop()
    time.sleep(delay)
    red.turnOff()

if __name__ == '__main__':
    main()
