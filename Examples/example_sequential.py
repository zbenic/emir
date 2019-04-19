import emir.core as emir
import time

def main():
    delay = 0.5

    red = emir.Emir("eMIR-Red")
    red.connect()
    red.startReceivingRobotStatus(True)

    red.setMaxSpeed(50)
    time.sleep(delay)

    red.setMaxRotation(50)
    time.sleep(delay)

    red.translate(50, 100)
    time.sleep(10)

    red.rotate(10, 100)
    time.sleep(3)

    red.rotate(-10, 50)
    time.sleep(3)

    red.translate(-50, 100)
    time.sleep(10)

    red.stopReceivingRobotStatus()
    time.sleep(delay)

    red.beep(1)
    time.sleep(delay)

    red.stop()
    time.sleep(delay)
    red.turnOff()


if __name__ == '__main__':
    main()
