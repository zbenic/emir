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

    red.move(100, 0)
    time.sleep(delay)

    red.rotate(10, 100)
    time.sleep(delay)

    red.rotate(-10, 50)
    time.sleep(delay)

    red.move(-100, 50)
    time.sleep(delay)

    red.stopReceivingRobotStatus()
    time.sleep(delay)

    red.beep(1)
    time.sleep(delay)

    red.stop()
    time.sleep(delay)
    red.turnOff()


if __name__ == '__main__':
    main()
