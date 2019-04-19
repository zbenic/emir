import emir.core as emir
import time

def main():
    delay = 0.5

    red = emir.Emir("eMIR-Red")
    red.connect()
    red.startReceivingRobotStatus(False)

    red.setMaxSpeed(50)
    time.sleep(delay)

    red.setMaxRotation(50)
    time.sleep(delay)

    print(str(red.workMode))

    red.translate(50, 100)
    time.sleep(6)

    red.rotate(20, 100)
    time.sleep(2)

    red.rotate(-20, 50)
    time.sleep(2)

    red.translate(-50, 100)
    time.sleep(6)

    red.stopReceivingRobotStatus()
    time.sleep(delay)

    red.beep(1)
    time.sleep(delay)

    red.stop()
    time.sleep(delay)
    print(str(red.workMode))
    red.turnOff()


if __name__ == '__main__':
    main()
