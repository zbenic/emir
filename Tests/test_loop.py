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

    for counter in range(0, 100):
        red.move(100, 0)
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
