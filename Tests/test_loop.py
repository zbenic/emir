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

    red.setMinDistance(20)
    time.sleep(delay)

    for counter in range(0, 100):
        red.move(50, 0)
        time.sleep(delay)

    red.stop()
    red.turnOff()

if __name__ == '__main__':
    main()
