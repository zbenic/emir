import emir
import time
import tkinter
from threading import Thread

offline = False


class StatusMessageWorker(Thread):
    def __init__(self, robot: emir.Emir):
        Thread.__init__(self)
        self.robot = robot

    def run(self):
        while True:
            try:
                self.robot.getRobotStatus()
                self.robot.printStatusMessageValues()
            finally:
                self.robot.update = False

def main():
    red = emir.Emir("eMIR-Red")
    blue = emir.Emir("eMIR-Blue")
    if offline:
        red.statusMessage = b'30000000000F00019C123/*505050505050800000000000F00019C1D6/*505050505050330000000000F00019C123/*505050505050800000000000F00019C'
        blue.statusMessage = b'30000000000F00019C123/*505050505050800000000000F00019C1D6/*505050505050330000000000F00019C123/*505050505050800000000000F00019C'
        red.getRobotStatus()
        blue.getRobotStatus()
    else:
        red.connect()
        # blue.connect()
        redWorker = StatusMessageWorker(red)
        # blueWorker = StatusMessageWorker(blue)
        redWorker.daemon = True
        # blueWorker.daemon = True
        redWorker.start()
        # blueWorker.start()

    robotDict = {0: red,
                 1: blue}

    delay = 0.25

    if not offline:
        red.setMaxSpeed(40)
        # blue.setMaxSpeed(20)
        time.sleep(delay)

        red.setMaxRotation(70)
        # blue.setMaxRotation(50)
        time.sleep(delay)

        red.setMinDistance(20)
        # blue.setMinDistance(40)
        time.sleep(delay)

        red.sendInfoOn()
        # blue.sendInfoOn()
        time.sleep(delay)

    for counter in range(0, 100):
        if offline:
            for robotIdx in range(0, 2):
                robot = robotDict[robotIdx]
                robot.printStatusMessageValues()
        else:
            # red.translate(10, 100)
            # # blue.translate(10, 80)
            # time.sleep(delay)
            #
            # red.rotate(10, 50)
            # # blue.rotate(10, 80)
            # time.sleep(delay)
            #
            # red.rotate(-10, 50)
            # # blue.rotate(-10, 80)
            # time.sleep(delay)

            # if counter == 5:
            #     red.beep(1)
            #     time.sleep(1.5)
            #     blue.beep(1)
            #     time.sleep(0.1)

            red.move(-50, 0)
            # blue.move(30, 60)
            time.sleep(delay)

            if counter == 10:
                print("Azimuth and counters are reset!\n")
                red.resetAzimuth()
                # blue.resetAzimuth()
                time.sleep(delay)
                red.resetCounters()
                # blue.resetCounters()
                time.sleep(delay)

            red.getRobotStatus()
            # blue.getRobotStatus()

            time.sleep(0.25)

            # for robotIdx in range(0, 1):
            #     robot = robotDict[robotIdx]
            #     robot.printStatusMessageValues()

            red.stop()
            # blue.stop()


if __name__ == '__main__':
    main()
