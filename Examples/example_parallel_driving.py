import emir.core as emir
import time

def main():
    delay = 0.5

    red = emir.Emir("eMIR-Red")
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

    k_P_distance = 0.5
    k_D_distance = 0.2
    k_P_parallel = 0.1
    k_D_parallel = 0.05
    wantedDistanceFromWall = 20
    oldDistanceError = 0
    oldParallelError = 0
    oldTime = 0

    while red.proximitySensors[0] > 15:
        print(red.proximitySensors[4])
        distanceErrorSen4 = wantedDistanceFromWall - red.proximitySensors[4]
        distanceErrorSen2 = wantedDistanceFromWall - red.proximitySensors[2]
        # distanceError = min(distanceErrorSen2, distanceErrorSen4)
        distanceError = distanceErrorSen4
        parallelError = red.proximitySensors[2] - (9**2 + (red.proximitySensors[4] + 3.8)**2)**0.5

        newTime = time.time()
        timeStep = newTime - oldTime
        print(timeStep)

        distanceError_d = int((distanceError - oldDistanceError) / timeStep)
        parallelError_d = int((parallelError - oldParallelError) / timeStep)

        red.setRotation = int(k_P_distance * distanceError + k_D_distance * distanceError_d + k_P_parallel * parallelError + k_D_parallel * parallelError_d)

        oldDistanceError = distanceError
        oldParallelError = parallelError
        oldTime = newTime
        time.sleep(0.01)

    red.setSpeed = 0
    red.setRotation = 0

    red.stopReceivingRobotStatus()
    red.stopSendingMoveCommands()

    red.beep(1)
    time.sleep(delay)

    red.stop()
    time.sleep(delay)
    # red.turnOff()

def sign(number):
    return number and (1, -1)[number < 0]


if __name__ == '__main__':
    main()
