import emir
import time

offline = True

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
        blue.connect()

    robotDict = {0: red,
                 1: blue}

    for counter in range(0, 10):
        if offline:
            for robotIdx in range(0, 2):
                robot = robotDict[robotIdx]
                for sensorIdx in range(0, 6):
                    print(robot.name + " sensor " + str(sensorIdx) + ":" + str(robot.proximitySensors[sensorIdx]))

                print(robot.name + " battery voltage:" + str(robot.battery))
                print(robot.name + " charging voltage:" + str(robot.chargeVoltage))
                print(robot.name + " translation speed:" + str(robot.speed))
                print(robot.name + " rotation speed:" + str(robot.rotation))
                print(robot.name + " left motor PWM:" + str(robot.leftMotorPwm))
                print(robot.name + " right motor PWM:" + str(robot.rightMotorPwm))
                print(robot.name + " work mode:" + str(robot.workMode))
                print(robot.name + " digital input:" + str(robot.digitalIn))
                print(robot.name + " azimuth:" + str(robot.azimuth))
                print(robot.name + " path:" + str(robot.path))
                print(robot.name + " angle:" + str(robot.angle))
                time.sleep(0.5)
        else:
            red.setMaxSpeed(40)
            blue.setMaxSpeed(20)
            time.sleep(0.1)

            red.setMaxRotation(70)
            blue.setMaxRotation(50)
            time.sleep(0.1)

            red.translate(10, 100)
            blue.translate(10, 80)
            time.sleep(0.1)

            red.rotate(180, 50)
            blue.translate(-180, 80)
            time.sleep(0.1)

            red.rotate(180, 50)
            blue.translate(-180, 80)
            time.sleep(0.1)

            if counter == 5:
                red.beep(1)
                time.sleep(1.5)
                blue.beep(1)
                time.sleep(0.1)

            red.setMinDistance(20)
            blue.setMinDistance(40)
            time.sleep(0.1)

            red.sendInfoOn()
            blue.sendInfoOn()

            red.move(50, 60)
            blue.move(30, 60)

            time.sleep(0.1)

            if counter == 500:
                print("Azimuth and counters are reset!\n")
                red.resetAzimuth()
                blue.resetAzimuth()
                red.resetCounters()
                blue.resetCounters()

            red.getRobotStatus()
            blue.getRobotStatus()

            time.sleep(0.1)

            for robotIdx in range(0, 2):
                robot = robotDict[robotIdx]
                for sensorIdx in range(0, 6):
                    print(robot.name + " sensor " + str(sensorIdx) + ":" + str(robot.proximitySensors[sensorIdx]))

                print(robot.name + " battery voltage:" + str(robot.battery))
                print(robot.name + " charging voltage:" + str(robot.chargeVoltage))
                print(robot.name + " translation speed:" + str(robot.speed))
                print(robot.name + " rotation speed:" + str(robot.rotation))
                print(robot.name + " left motor PWM:" + str(robot.leftMotorPwm))
                print(robot.name + " right motor PWM:" + str(robot.rightMotorPwm))
                print(robot.name + " work mode:" + str(robot.workMode))
                print(robot.name + " digital input:" + str(robot.digitalIn))
                print(robot.name + " azimuth:" + str(robot.azimuth))
                print(robot.name + " path:" + str(robot.path))
                print(robot.name + " angle:" + str(robot.angle))

            red.stop()
            blue.stop()


if __name__ == '__main__':
    main()
