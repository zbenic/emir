import emir
import time
import keyboard

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

    while True:
        if keyboard.is_pressed('w'):
            red.move(50, 0)
            time.sleep(delay)

        if keyboard.is_pressed('s'):
            red.move(-50, 0)
            time.sleep(delay)

        if keyboard.is_pressed('a'):
            red.move(0, 50)
            time.sleep(delay)

        if keyboard.is_pressed('d'):
            red.move(0, -50)
            time.sleep(delay)

        if keyboard.is_pressed('q'):
            break

    red.stop()
    red.turnOff()

if __name__ == '__main__':
    main()
