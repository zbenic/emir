import bluetooth
import warnings
import struct
import binascii
warnings.simplefilter('always', UserWarning)

VectorInt = [int]

class Emir:
    def __init__(self, name, numOfProximitySensors=6):
        self.name = name
        self.address = None
        self.numberOfRetries = 5
        self.port = None
        self.protocol = None
        self.service = None
        self.sock = None
        self.commands = {'stop':                     '00',
                         'move':                     '01',
                         'translate':                '02',
                         'rotate':                   '03',
                         'beep':                     '04',
                         'sensorsOff':               '05',
                         'sensorsOn':                '05',
                         'setDigitalOutput':         '06',
                         'sendInfoOn':               '10',
                         'sendInfoOff':              '10',
                         'sendInfoEEPROM':           '10',
                         'setMinDistance':           '11',
                         'setMaxSpeed':              '12',
                         'setMaxRotation':           '13',
                         'resetAzimuth':             '14',
                         'resetCounters':            '15',
                         'turnOff':                  'FF'}
        self.connected = False
        self.statusString = None  # TODO: is this needed??
        self.proximitySensors = [None] * numOfProximitySensors
        self.battery = None
        self.lowBattery = False
        self.chargeVoltage = None
        self.charging = False
        self.speed = None
        self.rotation = None
        self.leftMotorPwm = None
        self.rightMotorPwm = None
        self.workMode = None
        self.digitalIn = None
        self.azimuth = None
        self.path = None
        self.angle = None
        self.update = False

    def __getRobotAddress(self):
        retryNumber = 0
        while self.address is None and retryNumber < self.numberOfRetries:
            nearbyDevices = bluetooth.discover_devices()
            for device in nearbyDevices:
                if self.name == bluetooth.lookup_name(device):
                    self.address = device
                    print("Found " + self.name + " with address " + str(self.address))
                    break
            if len(nearbyDevices) == 0:
                warnings.warn("Trial " + str(retryNumber) + ". Could not find " + self.name + " among nearby bluetooth devices.")
                retryNumber += 1

    def __getRobotService(self):
        self.service = bluetooth.find_service(address=self.address)

    def __getRobotPort(self):
        if self.service[0] is not None:
            self.port = self.service[0]["port"]  # TODO: check what services eMIR has
        else:
            warnings.warn("No service could be found for device " + self.name + ".")

    def __getRobotProtocol(self):
        if self.service[0] is not None:
            self.protocol = self.service[0]["protocol"]  # TODO: check what services eMIR has
            if self.protocol == "RFCOMM":
                return bluetooth.RFCOMM
            elif self.protocol == "L2CAP":
                return bluetooth.L2CAP
            else:
                return None
        else:
            warnings.warn("No service could be found for device " + self.name + ".")

    def __getCheckSum(self, *args: int):
        argumentSum = 0
        for arg in args:
            argumentSum += arg

        return ((self.__getMax8bitIntegerValue() + 1) - argumentSum % (self.__getMax8bitIntegerValue() + 1)) % (self.__getMax8bitIntegerValue() + 1)

    @staticmethod
    def __getTwosComplement8bit(decimalBaseInteger: int):
        if (decimalBaseInteger.bit_length() > 8) or (-128 > decimalBaseInteger or decimalBaseInteger > 127):
            warnings.warn("The number does not fit in 8 bits or it's outside of [-128, 127] interval!")
            return None
        else:
            return decimalBaseInteger if decimalBaseInteger >= 0 else 256 + decimalBaseInteger

    @staticmethod
    def __clipValue(value, minValue, maxValue):
        return max(minValue, min(value, maxValue))

    @staticmethod
    def __getMax8bitIntegerValue():
        return 2**8 - 1

    def __messageIsValid(self, *args: int):
        argumentSum = 0
        for arg in args:
            argumentSum += arg

        if argumentSum % (self.__getMax8bitIntegerValue() + 1) == 0:
            return True
        else:
            return False

    def __sendCommand(self, commandName: str, firstArg: int = 0, secondArg: int = 0, firstArgLimits: VectorInt = [0, 0], secondArgLimits: VectorInt = [0, 0], useTwosComplement: bool = True, useValueClipping: bool = True):
        """
        Generic method for sending commands to the robot.
        Command format is #NNaabbCS/, where:
        #  - message start
        NN - command number
        aa - first parameter
        bb - second parameter
        CS - checksum (NN+aa+bb+CS=0)
        /  - message end

        :param commandName:
        :param firstArg:
        :param secondArg:
        :param firstArgLimits:
        :param secondArgLimits:
        :param useTwosComplement:
        :param useValueClipping:
        :return:
        """

        # get the command number from command name
        commandNumber = self.commands[commandName]
        commandNumberInt = int(commandNumber, 16)

        # value clipping
        if useValueClipping:
            firstArg = self.__clipValue(firstArg, firstArgLimits[0], firstArgLimits[1])
            secondArg = self.__clipValue(secondArg, secondArgLimits[0], secondArgLimits[1])

        # calculate 8 bit two's complement
        if useTwosComplement:
            firstArg = self.__getTwosComplement8bit(firstArg)
            secondArg = self.__getTwosComplement8bit(secondArg)

        # calculate checksum parameter of the command
        checkSum = self.__getCheckSum(commandNumberInt, firstArg, secondArg)

        # convert values form decimal to hex base
        firstArg_hex = format(firstArg, '02X')
        secondArg_hex = format(secondArg, '02X')
        checkSum_hex = format(checkSum, '02X')

        # send message if the message is valid (NN+aa+bb+CS=0)
        # raise warning in the case of invalid message
        if self.__messageIsValid(commandNumberInt, firstArg, secondArg, checkSum): #and self.connected:
            print("#" + commandNumber + firstArg_hex + secondArg_hex + checkSum_hex + "/")
            # self.sock.send("#" + commandNumber + firstArg_hex + secondArg_hex + checkSum_hex + "/")
        elif not self.connected:
            warnings.warn("Command '" + commandName + "': Robot " + self.name + " no connected!")
        else:
            warnings.warn("Command '" + commandName + "': checksum is not 0!")

    def __parseStatusMessage(self):
        statusMessage = self.statusString
        if statusMessage[0] != '*':
            warnings.warn("Problem occurred with parsing the robot status message. First character is not '*', but " + statusMessage[0] + ".")
        if statusMessage[-1] != '/':
            warnings.warn("Problem occurred with parsing the robot status message. Last character is not '/', but " +
                          statusMessage[-1] + ".")

        # read proximity sensors data (bytes from 1:13) [cm]
        for sensorIndex in range (0, self.proximitySensors.count()):
            self.proximitySensors[sensorIndex] = int(statusMessage[(sensorIndex * 2 + 1):(sensorIndex * 2 + 1) + 2], 16)

        # read battery/charging voltage [V]
        msb = bool(format(int(statusMessage[13:15], 16), '0>8b')[0])
        messageBatteryValueBinary = format(int(statusMessage[13:15], 16), '0>8b')[1:]
        messageBatteryValueInt = int(messageBatteryValueBinary, 2)
        if msb:  # if MSB is 1
            if messageBatteryValueInt > 20:
                self.chargeVoltage = 9 + messageBatteryValueInt / 17
            else:
                self.chargeVoltage = None
            self.charging = False if self.chargeVoltage < 9 else True
        else:
            self.battery = 9 + messageBatteryValueInt / 17
            if self.battery < 11.1:
                self.lowBattery = True
            else:
                self.lowBattery = False

        # read speed [%]
        messageSpeedValueInt = int(statusMessage[15:17], 16)
        self.speed = messageSpeedValueInt if messageSpeedValueInt < 128 else messageSpeedValueInt - 256

        # read rotation [%]
        messageRotationValueInt = int(statusMessage[17:19], 16)
        self.rotation = messageRotationValueInt if messageRotationValueInt < 128 else messageRotationValueInt - 256

        # read left motor PWM [%]
        messageLeftMotorPwmValueInt = int(statusMessage[19:21], 16)
        self.leftMotorPwm = messageLeftMotorPwmValueInt if messageLeftMotorPwmValueInt < 128 else messageLeftMotorPwmValueInt - 256

        # read right motor PWM [%]
        messageRightMotorPwmValueInt = int(statusMessage[21:23], 16)
        self.leftMotorPwm = messageRightMotorPwmValueInt if messageRightMotorPwmValueInt < 128 else messageRightMotorPwmValueInt - 256

        # read work mode
        self.workMode =  int(statusMessage[23:25], 16)

        # read digital input state
        self.digitalIn =  int(statusMessage[25:27], 16)

        # read azimuth [deg]
        self.azimuth =  int(statusMessage[27:29], 16) * 2  # TODO: check if multiplication with factor 2 is correct

        # read path [cm]
        self.path =  int(statusMessage[29:31], 16)

        # read angle [deg]
        self.path =  int(statusMessage[31:33], 16) * 2  # the value from the message is showing deg/2 value

        # read checksum
        checksum = int(statusMessage[33:35], 16)

        if not self.__messageIsValid(
            sum(self.proximitySensors),
            self.battery,
            self.speed,
            self.rotation,
            self.leftMotorPwm,
            self.rightMotorPwm,
            self.workMode,
            self.digitalIn,
            self.azimuth,
            self.path,
            int(self.angle / 2),
            checksum):
            warnings.warn("Message '" + self.statusString + "': checksum is not 0!")

    def connect(self):
        self.__getRobotAddress()
        self.__getRobotService()
        self.__getRobotPort()
        protocol = self.__getRobotProtocol()

        if protocol is not None:
            self.sock = bluetooth.BluetoothSocket(protocol)
            self.sock.connect((self.address, self.port))
            self.connected = self.sock.connected
            if self.connected:
                print(self.name + " connected.")
            else:
                raise ConnectionError("Could not connect to" + self.name + ".")
        else:
            warnings.warn("Unknown protocol. Only works with RFCOMM or L2CAP protocols.")

    def getRobotStatusMessage(self):
        self.statusString = self.sock.recv(64)  #TODO: check if 64bytes is enough
        if not self.statusString:
            warnings.warn("Problem occurred while getting robot status message which resulted in empty status message.")
            self.update = False
        else:
            self.__parseStatusMessage()
            self.update = True

    def stop(self):
        """

        :return:

        """

        self.__sendCommand('stop')

    def move(self, translationSpeed: int, rotationSpeed: int):
        """
        Move robot with defined translation and rotational speed

        :param translationSpeed: translation speed
        :param rotationSpeed: rotation speed
        :return: N/A

        """

        firstArgLimits = [-100, 100]  # [%]
        secondArgLimits = [-100, 100]  # [%]

        self.__sendCommand('move', translationSpeed, rotationSpeed, firstArgLimits, secondArgLimits)

    def translate(self, path: int, speed: int):
        """

        :param path:
        :param speed:
        :return:

        """

        firstArgLimits = [-100, 100]  # [cm]
        secondArgLimits = [10, 100]  # [%]

        self.__sendCommand('translate', path, speed, firstArgLimits, secondArgLimits)

    def rotate(self, angle: int, speed: int):
        """

        :param angle:
        :param speed:
        :return:

        """

        firstArgLimits = [-180, 180]  # [deg]
        secondArgLimits = [10, 100]  # [%]

        angle = int(angle / 2)

        self.__sendCommand('rotate', angle, speed, firstArgLimits, secondArgLimits)

    def setMinDistance(self, distance: int):
        """

        :param distance:
        :return:

        """

        firstArgLimits = [0, 50]  # [cm]

        self.__sendCommand('setMinDistance', distance, firstArgLimits=firstArgLimits)

    def setMaxSpeed(self, translationSpeed):
        """

        :param translationSpeed:
        :return:

        """

        firstArgLimits = [10, 100]  # [%]

        self.__sendCommand('setMaxSpeed', translationSpeed, firstArgLimits=firstArgLimits)

    def setMaxRotation(self, rotationSpeed):
        """

        :param rotationSpeed:
        :return:

        """

        firstArgLimits = [10, 100]  # [%]

        self.__sendCommand('setMaxRotation', rotationSpeed, firstArgLimits=firstArgLimits)

    def resetAzimuth(self):
        """

        :return:

        """

        self.__sendCommand('resetAzimuth')

    def resetCounters(self):
        """

        :return:

        """

        self.__sendCommand('resetCounters')

    def setDigitalOutput(self, bitNumber, state):
        """

        :param bitNumber:
        :param state:
        :return:

        """

        firstArgLimits = [0, 4]

        if bitNumber > 3:
            secondArgLimits = [0, 15]
        else:
            secondArgLimits = [0, 1]

        self.__sendCommand('setDigitalOutput', bitNumber, state, firstArgLimits, secondArgLimits)

    def beep(self, duration):
        """

        :param duration:
        :return:

        """

        firstArgLimits = [0, 255]  # [1/10s]

        self.__sendCommand('beep', duration, firstArgLimits=firstArgLimits, useTwosComplement=False)

    def sendInfoEEPROM(self):
        """

        :return:

        """

        self.__sendCommand('sendInfoEEPROM', 240, useTwosComplement=False, useValueClipping=False)  # 240 is F0

    def sendInfoOn(self):
        """

        :return:

        """

        self.__sendCommand('sendInfoOn', 1, useValueClipping=False)  # 240 is F0

    def sendInfoOff(self):
        """

        :return:

        """

        self.__sendCommand('sendInfoOff', 0, useValueClipping=False)  # 240 is F0

    def sensorsOn(self):
        """

        :return:

        """

        self.__sendCommand('sensorsOn', 1, useValueClipping=False)

    def sensorsOff(self):
        """

        :return:

        """

        self.__sendCommand('sensorsOff', 1, useValueClipping=False)

    def turnOff(self):
        """

        :return:

        """

        self.__sendCommand('turnOff')
        # self.sock.close()

robot = Emir("eMIR-Yellow")
robot.move(33, 0)
robot.translate(10, 33)
robot.rotate(10, 33)
robot.setMinDistance(33)
robot.setMaxSpeed(33)
robot.setMaxRotation(33)
robot.resetAzimuth()
robot.resetCounters()
robot.setDigitalOutput(0, 0)
robot.setDigitalOutput(0, 1)
robot.setDigitalOutput(1, 0)
robot.setDigitalOutput(1, 1)
robot.setDigitalOutput(2, 0)
robot.setDigitalOutput(2, 1)
robot.setDigitalOutput(3, 0)
robot.setDigitalOutput(3, 1)
robot.setDigitalOutput(4, 0)
robot.setDigitalOutput(4, 10)
robot.setDigitalOutput(4, 15)
robot.beep(-10)
robot.beep(0)
robot.beep(255)
robot.beep(298)
robot.sensorsOff()
robot.sensorsOn()
robot.sendInfoEEPROM()
robot.sendInfoOff()
robot.sendInfoOn()
robot.stop()
robot.turnOff()

robot.connect()