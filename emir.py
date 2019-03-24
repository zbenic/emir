import bluetooth
from typing import List
import warnings
warnings.simplefilter('always', UserWarning)

VectorInt = List[int]

class Emir:
    def __init__(self, name):
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
                        ('sensorsOff', 'sensorsOn'): '05',
                         'setDigitalOutput':         '06',
                        ('sendInfoOn', 'sendInfoOff', 'sendInfoEEPROM'): '10',
                         'setMinDistance':           '11',
                         'setMaxSpeed':              '12',
                         'setMaxRotation':           '13',
                         'resetAzimuth':             '14',
                         'resetCounters':            '15',
                         'turnOff':                  'FF'
                        }

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

        return (self.__getMax8bitIntegerValue() + 1) - argumentSum % (self.__getMax8bitIntegerValue() + 1)

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

    def __commandIsValid(self, *args: int):
        argumentSum = 0
        for arg in args:
            argumentSum += arg

        if argumentSum % (self.__getMax8bitIntegerValue() + 1) == 0:
            return True
        else:
            return False

    def __sendCommand(self, commandName: str, firstArg: int = 0, secondArg: int = 0, firstArgLimits: VectorInt = [0, 0], secondArgLimits: VectorInt = [0, 0], useTwosComplement: bool = True):
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
        :return:
        """

        # get the command number from command name
        commandNumber = self.commands[commandName]
        commandNumberInt = int(commandNumber, 16)

        # value clipping
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
        if self.__commandIsValid(commandNumberInt, firstArg, secondArg, checkSum):
            print("#" + commandNumber + firstArg_hex + secondArg_hex + checkSum_hex + "/")
            # self.sock.send("#" + commandNumber + firstArgument_hex + secondArgument_hex + checkSum_hex + "/")
        else:
            warnings.warn("Command '" + commandName + "': checksum is not 0!")


    def connect(self):
        self.__getRobotAddress()
        self.__getRobotService()
        self.__getRobotPort()
        protocol = self.__getRobotProtocol()

        if protocol is not None:
            self.sock = bluetooth.BluetoothSocket(protocol)
            self.sock.connect((self.address, self.port))
        else:
            warnings.warn("Unknown protocol. Only works with RFCOMM or L2CAP protocols.")

    def stop(self):
        raise NotImplementedError

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

        duration = self.__clipValue(duration, firstArgLimits[0], firstArgLimits[1])

        self.__sendCommand('beep', duration, firstArgLimits=firstArgLimits, useTwosComplement=False)


    def sendInfoEEPROM(self):
        raise NotImplementedError

    def sendInfoOn(self):
        raise NotImplementedError

    def sendInfoOff(self):
        raise NotImplementedError

    def sensorsOn(self):
        raise NotImplementedError

    def sensorsOff(self):
        raise NotImplementedError

    def turnOff(self):
        raise NotImplementedError

robot = Emir("Galaxy S6")
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

robot.connect()