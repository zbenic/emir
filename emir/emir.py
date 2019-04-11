import bluetooth
import threading
import time
import warnings

warnings.simplefilter('always', UserWarning)

VectorInt = [int]


class Emir:
    """
    Class for FMENA eMIR robot.
    Implemented methods can be used to achieve basic robot movement.
    Implementation of parsing of the return status message can also be used.
    """

    def __init__(self, name: str, numOfProximitySensors: int=6):
        """
        Constructor.

        Args:
            name: Name of the robot instance
            numOfProximitySensors: Number of the proximity sensors (default is 6)
        """

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
        self.statusMessage = None
        self.proximitySensors = [0] * numOfProximitySensors
        self.battery = 0
        self.lowBattery = False
        self.chargeVoltage = 0
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
        self.setSpeed = 0
        self.SetRotation = 0
        self.badCheckSum = 0
        self.statusWorker = None

    def __getRobotAddress(self):
        """
        Private method.
        Retrieves the bluetooth address from the robot whose bluetooth device is the same name as the self.name.
        The address is saved in the self.address.

        Returns:
            N/A
        """

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

    def __getRobotPort(self):
        """
        Private method.
        Updates self.port variable with one of the three eMIR robot port numbers, depending on the instance name.

        Returns:
            N/A
        """

        if self.name == "eMIR-Yellow":
            self.port = 1  # TODO: documentation says port num is 27
        elif self.name == "eMIR-Blue":
            self.port = 1  # TODO: documentation says port num is 34
        elif self.name == "eMIR-Red":
            self.port = 1  # TODO: documentation says port num is 50
        else:
            self.port = None

    def __getCheckSum(self, *args: int):
        """
        Private method.
        Returns sum(args) mod 2^8+1 (max. 8bit number + 1).

        Args:
            args: Variable number of integer arguments

        Returns:
            Checksum
        """

        argumentSum = 0
        for arg in args:
            argumentSum += arg

        return ((self.getMax8bitIntegerValue() + 1) - argumentSum % (self.getMax8bitIntegerValue() + 1)) % (self.getMax8bitIntegerValue() + 1)

    @staticmethod
    def getTwosComplement8bit(decimalBaseInteger: int):
        """
        Static method.
        Returns 8bit two's complement.

        Args:
            decimalBaseInteger: Integer number in decimal base

        Returns
            8 bit two's complement
        """

        if (decimalBaseInteger.bit_length() > 8) or (-128 > decimalBaseInteger or decimalBaseInteger > 127):
            warnings.warn("The number does not fit in 8 bits or it's outside of [-128, 127] interval!")
            return None
        else:
            return decimalBaseInteger if decimalBaseInteger >= 0 else 256 + decimalBaseInteger

    @staticmethod
    def clipValue(value, minValue, maxValue):
        """
        Static method.
        Returns value clipped by the min and max limits.

        Args:
            value: Input value
            minValue: Lower clipping bound
            maxValue: Upper clipping bound

        Returns:
            Clipped input value
        """

        return max(minValue, min(value, maxValue))

    @staticmethod
    def getMax8bitIntegerValue():
        """
        Static method.
        Returns maximal 8bit integer number.

        Returns:
            Maximal 8 bit integer number
        """

        return 2**8 - 1

    @staticmethod
    def stringOfHexToListOfDecIntegers(string: str):
        """
        Converts string to list of integers.

        Args:
            string: Input string

        Returns:
            List of integers in decimal base
        """

        groupSize = 2  # 2 strings in each group
        listOfHexStrings = [string[i:i + groupSize] for i in range(0, len(string), groupSize)]
        listOfDecimalIntegers = [int(hexStr, 16) for hexStr in listOfHexStrings]

        return listOfDecimalIntegers

    def __messageIsValid(self, *args):
        """
        Private method.
        Checks if incoming or outgoing message is valid, i.e. if the checksum is equal to zero.

        Args:
            args: Variable number of integer arguments

        Returns:
            Information if the message is valid
        """

        if type(args[0]) is list:
            argumentSum = sum(args[0])
        else:
            argumentSum = sum(args)

        if argumentSum % (self.getMax8bitIntegerValue() + 1) == 0:
            return True
        else:
            self.badCheckSum += 1
            return False

    def __sendCommand(self,
                      commandName: str,
                      firstArg: int = 0,
                      secondArg: int = 0,
                      firstArgLimits: VectorInt = [0, 0],
                      secondArgLimits: VectorInt = [0, 0],
                      useTwosComplement: bool = True,
                      useValueClipping: bool = True,
                      verbose: bool = False):
        """
        Private method.
        Generic method for sending commands to the robot.
        Command format is #NNaabbCS/, where:
            #  - message start
            NN - command number
            aa - first parameter
            bb - second parameter
            CS - checksum (NN+aa+bb+CS=0)
            /  - message end

        Args:
            commandName: The name of the command (move, translate, ...)
            firstArg: The first argument of the robot command
            econdArg: The second argument of the robot command
            firstArgLimits: First argument's numeric limits
            secondArgLimits: Second argument's numeric limits
            useTwosComplement: Flag for using two's complement
            useValueClipping: Flag for using value clipping
            verbose: If used, commands are also printed to the console

        Returns:
            N/A  # TODO: return bool if the command is successfully sent.
        """

        # get the command number from command name
        commandNumber = self.commands[commandName]
        commandNumberInt = int(commandNumber, 16)

        # value clipping
        if useValueClipping:
            firstArg = self.clipValue(firstArg, firstArgLimits[0], firstArgLimits[1])
            secondArg = self.clipValue(secondArg, secondArgLimits[0], secondArgLimits[1])

        # calculate 8 bit two's complement
        if useTwosComplement:
            firstArg = self.getTwosComplement8bit(firstArg)
            secondArg = self.getTwosComplement8bit(secondArg)

        # calculate checksum parameter of the command
        checkSum = self.__getCheckSum(commandNumberInt, firstArg, secondArg)

        # convert values form decimal to hex base
        firstArg_hex = format(firstArg, '02X')
        secondArg_hex = format(secondArg, '02X')
        checkSum_hex = format(checkSum, '02X')

        # send message if the message is valid (NN+aa+bb+CS=0)
        # raise warning in the case of invalid message
        if self.__messageIsValid(commandNumberInt, firstArg, secondArg, checkSum):
            if verbose:
                print("#" + commandNumber + firstArg_hex + secondArg_hex + checkSum_hex + "/")  # for debugging
            self.sock.send("#" + commandNumber + firstArg_hex + secondArg_hex + checkSum_hex + "/")
        else:
            warnings.warn("Command '" + commandName + "': checksum is not 0!")

    def __parseStatusMessage(self):
        """
        Private method.
        Parses received robot's status message.
        Updates corresponding variables with received data (self.battery, self.speed, self.angle, ...).

        Returns:
            N/A
        """

        statusMessage = self.statusMessage.decode('utf-8')
        if statusMessage[0] != '*':
            warnings.warn("Problem occurred with parsing the robot status message. First character is not '*', but " + statusMessage[0] + ".")
        if statusMessage[-1] != '/':
            warnings.warn("Problem occurred with parsing the robot status message. Last character is not '/', but " + statusMessage[-1] + ".")

        statusMessageParts = self.stringOfHexToListOfDecIntegers(statusMessage[1:-1])

        if not self.__messageIsValid(statusMessageParts):
            warnings.warn("Message '" + statusMessage + "' invalid: checksum is not 0!")

        # read proximity sensors data (bytes from 1:13) [cm]
        for sensorIndex in range(0, len(self.proximitySensors)):
            self.proximitySensors[sensorIndex] = int(statusMessage[(sensorIndex * 2 + 1):(sensorIndex * 2 + 1) + 2], 16)

        # read battery/charging voltage [V]
        msb = bool(format(int(statusMessage[13:15], 16), '0>8b')[0])
        messageBatteryValueBinary = format(int(statusMessage[13:15], 16), '0>8b')[1:]
        messageBatteryValueInt = int(messageBatteryValueBinary, 2)
        if msb:  # if MSB is 1
            if messageBatteryValueInt > 20:
                self.chargeVoltage = int(9 + messageBatteryValueInt / 17)
            else:
                self.chargeVoltage = 0
            self.charging = False if self.chargeVoltage < 9 else True
        else:
            self.battery = int(9 + messageBatteryValueInt / 17)
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
        self.rightMotorPwm = messageRightMotorPwmValueInt if messageRightMotorPwmValueInt < 128 else messageRightMotorPwmValueInt - 256

        # read work mode
        self.workMode = int(statusMessage[23:25], 16)

        # read digital input state
        self.digitalIn = int(statusMessage[25:27], 16)

        # read azimuth [deg]
        self.azimuth = int(statusMessage[27:29], 16) * 2  # TODO: check if multiplication with factor 2 is correct

        # read path [cm]
        messagePathValueInt = int(statusMessage[29:31], 16)
        self.path = int(messagePathValueInt * 1.03) if messagePathValueInt < 128 else int((messagePathValueInt - 256) * 1.03)

        # read angle [deg]
        messageAngleValueInt = int(statusMessage[31:33], 16)
        self.angle = int(messageAngleValueInt * 2.14) if messageAngleValueInt < 128 else int((messageAngleValueInt - 256) * 2.14)

    def connect(self):
        """
        Connects with the bluetooth module on the eMIR robot.
        If everything goes fine, self.sock is updated with the instance of created socket.
        If unable to connect, program exits with -1.

        Returns:
            N/A
        """

        self.__getRobotAddress()
        self.__getRobotPort()

        try:
            self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.sock.connect((self.address, self.port))
            print(self.name + " is connected.")
        except bluetooth.BluetoothError as error:
            self.sock.close()
            warnings.warn("Could not connect:", error)
            exit(-1)

    def getRobotStatus(self):
        """
        Receives and parses robot status message.
        If everything goes fine, self.update is set to True, else is False.

        The message format is *aabbccddeeffuuvvrrgghhmmjjkkppssCS/, where:\n
            *      - message start                                                  \n
            aa..ff - SHARP proximity sensors [cm]                                   \n
            uu     - battery or charging voltage (MSB=0, or MSB=1, respectively) [V]\n
            vv     - translation speed [%]                                          \n
            rr     - rotation speed [%]                                             \n
            gg     - left motor PWM [%]                                             \n
            hh     - right motor PWM [%]                                            \n
            mm     - work mode (see eMIR documentation)                             \n
            jj     - digital inputs status                                          \n
            kk     - azimuth [deg]                                                  \n
            pp     - path [cm]                                                      \n
            ss     - angle [deg/2]                                                  \n
            CS     - checksum (aa+bb+..+ss+CS=0)                                    \n
            /      - message end                                                    \n

        Returns:
            N/A
        """
        self.statusMessage = None  # TODO: comment this and the next line when testing offline
        self.statusMessage = self.sock.recv(128)  # TODO: check if 64bytes is enough
        messageStart = self.statusMessage.find(b'*')
        messageEnd = messageStart + self.statusMessage[messageStart:].find(b'/') + 1
        self.statusMessage = self.statusMessage[messageStart:messageEnd]
        if not self.statusMessage:
            # warnings.warn("Problem occurred while getting robot status message which resulted in an empty status message.")  # For debugging purposes
            self.update = False
        else:
            self.__parseStatusMessage()
            self.update = True

    def startReceivingRobotStatus(self, printMessages: bool = False):
        """
        Starts receiving robot status messages and refreshing robot internal status variables in parallel thread.

        Args:
            printMessages: True if internal status values are to be printed in console, false otherwise

        Returns:
            N/A
        """

        self.sendInfoOn()
        time.sleep(0.2)

        self.statusWorker = StatusMessageWorker(self, printMessages)
        self.statusWorker.daemon = True
        self.statusWorker.start()

    def stopReceivingRobotStatus(self):
        """
        Stops receiving robot status messages and refreshing robot internal status variables in parallel thread.

        Returns:
            N/A
        """

        self.sendInfoOff()
        time.sleep(0.2)

        self.statusWorker.stop()
        self.statusWorker.join()

    def stop(self):
        """
        Sends command to stop the robot in the current execution.

        Returns:
            N/A
        """

        self.__sendCommand('stop')

    def move(self, translationSpeed: int, rotationSpeed: int):
        """
        Sends "move" command to the robot with defined translation and rotational speed.

        Args:
            translationSpeed: Translation speed [%]
            rotationSpeed: Rotation speed [%]

        Returns:
            N/A
        """

        firstArgLimits = [-100, 100]  # [%]
        secondArgLimits = [-100, 100]  # [%]

        self.__sendCommand('move', translationSpeed, rotationSpeed, firstArgLimits, secondArgLimits)

    def translate(self, path: int, speed: int):
        """
        Sends "translate" command to the robot with defined path and translation speed.

        Args:
            path: Length for robot to travel [cm]
            speed: Translation speed [%]

        Returns:
            N/A
        """

        firstArgLimits = [-100, 100]  # [cm]
        secondArgLimits = [10, 100]  # [%]

        self.__sendCommand('translate', path, speed, firstArgLimits, secondArgLimits)

    def rotate(self, angle: int, speed: int):
        """
        Sends "rotate" command to the robot with defined angle and rotation speed.

        Args:
            angle: Robot rotation angle [deg]
            speed: Rotation speed [%]

        Returns:
            N/A
        """

        firstArgLimits = [-180, 180]  # [deg]
        secondArgLimits = [10, 100]  # [%]

        angle = int(angle / 2)

        self.__sendCommand('rotate', angle, speed, firstArgLimits, secondArgLimits)

    def setMinDistance(self, distance: int):
        """
        Sends command to the robot which sets minimal allowed distance between robot and obstacle.

        Args:
            distance: Minimal distance between the robot and obstacle [cm]

        Returns:
            N/A
        """

        firstArgLimits = [0, 50]  # [cm]

        self.__sendCommand('setMinDistance', distance, firstArgLimits=firstArgLimits)

    def setMaxSpeed(self, translationSpeed: int):
        """
        Sends command to the robot which sets maximal translation speed.

        Args:
            translationSpeed: Maximal robot translation speed [%]

        Returns:
            N/A
        """

        firstArgLimits = [10, 100]  # [%]

        self.__sendCommand('setMaxSpeed', translationSpeed, firstArgLimits=firstArgLimits)

    def setMaxRotation(self, rotationSpeed: int):
        """
        Sends command to the robot which sets maximal rotation speed.

        Args:
            rotationSpeed: Maximal robot rotation speed [%]

        Returns:
            N/A
        """

        firstArgLimits = [10, 100]  # [%]

        self.__sendCommand('setMaxRotation', rotationSpeed, firstArgLimits=firstArgLimits)

    def resetAzimuth(self):
        """
        Sends command to the robot which sets azimuth value to zero.

        Returns:
            N/A
        """

        self.__sendCommand('resetAzimuth')

    def resetCounters(self):
        """
        Sends command to the robot which sets left and right wheel encoder counters to zero.

        Returns:
            N/A
        """

        self.__sendCommand('resetCounters')

    def setDigitalOutput(self, bitNumber: int, state: int):
        """
        Sends command which sets digital inputs defined with the bitNumber argument to the desired state

        Args:
            bitNumber: Number of the bit (0, 1, 2, 3). If > 3, state argument sets multiple bits
            state: State of the bit defined with bitNumber

        Returns:
            N/A
        """

        firstArgLimits = [0, 4]

        if bitNumber > 3:
            secondArgLimits = [0, 15]
        else:
            secondArgLimits = [0, 1]

        self.__sendCommand('setDigitalOutput', bitNumber, state, firstArgLimits, secondArgLimits)

    def beep(self, duration: int):
        """
        Sends command which activates the horn for the defined duration.

        Args:
            duration: Duration for which to activate the horn (duration/10 s)

        Returns:
            N/A
        """

        firstArgLimits = [0, 255]  # [1/10s]

        self.__sendCommand('beep', duration, firstArgLimits=firstArgLimits, useTwosComplement=False)

    def sendInfoEEPROM(self):
        """
        Sends command which enables the EEPROM data to be returned via status message.

        Returns:
            N/A
        """

        self.__sendCommand('sendInfoEEPROM', 240, useTwosComplement=False, useValueClipping=False)  # 240 is F0

    def sendInfoOn(self):
        """
        Sends command which enables the robot to send status messages.

        Returns:
            N/A
        """

        self.__sendCommand('sendInfoOn', 1, useValueClipping=False)  # 240 is F0

    def sendInfoOff(self):
        """
        Sends command which disables the robot to send status messages.

        Returns:
            N/A
        """

        self.__sendCommand('sendInfoOff', 0, useValueClipping=False)  # 240 is F0

    def sensorsOn(self):
        """
        Sends command which enables robot's sensors.

        Returns:
            N/A
        """

        self.__sendCommand('sensorsOn', 1, useValueClipping=False)

    def sensorsOff(self):
        """
        Sends command which disables robot's sensors.

        Returns:
            N/A
        """

        self.__sendCommand('sensorsOff', 1, useValueClipping=False)

    def turnOff(self):
        """
        Sends command to turn off the robot.

        Returns:
            N/A
        """

        self.__sendCommand('turnOff')
        self.sock.close()
        # TODO: do thread cleanup

    def printStatusMessageValues(self):
        """
        Prints out values contained in the status message.

        Returns:
            N/A
        """

        for sensorIdx in range(len(self.proximitySensors)):
            print(self.name + " sensor " + str(sensorIdx) + ":" + str(self.proximitySensors[sensorIdx]))

        print(self.name + " battery voltage:" + str(self.battery))
        print(self.name + " charging voltage:" + str(self.chargeVoltage))
        print(self.name + " translation speed:" + str(self.speed))
        print(self.name + " rotation speed:" + str(self.rotation))
        print(self.name + " left motor PWM:" + str(self.leftMotorPwm))
        print(self.name + " right motor PWM:" + str(self.rightMotorPwm))
        print(self.name + " work mode:" + str(self.workMode))
        print(self.name + " digital input:" + str(self.digitalIn))
        print(self.name + " azimuth:" + str(self.azimuth))
        print(self.name + " path:" + str(self.path))
        print(self.name + " angle:" + str(self.angle))


class StatusMessageWorker(threading.Thread):
    """
    Class that represents a thread of control.
    Deals with receiving and parsing of robot status message and updating internal robot state variables.
    """

    def __init__(self, robot: Emir, printMessages: bool):
        """
        Constructor

        Args:
            robot: Instance of Emir class
            printMessages: True if internal status values are to be printed in console, false otherwise
        """
        threading.Thread.__init__(self)
        self.robot = robot
        self.printMessages = printMessages
        self.stop = threading.Event()

    def stop(self):
        """
        Signals an event which the main thread is waiting for.
        Used to stop the status message worker thread.

        Returns:
            N/A
        """
        self.stop.set()

    def stopped(self):
        """
        Indicates that we want to exit the status message worker thread.

        Returns:
            N/A
        """
        return self.stop.isSet()

    def run(self):
        """
        Method representing the thread's activity.
        Charged with receiving and parsing of the robot status message and updating robot internal state variables.

        Returns:
            N/A
        """
        while True:
            if self.stopped():
                print("Thread closed.")
                return
            try:
                self.robot.getRobotStatus()
                if self.printMessages:
                    self.robot.printStatusMessageValues()
                time.sleep(0.2)  # Refresh rate is ~5 Hz
            except:
                print("Error occurred when getting robot status message!")
