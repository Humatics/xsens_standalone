"""Constant and messages definition for MT communication."""


class DeviceState:
    """State of the device"""

    # measurement state
    Measurement = 0
    # config state
    Config = 1


class MID:
    """Values for the message id (MID)"""

    # Error message, 1 data byte
    Error = 0x42
    # Warning message
    Warning = 0x43

    # State MID
    # Wake up procedure
    WakeUp = 0x3E
    # Wake up ack to put device in config mode
    WakeUpAck = 0x3F
    # Switch to config state
    GoToConfig = 0x30
    # Switch to measurement state
    GoToMeasurement = 0x10
    # Reset device
    Reset = 0x40

    # Informational messages
    # Request device id
    ReqDID = 0x00
    # DeviceID, 4 bytes: HH HL LH LL
    DeviceID = 0x01
    # Request product code in plain text
    ReqProductCode = 0x1C
    # Product code (max 20 bytes data)
    ProductCode = 0x1D
    # Request firmware revision
    ReqFWRev = 0x12
    # Firmware revision, 3 bytes: major minor rev
    FirmwareRev = 0x13

    # Device specific messages
    # Restore factory defaults
    RestoreFactoryDef = 0x0E

    # Baudrate, 1 byte
    # * The SetBaudrate message is deprecated on the MTi 600-series. Use
    #   SetPortConfig (0x8C) instead.
    # https://www.xsens.com/hubfs/Downloads/Manuals/MT_Low-Level_Documentation.pdf
    SetBaudrate = 0x18

    # Run the built-in self test (MTi-1/10/100 series)
    RunSelftest = 0x24
    # Self test results, 2 bytes
    SelftestAck = 0x25
    # Set state of OptionFlags (MTi-1/2/3), 4 + 4 bytes
    SetOptionFlags = 0x48
    # Location ID, 2 bytes, arbitrary, default is 0
    SetLocationID = 0x84
    # Set transmit delay (RS485 only), 2 bytes
    SetTransmitDelay = 0xDC

    # Synchronization messages
    # Synchronization settings (MTi-1/10/100 series only), N*12 bytes
    SetSyncSettings = 0x2C

    # Configuration messages
    # Request configuration
    ReqConfiguration = 0x0C
    # Configuration, 118 bytes
    Configuration = 0x0D
    # Extended output mode (MTi-10/100), 2 bytes, bit 4 for extended UART
    SetExtOutputMode = 0x86
    # Output configuration (MTi-1/10/100 series only), N*4 bytes
    SetOutputConfiguration = 0xC0
    # Configure NMEA data output (MTi-10/100), 2 bytes
    SetStringOutputType = 0x8E
    # Set sensor of local alignment quaternion
    SetAlignmentRotation = 0xEC


    # Data messages
    # Request MTData message (for 65535 skip factor)
    ReqData = 0x34
    # Legacy data packet
    MTData = 0x32
    # Newer data packet (MTi-10/100 series only)
    MTData2 = 0x36

    # Filter messages
    # Reset orientation, 2 bytes
    ResetOrientation = 0xA4
    # Request or set UTC time from sensor (MTI-G and MTi-10/100 series)
    SetUTCTime = 0x60
    # Set correction ticks to UTC time
    AdjustUTCTime = 0xA8
    # UTC Time (MTI-G and MTi-10/100 series), 12 bytes
    UTCTime = 0x61
    # Request the available XKF scenarios on the device
    ReqAvailableScenarios = 0x62
    # Available Scenarios
    AvailableScenarios = 0x63
    # Current XKF scenario, 2 bytes
    SetCurrentScenario = 0x64
    # Magnitude of the gravity used for the sensor fusion mechanism, 4 bytes
    SetGravityMagnitude = 0x66
    # Latitude, Longitude and Altitude for local declination and gravity
    # (MTi-10/100 series only), 24 bytes
    SetLatLonAlt = 0x6E
    # Initiate No Rotation procedure (not on MTi-G), 2 bytes
    SetNoRotation = 0x22

    SetCanConfig = 0xE6
    SetCanOutputConfig = 0xE8

    SetGnssLeverArm = 0x68
    SetGnssReceiverSettings = 0xAC
    SetHardwareVersion = 0x1E

    XbuxInterface = 0x01
    RtcmInterface = 0x06
    SetPortConfig = 0x8C


class DeprecatedMID:
    """Deprecated message Ids."""

    # Informational messages
    # Compatibility for XBus Master users
    InitMT = 0x02
    InitMTResults = 0x03
    # Request data length according to current configuration
    ReqDataLength = 0x0A
    # Data Length, 2 bytes
    DataLength = 0x0B
    # Request GPS status (MTi-G only)
    ReqGPSStatus = 0xA6
    # GPS status (MTi-G only)
    GPSStatus = 0xA7

    # Synchronization messages
    # SyncIn setting (MTi only), (1+) 2 or 4 bytes depending on request
    SetSyncInSettings = 0xD6
    # SyncOut setting (MTi/MTi-G only), (1+) 2 or 4 bytes depending on request
    SetSyncOutSettings = 0xD8

    # Configuration messages
    # Skip factor (MTi/MTi-G only), 2 bytes
    SetOutputSkipFactor = 0xD4
    # Object alignment matrix, 9*4 bytes
    SetObjectAlignment = 0xE0

    # XKF Filter messages
    # Heading (MTi only), 4 bytes
    SetHeading = 0x82
    # Lever arm of the GPSin sensor coordinates (MTi-G and MTi-700 only),
    # 3*4 bytes
    SetLeverArmGPS = 0x68
    # Magnetic declination (MTi-G only), 4 bytes
    SetMagneticDeclination = 0x6A
    # Latitude, Longitude and Altitude for local declination and gravity
    # Processing flags (not on firmware 2.2 or lower for MTi/MTi-g), 1 byte
    SetProcessingFlags = 0x20


def getName(cls, value):
    '''Return the name of the first found member of class cls with given
    value.'''
    for k, v in cls.__dict__.items():
        if v == value:
            return k
    return ''


def getMIDName(mid):
    '''Return the name of a message given the message id.'''
    name = getName(MID, mid)
    if name:
        return name
    if mid & 1:
        name = getName(MID, mid - 1)
        if name:
            return name + 'Ack'
    return 'unknown MID'


class Baudrates(object):
    """Baudrate information and conversion."""

    # Baudrate mapping between ID and value
    Baudrates = [
        (0x80, 921600),
        (0x0A, 921600),
        (0x0C, 2000000),  # 2 Mbaud support
        (0x00, 460800),
        (0x01, 230400),
        (0x02, 115200),
        (0x03, 76800),
        (0x04, 57600),
        (0x05, 38400),
        (0x06, 28800),
        (0x07, 19200),
        (0x08, 14400),
        (0x09, 9600),
        (0x0B, 4800),
        (0x80, 921600),
    ]

    Can_Baudrates = [
        (0x0C, 1000000),
        (0x0B, 800000),
        (0x0A, 500000),
        (0x00, 250000),
        (0x01, 125000),
        (0x02, 100000),
        (0x03, 83300),
        (0x04, 62500),
        (0x05, 50000),
        (0x06, 33300),
        (0x07, 20000),
        (0x08, 10000),
        (0x09, 5000),
    ]

    @classmethod
    def get_can_BRID(cls, baudrate):
        """Get can_baudrate id for a given baudrate."""
        for brid, br in cls.Can_Baudrates:
            if baudrate == br:
                return brid
        raise MTException("unsupported baudrate.")

    @classmethod
    def get_can_BR(cls, baudrate_id):
        """Get baudrate for a given baudrate id."""
        for brid, br in cls.Can_Baudrates:
            if baudrate_id == brid:
                return br
        raise MTException("unknown baudrate id.")

    @classmethod
    def get_BRID(cls, baudrate):
        """Get baudrate id for a given baudrate."""
        for brid, br in cls.Baudrates:
            if baudrate == br:
                return brid
        raise MTException("unsupported baudrate.")

    @classmethod
    def get_BR(cls, baudrate_id):
        """Get baudrate for a given baudrate id."""
        for brid, br in cls.Baudrates:
            if baudrate_id == brid:
                return br
        raise MTException("unknown baudrate id.")


class OutputMode:
    """Values for the output mode."""

    Temp = 0x0001
    Calib = 0x0002
    Orient = 0x0004
    Auxiliary = 0x0008
    Position = 0x0010
    Velocity = 0x0020
    Status = 0x0800
    RAWGPS = 0x1000  # supposed to be incompatible with previous
    RAW = 0x4000  # incompatible with all except RAWGPS


class OutputSettings:
    """Values for the output settings."""

    Timestamp_None = 0x00000000
    Timestamp_SampleCnt = 0x00000001
    Timestamp_UTCTime = 0x00000002
    OrientMode_Quaternion = 0x00000000
    OrientMode_Euler = 0x00000004
    OrientMode_Matrix = 0x00000008
    CalibMode_AccGyrMag = 0x00000000
    CalibMode_GyrMag = 0x00000010
    CalibMode_AccMag = 0x00000020
    CalibMode_Mag = 0x00000030
    CalibMode_AccGyr = 0x00000040
    CalibMode_Gyr = 0x00000050
    CalibMode_Acc = 0x00000060
    CalibMode_Mask = 0x00000070
    DataFormat_Float = 0x00000000
    DataFormat_12_20 = 0x00000100  # not supported yet
    DataFormat_16_32 = 0x00000200  # not supported yet
    DataFormat_Double = 0x00000300  # not supported yet
    AuxiliaryMode_NoAIN1 = 0x00000400
    AuxiliaryMode_NoAIN2 = 0x00000800
    PositionMode_LLA_WGS84 = 0x00000000
    VelocityMode_MS_XYZ = 0x00000000
    Coordinates_NED = 0x80000000


class XDIGroup:
    """Values for the XDI groups."""

    Temperature = 0x0800
    Timestamp = 0x1000
    OrientationData = 0x2000
    Pressure = 0x3000
    Acceleration = 0x4000
    Position = 0x5000
    GNSS = 0x7000
    AngularVelocity = 0x8000
    GPS = 0x8800
    SensorComponentReadout = 0xA000
    AnalogIn = 0xB000  # deprecated
    Magnetic = 0xC000
    Velocity = 0xD000
    Status = 0xE000


class MTException(Exception):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return self.message


class MTTimeoutException(MTException):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return 'Timeout: %s' % self.message


class MTErrorMessage(MTException):
    ErrorCodes = {
        0x03: 'Period sent is not within valid range',
        0x04: 'Message sent is invalid',
        0x1E: 'Timer overflow. This can be caused by a too high output frequency or sending too much data to the MT during measurement',
        0x20: 'Baud rate requested is not within valid range',
        0x21: 'Parameter sent is invalid or not within range',
        0x28: 'Device Error - try updating the firmware; extra device error contains 5 bytes',
        0x29: 'Data overflow, the device generates more data than the bus communication can handle (baud rate may be too low)',
        0x2A: 'Buffer overflow, the sample buffer of the device was full during a communication outage',
    }

    def __init__(self, code):
        self.code = code
        self.message = self.ErrorCodes.get(code, 'Unknown error: 0x%02X' % code)

    def __str__(self):
        return 'Error message 0x%02X: %s' % (self.code, self.message)


class MTWarningMessage(MTException):
    WarningCodes = {
        0x0191: (
            'A configuration item was refused by the GNSS receiver',
            [
                'The configured baud rate (see SetGnssReceiverSettings) is not supported by the GNSS receiver',
                'The configured GNSS receiver model (see SetGnssReceiverSettings) does not match the connected GNSS receiver',
            ],
        ),
        0x0192: (
            'Communication with the GNSS receiver has timed out',
            [
                'No GNSS receiver connected',
                'The configured GNSS receiver model (see SetGnssReceiverSettings) does not match the connected GNSS receiver',
            ],
        ),
        0x0193: (
            'Communication with the GNSS receiver failed',
            [
                'Generic GNSS receiver communication error; various root causes are possible. Please contact Xsens support for further troubleshooting'
            ],
        ),
        0x0195: (
            'Communication with the GNSS receiver was lost',
            [
                'The GNSS receiver stopped providing data, e.g. due to power loss',
                'Disconnection of the data line(s) between the MTi and the GNSS receiver',
            ],
        ),
        0x0197: (
            'Incomplete dataset delivered by the GNSS receiver',
            [
                'The GNSS receiver is not sending at least one of the required input messages. Refer to your MTi product\'s datasheet for a list of required GNSS messages',
                'The GNSS receiver\'s output rate is too low',
            ],
        ),
    }

    def __init__(self, code, description_string=""):
        self.code = code
        self.description_string = description_string
        if code in self.WarningCodes:
            self.description, self.possible_causes = self.WarningCodes[code]
            self.message = f'{self.description}'
        else:
            self.description = f'Unknown warning: 0x{code:04X}'
            self.possible_causes = []
            self.message = self.description

    def __str__(self):
        result = f'Warning 0x{self.code:04X}: {self.description}'
        if self.description_string:
            result += f' - {self.description_string}'
        if self.possible_causes:
            result += f'\nPossible causes:\n'
            for cause in self.possible_causes:
                result += f'  - {cause}\n'
        return result.rstrip()
