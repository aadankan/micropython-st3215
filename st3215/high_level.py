from typing import List, Optional
from .low_level import ServoLowLevel, INST_READ, INST_WRITE, FACTORY_RESET, INST_REG_ACTION, INST_REG_WRITE, INST_REBOOT

DEFAULT_S_RXD = 18
DEFAULT_S_TXD = 19
DEFAULT_UART_NUM = 1
DEFAULT_BAUDRATE = 1000000
DEFAULT_SERVO_ID = 1

REG_FIRMWARE_MAJOR_VERSION       = 0x00  # Firmware major version number
REG_FIRMWARE_SUB_VERSION         = 0x01  # Firmware sub version number
REG_SERVO_MAJOR_VERSION          = 0x03  # Servo Main Version Number
REG_SERVO_SUB_VERSION            = 0x04  # Servo sub version number
REG_ID                           = 0x05  # ID
REG_BAUD_RATE                    = 0x06  # Baud rate
REG_RETURN_DELAY                 = 0x07  # Return delay
REG_STATUS_RETURN_LEVEL          = 0x08  # Response status level
REG_MIN_ANGLE_LIMIT              = 0x09  # Minimum Angle Limitation
REG_MAX_ANGLE_LIMIT              = 0x0B  # Maximum Angle Limitation
REG_MAX_TEMPERATURE              = 0x0D  # Maximum Temperature Limit
REG_MAX_VOLTAGE                  = 0x0E  # Maximum input voltage
REG_MIN_VOLTAGE                  = 0x0F  # Minimum input voltage
REG_MAX_TORQUE                   = 0x10  # Maximum torque
REG_PHASE                        = 0x12  # Phase
REG_UNLOAD_CONDITION             = 0x13  # Unloading condition
REG_LED_ALARM                    = 0x14  # LED Alarm condition
REG_P_COEFFICIENT                = 0x15  # P Proportionality coefficient
REG_D_COEFFICIENT                = 0x16  # D Differential coefficient
REG_I_COEFFICIENT                = 0x17  # I Integral coefficient
REG_MIN_STARTUP_FORCE            = 0x18  # Minimum startup force
REG_CW_DEAD_ZONE                 = 0x1A  # Clockwise insensitive area
REG_CCW_DEAD_ZONE                = 0x1B  # Counterclockwise insensitive region
REG_PROTECTION_CURRENT           = 0x1C  # Protection current
REG_ANGULAR_RESOLUTION           = 0x1E  # Angular resolution
REG_POSITION_CORRECTION          = 0x1F  # Position correction
REG_OPERATION_MODE               = 0x21  # Operation mode
REG_PROTECTIVE_TORQUE            = 0x22  # Protective torque
REG_PROTECTION_TIME              = 0x23  # Protection time
REG_OVERLOAD_TORQUE              = 0x24  # Overload torque
REG_SPEED_CLOSED_LOOP_P_COEFF    = 0x25  # Speed closed loop P proportional coefficient
REG_OVER_CURRENT_PROTECTION_TIME = 0x26  # Over current protection time
REG_VELOCITY_CLOSED_LOOP_I_COEFF = 0x27  # Velocity closed loop I integral coefficient
REG_TORQUE_SWITCH                = 0x28  # Torque switch
REG_ACCELERATION                 = 0x29  # Acceleration
REG_TARGET_LOCATION              = 0x2A  # Target location
REG_RUNNING_TIME                 = 0x2C  # Running time
REG_RUNNING_SPEED                = 0x2E  # Running speed
REG_TORQUE_LIMIT                 = 0x30  # Torque limit
REG_LOCK_MARK                    = 0x37  # Lock mark
REG_CURRENT_LOCATION             = 0x38  # Current location
REG_CURRENT_SPEED                = 0x3A  # Current speed
REG_CURRENT_LOAD                 = 0x3C  # Current load
REG_CURRENT_VOLTAGE              = 0x3E  # Current voltage
REG_CURRENT_TEMPERATURE          = 0x3F  # Current temperature
REG_ASYNCHRONOUS_WRITE_FLAG      = 0x40  # Asynchronous write flag
REG_SERVO_STATUS                 = 0x41  # Servo status
REG_MOBILE_SIGN                  = 0x42  # Mobile sign
REG_CURRENT_CURRENT              = 0x45  # Current current

REG_MAP = {
 REG_FIRMWARE_MAJOR_VERSION: {'fn': 'Firmware major version number', 'bytes': 1, 'initial': 3, 'storage': 'EPROM', 'access': 'R', 'min': -1, 'max': -1, 'multiplayer': 1, 'unit': 'number', 'description': 'Firmware major version number'},
 REG_FIRMWARE_SUB_VERSION: {'fn': 'Firmware sub version number', 'bytes': 1, 'initial': 6, 'storage': 'EPROM', 'access': 'R', 'min': -1, 'max': -1, 'multiplayer': 1, 'unit': 'number', 'description': 'Firmware sub version number'},
 REG_SERVO_MAJOR_VERSION: {'fn': 'Servo Main Version Number', 'bytes': 1, 'initial': 9, 'storage': 'EPROM', 'access': 'R', 'min': -1, 'max': -1, 'multiplayer': 1, 'unit': 'number', 'description': 'Servo Main Version Number'},
 REG_SERVO_SUB_VERSION: {'fn': 'Servo sub version number', 'bytes': 1, 'initial': 3, 'storage': 'EPROM', 'access': 'R', 'min': -1, 'max': -1, 'multiplayer': 1, 'unit': 'number', 'description': 'Servo sub version number'},
 REG_ID: {'fn': 'ID', 'bytes': 1, 'initial': 1, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 253, 'multiplayer': 1, 'unit': 'number', 'description': 'Unique identification code on the bus. Duplicate ID number is not allowed on the same bus,254 (oxfe) is the broadcast ID, broadcast does not return a reply packet“'},
 REG_BAUD_RATE: {'fn': 'Baud rate', 'bytes': 1, 'initial': 0, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 7, 'multiplayer': 1, 'unit': 'no', 'description': '0-7 represents baud rate as follows:\n1000000，500000，250000，128000，115200，76800，57600，38400'},
 REG_RETURN_DELAY: {'fn': 'Return delay', 'bytes': 1, 'initial': 0, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 2, 'unit': 'us', 'description': 'The minimum unit is 2us; and the maximum set return delay is 254 * 2 = 508us'},
 REG_STATUS_RETURN_LEVEL: {'fn': 'Response status level', 'bytes': 1, 'initial': 1, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 1, 'multiplayer': 1, 'unit': 'no', 'description': '0: except for read instruction and Ping instruction; other instructions do not return reply packet;1: Returns a reply packet for all instructions“'},
 REG_MIN_ANGLE_LIMIT: {'fn': 'Minimum Angle Limitation', 'bytes': 2, 'initial': 0, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 4094, 'multiplayer': 1, 'unit': 'step', 'description': 'Set the minimum limit of motion stroke; the value is less than the maximum angle limit; and this value is 0 when the multi cycle absolute position control is carried out'},
 REG_MAX_ANGLE_LIMIT: {'fn': 'Maximum Angle Limitation', 'bytes': 2, 'initial': 4095, 'storage': 'EPROM', 'access': 'RW', 'min': 1, 'max': 4095, 'multiplayer': 1, 'unit': 'step', 'description': 'Set the maximum limit of motion stroke; which is greater than the minimum angle limit; and the value is 0 when the multi turn absolute position control is adopted.\n'},
 REG_MAX_TEMPERATURE: {'fn': 'Maximum Temperature Limit', 'bytes': 1, 'initial': 70, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 100, 'multiplayer': 1, 'unit': '°C', 'description': 'The maximum operating temperature limit; if set to 70; the maximum temperature is 70 ℃; and the setting accuracy is 1 ℃'},
 REG_MAX_VOLTAGE: {'fn': 'Maximum input voltage', 'bytes': 1, 'initial': 80, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 0.1, 'unit': 'V', 'description': 'If the maximum input voltage is set to 80; the maximum working voltage is limited to 8.0V and the setting accuracy is 0.1V'},
 REG_MIN_VOLTAGE: {'fn': 'Minimum input voltage', 'bytes': 1, 'initial': 40, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 0.1, 'unit': 'V', 'description': 'If the minimum input voltage is set to 40; the minimum working voltage is limited to 4.0V and the setting accuracy is 0.1V'},
 REG_MAX_TORQUE: {'fn': 'Maximum torque', 'bytes': 2, 'initial': 1000, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 1000, 'multiplayer': 0.1, 'unit': '%', 'description': 'Set the maximum output torque limit of the servo; and set 1000 = 100% * locked torque;Power on assigned to address 48 torque limit“'},
 REG_PHASE: {'fn': 'Phase', 'bytes': 1, 'initial': 12, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 1, 'unit': 'no', 'description': 'Special function byte; which cannot be modified without special requirements. See special byte bit analysis for details'},
 REG_UNLOAD_CONDITION: {'fn': 'Unloading condition', 'bytes': 1, 'initial': 44, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 1, 'unit': 'no', 'description': 'Bit0 Bit1 bit2 bit3 bit4 bit5 corresponding bit is set to enable corresponding protection'},
 REG_LED_ALARM: {'fn': 'LED Alarm condition', 'bytes': 1, 'initial': 47, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 1, 'unit': 'no', 'description': 'The corresponding bit of temperature current angle overload of voltage sensor is set to 0 to close the corresponding protection“Bit0 Bit1 bit2 bit3 bit4 bit5 corresponding bit is set to enable flashing alarm\nThe corresponding bit of temperature current angle overload of voltage sensor is set to 0 to turn off flashing light alarm'},
 REG_P_COEFFICIENT: {'fn': 'P Proportionality coefficient', 'bytes': 1, 'initial': 32, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 1, 'unit': 'no', 'description': 'Proportional factor of control motor'},
 REG_D_COEFFICIENT: {'fn': 'D Differential coefficient', 'bytes': 1, 'initial': 32, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 1, 'unit': 'no', 'description': 'Differential coefficient of control motor'},
 REG_I_COEFFICIENT: {'fn': 'I Integral coefficient', 'bytes': 1, 'initial': 0, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 1, 'unit': 'no', 'description': 'Integral coefficient of control motor'},
 REG_MIN_STARTUP_FORCE: {'fn': 'Minimum startup force', 'bytes': 2, 'initial': 16, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 1000, 'multiplayer': 0.1, 'unit': '%', 'description': 'Set the minimum output starting torque of servo and set 1000 = 100% * locked torque'},
 REG_CW_DEAD_ZONE: {'fn': 'Clockwise insensitive area', 'bytes': 1, 'initial': 1, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 32, 'multiplayer': 1, 'unit': 'step', 'description': 'The minimum unit is a minimum resolution angle'},
 REG_CCW_DEAD_ZONE: {'fn': 'Counterclockwise insensitive region', 'bytes': 1, 'initial': 1, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 32, 'multiplayer': 1, 'unit': 'step', 'description': 'The minimum unit is a minimum resolution angle'},
 REG_PROTECTION_CURRENT: {'fn': 'Protection current', 'bytes': 2, 'initial': 500, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 511, 'multiplayer': 6.5, 'unit': 'mA', 'description': 'The maximum current can be set at 3255ma'},
 REG_ANGULAR_RESOLUTION: {'fn': 'Angular resolution', 'bytes': 1, 'initial': 1, 'storage': 'EPROM', 'access': 'RW', 'min': 1, 'max': 100, 'multiplayer': 1, 'unit': 'no', 'description': 'For the amplification factor of minimum resolution angle (degree / step); the number of control turns can be extended by modifying this value'},
 REG_POSITION_CORRECTION: {'fn': 'Position correction', 'bytes': 2, 'initial': 0, 'storage': 'EPROM', 'access': 'RW', 'min': -2047, 'max': 2047, 'multiplayer': 1, 'unit': 'step', 'description': 'Bit11 is the direction bit; indicating the positive and negative directions. Other bits can represent the range of 0-2047 steps'},
 REG_OPERATION_MODE: {'fn': 'Operation mode', 'bytes': 1, 'initial': 0, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 2, 'multiplayer': 1, 'unit': 'no', 'description': '0: position servo mode\n\n1: The motor is in constant speed mode; which is controlled by parameter 0x2e; and the highest bit 15 is the direction bit\n\n2: PWM open-loop speed regulation mode; with parameter 0x2c running time parameter control; bit11 as direction bit\n\n3: In step servo mode; the number of step progress is represented by parameter 0x2a; and the highest bit 15 is the direction bit“'},
 REG_PROTECTIVE_TORQUE: {'fn': 'Protective torque', 'bytes': 1, 'initial': 20, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 0.1, 'unit': '%', 'description': 'After entering the overload protection; the output torque; if set to 20; means 20% of the maximum torque'},
 REG_PROTECTION_TIME: {'fn': 'Protection time', 'bytes': 1, 'initial': 200, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 10, 'unit': 'ms', 'description': 'The timing time when the current load output exceeds the overload torque and remains. If 200 is set to 2 seconds; the maximum can be set to 2.5 seconds'},
 REG_OVERLOAD_TORQUE: {'fn': 'Overload torque', 'bytes': 1, 'initial': 80, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 0.1, 'unit': '%', 'description': 'The maximum torque threshold of starting overload protection time meter; if set to 80; means 80% of the maximum torque'},
 REG_SPEED_CLOSED_LOOP_P_COEFF: {'fn': 'Speed closed loop P proportional coefficient', 'bytes': 1, 'initial': 10, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 1, 'unit': 'no', 'description': 'In the motor constant speed mode (mode 1); the speed loop proportional coefficient'},
 REG_OVER_CURRENT_PROTECTION_TIME: {'fn': 'Over current protection time', 'bytes': 1, 'initial': 200, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 10, 'unit': 'ms', 'description': 'The maximum setting is 254 * 10ms = 2540ms'},
 REG_VELOCITY_CLOSED_LOOP_I_COEFF: {'fn': 'Velocity closed loop I integral coefficient', 'bytes': 1, 'initial': 10, 'storage': 'EPROM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 1, 'unit': 'no', 'description': 'In the motor constant speed mode (mode 1); the speed loop integral coefficient'},
 REG_TORQUE_SWITCH: {'fn': 'Torque switch', 'bytes': 1, 'initial': 0, 'storage': 'SRAM', 'access': 'RW', 'min': 0, 'max': 128, 'multiplayer': 1, 'unit': 'no', 'description': 'Write 0: turn off torque output; write 1: turn on torque output; write 128: current position correction is 2048'},
 REG_ACCELERATION: {'fn': 'Acceleration', 'bytes': 1, 'initial': 0, 'storage': 'SRAM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 100, 'unit': 'step/s^2', 'description': 'If it is set to 10; the speed will be changed by 1000 steps per second'},
 REG_TARGET_LOCATION: {'fn': 'Target location', 'bytes': 2, 'initial': 0, 'storage': 'SRAM', 'access': 'RW', 'min': -32766, 'max': 32766, 'multiplayer': 1, 'unit': 'step', 'description': 'Each step is a minimum resolution angle; absolute position control mode; the maximum corresponding to the maximum effective angle\n\n'},
 REG_RUNNING_TIME: {'fn': 'Running time', 'bytes': 2, 'initial': 0, 'storage': 'SRAM', 'access': 'RW', 'min': 0, 'max': 1000, 'multiplayer': 0.1, 'unit': '%', 'description': 'Running time'},
 REG_RUNNING_SPEED: {'fn': 'Running speed', 'bytes': 2, 'initial': 0, 'storage': 'SRAM', 'access': 'RW', 'min': 0, 'max': 254, 'multiplayer': 1, 'unit': 'step/s', 'description': 'Number of steps in unit time (per second); 50 steps / second = 0.732 RPM (cycles per minute)'},
 REG_TORQUE_LIMIT: {'fn': 'Torque limit', 'bytes': 2, 'initial': 1000, 'storage': 'SRAM', 'access': 'RW', 'min': 0, 'max': 1000, 'multiplayer': 0.1, 'unit': '%', 'description': 'The initial value of power on is assigned by the maximum torque (0x10); which can be modified by the user to control the output of the maximum torque'},
 REG_LOCK_MARK: {'fn': 'Lock mark', 'bytes': 1, 'initial': 0, 'storage': 'SRAM', 'access': 'RW', 'min': 0, 'max': 1, 'multiplayer': 1, 'unit': 'no', 'description': 'Write 0 closes the write lock; and the value written to EPROM address is saved after power failure.\nWrite 1 opens the write lock; and the value written to EPROM address is not saved after power failure“'},
 REG_CURRENT_LOCATION: {'fn': 'Current location', 'bytes': 2, 'initial': 0, 'storage': 'SRAM', 'access': 'R', 'min': -1, 'max': -1, 'multiplayer': 1, 'unit': 'step', 'description': 'In the absolute position control mode; the maximum value corresponds to the maximum effective angle'},
 REG_CURRENT_SPEED: {'fn': 'Current speed', 'bytes': 2, 'initial': 0, 'storage': 'SRAM', 'access': 'R', 'min': -1, 'max': -1, 'multiplayer': 1, 'unit': 'step/s', 'description': 'Feedback the current speed of motor rotation; the number of steps in unit time (per second)'},
 REG_CURRENT_LOAD: {'fn': 'Current load', 'bytes': 2, 'initial': 0, 'storage': 'SRAM', 'access': 'R', 'min': -1, 'max': -1, 'multiplayer': 0.1, 'unit': '%', 'description': 'Voltage duty cycle of current control output drive motor'},
 REG_CURRENT_VOLTAGE: {'fn': 'Current voltage', 'bytes': 1, 'initial': 0, 'storage': 'SRAM', 'access': 'R', 'min': -1, 'max': -1, 'multiplayer': 0.1, 'unit': 'V', 'description': 'Current servo working voltage'},
 REG_CURRENT_TEMPERATURE: {'fn': 'Current temperature', 'bytes': 1, 'initial': 0, 'storage': 'SRAM', 'access': 'R', 'min': -1, 'max': -1, 'multiplayer': 1, 'unit': '°C', 'description': 'Current internal operating temperature of the servo'},
 REG_ASYNCHRONOUS_WRITE_FLAG: {'fn': 'Asynchronous write flag', 'bytes': 1, 'initial': 0, 'storage': 'SRAM', 'access': 'R', 'min': -1, 'max': -1, 'multiplayer': 1, 'unit': 'no', 'description': 'When using asynchronous write instruction; flag bit'},
 REG_SERVO_STATUS: {'fn': 'Servo status', 'bytes': 1, 'initial': 0, 'storage': 'SRAM', 'access': 'R', 'min': -1, 'max': -1, 'multiplayer': 1, 'unit': 'no', 'description': 'Bit0 Bit1 bit2 bit3 bit4 bit5 corresponding bit is set to 1; indicating that the corresponding error occurs;Voltage sensor temperature current angle overload corresponding bit 0 is no phase error.'},
 REG_MOBILE_SIGN: {'fn': 'Mobile sign', 'bytes': 1, 'initial': 0, 'storage': 'SRAM', 'access': 'R', 'min': -1, 'max': -1, 'multiplayer': 1, 'unit': 'no', 'description': 'When the servo is moving; it is marked as 1; and when the servo is stopped; it is 0'},
 REG_CURRENT_CURRENT: {'fn': 'Current current', 'bytes': 2, 'initial': 0, 'storage': 'SRAM', 'access': 'R', 'min': -1, 'max': -1, 'multiplayer': 6.5, 'unit': 'mA', 'description': 'The maximum measurable current is 500 * 6.5ma = 3250ma'}
}

STATUS_REGISTERS = [REG_FIRMWARE_MAJOR_VERSION, REG_FIRMWARE_SUB_VERSION, REG_SERVO_MAJOR_VERSION, REG_SERVO_SUB_VERSION, REG_ID, REG_BAUD_RATE, REG_RETURN_DELAY, REG_STATUS_RETURN_LEVEL, REG_MIN_ANGLE_LIMIT, REG_MAX_ANGLE_LIMIT, REG_MAX_TEMPERATURE, REG_MAX_VOLTAGE, REG_MIN_VOLTAGE, REG_MAX_TORQUE, REG_PHASE, REG_UNLOAD_CONDITION, REG_LED_ALARM, REG_P_COEFFICIENT, REG_D_COEFFICIENT, REG_I_COEFFICIENT, REG_MIN_STARTUP_FORCE, REG_CW_DEAD_ZONE, REG_CCW_DEAD_ZONE, REG_PROTECTION_CURRENT, REG_ANGULAR_RESOLUTION, REG_POSITION_CORRECTION, REG_OPERATION_MODE, REG_PROTECTIVE_TORQUE, REG_PROTECTION_TIME, REG_OVERLOAD_TORQUE, REG_SPEED_CLOSED_LOOP_P_COEFF, REG_OVER_CURRENT_PROTECTION_TIME, REG_VELOCITY_CLOSED_LOOP_I_COEFF]
STATE_REGISTERS = [REG_TORQUE_SWITCH, REG_ACCELERATION, REG_TARGET_LOCATION, REG_RUNNING_TIME, REG_RUNNING_SPEED, REG_TORQUE_LIMIT, REG_LOCK_MARK, REG_CURRENT_LOCATION, REG_CURRENT_SPEED, REG_CURRENT_LOAD, REG_CURRENT_VOLTAGE, REG_CURRENT_TEMPERATURE, REG_ASYNCHRONOUS_WRITE_FLAG, REG_SERVO_STATUS, REG_MOBILE_SIGN, REG_CURRENT_CURRENT]


class HexUtils:
    def get_16_bit_value(self, value) -> List[int]:
        """
        Converts a 16-bit signed integer to a list of two bytes.

        Parameters:
            value (int): The 16-bit signed integer to convert.

        Returns:
            list[int]: Two-byte representation of the value.
        """
        if value < 0:
            value = -value
            value |= (1 << 15)

        return [value & 0xFF, (value >> 8) & 0xFF]

    def extract_signed_int_from_bytes(self, byte_data: bytes, sign_bit_position: Optional[int] = None, lsb_first: bool = True) -> int:
        """
        Extracts a signed integer from byte data.

        Parameters:
            byte_data (bytes): The bytes to extract the integer from.
            sign_bit_position (int, optional): The bit position used for the sign.
            lsb_first (bool): Whether the byte order is LSB first.

        Returns:
            int: The extracted signed integer.
        """
        if not byte_data:
            raise ValueError("byte_data cannot be empty")

        integer_value = 0
        bytes_iter = enumerate(byte_data) if lsb_first else enumerate(reversed(byte_data))

        for i, byte in bytes_iter:
            integer_value |= byte << (8 * i)

        #  If sign_bit_position is not provided, use the last bit of the last byte
        sign_bit_position = sign_bit_position or (len(byte_data) * 8 - 1)

        # Check if the sign bit is set
        if integer_value & (1 << sign_bit_position):
            integer_value = -(integer_value & ~(1 << sign_bit_position))

        return integer_value


class ServoReadUtils:

    def read_register(self, reg: int) -> Optional[int]:
        """
        Reads the value of a given register.

        Parameters:
            reg (int): The register address to read from.

        Returns:
            int or None: The processed value from the register or None on error.
        """
        length = REG_MAP[reg]['bytes']
        multiplayer = REG_MAP[reg]['multiplayer']
        self.send_packet(INST_READ, [reg, length])
        error = self.ack()
        if error:
            return None
        return self.extract_signed_int_from_bytes(self.register) * multiplayer

    def read_registers(self, registers: list, format: str='list') -> list | dict:
        """
        Reads multiple registers and returns their values.

        Parameters:
            registers (list): List of register addresses to read.
            format (str): Output format: 'list' or 'dict'.

        Returns:
            list or dict: Register values as list or dictionary.
        """
        if format == 'list':
            vals = []
            for reg in registers:
                val = self.read_register(reg)
                vals.append(val)
            return vals
        if format == 'dict':
            ret = []
            for reg in registers:
                val = self.read_register(reg)
                ret.append({'fn': REG_MAP[reg]['fn'], 'value': val, 'unit': REG_MAP[reg]['unit']})
            return ret

    def get_mode(self):
        """Returns the current operation mode."""
        return self.read_register(REG_OPERATION_MODE)
    
    def get_position(self) -> int:
        """Returns the current position."""
        return self.read_register(REG_CURRENT_LOCATION)

    def get_speed(self):
        """Returns the current speed."""
        return self.read_register(REG_CURRENT_SPEED)

    def get_load(self):
        """Returns the current load."""
        return self.read_register(REG_CURRENT_LOAD)

    def get_voltage(self):
        """Returns the current voltage."""
        return self.read_register(REG_CURRENT_VOLTAGE)

    def get_current(self):
        """Returns the current consumption."""
        return self.read_register(REG_CURRENT_CURRENT)

    def get_temperature(self):
        """Returns the current temperature."""
        return self.read_register(REG_CURRENT_TEMPERATURE)
    
    def get_protection_status(self) -> dict:
        """
        Returns the protection status of the servo.

        Returns:
            dict: Dictionary with individual protection flags.
        """
        status = self.read_register(REG_SERVO_STATUS)
        return {
            'overload_protection': bool(status & 0x01),
            'overtemperature protection': bool(status & 0x02),
            'stall_protection': bool(status & 0x04),
            'overvoltage_protection': bool(status & 0x08),
            'overcurrent_protection': bool(status & 0x10),
        }

    def get_state(self, format: str='dict') -> dict | list:
        """
        Returns the state of the servo.

        Parameters:
            format (str): 'dict' or 'list'.

        Returns:
            dict or list: The current state information.
        """
        return self.read_registers(STATE_REGISTERS, format=format)

    def get_status(self, format: str='dict') -> dict | list:
        """
        Returns the status of the servo.

        Parameters:
            format (str): 'dict' or 'list'.

        Returns:
            dict or list: The current status information.
        """
        return self.read_registers(STATUS_REGISTERS, format=format)


class ServoWriteUtils:

    def validate_register_value(self, reg: int, value: int):
        """
        Validates a value against the register's constraints.

        Parameters:
            reg (int): Register address.
            value (int): Value to validate.
        """
        reg_info = REG_MAP[reg]
        if not (reg_info['min'] <= value <= reg_info['max']):
            raise ValueError(f"Value {value} for register {reg} is out of bounds ({reg_info['min']} - {reg_info['max']})")

    def validate(self, value, min, max):
        """
        Validates whether a value is within the specified bounds.

        Parameters:
            value (int): Value to check.
            min (int): Minimum allowed value.
            max (int): Maximum allowed value.
        """
        if value is None:
            raise ValueError("Value cannot be None")
        if not (min <= value <= max):
            raise ValueError(f"Value {value} is out of bounds ({min} - {max})")

    def set_register(self, reg: int, value: int, async_write: bool = False) -> int:
        """
        Writes a value to a register.

        Parameters:
            reg (int): Register address.
            value (int): Value to write.
            async_write (bool): Whether to queue the command.

        Returns:
            int: 0 if successful, otherwise an error code.
        """
        write = INST_REG_WRITE if async_write else INST_WRITE
        self.validate_register_value(reg, value)
        self.send_packet(write, [reg, value])
        error = self.ack()
        if error:
            return error
        return 0

    def async_set_register(self, reg: int, value: int) -> int:
        """
        Writes a value to a register asynchronously.

        Parameters:
            reg (int): Register address.
            value (int): Value to write.

        Returns:
            int: Result of the write.
        """
        return self.set_register(reg, value, async_write=True)

    def set_id(self, new_id: int):
        """Sets a new ID for the servo."""
        return self.set_register(REG_ID, new_id)

    def lock_mark(self):
        """Locks servo mark for configuration protection."""
        return self.set_register(REG_LOCK_MARK, 0x00)
    
    def unlock_mark(self):
        """Unlocks servo mark for configuration."""
        return self.set_register(REG_LOCK_MARK, 0x01)

    def tork_lock(self):
        """Enables torque output."""
        return self.set_register(REG_TORQUE_SWITCH, 0x01)
    
    def tork_unlock(self):
        """Disables torque output."""
        return self.set_register(REG_TORQUE_SWITCH, 0x00)

    def tork_auto(self):
        """
        Write 0: turn off torque output; write 1: turn on torque output; write 128: current position correction is 2048
        """
        return self.set_register(REG_TORQUE_SWITCH, 128)
    
    def servo_mode(self):
        """Sets the servo to position control mode."""
        return self.set_register(REG_OPERATION_MODE, 0x00)
    
    def speed_mode(self):
        """Sets the servo to speed control mode."""
        return self.set_register(REG_OPERATION_MODE, 0x01)
    
    def pwm_mode(self):
        """Sets the servo to PWM control mode."""
        return self.set_register(REG_OPERATION_MODE, 0x02)
    
    def step_mode(self):
        """Sets the servo to step mode."""
        return self.set_register(REG_OPERATION_MODE, 0x03)

    def set_speed(self, speed: int, reverse: bool = False) -> int:
        """
        Sets the speed of the servo.

        Parameters:
            speed (int): Speed value.
            reverse (bool): Whether the speed is in reverse.

        Returns:
            int: Result of the operation.
        """
        # if reverse:
        #   speed |= (1 << 15)
        return self.set_register(REG_RUNNING_SPEED, speed)
    
    def move_target_location(self, position, delta_time=None, speed=None) -> int:
        """
        Moves the servo to a target location.

        Parameters:
            position (int): Target position.
            delta_time (int, optional): Time delay (SC model).
            speed (int, optional): Movement speed (ST model).

        Returns:
            int: Result of the operation.
        """
        if position < 0:
            position = -position
            position |= (1 << 15)

        delta_time = delta_time or 0  # parametre only for SC model
        speed = speed or 150  # parametre only for ST model
         
        self.validate(speed, 0, 1600)
        self.validate(position, 0, 4095)
        self.validate(delta_time, 0, 2000)
        data = [REG_TARGET_LOCATION, self.get_16_bit_value(position), self.get_16_bit_value(delta_time), self.get_16_bit_value(speed)]
        self.send_packet(INST_WRITE, data)
        return self.ack()

    def move_acceleration(self, position: int, acceleration: int=None, speed: int=None) -> int:
        """
        Moves the servo using acceleration profile.

        Parameters:
            position (int): Target position.
            acceleration (int, optional): Acceleration value.
            speed (int, optional): Speed value.

        Returns:
            int: Result of the operation.
        """
        if position < 0:
            position = -position
            position |= (1 << 15)

        speed = speed or 1023
        acceleration = acceleration or 50

        self.validate(position, 0, 4095)
        self.validate(acceleration, 0, 255)
        self.validate(speed, 0, 1600)
        data = [REG_ACCELERATION, acceleration, self.get_16_bit_value(position), self.get_16_bit_value(0), self.get_16_bit_value(speed)]
        self.send_packet(INST_WRITE, data)
        return self.ack()
        

class ST3215(ServoLowLevel, ServoReadUtils, ServoWriteUtils, HexUtils):

    def __init__(self, id=DEFAULT_SERVO_ID, uart_num=DEFAULT_UART_NUM, tx_pin=DEFAULT_S_TXD, rx_pin=DEFAULT_S_RXD, baudrate=DEFAULT_BAUDRATE):
        """
        Initializes the ST3215 servo class.

        Parameters:
            id (int): The servo ID.
            uart_num (int): UART number.
            tx_pin (int): TX pin.
            rx_pin (int): RX pin.
            baudrate (int): Communication baud rate.
        """
        super().__init__(id, uart_num, tx_pin, rx_pin, baudrate)
        ack = self.ping()
        if ack != 0:
            raise ValueError(f"Failed to ping servo with ID {id}. Error code: {ack}")

    def sync(self):
        """
        Sends a sync action command to the servo.
        """
        self.send_packet(INST_REG_ACTION, [0x01])

    def reset(self):
        """
        Resets the servo to its factory settings.
        """
        self.send_packet(FACTORY_RESET, [0x01])
    
    def reboot(self):
        """
        Reboots the servo.
        """
        self.send_packet(INST_REBOOT, [0x01])
    
    def ping(self) -> int:
        """
        Pings the servo to check if it is responsive.

        Returns:
            int: 0 if successful, or an error code.
        """
        self.send_packet(INST_READ, [REG_ID, 0x01])
        return self.ack()
    
    def version(self) -> dict:
        """
        Returns the firmware and servo version.

        Returns:
            dict: Firmware and servo version info.
        """
        firmware_major = self.read_register(REG_FIRMWARE_MAJOR_VERSION)
        firmware_minor = self.read_register(REG_FIRMWARE_SUB_VERSION)
        servo_major = self.read_register(REG_SERVO_MAJOR_VERSION)
        servo_minor = self.read_register(REG_SERVO_SUB_VERSION)
        return {"firmware": f"{firmware_major}.{firmware_minor}", "servo": f"{servo_major}.{servo_minor}"}
    
    def reg_map(self) -> dict:
        """
        Returns the register map of the servo.

        Returns:
            dict: Register metadata.
        """
        return REG_MAP

