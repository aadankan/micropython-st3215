from machine import UART
import time

BROADCAST_ID = 0xFE # 254

INST_PING = 0x01
INST_READ = 0x02 # read register function
INST_WRITE = 0x03  # write register function
INST_REG_WRITE = 0x04
INST_REG_ACTION = 0x05
FACTORY_RESET = 0x06
INST_REBOOT = 0x08
INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83

ERROR_NO_RESPONSE = 0x01
ERROR_RESPONSE_TOO_SHORT = 0x02
ERROR_INVALID_HEADER = 0x03
ERROR_LENGTH_MISMATCH = 0x04
ERROR_CHECKSUM = 0x05

DEBUG = False


def log(*args, **kwargs):
    """Logs a message to the console."""
    if DEBUG:
        print(args, kwargs)
        


class ServoLowLevel:
    buf = b''

    def __init__(self, id, uart_num, tx_pin, rx_pin, baudrate):
        """
        Initializes the ServoLowLevel class.

        Parameters:
            id (int): The ID of the servo motor. If 0, performs broadcast ping to detect servo.
            uart_num (int): The UART number to use for communication.
            tx_pin (int): The TX (transmit) pin for UART communication.
            rx_pin (int): The RX (receive) pin for UART communication.
            baudrate (int): The baud rate for UART communication.
        """
        self.uart = UART(uart_num, baudrate=baudrate, bits=8, parity=None, tx=tx_pin, rx=rx_pin)
        if id == 0:
            id = self.broadcast_ping()
            if id == b'-1':
                raise ValueError("No servo responded to broadcast ping")
        self._id = id

    @property
    def register(self):
        """
        Extracts and returns register value from response buffer.

        Returns:
            bytes: Register data from the buffer.

        Raises:
            ValueError: If buffer is too short.
        """
        if len(self.buf) < 6:
            raise ValueError("Buffer too short to extract value")
        return self.buf[5:-1]

    def read_servo(self):
        """
        Reads incoming data from the UART interface.

        Returns:
            bytes: The data read from the servo or empty byte string if nothing is read.
        """
        self.buf = self.uart.read() or b''
        return self.buf if self.buf else b''

    def flush(self):
        """
        Flushes the UART buffer to discard any existing data.
        """
        self.uart.read()
        self.buf = b''

    def send_packet(self, instruction, parameters=None, broadcast=False):
        """
        Constructs and sends a packet to the servo motor.

        Parameters:
            instruction (int): The instruction byte (e.g., INST_PING, INST_READ).
            parameters (list, optional): Optional list of parameters for the instruction.
            broadcast (bool, optional): Whether to broadcast the packet to all servos. Defaults to False.
        """

        _id = BROADCAST_ID if broadcast else self._id
        packet = [0xFF, 0xFF, _id]
        length = 2

        args = []
        # Add parameters if provided
        if parameters:
            for param in parameters:
                i = 1
                if isinstance(param, int):
                    args.append(param)
                elif isinstance(param, list):
                    i = len(param)
                    args.extend(param)
                length += i

        packet.append(length)

        # Add the instruction to the packet
        packet.append(instruction)
        # Add the parameters to the packet
        packet.extend(args)
        
        # Calculate checksum
        checksum_data = packet[2:]
        checksum = (~sum(checksum_data)) & 0xFF
        packet.append(checksum)

        # Send the packet to the servo
        self.flush()
        self.uart.write(bytearray(packet))

        # Debug
        log("Sent:", "-".join(f"{b:02X}" for b in packet))
    
    def broadcast_ping(self) -> bytes:
        """
        Sends a ping to all servos and checks for a response.

        Returns:
            bytes: The ID of the responding servo or b'-1' if no response is received.
        """
        self.send_packet(INST_PING, broadcast=True)
        error = self.ack()
        if error == ERROR_LENGTH_MISMATCH:
            log("Comunication error: Length mismatch probably many servos are connected. Try connecting only one servo.")
            return b'-1'
        elif error or (len(self.buf) < 3):
            log("Comunication error: No response or invalid response")
            return b'-1'
        
        log(f"ID: {self.buf[2]}[{self.buf[2]:02X}]", )
        return self.buf[2]

    def iterate_ping(self):
        """
        Pings all possible servo IDs (1–254) to find connected servos.

        Returns:
            None
        """
        __id = self._id
        connected_ids = []
        for i in range(1, 255):
            time.sleep(0.01)
            self._id = i
            self.send_packet(INST_PING, )
            error = self.ack()
            if error or (len(self.buf) < 3):
                log(f"Id {i} not responding")
                continue
            connected_ids.append(i)
        
        for i in connected_ids:
            log(f"Id: {i}[{i:02X}] responding")
        self._id = __id
        
    def ack(self) -> int:
        """
        Reads and validates a response packet from the servo.

        Returns:
            int: Error code indicating the result of the acknowledgement:
                0 - Success
                ERROR_NO_RESPONSE - No response received
                ERROR_RESPONSE_TOO_SHORT - Response too short
                ERROR_INVALID_HEADER - Invalid header in response
                ERROR_LENGTH_MISMATCH - Declared length does not match actual
                ERROR_CHECKSUM - Checksum mismatch
        """
        response = self.read_servo()
        if not response:
            log("No response received")
            return ERROR_NO_RESPONSE
            
        log("Received:", "-".join(f"{b:02X}" for b in response))
        
        # Minimal response length: header(2) + ID(1) + length(1) + error(1) + checksum(1) = 6
        if len(response) < 6:
            log("Response too short")
            return ERROR_RESPONSE_TOO_SHORT
            
        # Check header
        if response[0] != 0xFF or response[1] != 0xFF:
            log("Invalid header")
            return ERROR_INVALID_HEADER
        
        # Verify length
        if len(response) != response[3] + 4:  # header(2) + ID(1) + length(1) + data + checksum(1)
            log("Length mismatch")
            return ERROR_LENGTH_MISMATCH
            
        # Verify checksum
        check_data = response[2:-1]  # everything after header and before checksum
        checksum = response[-1]
        if (~sum(check_data) & 0xFF) != checksum:
            log("Checksum error")
            return ERROR_CHECKSUM
        return 0


if __name__ == "__main__":
    DEBUG = True
    servo = ServoLowLevel(id=0, uart_num=1, tx_pin=19, rx_pin=18, baudrate=1000000)
    #servo.broadcast_ping()
    #servo.iterate_ping()
