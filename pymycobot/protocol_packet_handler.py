import time

TXPACKET_MAX_LEN = 250
RXPACKET_MAX_LEN = 250

# for Protocol Packet
PKT_HEADER0 = 0
PKT_HEADER1 = 1
PKT_ID = 2
PKT_LENGTH = 3
PKT_INSTRUCTION = 4
PKT_ERROR = 4
PKT_PARAMETER0 = 5

# Protocol Error bit
ERRBIT_VOLTAGE = 1
ERRBIT_ANGLE = 2
ERRBIT_OVERHEAT = 4
ERRBIT_OVERELE = 8
ERRBIT_OVERLOAD = 32

BROADCAST_ID = 0xFE  # 254
MAX_ID = 0xFC  # 252
SCS_END = 0

# Instruction for SCS Protocol
INST_PING = 1
INST_READ = 2
INST_WRITE = 3
INST_REG_WRITE = 4
INST_ACTION = 5
INST_SYNC_WRITE = 131  # 0x83
INST_SYNC_READ = 130  # 0x82

# Communication Result
COMM_SUCCESS = 0  # tx or rx packet communication success
COMM_PORT_BUSY = -1  # Port is busy (in use)
COMM_TX_FAIL = -2  # Failed transmit instruction packet
COMM_RX_FAIL = -3  # Failed get status packet
COMM_TX_ERROR = -4  # Incorrect instruction packet
COMM_RX_WAITING = -5  # Now recieving status packet
COMM_RX_TIMEOUT = -6  # There is no status packet
COMM_RX_CORRUPT = -7  # Incorrect status packet
COMM_NOT_AVAILABLE = -9  #



class protocol_packet_handler(object):
    def __init__(self, portHandler, protocol_end):
        #self.scs_setend(protocol_end)# SCServo bit end(STS/SMS=0, SCS=1)
        self.portHandler = portHandler
        self.scs_end = protocol_end
        
    def save_command(self):
        save_command = [255, 255, 254, 4, 3, 55, 0]
        save_command.append((sum(save_command[2:]) & 0xFF) ^ 0xFF)
        self.portHandler.write(save_command)
        self.portHandler.flush()

        
    def txPacket(self, txpacket):
            checksum = 0
            total_packet_length = txpacket[PKT_LENGTH] + 4  # 4: HEADER0 HEADER1 ID LENGTH

            # if self.portHandler.is_using:
            #     return COMM_PORT_BUSY
            # self.portHandler.is_using = True

            # # check max packet length
            # if total_packet_length > TXPACKET_MAX_LEN:
            #     self.portHandler.is_using = False
            #     return COMM_TX_ERROR

            # make packet header
            txpacket[PKT_HEADER0] = 0xFF
            txpacket[PKT_HEADER1] = 0xFF

            # add a checksum to the packet
            for idx in range(2, total_packet_length - 1):  # except header, checksum
                checksum += txpacket[idx]

            txpacket[total_packet_length - 1] = ~checksum & 0xFF

            #print "[TxPacket] %r" % txpacket

            # tx packet
            self.portHandler.flush()
            written_packet_length = self.portHandler.write(txpacket)
            # if total_packet_length != written_packet_length:
            #     self.portHandler.is_using = False
            #     return COMM_TX_FAIL

            return COMM_SUCCESS

    def txRxPacket(self, txpacket):
            rxpacket = None
            error = 0

            # tx packet
            result = self.txPacket(txpacket)
            if result != COMM_SUCCESS:
                return rxpacket, result, error

            # (ID == Broadcast ID) == no need to wait for status packet or not available
            # if (txpacket[PKT_ID] == BROADCAST_ID):
                # self.portHandler.is_using = False
                # return rxpacket, result, error

            # set packet timeout
            # if txpacket[PKT_INSTRUCTION] == INST_READ:
            #     self.portHandler.setPacketTimeout(txpacket[PKT_PARAMETER0 + 1] + 6)
            # else:
            #     self.portHandler.setPacketTimeout(6)  # HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM

            # rx packet
            result = -1
            while True:
                rxpacket, result = self.rxPacket()
                if result != COMM_SUCCESS or txpacket[PKT_ID] == rxpacket[PKT_ID]:
                    break

            if result == COMM_SUCCESS and txpacket[PKT_ID] == rxpacket[PKT_ID]:
                error = rxpacket[PKT_ERROR]

            return rxpacket, result, error

    def rxPacket(self):
        rxpacket = []

        result = COMM_TX_FAIL
        checksum = 0
        rx_length = 0
        wait_length = 6  # minimum length (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)
        t = time.time()
        while time.time() - t < 0.1:
            rxpacket.extend(self.portHandler.read(wait_length - rx_length))
            rx_length = len(rxpacket)
            if rx_length >= wait_length:
                # find packet header
                for idx in range(0, (rx_length - 1)):
                    if (rxpacket[idx] == 0xFF) and (rxpacket[idx + 1] == 0xFF):
                        break

                if idx == 0:  # found at the beginning of the packet
                    if (rxpacket[PKT_ID] > 0xFD) or (rxpacket[PKT_LENGTH] > RXPACKET_MAX_LEN) or (
                            rxpacket[PKT_ERROR] > 0x7F):
                        # unavailable ID or unavailable Length or unavailable Error
                        # remove the first byte in the packet
                        del rxpacket[0]
                        rx_length -= 1
                        continue

                    # re-calculate the exact length of the rx packet
                    if wait_length != (rxpacket[PKT_LENGTH] + PKT_LENGTH + 1):
                        wait_length = rxpacket[PKT_LENGTH] + PKT_LENGTH + 1
                        continue

                    # if rx_length < wait_length:
                    #     # check timeout
                    #     if self.portHandler.isPacketTimeout():
                    #         if rx_length == 0:
                    #             result = COMM_RX_TIMEOUT
                    #         else:
                    #             result = COMM_RX_CORRUPT
                    #         break
                    #     else:
                    #         continue

                    # calculate checksum
                    for i in range(2, wait_length - 1):  # except header, checksum
                        checksum += rxpacket[i]
                    checksum = ~checksum & 0xFF

                    # verify checksum
                    if rxpacket[wait_length - 1] == checksum:
                        result = COMM_SUCCESS
                    else:
                        result = COMM_RX_CORRUPT
                    break

                else:
                    # remove unnecessary packets
                    del rxpacket[0: idx]
                    rx_length -= idx

            # else:
            #     # check timeout
            #     if self.portHandler.isPacketTimeout():
            #         if rx_length == 0:
            #             result = COMM_RX_TIMEOUT
            #         else:
            #             result = COMM_RX_CORRUPT
            #         break
        else:
            None, None
        # self.portHandler.is_using = False
        return rxpacket, result
        
    def readTxRx(self, scs_id, address, length):
        txpacket = [0] * 8
        data = []

        if scs_id >= BROADCAST_ID:
            return data, COMM_NOT_AVAILABLE, 0

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = 4
        txpacket[PKT_INSTRUCTION] = INST_READ
        txpacket[PKT_PARAMETER0 + 0] = address
        txpacket[PKT_PARAMETER0 + 1] = length

        rxpacket, result, error = self.txRxPacket(txpacket)
        if result == COMM_SUCCESS:
            error = rxpacket[PKT_ERROR]

            data.extend(rxpacket[PKT_PARAMETER0 : PKT_PARAMETER0+length])

        return data, result, error

    def read1ByteTxRx(self, scs_id, address):
            data, result, error = self.readTxRx(scs_id, address, 1)
            data_read = data[0] if (result == COMM_SUCCESS) else 0
            return data_read, result, error
        
    def read2ByteTxRx(self, scs_id, address):
        data, result, error = self.readTxRx(scs_id, address, 2)
        data_read = self.scs_makeword(data[0], data[1]) if (result == COMM_SUCCESS) else 0
        return data_read, result, error
    
    def scs_makeword(self,a, b):
        if self.scs_end==0:
            return (a & 0xFF) | ((b & 0xFF) << 8)
        else:
            return (b & 0xFF) | ((a & 0xFF) << 8)
    def scs_tohost(self, a, b):
        if (a & (1<<b)):
            return -(a & ~(1<<b))
        else:
            return a
        
    def write1ByteTxRx(self, scs_id, address, data):
        data_write = [data]
        return self.writeTxRx(scs_id, address, 1, data_write)
    
    def writeTxRx(self, scs_id, address, length, data):
        if address in [6,7,18,13,14,21,22,23,26,27,81,24]:
            self.save_command()
        txpacket = [0] * (length + 7)

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = length + 3
        txpacket[PKT_INSTRUCTION] = INST_WRITE
        txpacket[PKT_PARAMETER0] = address

        txpacket[PKT_PARAMETER0 + 1: PKT_PARAMETER0 + 1 + length] = data[0: length]
        rxpacket, result, error = self.txRxPacket(txpacket)
        
        return result, error
    
    def write2ByteTxRx(self, scs_id, address, data):
        data_write = [self.scs_lobyte(data), self.scs_hibyte(data)]
        return self.writeTxRx(scs_id, address, 2, data_write)
    
    def scs_lobyte(self, w):
        if self.scs_end==0:
            return w & 0xFF
        else:
            return (w >> 8) & 0xFF

    def scs_hibyte(self, w):
        if self.scs_end==0:
            return (w >> 8) & 0xFF
        else:
            return w & 0xFF
        
    def ping(self, scs_id):
        model_number = 0
        error = 0

        txpacket = [0] * 6

        if scs_id >= BROADCAST_ID:
            return model_number, COMM_NOT_AVAILABLE, error

        txpacket[PKT_ID] = scs_id
        txpacket[PKT_LENGTH] = 2
        txpacket[PKT_INSTRUCTION] = INST_PING

        rxpacket, result, error = self.txRxPacket(txpacket)
        if result == COMM_SUCCESS:
            data_read, result, error = self.readTxRx(scs_id, 3, 2)  # Address 3 : Model Number
            if result == COMM_SUCCESS:
                model_number = self.scs_makeword(data_read[0], data_read[1])

        return result