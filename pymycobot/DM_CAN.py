import time
from time import sleep
import numpy as np
from enum import IntEnum
from struct import unpack
from struct import pack




class Motor:
    def __init__(self, MotorType, SlaveID, MasterID):
        self.Pd = float(0)
        self.Vd = float(0)
        self.state_q = float(0)
        self.state_dq = float(0)
        self.state_tau = float(0)
        self.SlaveID = SlaveID
        self.MasterID = MasterID
        self.MotorType = MotorType
        self.isEnable = False
        self.NowControlMode = Control_Type.MIT
        self.temp_param_dict = {}
        self.err = 0
        self.t_mos = 0
        self.t_coil = 0
        self.recv_count = 0
        self.req_count = 0

    def recv_data(self, q: float, dq: float, tau: float, err: int = 0, t_mos: int = 0, t_coil: int = 0, is_enabled: bool = False):
        self.state_q = q
        self.state_dq = dq
        self.state_tau = tau
        self.err = err
        self.t_mos = t_mos
        self.t_coil = t_coil
        self.isEnable = is_enabled
        self.recv_count += 1

    def getPosition(self):
        """
        get the position of the motor （）
        ：DM，
        `MotorControl.refresh_motor_status(motor)` ，Motor object。
        ，。
        :return: the position of the motor （）
        """
        return self.state_q

    def getVelocity(self):
        """
        get the velocity of the motor （）
        ： `refresh_motor_status` 。
        ，。
        :return: the velocity of the motor （）
        """
        return self.state_dq

    def getTorque(self):
        """
        get the torque of the motor （）
        ： `refresh_motor_status` 。
        ，。
        :return: the torque of the motor （）
        """
        return self.state_tau

    def getTemp(self):
        """ MOS  (T_MOS, T_Coil)"""
        return self.t_mos, self.t_coil

    def getError(self):
        """"""
        return self.err

    def getParam(self, RID):
        """
        get the parameter of the motor ，
        :param RID: DM_variable 
        :return: the parameter of the motor 
        """
        if RID in self.temp_param_dict:
            return self.temp_param_dict[RID]
        else:
            return None

class MotorControl:
    send_data_frame = np.array(
        [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0x00, 0x08, 0x00,
         0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)
    Limit_Param = [
        [12.5,    30,  10],  # DM4310
        [12.5,    50,  10],  # DM4310_48
        [12.5,    10,  28],  # DM4340
        [12.5,    10,  28],  # DM4340_48
        [12.5,    45,  20],  # DM6006
        [12.5,    45,  40],  # DM8006
        [12.5,    45,  54],  # DM8009
        [12.5,    25, 200],  # DM10010L
        [12.5,    20, 200],  # DM10010
        [12.5,   280,   1],  # DMH3510
        [12.5,    45,  10],  # DMG6215
        [12.5,    45,  10],  # DMH6220
        [12.5 ,   10 , 12],  # DMJH11
        [12.566,  20, 120],  # DM6248P
        [12.566,  50,   5],  # DM3507
        [12.5,   45,  10],  # DM2325
    ]

    def __init__(self, serial_device, debug=False):
        """
        define MotorControl object 
        :param serial_device: serial object 
        """
        self.debug = debug
        self.serial_ = serial_device
        self.motors_map = dict()
        self.last_serial_logs = {"write": [], "read": []}
        self.data_save = bytes()  # save data
        if self.serial_.is_open:  # open the serial port
            serial_device.close()
        self.serial_.open()

    def controlMIT(self, DM_Motor, kp: float, kd: float, q: float, dq: float, tau: float):
        """
        MIT Control Mode Function MITcontrol mode
        :param DM_Motor: Motor object Motor object
        :param kp: kp
        :param kd:  kd
        :param q:  position  
        :param dq:  velocity  
        :param tau: torque  
        :return: None
        """
        if DM_Motor.SlaveID not in self.motors_map:
            print("controlMIT ERROR : Motor ID not found")
            return
        kp_uint = float_to_uint(kp, 0, 500, 12)
        kd_uint = float_to_uint(kd, 0, 5, 12)
        MotorType = DM_Motor.MotorType
        Q_MAX = self.Limit_Param[MotorType][0]
        DQ_MAX = self.Limit_Param[MotorType][1]
        TAU_MAX = self.Limit_Param[MotorType][2]
        q_uint = float_to_uint(q, -Q_MAX, Q_MAX, 16)
        dq_uint = float_to_uint(dq, -DQ_MAX, DQ_MAX, 12)
        tau_uint = float_to_uint(tau, -TAU_MAX, TAU_MAX, 12)
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        data_buf[0] = (q_uint >> 8) & 0xff
        data_buf[1] = q_uint & 0xff
        data_buf[2] = dq_uint >> 4
        data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf)
        data_buf[4] = kp_uint & 0xff
        data_buf[5] = kd_uint >> 4
        data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf)
        data_buf[7] = tau_uint & 0xff
        self.__send_data(DM_Motor.SlaveID, data_buf)
        self.recv()  # receive the data from serial port

    def control_delay(self, DM_Motor, kp: float, kd: float, q: float, dq: float, tau: float, delay: float):
        """
        MIT Control Mode Function with delay MITcontrol mode
        :param DM_Motor: Motor object Motor object
        :param kp: kp
        :param kd: kd
        :param q:  position  
        :param dq:  velocity  
        :param tau: torque  
        :param delay: delay time  
        """
        self.controlMIT(DM_Motor, kp, kd, q, dq, tau)
        sleep(delay)

    def control_Pos_Vel(self, Motor, P_desired: float, V_desired: float):
        """
        control the motor in position and velocity control mode control mode
        :param Motor: Motor object Motor object
        :param P_desired: desired position 
        :param V_desired: desired velocity 
        :return: None
        """
        if Motor.SlaveID not in self.motors_map:
            print("Control Pos_Vel Error : Motor ID not found")
            return
        motorid = 0x100 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        P_desired_uint8s = float_to_uint8s(P_desired)
        V_desired_uint8s = float_to_uint8s(V_desired)
        data_buf[0:4] = P_desired_uint8s
        data_buf[4:8] = V_desired_uint8s
        self.__send_data(motorid, data_buf)
        time.sleep(0.001)
        self.recv()  # receive the data from serial port

    def control_Vel(self, Motor, Vel_desired):
        """
        control the motor in velocity control mode control mode
        :param Motor: Motor object Motor object
        :param Vel_desired: desired velocity 
        """
        Motor.Vd = float(Vel_desired)
        if Motor.SlaveID not in self.motors_map:
            print("control_VEL ERROR : Motor ID not found")
            return
        motorid = 0x200 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        Vel_desired_uint8s = float_to_uint8s(Vel_desired)
        data_buf[0:4] = Vel_desired_uint8s
        self.__send_data(motorid, data_buf)
        self.recv()  # receive the data from serial port

    def control_pos_force(self, Motor, Pos_des: float, Vel_des, i_des):
        """
        control the motor in EMIT control mode 
        :param Pos_des: desired position rad   rad
        :param Vel_des: desired velocity rad/s   100
        :param i_des: desired current rang 0-10000 10000
        ：，
        """
        if Motor.SlaveID not in self.motors_map:
            print("control_pos_vel ERROR : Motor ID not found")
            return
        motorid = 0x300 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        Pos_desired_uint8s = float_to_uint8s(Pos_des)
        data_buf[0:4] = Pos_desired_uint8s
        Vel_uint = np.uint16(Vel_des)
        ides_uint = np.uint16(i_des)
        data_buf[4] = Vel_uint & 0xff
        data_buf[5] = Vel_uint >> 8
        data_buf[6] = ides_uint & 0xff
        data_buf[7] = ides_uint >> 8
        self.__send_data(motorid, data_buf)
        self.recv()  # receive the data from serial port

    def control_Pos_Vel_CSP(self, Motor, P_desired: float, V_desired: float):#JH11
        """
        control the motor in position and velocity control mode control mode
        :param Motor: Motor object Motor object
        :param P_desired: desired position 
        :param V_desired: desired velocity 
        :return: None
        """
        if Motor.SlaveID not in self.motors_map:
            print("Control Pos_Vel Error : Motor ID not found")
            return
        motorid = 0x400 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        P_desired_uint8s = float_to_uint8s(P_desired)
        V_desired_uint8s = float_to_uint8s(V_desired)
        data_buf[0:4] = P_desired_uint8s
        data_buf[4:8] = V_desired_uint8s
        self.__send_data(motorid, data_buf)
        # time.sleep(0.001)
        self.recv()  # receive the data from serial port

    def control_Vel_CSP(self, Motor, Vel_desired):#JH11
        """
        control the motor in velocity control mode control mode
        :param Motor: Motor object Motor object
        :param Vel_desired: desired velocity 
        """
        if Motor.SlaveID not in self.motors_map:
            print("control_VEL ERROR : Motor ID not found")
            return
        motorid = 0x500 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        Vel_desired_uint8s = float_to_uint8s(Vel_desired)
        data_buf[0:4] = Vel_desired_uint8s
        self.__send_data(motorid, data_buf)
        self.recv()  # receive the data from serial port

    def control_Tor_CSP(self, Motor, Tor_desired):#JH11
        """
        control the motor in velocity control mode control mode
        :param Motor: Motor object Motor object
        :param Vel_desired: desired velocity 
        """
        if Motor.SlaveID not in self.motors_map:
            print("control_VEL ERROR : Motor ID not found")
            return
        motorid = 0x600 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        Tor_desired_uint8s = float_to_uint8s(Tor_desired)
        data_buf[0:4] = Tor_desired_uint8s
        self.__send_data(motorid, data_buf)
        self.recv()  # receive the data from serial port

    def enable(self, Motor):
        """
        enable motor enable motor
        wait a few seconds after power on before enabling
        :param Motor: Motor object Motor object
        """
        self.__control_cmd(Motor, np.uint8(0xFC))
        sleep(0.1)
        self.recv()  # receive the data from serial port

    def set_motors_state(self, motors_list, state):
        """
        Batch set enable/disable state and VEL mode for multiple motors.
        :param motors_list: list of Motor objects
        :param state: 1 to enable, 0 to disable
        """
        if state == 1:
            for m in motors_list:
                self.enable(m)
                self.switchControlMode(m, Control_Type.VEL)
                m.isEnable = True
        else:
            for m in motors_list:
                self.disable(m)
                m.isEnable = False

    def enable_old(self, Motor ,ControlMode):
        """
        enable motor old firmware enable motor (legacy firmware compatibility)
        legacy firmware requires offset
        wait a few seconds after power on before enabling
        :param Motor: Motor object Motor object
        """
        data_buf = np.array([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc], np.uint8)
        enable_id = ((int(ControlMode)-1) << 2) + Motor.SlaveID
        self.__send_data(enable_id, data_buf)
        sleep(0.1)
        self.recv()  # receive the data from serial port

    def disable(self, Motor):
        """
        disable motor disable motor
        :param Motor: Motor object Motor object
        """
        self.__control_cmd(Motor, np.uint8(0xFD))
        sleep(0.01)
        self.recv()

    def set_zero_position(self, Motor):
        """
        set the zero position of the motor set motor zero position
        :param Motor: Motor object Motor object
        """
        was_enabled = Motor.isEnable
        self.disable(Motor)
        sleep(0.05)
        self.__control_cmd(Motor, np.uint8(0xFE))
        sleep(0.2)  # Allow time for zero calibration to complete
        self.recv()
        if was_enabled:
            self.enable(Motor)
        # Refresh to ensure encoder has settled, then force zero
        self.refresh_motor_status(Motor)
        Motor.state_q = 0.0

    def recv(self):
        # wait up to 20ms for data (at least 16 bytes representing 1 packet)
        start_time = time.time()
        while self.serial_.in_waiting < 16 and (time.time() - start_time) < 0.020:
            time.sleep(0.0005)
        # append remaining unparsed data
        read_all_bytes = self.serial_.read_all()
        if hasattr(self, 'last_serial_logs') and read_all_bytes:
            self.last_serial_logs["read"].append(" ".join([f"{b:02X}" for b in read_all_bytes]))
        if getattr(self, 'debug', False) in (True, 1) and read_all_bytes:
            import datetime
            now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            hex_str = " ".join([f"{b:02X}" for b in read_all_bytes])
            if True:
                print(f"{now} DEBU [pymycobot.dm_can] _read: {hex_str}")
        data_recv = b''.join([self.data_save, read_all_bytes])
        # print(data_recv)
        packets = self.__extract_packets(data_recv)
        for packet in packets:
            data = packet[7:15]
            CANID = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
            CMD = packet[1]
            self.__process_packet(data, CANID, CMD)

    def recv_set_param_data(self):
        # wait up to 10ms for data (at least 16 bytes representing 1 packet)
        start_time = time.time()
        while self.serial_.in_waiting < 16 and (time.time() - start_time) < 0.010:
            time.sleep(0.0005)
        data_recv = self.serial_.read_all()
        if hasattr(self, 'last_serial_logs') and data_recv:
            self.last_serial_logs["read"].append(" ".join([f"{b:02X}" for b in data_recv]))
        if getattr(self, 'debug', False) in (True, 1) and data_recv:
            import datetime
            now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            hex_str = " ".join([f"{b:02X}" for b in data_recv])
            if True:
                print(f"{now} DEBU [pymycobot.dm_can] _read: {hex_str}")
        packets = self.__extract_packets(data_recv)
        for packet in packets:
            data = packet[7:15]
            CANID = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
            CMD = packet[1]
            self.__process_set_param_packet(data, CANID, CMD)

    def __process_packet(self, data, CANID, CMD):
        if CMD == 0x11:
            if CANID != 0x00:
                if CANID in self.motors_map:
                    q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                    dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                    tau_uint = np.uint16(((data[4] & 0xf) << 8) | data[5])
                    MotorType_recv = self.motors_map[CANID].MotorType
                    Q_MAX = self.Limit_Param[MotorType_recv][0]
                    DQ_MAX = self.Limit_Param[MotorType_recv][1]
                    TAU_MAX = self.Limit_Param[MotorType_recv][2]
                    recv_q = uint_to_float(q_uint, -Q_MAX, Q_MAX, 16)
                    recv_dq = uint_to_float(dq_uint, -DQ_MAX, DQ_MAX, 12)
                    recv_tau = uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)
                    state = (data[0] >> 4) & 0x0F  # High 4 bits: Status Code
                    err = 0 if state in (0, 1) else state # 0:Disabled, 1:Enabled both normal
                    is_enabled = (state == 1)
                    t_mos = data[6]
                    t_coil = data[7]
                    self.motors_map[CANID].recv_data(recv_q, recv_dq, recv_tau, err, t_mos, t_coil, is_enabled)
            else:
                MasterID=data[0] & 0x0f
                if MasterID in self.motors_map:
                    q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                    dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                    tau_uint = np.uint16(((data[4] & 0xf) << 8) | data[5])
                    MotorType_recv = self.motors_map[MasterID].MotorType
                    Q_MAX = self.Limit_Param[MotorType_recv][0]
                    DQ_MAX = self.Limit_Param[MotorType_recv][1]
                    TAU_MAX = self.Limit_Param[MotorType_recv][2]
                    recv_q = uint_to_float(q_uint, -Q_MAX, Q_MAX, 16)
                    recv_dq = uint_to_float(dq_uint, -DQ_MAX, DQ_MAX, 12)
                    recv_tau = uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)
                    state = (data[0] >> 4) & 0x0F  # High 4 bits: Status Code
                    err = 0 if state in (0, 1) else state # 0:Disabled, 1:Enabled both normal
                    is_enabled = (state == 1)
                    t_mos = data[6]
                    t_coil = data[7]
                    self.motors_map[MasterID].recv_data(recv_q, recv_dq, recv_tau, err, t_mos, t_coil, is_enabled)

    def __process_set_param_packet(self, data, CANID, CMD):
        if CMD == 0x11 and (data[2] == 0x33 or data[2] == 0x55):
            masterid=CANID
            slaveId = ((data[1] << 8) | data[0])
            if CANID==0x00:  #Prevent MasterID=0 issues
                masterid=slaveId

            if masterid not in self.motors_map:
                if slaveId not in self.motors_map:
                    return
                else:
                    masterid=slaveId

            RID = data[3]
            # 
            if is_in_ranges(RID):
                #uint32
                num = uint8s_to_uint32(data[4], data[5], data[6], data[7])
                self.motors_map[masterid].temp_param_dict[RID] = num

            else:
                #float
                num = uint8s_to_float(data[4], data[5], data[6], data[7])
                self.motors_map[masterid].temp_param_dict[RID] = num


    def addMotor(self, Motor):
        """
        add motor to the motor control object 
        :param Motor: Motor object Motor object
        """
        self.motors_map[Motor.SlaveID] = Motor
        if Motor.MasterID != 0:
            self.motors_map[Motor.MasterID] = Motor
        return True

    def __control_cmd(self, Motor, cmd: np.uint8):
        data_buf = np.array([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd], np.uint8)
        self.__send_data(Motor.SlaveID, data_buf)

    def __send_data(self, motor_id, data):
        """
        send data to the motor 
        :param motor_id:
        :param data:
        :return:
        """
        # hex
        # print(f"Sent raw HEX data (MotorID 0x{motor_id:02X}): {data.tobytes().hex()}")
        self.send_data_frame[13] = motor_id & 0xff
        self.send_data_frame[14] = (motor_id >> 8)& 0xff  #id high 8 bits
        self.send_data_frame[21:29] = data
        
        # Track req_count
        master_id = motor_id & 0x0F
        if master_id in self.motors_map:
            self.motors_map[master_id].req_count += 1
            
        send_bytes = bytes(self.send_data_frame.T)
        if hasattr(self, 'last_serial_logs'):
            self.last_serial_logs["write"].append(" ".join([f"{b:02X}" for b in send_bytes]))
        if getattr(self, 'debug', False) in (True, 1):
            import datetime
            now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            hex_str = " ".join([f"{b:02X}" for b in send_bytes])
            if True:
                print(f"{now} DEBU [pymycobot.dm_can] _write: {hex_str}")
        self.serial_.write(send_bytes)

    def __read_RID_param(self, Motor, RID):
        can_id_l = Motor.SlaveID & 0xff #id low 8 bits
        can_id_h = (Motor.SlaveID >> 8)& 0xff  #id high 8 bits
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0x33, np.uint8(RID), 0x00, 0x00, 0x00, 0x00], np.uint8)
        self.__send_data(0x7FF, data_buf)

    def __write_motor_param(self, Motor, RID, data):
        can_id_l = Motor.SlaveID & 0xff #id low 8 bits
        can_id_h = (Motor.SlaveID >> 8)& 0xff  #id high 8 bits
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0x55, np.uint8(RID), 0x00, 0x00, 0x00, 0x00], np.uint8)
        if not is_in_ranges(RID):
            # data is float
            data_buf[4:8] = float_to_uint8s(data)
        else:
            # data is int
            data_buf[4:8] = data_to_uint8s(int(data))
        self.__send_data(0x7FF, data_buf)

    def switchControlMode(self, Motor, ControlMode):
        """
        switch the control mode of the motor control mode
        :param Motor: Motor object Motor object
        :param ControlMode: Control_Type control mode example:MIT:Control_Type.MIT MIT
        """
        max_retries = 10
        retry_interval = 0.05  #retry times
        RID = 10
        self.__write_motor_param(Motor, RID, np.uint8(ControlMode))
        for _ in range(max_retries):
            sleep(retry_interval)
            self.recv_set_param_data()
            if Motor.SlaveID in self.motors_map:
                if RID in self.motors_map[Motor.SlaveID].temp_param_dict:
                    if self.motors_map[Motor.SlaveID].temp_param_dict[RID] == ControlMode:
                        return True
                    else:
                        return False
        return False

    def save_motor_param(self, Motor):
        """
        save the all parameter  to flash 
        :param Motor: Motor object Motor object
        :return:
        """
        can_id_l = Motor.SlaveID & 0xff #id low 8 bits
        can_id_h = (Motor.SlaveID >> 8)& 0xff  #id high 8 bits
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        self.disable(Motor)  # before save disable the motor
        self.__send_data(0x7FF, data_buf)
        sleep(0.001)

    def change_limit_param(self, Motor_Type, PMAX, VMAX, TMAX):
        """
        change the PMAX VMAX TMAX of the motor PMAX VMAX TMAX
        :param Motor_Type:
        :param PMAX: PMAX
        :param VMAX: VMAX
        :param TMAX: TMAX
        :return:
        """
        self.Limit_Param[Motor_Type][0] = PMAX
        self.Limit_Param[Motor_Type][1] = VMAX
        self.Limit_Param[Motor_Type][2] = TMAX

    def refresh_motor_status(self, Motor):
        """
        get the motor status 
        """
        # Re-send last known velocity to trigger fresh feedback (0xCC unsupported on DM2325)
        self.control_Vel(Motor, getattr(Motor, 'Vd', 0.0))

    def get_filtered_velocity(self, Motor):
        self.refresh_motor_status(Motor)
        v = Motor.getVelocity()
        return round(float(v), 2) if abs(v) >= 0.05 else 0.0

    def get_filtered_torque(self, Motor):
        self.refresh_motor_status(Motor)
        t = Motor.getTorque()
        return round(float(t), 2) if abs(t) >= 0.05 else 0.0

    def get_mos_temp(self, Motor):
        self.refresh_motor_status(Motor)
        t_mos, _ = Motor.getTemp()
        return float(t_mos)

    def change_motor_param(self, Motor, RID, data):
        """
        change the RID of the motor 
        :param Motor: Motor object Motor object
        :param RID: DM_variable 
        :param data: 
        :return: True or False ,True means success, False means fail
        """
        max_retries = 20
        retry_interval = 0.05  #retry times

        self.__write_motor_param(Motor, RID, data)
        for _ in range(max_retries):
            self.recv_set_param_data()
            if Motor.SlaveID in self.motors_map and RID in self.motors_map[Motor.SlaveID].temp_param_dict:
                if abs(self.motors_map[Motor.SlaveID].temp_param_dict[RID] - data) < 0.1:
                    return True
                else:
                    return False
            sleep(retry_interval)
        return False

    def read_motor_param(self, Motor, RID):
        """
        read only the RID of the motor  
        :param Motor: Motor object Motor object
        :param RID: DM_variable 
        :return: 
        """
        max_retries = 20
        retry_interval = 0.05  #retry times
        self.__read_RID_param(Motor, RID)
        for _ in range(max_retries):
            sleep(retry_interval)
            self.recv_set_param_data()
            if Motor.SlaveID in self.motors_map:
                if RID in self.motors_map[Motor.SlaveID].temp_param_dict:
                    return self.motors_map[Motor.SlaveID].temp_param_dict[RID]
                else:
                    return None
        return None

    # -------------------------------------------------
    # Extract packets from the serial data
    def __extract_packets(self, data):
        frames = []
        header = 0xAA
        tail = 0x55
        frame_length = 16
        i = 0
        remainder_pos = 0

        while i <= len(data) - frame_length:
            if data[i] == header and data[i + frame_length - 1] == tail:
                frame = data[i:i + frame_length]
                frames.append(frame)
                i += frame_length
                remainder_pos = i
            else:
                i += 1
        self.data_save = data[remainder_pos:]
        return frames


def LIMIT_MIN_MAX(x, min, max):
    if x <= min:
        return min
    elif x > max:
        return max
    return x


def float_to_uint(x: float, x_min: float, x_max: float, bits):
    x = LIMIT_MIN_MAX(x, x_min, x_max)
    span = x_max - x_min
    data_norm = (x - x_min) / span
    return np.uint16(data_norm * ((1 << bits) - 1))


def uint_to_float(x: np.uint16, min: float, max: float, bits):
    span = max - min
    data_norm = float(x) / ((1 << bits) - 1)
    temp = data_norm * span + min
    return np.float32(temp)


def float_to_uint8s(value):
    # Pack the float into 4 bytes
    packed = pack('f', value)
    # Unpack the bytes into four uint8 values
    return unpack('4B', packed)


def data_to_uint8s(value):
    # Check if the value is within the range of uint32
    if isinstance(value, int) and (0 <= value <= 0xFFFFFFFF):
        # Pack the uint32 into 4 bytes
        packed = pack('I', value)
    else:
        raise ValueError("Value must be an integer within the range of uint32")

    # Unpack the bytes into four uint8 values
    return unpack('4B', packed)


def is_in_ranges(number):
    """
    check if the number is in the range of uint32
    :param number:
    :return:
    """
    if (7 <= number <= 10) or (13 <= number <= 16) or (35 <= number <= 36):
        return True
    return False


def uint8s_to_uint32(byte1, byte2, byte3, byte4):
    # Pack the four uint8 values into a single uint32 value in little-endian order
    packed = pack('<4B', byte1, byte2, byte3, byte4)
    # Unpack the packed bytes into a uint32 value
    return unpack('<I', packed)[0]


def uint8s_to_float(byte1, byte2, byte3, byte4):
    # Pack the four uint8 values into a single float value in little-endian order
    packed = pack('<4B', byte1, byte2, byte3, byte4)
    # Unpack the packed bytes into a float value
    return unpack('<f', packed)[0]


def print_hex(data):
    hex_values = [f'{byte:02X}' for byte in data]
    print(' '.join(hex_values))


def get_enum_by_index(index, enum_class):
    try:
        return enum_class(index)
    except ValueError:
        return None


class DM_Motor_Type(IntEnum):
    DM4310 = 0
    DM4310_48V = 1
    DM4340 = 2
    DM4340_48V = 3
    DM6006 = 4
    DM8006 = 5
    DM8009 = 6
    DM10010L = 7
    DM10010 = 8
    DMH3510 = 9
    DMH6215 = 10
    DMG6220 = 11
    DMJH11 = 12
    DM6248P = 13
    DM3507 = 14
    DM2325 = 15

class DM_variable(IntEnum):
    UV_Value = 0      #  (Under Voltage threshold)
    KT_Value = 1      # Calibration coefficient (Torque constant / calibration)
    OT_Value = 2      #  (Over Temperature threshold)
    OC_Value = 3      #  (Over Current threshold)
    ACC = 4           #  (Acceleration)
    DEC = 5           #  (Deceleration)
    MAX_SPD = 6       #  (Maximum speed)
    MST_ID = 7        # / ID (Master ID)
    ESC_ID = 8        #  ESC（）ID (ESC / slave ID)
    TIMEOUT = 9       #  (Timeout duration)
    CTRL_MODE = 10    # control mode（//）(Control mode)
    Damp = 11         #  (Damping)
    Inertia = 12      #  (Inertia)
    hw_ver = 13       #  (Hardware version)
    sw_ver = 14       #  (Software/firmware version)
    SN = 15           #  (Serial Number)
    NPP = 16          #  (Number of pulses per revolution)
    Rs = 17           #  (Stator resistance Rs)
    LS = 18           #  (Stator inductance Ls)
    Flux = 19         #  (Flux linkage)
    Gr = 20           #  (Gear ratio)
    PMAX = 21         #  (Maximum power)
    VMAX = 22         #  (Maximum voltage/speed limit)
    TMAX = 23         # / (Maximum torque/temperature)
    I_BW = 24         # （）(Current loop bandwidth)
    KP_ASR = 25       # ASR  Kp (Proportional gain for ASR)
    KI_ASR = 26       # ASR  Ki (Integral gain for ASR)
    KP_APR = 27       # APR  Kp (Proportional gain for APR)
    KI_APR = 28       # APR  Ki (Integral gain for APR)
    OV_Value = 29     #  (Over Voltage threshold)
    GREF = 30         # / (Reference gain or frequency)
    Deta = 31         #  (Delta / small offset)
    V_BW = 32         # velocity loop bandwidth (Velocity loop bandwidth)
    IQ_c1 = 33        #  IQ_c1 (Current-related constant)
    VL_c1 = 34        #  VL_c1 (Velocity/voltage constant)
    can_br = 35       # CAN bus baudrate (CAN bus baud rate)
    sub_ver = 36      # sub version number (Sub-version)
    u_off = 50        # Phase U offset/ (Phase U offset)
    v_off = 51        # Phase V offset/ (Phase V offset)
    k1 = 52           # Calibration coefficient k1 (Calibration coefficient)
    k2 = 53           # Calibration coefficient k2 (Calibration coefficient)
    m_off = 54        #  (Mechanical offset)
    dir = 55          # （/）(Direction)
    p_m = 80          # Pole pairs (Pole pairs / mechanical poles)
    xout = 81         # External output (External output / diagnostic output)

class Control_Type(IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    Torque_Pos = 4
    POS_VEL_CSP = 5
    VEL_CSP = 6
    Torque_CSP = 7