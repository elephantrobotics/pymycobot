
from .protocol_packet_handler import *
# 1 byte
FIRMWARE_MAJOR = 0x00
FIRMWARE_MINOR = 0x01
SERVO_MAJOR = 0x03
SERVO_MINOR = 0x04
BAUD = 6
RETURN_DELAY = 7
PHASE = 18
MAX_TEMPERATURE = 13
MAX_VOLTAGE = 14
SERVO_P = 21
SERVO_D = 22
SERVO_I = 23
CLOCKWISE_INSENSITIVE_ZONE = 26
COUNTERCLOCKWISE_INSENSITIVE_ZONE = 27
D_CONTROL_TIME = 81
# 2 bytes
MIN_START_FORCE = 24


class sms_sts(protocol_packet_handler):
    def __init__(self, portHandler):
        protocol_packet_handler.__init__(self, portHandler, 0)
        
    def get_servo_baud(self, id):
        """获取舵机波特率
        
        Args:
            id: 电机Id
            
        Return:
            None: 获取失败
        """
        res = self.read1ByteTxRx(id, BAUD)
        return res[0] if res[1] != -2 else None
    
    def set_servo_baud(self, id, value):
        """设置舵机波特率"""
        return self.write1ByteTxRx(id, BAUD, value)
    
    def get_servo_response_speed(self, id):
        """获取舵机响应速度
        
        Args:
            id: 电机Id
            
        Return:
            None: 获取失败
        """
        res = self.read1ByteTxRx(id, RETURN_DELAY)
        return res[0] if res[1] != -2 else None
    
    def set_servo_response_speed(self, id, value):
        """设置舵机响应速度"""
        return self.write1ByteTxRx(id, RETURN_DELAY, value)
    
    def get_servo_phase(self, id):
        """获取舵机相位
        
        Args:
            id: 电机Id
            
        Return:
            None: 获取失败
        """
        res = self.read1ByteTxRx(id, PHASE)
        return res[0] if res[1] != -2 else None
    
    def set_servo_phase(self, id, value):
        """设置舵机相位"""
        return self.write1ByteTxRx(id, PHASE, value)
    
    def get_servo_max_temperature(self, id):
        """获取舵机最大温度
        
        Args:
            id: 电机Id
            
        Return:
            None: 获取失败
        """
        res = self.read1ByteTxRx(id, MAX_TEMPERATURE)
        return res[0] if res[1] != -2 else None
    
    def set_servo_max_temperature(self, id, value):
        """设置舵机最大温度"""
        return self.write1ByteTxRx(id, MAX_TEMPERATURE, value)
       
    def get_servo_max_voltage(self, id):
        """获取舵机最大电压
        
        Args:
            id: 电机Id
            
        Return:
            None: 获取失败
        """
        res = self.read1ByteTxRx(id, MAX_VOLTAGE)
        return res[0] if res[1] != -2 else None
    
    def set_servo_max_voltage(self, id, value):
        """设置舵机最大电压"""
        return self.write1ByteTxRx(id, MAX_VOLTAGE, value)
    
    def get_servo_pid(self, id):
        """获取舵机pid
        
        Args:
            id: 电机Id
            
        Return:
            None: 获取失败
        """
        res_p = self.read1ByteTxRx(id, SERVO_P)
        res_i = self.read1ByteTxRx(id, SERVO_I)
        res_d = self.read1ByteTxRx(id, SERVO_D)
        return [res_p[0] if res_p[1] != -2 else None, res_i[0] if res_i[1] != -2 else None, res_d[0] if res_d[1] != -2 else None]
    
    def set_servo_pid(self, id, pid):
        """设置舵机pid"""
        self.write1ByteTxRx(id, SERVO_P, pid[0])
        self.write1ByteTxRx(id, SERVO_I, pid[1])
        self.write1ByteTxRx(id, SERVO_D, pid[2])
    
    def get_servo_clockwise(self, id):
        """获取舵机顺时针不灵敏区
        
        Args:
            id: 电机Id
            
        Return:
            None: 获取失败
        """
        res = self.read1ByteTxRx(id, CLOCKWISE_INSENSITIVE_ZONE)
        return res[0] if res[1] != -2 else None
    
    def set_servo_clockwise(self, id, value):
        """设置舵机顺时针不灵敏区"""
        return self.write1ByteTxRx(id, CLOCKWISE_INSENSITIVE_ZONE, value)
    
    def get_servo_counter_clockwise(self, id):
        """获取舵机逆时针不灵敏区
        
        Args:
            id: 电机Id
            
        Return:
            None: 获取失败
        """
        res = self.read1ByteTxRx(id, COUNTERCLOCKWISE_INSENSITIVE_ZONE)
        return res[0] if res[1] != -2 else None
    
    def set_servo_counter_clockwise(self, id, value):
        """设置舵机逆时针不灵敏区"""
        return self.write1ByteTxRx(id, COUNTERCLOCKWISE_INSENSITIVE_ZONE, value)
     
    def get_servo_d_time(self, id):
        """获取舵机D控制时间
        
        Args:
            id: 电机Id
            
        Return:
            None: 获取失败
        """
        res = self.read1ByteTxRx(id, D_CONTROL_TIME)
        return res[0] if res[1] != -2 else None
    
    def set_servo_d_time(self, id, value):
        """设置舵机D控制时间"""
        return self.write1ByteTxRx(id, D_CONTROL_TIME, value)
    
    def get_servo_min_start(self, id):
        """获取舵机最小启动力
        
        Args:
            id: 电机Id
            
        Return:
            None: 获取失败
        """
        scs_present_speed, scs_comm_result, scs_error = self.read2ByteTxRx(id, MIN_START_FORCE)
        return self.scs_tohost(scs_present_speed, 15) if scs_comm_result != -2 else None
    
    def set_servo_min_start(self, id, value):
        """设置舵机最小启动力"""
        return self.write2ByteTxRx(id, MIN_START_FORCE, value)
    
    def search_servo(self, id):
        """搜索舵机是否存在
        
        Args:
            id：舵机ID号.
        
        Return:
            0: 存在.
            -6：不存在.
            None: 获取失败.
        """
        res = self.ping(id)
        if res == -2:
            return None
        return res
    
    def get_servo_error(self, id):
        """获取舵机错误信息
        
        Return:
            error_info: 将返回的十进制数据转换为二进制
                Bit0  Bit1  Bit2 Bit3 Bit4 Bit5 对应位被置1表示相应错误出现\n
                电压  传感器 温度 电流 角度  过载 对应位0为无相应该错误\n
                None:表示获取失败
        """
        res = self.read1ByteTxRx(id, 0x41)
        if res[1] == -2:
            return None
        return res[0]
        
    def get_servo_firmware_version(self, id):
        """获取电机固件版本信息
        
        Return:
            list: [固件主版本号, 固件次版本号, 舵机主版本号, 舵机次版本号], -1 表示获取失败
        """
        res = []
        command = [FIRMWARE_MAJOR, FIRMWARE_MINOR, SERVO_MAJOR, SERVO_MINOR]
        for add in command:
            r = self.read1ByteTxRx(id, add)
            if r[1] == -2:
                res.append(-1)
            else:
                res.append(r[0])
        return res
                
