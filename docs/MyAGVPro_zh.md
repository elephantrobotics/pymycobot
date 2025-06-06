# class MyAGVPro()

### 1. ϵͳ & ��Ʒ��Ϣ

#### get_system_version():
- **����:** ��ȡ���̼��汾��
- **����ֵ:**
  - **float: version**

#### get_modify_version():
- **����:** ��ȡ�ι̼��汾��
- **����ֵ:**
  - **int: version**

#### power_on():
- **����:** ����������
- **����ֵ:**
  - **int: �������, 1: �ɹ�, 0: ʧ��**

#### power_on_only():
- **����:** ���������ˣ������������Ƴ���
- **����ֵ:**
  - **int: ���������, 1: �ɹ�, 0: ʧ��**

#### power_off():
- **����:** �رջ�����
- **����ֵ:**
  - **int: ���رս��, 1: �ɹ�, 0: ʧ��**

#### is_power_on():
- **����:** ���������Ƿ��ѿ���
- **����ֵ:**
  - **int: ��Դ״̬, 1: ����, 0: �ر�**

### 2. �˶�����

#### move_backward(speed):
- **����:** ƽ�ƻ��������
- **����:**
  - **speed(float): 0 ~ 1.5 m/s**
- **����ֵ:**
  - **int: 1: �ɹ�, 0: ʧ��**

#### move_forward(speed):
- **����:** ƽ�ƻ�������ǰ
- **����:**
  - **speed(float): 0 ~ 1.5 m/s**
- **����ֵ:**
  - **int: 1: �ɹ�, 0: ʧ��**

#### move_left_lateral(speed):
- **����:** ƽ�ƻ���������
- **����:**
  - **speed(float): 0 ~ 1 m/s**
- **����ֵ:**
  - **int: 1: �ɹ�, 0: ʧ��**

#### move_right_lateral(speed):
- **����:** ƽ�ƻ���������
- **����:**
  - **speed(float): 0 ~ 1 m/s**
- **����ֵ:**
  - **int: 1: �ɹ�, 0: ʧ��**

#### turn_left(speed):
- **����:** ������ת
- **����:**
  - **speed:**
- **����ֵ:**
  - **int: 1: �ɹ�, 0: ʧ��**

#### turn_right(speed):
- **����:** ������ת
- **����:**
  - **speed:**
- **����ֵ:**
  - **int: 1: �ɹ�, 0: ʧ��**

#### stop():
- **����:** ֹͣ�ƶ�
- **����ֵ:**
  - **int: 1: �ɹ�, 0: ʧ��**

#### set_auto_report_state(state):
- **����:** �����Զ�����״̬
- **����:**
  - **state(int): 0: �ر�, 1: ����**
- **����ֵ:**
  - **int: 1: �ɹ�, 0: ʧ��**

#### get_auto_report_state():
- **����:** ��ȡ�Զ�����״̬
- **����ֵ:**
  - **int: 0: �ر�, 1: ����**

#### get_auto_report_message():
- **����:** ��ȡ�Զ�������Ϣ
- **����ֵ:**
  - **list[int | list[int] | float]:**
  - **0 - (float)rx**
  - **1 - (float)ry**
  - **2 - (float)rw**
  - **3 - (list[int])����״̬**
  - **4 - (list[int])�����Ϣ**
  - **5 - (float)��ص�ѹ**
  - **6 - (int)���ʹ��״̬ 0: ����, 1: ����**

### 3. �������

#### get_motor_enable_status():
- **����:** ��ȡ���ʹ��״̬
- **����ֵ:**
  - **list[int]: ���ʹ��״̬**
  - **0: ����**
  - **1: ����**

#### get_motor_status():
- **����:** ��ȡ���״̬
- **����ֵ:**
  - **list[int]: ���״̬**
  - **0: ����**
  - **any: �������**

#### get_motor_temps():
- **����:** ��ȡ����¶�
- **����ֵ:**
  - **list[float]: ����¶�**

#### get_motor_speeds():
- **����:** ��ȡ����ٶ�
- **����ֵ:**
  - **list[float]: ����ٶ�**

#### get_motor_torques():
- **����:** ��ȡ���Ť��
- **����ֵ:**
  - **list[float]: ���Ť��**

#### set_communication_state(state):
- **����:** ����ͨѶ״̬
- **����:**
  - **state(int):**
  - **0: ����ͨѶ (Ĭ��)**
  - **1: Socket ͨѶ**
  - **2: ����ͨѶ (�� MAC ��ַд���ļ��Ͷ˵㣬Ȼ�󷵻ص���״̬)**
- **����ֵ:**
  - **int: 1: �ɹ�, 0: ʧ��**

#### get_communication_state():
- **����:** ��ȡͨѶ״̬
- **����ֵ:**
  - **int: ͨѶ״̬**
  - **0: ����ͨѶ,**
  - **1: Socket ͨѶ,**
  - **2: ����ͨѶ**

#### set_led_color(position, brightness, color):
- **����:** ���� LED ��ɫ
- **����:**
  - **position(int):**
  - **0: ��� LED**
  - **1: �Ҳ� LED**
  - **brightness(int): 0 - 255**
  - **color(tuple(int, int, int)): RGB ��ɫ**
- **����ֵ:**
  - **int: 1: �ɹ�, 0: ʧ��**

#### get_motor_loss_count():
- **����:** ��ȡ�����������
- **����ֵ:**
  - **list[int]: �����������**

### 4. IO ����

#### get_pin_input(pin):
- **����:** ��ȡ���� IO
- **����:**
  - **pin(int): 1, 2, 3, 4, 5, 6, 7, 8, 254**
- **����ֵ:**
  - **int: 0: �͵�ƽ, 1: �ߵ�ƽ, -1: û���������**

#### set_pin_output(pin, state):
- **����:** ������� IO
- **����:**
  - **pin(int): 1 - 6**
  - **state(int): 0: �͵�ƽ, 1: �ߵ�ƽ**
- **����ֵ:**
  - **int: 1: �ɹ�, 0: ʧ��**

### 5. WiFi & ����

#### get_wifi_ip_port():
- **����:** ��ȡ wi-fi ip �Ͷ˿�
- **����ֵ:**
  - **tuple(str, int): wi-fi ip, wi-fi �˿�**

#### get_wifi_account():
- **����:** ��ȡ wi-fi �˺�
- **����ֵ:**
  - **tuple(str, str): wi-fi �˺�, wi-fi ����**

#### get_bluetooth_address():
- **����:** ��ȡ���� MAC ��ַ
- **����ֵ:**
  - **str: ���� MAC ��ַ**

#### get_bluetooth_uuid():
- **����:** ��ȡ���� uuid
- **����ֵ:**
  - **tuple(str, str, str): ��������, ���� uuid, ���� uuid**

### 6. ʹ�ð���
#### 6.1 ��ȡ AGVPro ��ϵͳ�汾��
```python
from pymycobot import MyAGVPro
# ��ʼ�� AGVPro ����
agv_pro = MyAGVPro("/dev/ttyTHS1", baudrate=1000000, debug=True)
# ��ȡϵͳ�汾��
version = agv_pro.get_system_version()
print(version)
```
#### 6.2 ���� AGVPro��0.5m/s���ٶ���ǰ�ƶ�3��
```python
import time
from pymycobot import MyAGVPro

# ��ʼ�� AGVPro ����
agv_pro = MyAGVPro("/dev/ttyTHS1", baudrate=1000000, debug=True)

# ���� agv_pro �� 0.5m/s ���ٶ���ǰ�ƶ�
agv_pro.move_forward(0.5)

# ˯�� 3 ��
time.sleep(3)

# ֹͣ�ƶ�
agv_pro.stop()
```