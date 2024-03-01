import time
from pymycobot.myarm import MyArm

mc = MyArm('/dev/ttyAMA0')
time.sleep(0.1)


def main():
    """
    模仿myarm与人类下棋输赢状态的动作
    :return: None
    """
    init_angles = [
        [-60, 0, 0, -90, 0, -90, -0.79],  # first init point
        #[0, 49, 0, -104, 0, -45, -0.79],  # second init point
        [0, -49, 0, -100, 0, -45, -0.79],
        #[-0.26, 2.0, 0.43, -111.0, 0.43, 20.22, -0.79],  # third
        [-3.16, -11.77, 0.17, -100, -0.52, -3.25, 1.23],  # third
        [3.25, -8.61, 0.17, -99.14, 1.14, -2.81, -0.79],  # left
        [-10.01, -6.85, 0.17, -99.49, -2.63, -2.81, 3.07], # right
    ]

    #mc.send_angles(init_angles[0], 50)

    #time.sleep(3)

    #mc.send_angles(init_angles[1], 50)
    #time.sleep(3)

    #mc.send_angles(init_angles[2], 50)
    #time.sleep(3)
    #mc.set_gservo_round(12)
    #time.sleep(1.5)
    
    #for i in range(5):
        
    mc.send_angles(init_angles[2], 50)
    time.sleep(2.5)
    mc.set_gservo_round(12)
    time.sleep(1.5)
    mc.send_angles(init_angles[1], 50)
    time.sleep(3)
    for i in range(3):
        mc.send_angles([0, -49, 0, -100, 0, -65, -0.79], 90)
        time.sleep(1)
        mc.send_angles([0, -49, 0, -100, 0, -25, -0.79], 90)
        time.sleep(1)
        #mc.send_angles(init_angles[1], 50)
       # time.sleep(3)
    
        
        #mc.send_angles(init_angles[2], 50)
        #time.sleep(3)
       # mc.set_gservo_round(12)
       # time.sleep(1.5)
       # mc.send_angles(init_angles[1], 50)
       # time.sleep(3)
        
        #mc.send_angles(init_angles[2], 50)
       # time.sleep(3)
        #mc.set_gservo_round(12)
        #time.sleep(1.5)
        #mc.send_angles(init_angles[1], 50)
       # time.sleep(3)
        
   # mc.send_angles([0.26, -2.0, 0.43, -100.0, 0.43, 20.22, -0.79], 50)
    #time.sleep(3)
    
    # win
   # for i in range(5):
    #    mc.send_angles([0.0, -0.34, 0.17, -100.62, 0.17, 20.29, 0.08], 90)
    #    time.sleep(1)
    #    mc.send_angles([0.0, -0.34, 0.17, -100.62, 0.17, 20.29, 0.08], 90)
    #    time.sleep(1)
        #print(init_angles[2])

   # mc.send_angles(init_angles[2], 50)
   # time.sleep(3)
    # fail
   # mc.send_angles([0.0, -18.34, 0.17, -100.62, 0.17, -7.29, 0.08], 50)
   # time.sleep(3)
   # mc.send_angles([0.0, -18.34, 0.17, -100.62, 0.17, -26.29, 0.08], 5)
   # time.sleep(7)
   # mc.send_angle(4, -80, 30)
   # time.sleep(3)


# mc.send_angles(init_angles[2], 50)
# time.sleep(3)

if __name__ == '__main__':
    main()
