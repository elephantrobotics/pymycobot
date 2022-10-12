from pymycobot import MyBuddyEmoticon
import time

file_path = [
    ['/home/er/emo/look_happy.mp4', 10]
]

em = MyBuddyEmoticon(file_path, loop=True)

em.start()
time.sleep(5)

# 暂停
em.pause()

time.sleep(2)

# 继续播放
em.run()

# 等待播放结束
em.join()