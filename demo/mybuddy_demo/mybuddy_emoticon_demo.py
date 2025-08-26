from pymycobot import MyBuddyEmoticon
import time

# [video_path, Playback_duration(s)]
video1 = ["/home/er/pymycobot/emo/face_video_3_2.mp4", 10]

datas = [video1]

me = MyBuddyEmoticon(datas)
me.start()

time.sleep(3)
# Pause playback
me.pause()

time.sleep(3)
# Continue playing
me.run()

# Waiting for playback to complete
me.join()