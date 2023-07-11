# Instructions for playing animations on MyBuddy screen.
[中文]
1. 将[视频文件](emoticon.zip)和[案例代码](./mybuddy_emoticon_demo.py)下载并且解压到mybuddy系统内

2. 更改案例`pymycobot/demo/mybuddy_demo/mybuddy_emoticon_demo.py`文件,

将`/home/er/pymycobot/emo/face_video_3_2.mp4`替换成解压出来的视频文件路径
```python
from pymycobot import MyBuddyEmoticon
import time

# [video_path, Playback_duration(s)]
video1 = ["/home/er/pymycobot/emo/face_video_3_2.mp4", 10]

datas = [video1]

me = MyBuddyEmoticon(datas)
...
```
3. 执行mybuddy_emoticon_demo.py即可
    [API说明文档](../../docs/mybuddy_emoticon.md)
---
[English]
1. Download and unzip [video file](emoticon.zip) and [case code](./mybuddy_emoticon_demo.py) into the mybuddy system

2. Change the case `pymycobot/demo/mybuddy_demo/mybuddy_emoticon_demo.py` file,

Replace `/home/er/pymycobot/emo/face_video_3_2.mp4` with the decompressed video file path
```python
from pymycobot import MyBuddyEmoticon
import time

# [video_path, Playback_duration(s)]
video1 = ["/home/er/pymycobot/emo/face_video_3_2.mp4", 10]

datas = [video1]

me = MyBuddyEmoticon(datas)
...
```
3. Execute mybuddy_emoticon_demo.py

[API document](../../docs/mybuddy_emoticon.md)
