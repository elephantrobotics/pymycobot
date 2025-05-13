# MyBuddyEmoticon

```python
from pymycobot import MyBuddyEmoticon
import time

# [video_path, Playback_duration(s)]
video1 = ["/home/er/pymycobot/emo/face_video_3_2.mp4", 10]

datas = [video1]

me = MyBuddyEmoticon(datas)
me.start()
```

## MyBuddyEmoticon(file_path: list = [], window_size: tuple = (), loop=False)

* **功能**：播放表情的 API

* **参数**：

  - **file_path (list)**：`[[path, time]]` 表情视频的绝对路径及播放时长，单位为秒。
  - **window_size (tuple)**：`(长, 宽)` 播放窗口大小（默认全屏）。
  - **loop (bool)**：是否循环播放（默认 False，仅循环播放一次）。

### file_path()

* **功能：** 获取播放文件列表
* **返回**
  * 视频路径列表

### add_file_path(path_time)

* **功能：** 添加播放文件
* **参数**
    **path_time(list)**: `[path, time]` 待添加的视频地址及播放时长

### del_file_path(index)

* **函数：** 删除播放列表中指定下标的元素
* **参数：**

    **index(int)**:: 待删除元素在播放列表中的下标

### pause()

**功能：** 暂停播放

### run()

**功能：** 继续播放

### start()

* **功能：** 开始播放视频

### join()

* **功能**：等待播放视频的线程完成。

[--->>> 播放演示](../demo/mybuddy_demo/mybuddy_emoticon_demo.py)