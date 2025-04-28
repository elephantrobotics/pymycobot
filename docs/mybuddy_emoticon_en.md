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

* **Function:** API for playing emoticons

* **Parameters:**

    - **file_path (list)**: `[[path, time]]`The absolute path of facial expression video and the length of time to play.Time in seconds.
    - **window_size (tuple)**: `(Length, width) `Size of the playback window (default is full screen).
    - **loop (bool)**: Loop playback or not (default False. only once by default).

### file_path()

* **Function:** Get Playfile List
* **Return**
  * video path list

### add_file_path(path_time)

* **Function:** Add Playback File
* **Parameters**
    **path_time(list)**: `[path, time]` The video address to be added and the running time

### del_file_path(index)

* **Function:** Delete the element with the specified subscript in the playlist list
* **Parameters**

    **index(int)**:: The subscript of the element in the playlist to be deleted

### pause()

* **Function:** Pause playback

### run()

* **Function:** Continue playing

### start()

* **Function:** start playing video

### join()

* **Function:** Wait for the thread playing the video to finish.

[--->>> play_demo](../demo/mybuddy_demo/mybuddy_emoticon_demo.py)