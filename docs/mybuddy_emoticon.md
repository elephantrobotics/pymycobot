# MyBuddyEmoticon

## MyBuddyEmoticon(file_path: list = [], window_size: tuple = (), loop=False)
API for playing emoticons

* **Parameters**

    **file_path (list)**: `[[path, time]]`The absolute path of facial expression video and the length of time to play.Time in seconds.
    **window_size (tuple)**: `(Length, width) `Size of the playback window (default is full screen).
    **loop (bool)**: Loop playback or not (default False. only once by default).

### file_path()
Get Playfile List
* **Return**
  * video path list

### add_file_path(path_time)
Add Playback File
* **Parameters**
    **path_time(list)**: `[path, time]` The video address to be added and the running time

### del_file_path(index)
Delete the element with the specified subscript in the playlist list
* **Parameters**

    **index(int)**:: The subscript of the element in the playlist to be deleted

### pause()
Pause playback

### run()
Continue playing

### start()
start playing video

### join()
Wait for the thread playing the video to finish.

[--->>> play_demo](../demo/mybuddy_demo/mybuddy_emoticon_demo.py)