# encoding=utf-8
import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager
from pymycobot import MyCobot

mc = MyCobot("COM12", 115200)
 
POINTS = 7
sin_list = [0] * (POINTS -1)
indx = 0
 
fig, ax = plt.subplots()
ax.set_ylim([0, 3250])
ax.set_xlim([1, 6])
ax.set_autoscale_on(False)
ax.set_xticks(range(0, 3250, 500))
ax.set_yticks(range(0, 11, 1))
ax.grid(True)
 
line_sin, = ax.plot([i for i in range(1,POINTS)], sin_list, label='Current', color='cornflowerblue')
ax.legend(loc='upper center', ncol=4, prop=font_manager.FontProperties(size=10))
 
def sin_output(ax):
    global sin_list, line_sin
    sin_list = sin_list[1:] + mc.get_servo_currents() # get current
    # print(sin_list)
    line_sin.set_ydata(sin_list)
    ax.draw_artist(line_sin)
    ax.figure.canvas.draw()
 
timer = fig.canvas.new_timer(interval=100)
timer.add_callback(sin_output, ax)
timer.start()
plt.show()