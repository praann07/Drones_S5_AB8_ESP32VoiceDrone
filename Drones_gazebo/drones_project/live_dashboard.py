import collections
import matplotlib
# Force interactive GUI backend for WSL
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pymavlink import mavutil

# Buffer for live streaming data
history_len = 100
time_buf = collections.deque(maxlen=history_len)
roll_buf = collections.deque(maxlen=history_len)
pitch_buf = collections.deque(maxlen=history_len)

# Connect as client to PX4 stream
mav = mavutil.mavlink_connection('udpout:127.0.0.1:14550')

fig, ax = plt.subplots()
line_roll, = ax.plot([], [], label='Roll (deg)', color='r')
line_pitch, = ax.plot([], [], label='Pitch (deg)', color='b')
ax.set_ylim(-45, 45)
ax.set_xlim(0, history_len)
ax.legend(loc='upper right')
ax.set_title('Live Dynamic State Telemetry (PySINDy Input)')
ax.set_xlabel('Frame Count')
ax.set_ylabel('Angle (deg)')

def update(frame):
    msg = mav.recv_match(type='ATTITUDE', blocking=False)
    if msg:
        time_buf.append(frame)
        roll_buf.append(msg.roll * 57.2958)
        pitch_buf.append(msg.pitch * 57.2958)

        line_roll.set_data(range(len(roll_buf)), roll_buf)
        line_pitch.set_data(range(len(pitch_buf)), pitch_buf)
    return line_roll, line_pitch

# Disable cache warning by passing cache_frame_data=False
ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)

plt.show()
