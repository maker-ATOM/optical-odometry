import matplotlib.pyplot as plt
import numpy as np
import math

Mx = 1
My = 1
Mly = 1
Mlx = 0

sensor_yaw = np.linspace(0, 2 * math.pi, 50)
# sensor_yaw = np.linspace(0, 0, 50)

sensor_pose_yaw = math.atan2(Mly, Mlx)
# sensor_pose_yaw = np.linspace(0, 2 * math.pi, 50)

r = math.sqrt((Mlx ** 2) + (Mly ** 2))
# r = 1

# --- Math Models --- #
Rx = (Mx * np.cos(sensor_yaw)) - (My * np.sin(sensor_yaw))
Ry = (Mx * np.sin(sensor_yaw)) + (My * np.cos(sensor_yaw))
Rz = ((My * np.cos(sensor_pose_yaw - sensor_yaw)) - (Mx * np.cos(sensor_yaw - sensor_pose_yaw + (math.pi/2)))) / r
# --- Math Models --- #

plt.plot(sensor_yaw, Rx, label=f'Rx', color='blue')
plt.plot(sensor_yaw, Ry, label=f'Ry', color='green')
plt.plot(sensor_yaw, Rz, label=f'Rz', color='red')


points_to_mark = [0, math.pi / 2, math.pi, math.pi * 1.5, 2 * math.pi]

for point in points_to_mark:
    idx = np.abs(sensor_yaw - point).argmin()
    plt.scatter(sensor_yaw[idx], Rz[idx], color='black', marker='o')
    plt.annotate(f'Rz={Rz[idx]:.2f}', (sensor_yaw[idx], Rz[idx]), textcoords="offset points", xytext=(0,5), ha='center')


plt.grid(True) 
plt.xlabel('x-axis')
plt.ylabel('y-axis')
plt.title('Math Models')
plt.legend()


plt.show()
