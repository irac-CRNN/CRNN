import csv
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

with open('data.csb', 'r') as f:
    joint_data = csv.reader(f)
    for i,line in enumerate(joint_data):
        if i == 0:
            base_control = line
        elif i == 1:
            base_robot = line
        elif i == 2:
            shoulder_control = line
        elif i == 3:
            shoulder_robot = line
        elif i == 4:
            elbow_control = line
        elif i == 5:
            elbow_robot = line
        elif i == 6:
            wrist3_control = line
        elif i == 7:
            wrist3_robot = line
        elif i == 8:
            time_check = line

for i in range(len(base_control)):
    base_control[i] = float(base_control[i])
    base_robot[i] = float(base_robot[i])
    shoulder_control[i] = float(shoulder_control[i])
    shoulder_robot[i] = float(shoulder_robot[i])
    elbow_control[i] = float(elbow_control[i])
    elbow_robot[i] = float(elbow_robot[i])
    wrist3_control[i] = float(wrist3_control[i])
    wrist3_robot[i] = float(wrist3_robot[i])

ax=plt.subplot(1, 1, 1)
ax.grid(True, axis = 'x')
ax.xaxis.set_major_locator(ticker.MultipleLocator(0.5))
ax.xaxis.set_minor_locator(ticker.MultipleLocator(0.5))
plt.ylim([-2, 2])
plt.plot(time_check, base_control, label='Theta_1')
plt.plot(time_check, base_robot, label='Base', linestyle='--')
plt.plot(time_check, shoulder_control, label='Theta_2')
plt.plot(time_check, shoulder_robot, label='Shoulder', linestyle='--')
plt.plot(time_check, elbow_control, label='Theta_3')
plt.plot(time_check, elbow_robot, label='Elbow', linestyle='--')
plt.plot(time_check, wrist3_control, label='Theta_6')
plt.plot(time_check, wrist3_robot, label='Wrist3', linestyle='--')
plt.xlabel('Time', fontsize=15)
plt.ylabel('Deg', fontsize=15)
plt.legend(loc = 2, fontsize=15)
plt.show()
