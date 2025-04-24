import csv
import matplotlib.pyplot as plt

# Cargo los datos
xs_real, ys_real = [], []
with open('real_tray.csv', 'r') as f:
    reader = csv.reader(f)
    for row in reader:
        xs_real.append(float(row[0]))
        ys_real.append(float(row[1]))
        
# 1) Carga los datos
xs_odom, ys_odom = [], []
with open('odom_tray.csv', 'r') as f:
    reader = csv.reader(f)
    for row in reader:
        xs_odom.append(float(row[0]))
        ys_odom.append(float(row[1]))

# grafico
plt.figure()
plt.plot(xs_real, ys_real, marker='o', linestyle='-', label='Real')
plt.plot(xs_odom, ys_odom, marker='s', linestyle='-', label='Odometría')
plt.axis('equal')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Comparación Trayectorias')
plt.legend()
plt.show()

