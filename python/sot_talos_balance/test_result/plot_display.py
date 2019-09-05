import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

data_com = np.transpose(np.loadtxt('dg_pg-comref.dat'))
data_dcom = np.transpose(np.loadtxt('dg_pg-dcomref.dat'))
data_ddcom = np.transpose(np.loadtxt('dg_pg-ddcomref.dat'))

time = data_com[0]
x_com = data_com[1]
y_com = data_com[2]
z_com = data_com[3]
x_dcom = data_dcom[1]
y_dcom = data_dcom[2]
z_dcom = data_dcom[3]
x_ddcom = data_ddcom[1]
y_ddcom = data_ddcom[2]
z_ddcom = data_ddcom[3]

plt.subplot(131)
plt.plot(time,x_com,'r',label='com X')
plt.plot(time,y_com,'b',label='com Y')
plt.plot(time,z_com,'g',label='com Z')
legend = plt.legend(loc='upper left')
plt.title("pg-com")

plt.subplot(132)
plt.plot(time,x_dcom,'r',label='dcom X')
plt.plot(time,y_dcom,'b',label='dcom Y')
plt.plot(time,z_dcom,'g',label='dcom Z')
legend = plt.legend(loc='upper left')
plt.title("pg-dcom")

plt.subplot(133)
plt.plot(time,x_ddcom,'r',label='ddcom X')
plt.plot(time,y_ddcom,'b',label='ddcom Y')
plt.plot(time,z_ddcom,'g',label='ddcom Z')
legend = plt.legend(loc='upper left')
plt.title("pg-ddcom")

plt.show()

data_LF = np.transpose(np.loadtxt('dg_pg-leftfootref.dat'))
data_RF = np.transpose(np.loadtxt('dg_pg-rightfootref.dat'))

time = data_LF[0]
x_LF = data_LF[4]
y_LF = data_LF[8]
z_LF = data_LF[12]
x_RF = data_RF[4]
y_RF = data_RF[8]
z_RF = data_RF[12]

plt.plot(time,x_LF,'r',label='LF X')
plt.plot(time,y_LF,'b',label='LF Y')
plt.plot(time,z_LF,'g',label='LF Z')
plt.plot(time,x_RF,'r--',label='RF X')
plt.plot(time,y_RF,'b--',label='RF Y')
plt.plot(time,z_RF,'g--',label='RF Z')
legend = plt.legend(loc='upper left')
plt.title("pg-feet")
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot3D(x_LF, y_LF, z_LF, 'r',label='Left foot')
ax.plot3D(x_RF, y_RF, z_RF, 'b',label='Right foot')
ax.plot3D(x_com, y_com, z_com, 'g',label='CoM')
# ax.plot3D(comX, comY, comZ, 'g',label='CoM')
legend = plt.legend(loc='upper left')
plt.show()