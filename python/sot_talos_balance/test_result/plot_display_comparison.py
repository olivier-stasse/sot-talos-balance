import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import misc

def derivate(liste):
	current_value = liste[1]
	prev_value = liste[0]
	deriv = []
	for k in range (2,len(liste)):
		deriv.append(current_value-prev_value)
		prev_value = current_value
		current_value = liste[k]
	return deriv

def first_local_max(liste):
	current_value = liste[1]
	prev_value = liste[0]
	max_liste = []
	for k in range (2,len(liste)):
		if liste [k] < current_value and current_value > prev_value and current_value > 0.06:
			max_liste.append(k-1)
		prev_value = current_value
		current_value = liste[k]
	return max_liste

online_data_com = np.transpose(np.loadtxt('dg_pg-comref.dat'))
online_data_dcom = np.transpose(np.loadtxt('dg_pg-dcomref.dat'))
online_data_ddcom = np.transpose(np.loadtxt('dg_pg-ddcomref.dat'))
offline_data = np.transpose(np.loadtxt('/local/imaroger/jrl-walkgen/test_result/10cm/TestNaveau2015Online64TestFGPIFull.dat'))

time_online = online_data_com [0]/1000
x_com_online = online_data_com [1]
y_com_online = online_data_com [2]
z_com_online = online_data_com [3]
x_dcom_online = online_data_dcom [1]
y_dcom_online = online_data_dcom [2]
z_dcom_online = online_data_dcom [3]
x_ddcom_online = online_data_ddcom [1]
y_ddcom_online = online_data_ddcom [2]
z_ddcom_online = online_data_ddcom [3]

time_offline = offline_data[0]
x_com_offline = offline_data[1]
y_com_offline = offline_data[2]
z_com_offline = offline_data[3]
x_dcom_offline = offline_data[5]
y_dcom_offline = offline_data[6]
z_dcom_offline = offline_data[7]
x_ddcom_offline = offline_data[9]
y_ddcom_offline = offline_data[10]
z_ddcom_offline = offline_data[11]

y_com_online_max = first_local_max(y_com_online)
y_com_offline_max = first_local_max(y_com_offline)

y_dcom_online_max = first_local_max(y_dcom_online)
y_dcom_offline_max = first_local_max(y_dcom_offline)
#print(time_online[y_dcom_online_max],time_offline[y_dcom_offline_max])

y_ddcom_online_max = first_local_max(y_ddcom_online)
y_ddcom_offline_max = first_local_max(y_ddcom_offline)

delta_t = time_online[y_com_online_max[0]]-time_offline[y_com_offline_max[0]]
i_delta_t = np.where(time_online > delta_t)[0][0]

plt.plot(time_online[0:len(time_online)-i_delta_t],
	x_com_online[i_delta_t:len(time_online)],'r',label='com X off')
plt.plot(time_online[0:len(time_online)-i_delta_t],
	y_com_online[i_delta_t:len(time_online)],'b',label='com Y off')
plt.plot(time_online[0:len(time_online)-i_delta_t],
	z_com_online[i_delta_t:len(time_online)],'g',label='com Z off')
plt.plot(time_offline,
	x_com_offline,'r--',label='com X on')
plt.plot(time_offline,
	y_com_offline,'b--',label='com Y on')
plt.plot(time_offline,
	z_com_offline,'g--',label='com Z on')
legend = plt.legend(loc='upper left')
plt.show()

################################################

delta_t = time_online[y_dcom_online_max[0]]-time_offline[y_dcom_offline_max[0]]
i_delta_t = np.where(time_online > delta_t)[0][0]

plt.plot(time_online[0:len(time_online)-i_delta_t],
	x_dcom_online[i_delta_t:len(time_online)],'r',label='com X off')
plt.plot(time_online[0:len(time_online)-i_delta_t],
	y_dcom_online[i_delta_t:len(time_online)],'b',label='com Y off')
plt.plot(time_online[0:len(time_online)-i_delta_t],
	z_dcom_online[i_delta_t:len(time_online)],'g',label='com Z off')
plt.plot(time_offline,
	x_dcom_offline,'r--',label='com X on')
plt.plot(time_offline,
	y_dcom_offline,'b--',label='com Y on')
plt.plot(time_offline,
	z_dcom_offline,'g--',label='com Z on')
legend = plt.legend(loc='upper left')
plt.show()


################################################

delta_t = time_online[y_ddcom_online_max[0]]-time_offline[y_ddcom_offline_max[0]]
i_delta_t = np.where(time_online > delta_t)[0][0]

plt.plot(time_online[0:len(time_online)-i_delta_t],
	x_ddcom_online[i_delta_t:len(time_online)],'r',label='com X off')
plt.plot(time_online[0:len(time_online)-i_delta_t],
	y_ddcom_online[i_delta_t:len(time_online)],'b',label='com Y off')
plt.plot(time_online[0:len(time_online)-i_delta_t],
	z_ddcom_online[i_delta_t:len(time_online)],'g',label='com Z off')
plt.plot(time_offline,
	x_ddcom_offline,'r--',label='com X on')
plt.plot(time_offline,
	y_ddcom_offline,'b--',label='com Y on')
plt.plot(time_offline,
	z_ddcom_offline,'g--',label='com Z on')
legend = plt.legend(loc='upper left')
plt.show()


