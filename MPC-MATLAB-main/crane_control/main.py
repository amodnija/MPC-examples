from apm import *

# server and application
s = 'http://byu.apmonitor.com'
a = 'crane_pendulum'

# clear application
apm(s,a,'clear all')

# load model and data file
apm_load(s,a,'pendulum.apm')
csv_load(s,a,'pendulum.csv')

# option: dynamic control mode
apm_option(s,a,'nlc.imode',6)

# classify: manipulated variable
apm_info(s,a,'MV','u')

# option: let optimizer use MV to minimize objective
apm_option(s,a,'u.status',1)

# solve
output = apm(s,a,'solve')
print(output)

# retrieve results
z = apm_sol(s,a)

import matplotlib.pyplot as plt
plt.figure(1)
plt.subplot(4,1,1)
plt.plot(z['time'],z['u'],'r-',linewidth=2)
plt.ylabel('Force on Cart')
plt.legend('u')

plt.subplot(4,1,2)
plt.plot(z['time'],z['y'],'b--',linewidth=2)
plt.ylabel('Cart Position')
plt.legend('y')

plt.subplot(4,1,3)
plt.plot(z['time'],z['v'],'g:',linewidth=2)
plt.ylabel('Cart Velocity')
plt.legend('v')

plt.subplot(4,1,4)
plt.plot(z['time'],z['theta'],'m.-',linewidth=2)
plt.plot(z['time'],z['q'],'k.',linewidth=2)
plt.ylabel('Pendulum Mass')
plt.legend(['theta (Angle)','q (Angle Rate)'])
plt.xlabel('Time')
plt.show()
