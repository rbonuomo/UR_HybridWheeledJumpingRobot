import numpy as np
import matplotlib.pyplot as plt

max_noise = 0.4
sigma_values = np.arange(0, max_noise+0.01, 0.02)

errors_array_mpc = np.load("errors_array_mpc.npy")
errors_array_pd = np.load("errors_array_pd.npy")

mean_array_mpc = np.mean(errors_array_mpc, axis=1)
std_array_mpc = np.std(errors_array_mpc, axis=1)

mean_array_pd = np.mean(errors_array_pd, axis=1)
std_array_pd = np.std(errors_array_pd, axis=1)

plt.plot(sigma_values, mean_array_mpc)
plt.fill_between(sigma_values,mean_array_mpc-std_array_mpc,mean_array_mpc+std_array_mpc,alpha=.1)

plt.plot(sigma_values, mean_array_pd)
plt.fill_between(sigma_values,mean_array_pd-std_array_pd,mean_array_pd+std_array_pd,alpha=.1)

plt.legend(["MPC", "PD"])
plt.ylabel(r'Final State L2-distance to x*',fontsize ='16')
plt.xlabel(r'Noise Strength ($\sigma$)',fontsize ='16')
plt.grid(True)
#plt.savefig('results/' + folder + f"/noise_plot.eps")
#plt.savefig('results/' + folder + f"/noise_plot.png", dpi=300)
plt.show()