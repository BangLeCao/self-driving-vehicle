import numpy as np 
import cv2
import matplotlib.pyplot as plt
from sklearn.neighbors import KernelDensity
from scipy.stats import norm
import time

f = open('/home/nguyen/thesis_ws/15.txt', 'r')
input_data = f.read()
d = []
for i in range(len(input_data)):
    data = input_data[i]
    if data == ' ' and i == 1:
        pre_i = 2
    elif data == ' ':
        d.append(float(input_data[pre_i:i]))
        pre_i = i + 1

focal = 473.40625
b = 0.1195186972618103
print(len(d))
start = time.time()
d = np.array(d[len(d)/2:])
#d = np.array(d)
x_d = np.linspace(d[0], d[-1], 2000)
kde = KernelDensity(bandwidth=1.0, kernel='gaussian').fit(d[:,None])
logprob = kde.score_samples(x_d[:, None])
d_mean = x_d[np.where(logprob == logprob.max())[0][0]]
print(d_mean)
print(focal*b/d_mean)
print('Sklearn:', time.time() - start)

start = time.time()
hist, bin_edges = np.histogram(d, bins = 6)
index = np.where(hist == hist[1:].max())[0][-1]
#print d[sum(hist[0:index]):sum(hist[0:index + 1])]
d_mean = np.mean(d[sum(hist[0:index]):sum(hist[0:index + 1])])
print(d_mean)
print(focal*b/d_mean)
print('Histogram:', time.time() - start)

#start = time.time()
#density = sum(norm(xi).pdf(x_d) for xi in d)
#print x_d[np.where(density == density.max())[0][0]]
#print('Scipy:', time.time() - start)

#start = time.time()
#density = np.convolve(d, np.ones(10000), mode='valid')
#d_mean = d[np.where(density == density.max())[0][0]]
#print(d_mean)
#print(focal*b/d_mean)
#print('Convolve:', time.time() - start)

plt.fill_between(x_d, np.exp(logprob), alpha=0.5)
plt.plot(d, np.full_like(d, -0.01), '|k', markeredgewidth=1)
# plt.plot(d, np.zeros(len(d)), 'or')
plt.show()
