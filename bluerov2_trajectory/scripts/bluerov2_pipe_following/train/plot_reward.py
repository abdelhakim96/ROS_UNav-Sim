from stable_baselines3.common.results_plotter import ts2xy, load_results
from matplotlib import pyplot as plt
import numpy as np
from matplotlib import rc

# rc('font', **{'family': 'serif', 'size': '16'})
# rc('text', usetex=True)


def plot_shaded(x, y):
    l = y.shape[0]
    k = 100
    a_std = []
    a_mean = []
    for i in range(l - k + 1):
        a_std.append(np.std(y[i:i + k]))
        a_mean.append(np.mean(y[i:i + k]))
    # a = np.array([y[k - 1 - i:l - i] for i in range(k)])
    # b = a.std(axis=0)
    # a_mean = a.mean(axis=0)
    a_std = np.array(a_std)/20
    a_mean = np.array(a_mean)/10
    x[x < -0.9] = 0
    reduced_x = x[k - 1:]
    # print(a.shape, reduced_x.shape, b.shape)
    # fig = plt.figure()
    plt.plot(reduced_x, a_mean, label="Reward")

    plt.grid()
    #plt.title("Mean episode reward during training")
    plt.xlabel("time steps")
    plt.ylabel("mean episode reward")
    plt.fill_between(reduced_x, a_mean + a_std, a_mean - a_std, alpha=0.4)


# dat = load_results("./log-11-02-23")
dat = load_results("./log/.")
x, y = np.cumsum(dat.l.values), dat.r.values
collided = 1*(y < -0.1)
missing = 1*(y == -0.0)
timeout = 1*(y == -0.01)
# x, y = ts2xy(load_results("./log/"), 'timesteps')
plot_shaded(x, y)
plt.legend()
plt.show()
