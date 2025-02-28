import matplotlib.pyplot as plt
import numpy as np

class lpf:
    def __init__(self, sample_rate, cutoff_freq):
        fr = sample_rate / cutoff_freq
        ohm = np.tan(np.pi / fr)
        c = 1 + 2 * np.cos(np.pi / 4) * ohm + ohm ** 2
        self.b0 = ohm ** 2 / c
        self.b1 = 2 * self.b0
        self.b2 = self.b0
        self.a1 = 2 * (ohm ** 2 - 1) / c
        self.a2 = (1 - 2 * np.cos(np.pi / 4) * ohm + ohm ** 2) / c
        self.xn1 = 0
        self.xn2 = 0

    def update(self, x):
        y = x - self.a1 * self.xn1 - self.a2 * self.xn2
        y = self.b0 * y + self.b1 * self.xn1 + self.b2 * self.xn2
        self.xn2 = self.xn1
        self.xn1 = y
        return y
    
    def reset(self):
        self.xn1 = 0
        self.xn2 = 0

# test filter
def draw_lpf(sample_rate, cutoff_freq):
    lpf_filter = lpf(sample_rate, cutoff_freq)
    x = np.linspace(0, 1, sample_rate)
    # step function
    y = np.zeros(sample_rate)
    y[sample_rate//2:] = 1
    y[sample_rate//2+10:] = 0

    y_noise = y + 0.5 * np.random.normal(0, 0.1, sample_rate)
    y_output = np.zeros(sample_rate)
    for i in range(sample_rate):
        y_output[i] = lpf_filter.update(y_noise[i])
    plt.plot(x, y_output)
    plt.plot(x, y_noise)
    plt.legend(['output', 'input'])
    plt.show()

draw_lpf(100, 20)