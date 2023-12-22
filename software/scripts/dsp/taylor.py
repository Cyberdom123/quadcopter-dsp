import math as m
import numpy as np
import matplotlib.pyplot as plt

def sin_t(x, order):
    y = np.full_like(np.array(x), 0)

    for k in range(1, order, 2):
        factor = 1 if (k - 1) / 2 % 2 == 0 else -1
        # print(k, factor)
        y += factor * (x ** k) / m.factorial(k)

    return y

def cos_t(x, order):
    y = np.full_like(np.array(x), 0)

    for k in range(0, order, 2):
        factor = 1 if (k) / 2 % 2 == 0 else -1
        # print(k, factor)
        y += factor * (x ** k) / m.factorial(k)

    return y

def main():
    samples_no = 1000
    x = np.linspace(-np.pi, np.pi, samples_no)

    y_sin    = np.sin(x)
    y_cos    = np.cos(x)
    y_taylor_sin    = x
    max_order = 22 

    for order in range(1, max_order):
        y_taylor_sin = sin_t(x, order)
        y_taylor_cos = cos_t(x, order)
        err_sin = y_sin - y_taylor_sin
        err_cos = y_cos - y_taylor_cos
        max_err_sin = np.max(np.abs(err_sin))
        max_err_cos = np.max(np.abs(err_cos))
        print(f"Order: {order}, max_error_sin: {max_err_sin}, max_error_cos: {max_err_cos}")

    plt.figure(1)
    plt.plot(x, y_sin)
    plt.plot(x, y_taylor_sin)

    plt.figure(2)
    plt.plot(x, y_cos)
    plt.plot(x, y_taylor_cos)

    plt.ylim([-3, 3])

    plt.show()

if __name__ == "__main__":
    main()