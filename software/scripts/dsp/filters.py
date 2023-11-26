def ema_filter(x, alpha, yPrev):
    out = alpha * x + (1-alpha)*yPrev
    yPrev = out
    return out