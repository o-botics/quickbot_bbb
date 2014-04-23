import time
import numpy as np

# Tic toc constants
TICTOC_START = 0
TICTOC_COUNT = 0
TICTOC_MEAN = 0
TICTOC_MAX = -float('inf')
TICTOC_MIN = float('inf')


def operatingPoint(uStar, uStarThreshold):
    """
    This function returns the steady state tick velocity given some PWM input.

    uStar: PWM input.
    uStarThreshold: Threshold on the minimum magnitude of a PWM input value

    returns: omegaStar - steady state tick velocity
    """
    # Matlab code to find beta values
    # X = [40; 80; 100]; % Air Test
    # Y = [0.85; 2.144; 3.5];
    #
    # r = 0.0325; % Wheel radius
    # c = 2*pi*r;
    # X = [  70;   70;   70;   75;   75;   75;   80;   80;   80; 85;     85;
    #        85;   90;   90;   90]; % Ground Test
    # Z = [4.25; 3.95; 4.23; 3.67; 3.53; 3.48; 3.19; 3.08; 2.93; 2.52; 2.59;
    #      2.56; 1.99; 2.02; 2.04]; % Time to go 1 m
    # Y = 1./(Z*c);
    # H = [X ones(size(X))];
    # beta = H \ Y
    # beta = [0.0425, -0.9504] # Air Test Results
    beta = [0.0606, -3.1475]  # Ground Test Results

    if np.abs(uStar) <= uStarThreshold:
        omegaStar = 0.0
    elif uStar > 0:
        omegaStar = beta[0] * uStar + beta[1]
    else:
        omegaStar = -1.0 * (beta[0] * np.abs(uStar) + beta[1])

    return omegaStar


def kalman(x, P, Phi, H, W, V, z):
    """
    This function returns an optimal expected value of the state and covariance
    error matrix given an update and system parameters.

    x:   Estimate of staet at time t-1.
    P:   Estimate of error covariance matrix at time t-1.
    Phi: Discrete time state tranistion matrix at time t-1.
    H:   Observation model matrix at time t.
    W:   Process noise covariance at time t-1.
    V:   Measurement noise covariance at time t.
    z:   Measurement at time t.

    returns: (x,P) tuple
    x: Updated estimate of state at time t.
    P: Updated estimate of error covariance matrix at time t.

    """
    x_p = Phi * x  # Prediction of setimated state vector
    P_p = Phi * P * Phi + W  # Prediction of error covariance matrix
    S = H * P_p * H + V  # Sum of error variances
    S_inv = 1 / S  # Inverse of sum of error variances
    K = P_p * H * S_inv  # Kalman gain
    r = z - H * x_p  # Prediction residual
    w = -K * r  # Process error
    x = x_p - w  # Update estimated state vector
    # v = z - H * x  # Measurement error
    if np.isnan(K * V):
        P = P_p
    else:
        # Updated error covariance matrix
        P = (1 - K * H) * P_p * (1 - K * H) + K * V * K
    return (x, P)


def tic():
    global TICTOC_START
    TICTOC_START = time.time()


def toc(tictocName='toc', printFlag=True):
    global TICTOC_START
    global TICTOC_COUNT
    global TICTOC_MEAN
    global TICTOC_MAX
    global TICTOC_MIN

    tictocTime = time.time() - TICTOC_START
    TICTOC_COUNT = TICTOC_COUNT + 1
    TICTOC_MEAN = tictocTime / TICTOC_COUNT + \
        TICTOC_MEAN * (TICTOC_COUNT - 1) / TICTOC_COUNT
    TICTOC_MAX = max(TICTOC_MAX, tictocTime)
    TICTOC_MIN = min(TICTOC_MIN, tictocTime)

    if printFlag:
        print tictocName + " time: " + str(tictocTime)


def tictocPrint():
    global TICTOC_COUNT
    global TICTOC_MEAN
    global TICTOC_MAX
    global TICTOC_MIN

    print "Tic Toc Stats:"
    print "Count = " + str(TICTOC_COUNT)
    print "Mean = " + str(TICTOC_MEAN)
    print "Max = " + str(TICTOC_MAX)
    print "Min = " + str(TICTOC_MIN)


def recursiveMeanVar(x, l, mu, sigma2):
    """
    This function calculates a new mean and variance given
    the current mean "mu", current variance "sigma2", current
    update count "l", and new samples "x"
    """
    m = len(x)
    n = l + m
    muPlus = l / n * mu + m / n * np.mean(x)
    if n > 1:
        sigma2Plus = 1 / (n - 1) * (
            (l - 1) * sigma2 + (m - 1) * np.var(x) + l * (
                mu - muPlus) ** 2 + m * (np.mean(x) - muPlus) ** 2)
    else:
        sigma2Plus = 0

    return (muPlus, sigma2Plus, n)
