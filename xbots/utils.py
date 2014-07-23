import time
import numpy as np

# Tic toc constants
TICTOC_START = 0
TICTOC_COUNT = 0
TICTOC_MEAN = 0
TICTOC_MAX = -float('inf')
TICTOC_MIN = float('inf')


def convertHEXtoDEC(hexString, N):
    """
    Return 2's compliment of hexString
    """
    for hexChar in hexString:
        asciiNum = ord(hexChar)
        if not ((asciiNum >= 48 and asciiNum <= 57) or
                (asciiNum >= 65 and asciiNum <= 70) or
                (asciiNum >= 97 and asciiNum <= 102)):
            val = float('nan')
            return val

    if len(hexString) == N:
        val = int(hexString, 16)
        bits = 4*len(hexString)
        if (val & (1 << (bits-1))) != 0:
            val = val - (1 << bits)
        return val


def operatingPoint(uStar, uStarThreshold):
    """
    This function returns the steady state tick velocity given some PWM input.

    uStar: PWM input.
    uStarThreshold: Threshold on the minimum magnitude of a PWM input value

    returns: omegaStar - steady state tick velocity
    """
    # Matlab code to find beta values
    # X = [40; 50; 60]; % Air Test
    # Y = [180; 305; 400]; % Tick per second
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
    beta = [11.0, -255.0] # Air Test Results
    # beta = [0.0606, -3.1475]  # Ground Test Results

    if np.abs(uStar) <= uStarThreshold:
        omegaStar = 0.0
    elif uStar > 0:
        omegaStar = beta[0] * uStar + beta[1]
    else:
        omegaStar = -1.0 * (beta[0] * np.abs(uStar) + beta[1])

    return omegaStar


def kalman(x, u, P, A, B, C, W, V, z=np.NaN):
    """
    This function returns an optimal expected value of the state and covariance
    error matrix given an update and system parameters.

    x:   Estimate of state at time t-1.
    u:   Input at time t-1.
    P:   Estimate of error covariance matrix at time t-1.
    A:   Discrete time state tranistion matrix at time t-1.
    B:   Input to state model matrix at time t-1.
    C:   Observation model matrix at time t.
    W:   Process noise covariance at time t-1.
    V:   Measurement noise covariance at time t.
    z:   Measurement at time t.

    returns: (x,P) tuple
    x: Updated estimate of state at time t.
    P: Updated estimate of error covariance matrix at time t.

    """

    x = np.atleast_2d(x)
    u = np.atleast_2d(u)
    P = np.atleast_2d(P)
    A = np.atleast_2d(A)
    B = np.atleast_2d(B)
    x_p = np.dot(A, x) + np.dot(B, u)  # Prediction of estimated state vector
    P_p = np.dot(A, np.dot(P, A.T)) + W  # Prediction of error covariance matrix

    if np.any(np.isnan(z)):
        return (x_p, P_p)
    else:
        C = np.atleast_2d(C)
        W = np.atleast_2d(W)
        V = np.atleast_2d(V)
        z = np.atleast_2d(z)

        [M,N] = np.shape(C)

        if W.shape[0] == 1 or W.shape[1] == 1:
            W = np.diag(np.squeeze(W))

        if (V.shape[0] == 1 or V.shape[1] == 1) and not (V.shape[0] == 1 and V.shape[1] == 1):
            V = np.diag(np.squeeze(V))

        I = np.eye(N) # N x N identity matrix

        S = np.dot(C, np.dot(P_p, C.T)) + V  # Sum of error variances
        S_inv = np.linalg.inv(S)  # Inverse of sum of error variances
        K = np.dot(P_p, np.dot(C.T, S_inv))  # Kalman gain
        r = z - np.dot(C, x_p)  # Prediction residual
        w = np.dot(-K, r) # Process error
        x = x_p - w  # Update estimated state vector
        # v = z - np.dot(C, x)  # Measurement error
        if np.any(np.isnan(np.dot(K, V))):
            P = P_p
        else:
            # Updated error covariance matrix
            P = np.dot((I - np.dot(K, C)), np.dot(P_p, (I - np.dot(K, C)).T)) + np.dot(K, np.dot(V, K.T))
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
