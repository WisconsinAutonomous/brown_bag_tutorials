import numpy as np
import matplotlib.pyplot as plt

### RESOURCES
# paper: https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf


## PREDICT
def predict(x, A, B, u, P, Q):
    # predict state estimate
    x_k_prime = A.dot(x) + B.dot(u)

    # predict uncertainty
    P_k_prime = A.dot(P.dot(A.T)) + Q

    predicted = (x_k_prime, P_k_prime)
    return predicted

## UPDATE
def update(x, P, H, R, z):

    # compute Kalman gain
    K = P.dot(H.T) / (H.dot(P.dot(H.T)) + R)

    # update state estimate
    x_k_hat = x + K.dot((z - H.dot(x)))

    # update state uncertainty
    I = np.ones(K.shape)
    P_k_hat = (I - K.dot(H))*P

    updated = (x_k_hat, P_k_hat, K)
    return updated



if __name__ == "__main__":

    ## INTIALIZATION
    x = np.array([[0]]) # states
    A = np.array([[1]]) # state timestep transformation
    P = np.array([[1]]) # state uncertainty covariance

    u = np.array([[0]]) # control inputs
    B = np.array([[1]]) # control input timestep transformation

    Q_scalar = 1e-2
    Q_prep = [Q_scalar**2 for i in range(x.shape[0])]
    Q = np.diag(Q_prep) # process noise covariance, when smaller filter will weight process model more heavily in estimation

    z = np.array([[1]]) # measurements
    H = np.array([[1]]) # state to measurement space transformation

    R_scalar = 1
    R_prep = [R_scalar**2 for i in range(P.shape[0])] # measurement noise covariance, when smaller filter will weight measurement data more heavily in estimation
    R = np.diag(R_prep)

    ## SIMULATION
    numMeasurements = 50
    meanMeasurements = 5 # true velocity
    sdMeasurements = 1
    np.random.seed(0)
    measurements = np.random.normal(meanMeasurements, sdMeasurements, numMeasurements)

    # data collection
    data_x = []
    data_P = []
    data_K = []
    data_velocity = []

    data_x.append(x)
    data_P.append(P)
    data_velocity.append(meanMeasurements)

    # run filter for each measurement
    for i in range(numMeasurements):

        # get new measurement
        z = measurements[i]

        # predict
        x, P = predict(x, A, B, u, P, Q)

        # update
        x, P, dataK = update(x, P, H, R, z)

        data_x.append(x[0])
        data_P.append(P[0])
        data_K.append(dataK[0])
        data_velocity.append(meanMeasurements)

    ## PLOT
    fig, ax = plt.subplots(1, 3)
    fig.suptitle("Kalman Filter for Velocity Data")

    iterationRange = range(numMeasurements+1)
    measurementRange = range(1, numMeasurements+1)

    ax[0].scatter(measurementRange, measurements, color="orange", marker="+", label="Noisy Measurements")
    ax[0].plot(iterationRange, data_x[0:], color="blue", label="State Estimate")
    ax[0].plot(iterationRange, data_velocity, color="green", label="True Velocity")
    ax[1].plot(iterationRange, data_P, color="red", label="Uncertainty")
    ax[2].plot(measurementRange, data_K, color="purple", label="Kalman Gain")

    ax[0].set_title(f"State Estimation: Q={Q_scalar}, R={R_scalar}")
    ax[0].set_xlabel("Iteration")
    ax[0].set_ylabel("Velocity")
    ax[0].legend()

    ax[1].set_title("Estimation Uncertainty")
    ax[1].set_xlabel("Iteration")
    ax[1].set_ylabel("P")

    ax[2].set_title("Kalman Gain")
    ax[2].set_xlabel("Iteration")
    ax[2].set_ylabel("K")

    plt.show()
