#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF
{
public:
    /**
    * Constructor
    */
    UKF();

    /**
    * Destructor
    */
    virtual ~UKF();

    /**
    * ProcessMeasurement
    * @param meas_package The latest measurement data of either radar or laser
    */
    void ProcessMeasurement(MeasurementPackage meas_package, bool use_lidar_ukf=true);

    /**
    * Prediction Predicts sigma points, the state, and the state covariance
    * matrix
    * @param delta_t Time between k and k+1 in s
    */
    void Prediction(double delta_t);

    /**
    * Updates the state and the state covariance matrix using a laser measurement
    * @param meas_package The measurement at k+1
    */
    void UpdateLidar(MeasurementPackage meas_package, bool use_lidar_ukf);

    /**
    * Updates the state and the state covariance matrix using a radar measurement
    * @param meas_package The measurement at k+1
    */
    void UpdateRadar(MeasurementPackage meas_package);

    // initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized;

    // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    Eigen::VectorXd x;

    // state covariance matrix
    Eigen::MatrixXd P;

    // predicted sigma points matrix
    Eigen::MatrixXd Xsig_pred;
    
    // time when the state is true, in us
    long long time_us;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a;

    // Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd;

    // Laser measurement noise standard deviation position1 in m
    double std_laspx;

    // Laser measurement noise standard deviation position2 in m
    double std_laspy;

    // Radar measurement noise standard deviation radius in m
    double std_radr;

    // Radar measurement noise standard deviation angle in rad
    double std_radphi;

    // Radar measurement noise standard deviation radius change in m/s
    double std_radrd;

    // Weights of sigma points
    Eigen::VectorXd weights;

    // State dimension
    int n_x;

    // Augmented state dimension
    int n_aug;
    
    // Measurement dimension
    int n_z;

    // Sigma point spreading parameter
    double lambda;
    
    // Normalized Innovation Squared initialization
    double NIS_radar, NIS_laser;
};

#endif  // UKF_H