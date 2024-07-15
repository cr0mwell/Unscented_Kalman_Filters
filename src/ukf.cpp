/* authors: Aaron Brown, Oleksandr Kashkevich */
#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
    is_initialized = false;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a = 5;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd = 1.4;
    
    /**
    * DO NOT MODIFY measurement noise values below.
    * These are provided by the sensor manufacturer.
    */

    // Laser measurement noise standard deviation position1 in m
    std_laspx = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd = 0.3;

    /**
    * End DO NOT MODIFY section for measurement noise values 
    */

    n_x = 5;
    n_aug = 7;
    
    // State vector x and its covariance matrix P
    x = VectorXd(n_x);
    P = MatrixXd::Identity(n_x, n_x);
    
    // Initial predicted sigma points matrix
    Xsig_pred = MatrixXd(n_x, 2*n_aug + 1);
    
    // Init weights matrix
    lambda = 3 - n_aug;
    
    weights = VectorXd(2*n_aug+1);
    weights.fill(0.5/(lambda + n_aug));
    weights(0) = lambda/(lambda + n_aug);
    
    // Init NIS for the sensors
    NIS_radar = 0.0;
    NIS_laser = 0.0;
    
    // Initialize time
    time_us = 0.0;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package, bool use_lidar_ukf)
{
    if (is_initialized)
    {
        // Prediction Function call to generate and predict sigma points
        double delta_t = (meas_package.timestamp_ - time_us) / 1000000.0;
        time_us = meas_package.timestamp_;

        Prediction(delta_t);
        
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
            UpdateRadar(meas_package);
        else
            UpdateLidar(meas_package, use_lidar_ukf);
    
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
        is_initialized = true;
        
        // Initialise the state vector and the covariance matrix
        float rho = meas_package.raw_measurements_(0);
        float phi = meas_package.raw_measurements_(1);
        float rho_h = meas_package.raw_measurements_(2);
        x << rho*cos(phi), rho*sin(phi), rho_h, rho, rho_h;
        
        P(0, 0) = pow(std_radr, 2);
        P(1, 1) = pow(std_radr, 2);
        P(2, 2) = pow(std_radrd, 2);
        P(3, 3) = pow(std_radphi, 2);
        P(4, 4) = pow(std_radrd, 2);
    
    } else
    {   
        is_initialized = true;
        
        // Initialise the state vector and the covariance matrix
        x.head(2) = meas_package.raw_measurements_;
        
        P(0, 0) = pow(std_laspx, 2);
        P(1, 1) = pow(std_laspy, 2);
        P(2, 2) = 1;
        P(3, 3) = 1;
        P(4, 4) = 1;
    }
}

void normalizeAngle(VectorXd &v, size_t i)
{
    while (v(i) > M_PI)
        v(i) -= 2.*M_PI;
    while (v(i) < -M_PI)
        v(i) += 2.*M_PI;
}

void UKF::Prediction(double delta_t)
{
    std::cout << "####  PREDICTION  ####" << std::endl;
    //************************
    // SIGMA POINTS GENERATION
    //************************
    // Sigma point matrix
    MatrixXd Xsig = MatrixXd(n_x, 2*n_x + 1);
    lambda = 3 - n_x;
    
    // Square root of P
    MatrixXd A = P.llt().matrixL();
    
    Xsig.col(0) = x;
    
    for (size_t i=0; i<n_x; ++i)
    {
        Xsig.col(i+1) = x + sqrt(lambda + n_x)*A.col(i);
        Xsig.col(i+1+n_x) = x - sqrt(lambda + n_x)*A.col(i);
    }
    
    //std::cout << "Xsig: \n" << Xsig << std::endl;
    
    //*************
    // AUGMENTATION 
    //*************
    lambda = 3 - n_aug;
    
    // Augmented mean
    VectorXd x_aug = VectorXd(n_aug);
    x_aug.head(n_x) = x;
    x_aug(n_x) = 0.0;
    x_aug(n_x+1) = 0.0;
    
    // Augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug, n_aug);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x, n_x) = P;
    P_aug(n_x, n_x) = pow(std_a, 2);
    P_aug(n_x+1, n_x+1) = pow(std_yawdd, 2);
    
    // Square root of P_aug_
    MatrixXd L = P_aug.llt().matrixL();
    
    // Augmented sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug, 2*n_aug + 1);
    Xsig_aug.col(0) = x_aug;

    for (size_t i=0; i<n_aug; ++i)
    {
      Xsig_aug.col(i+1) = x_aug + sqrt(lambda + n_aug)*L.col(i);
      Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda + n_aug)*L.col(i);
    }
    
    //std::cout << "Xsig_aug: " << Xsig_aug << std::endl;
    
    //***********************
    // SIGMA POINT PREDICTION 
    //***********************
    double p_x, p_y, v, yaw, yaw_r, nu_a, nu_yaw_r, sin_yaw, cos_yaw, pow_delta_t;
  
    for (size_t i=0; i<2*n_aug+1; ++i)
    {
        VectorXd delta_vector = VectorXd(n_x);
        VectorXd noise_vector = VectorXd(n_x);
        p_x = Xsig_aug(0, i);
        p_y = Xsig_aug(1, i);
        v = Xsig_aug(2, i);
        yaw = Xsig_aug(3, i);
        yaw_r = Xsig_aug(4, i);
        nu_a = Xsig_aug(5, i);
        nu_yaw_r = Xsig_aug(6, i);
        
        pow_delta_t = pow(delta_t, 2);
        sin_yaw = sin(yaw);
        cos_yaw = cos(yaw);

        if (fabs(yaw_r) < 1e-3)
            delta_vector << v*cos_yaw*delta_t, v*sin_yaw*delta_t, 0, yaw_r*delta_t, 0;
        else
            delta_vector << v*(sin(yaw + yaw_r*delta_t) - sin_yaw)/yaw_r, v*(-cos(yaw + yaw_r*delta_t) + cos_yaw)/yaw_r, 0, yaw_r*delta_t, 0;
        
        noise_vector << pow_delta_t*cos_yaw*nu_a*0.5, pow_delta_t*sin_yaw*nu_a*0.5, delta_t*nu_a, pow_delta_t*nu_yaw_r*0.5, delta_t*nu_yaw_r;
        Xsig_pred.col(i) = Xsig_aug.col(i).head(n_x) + delta_vector + noise_vector;
    }
    
    //std::cout << "Xsig_pred: \n" << Xsig_pred << std::endl;
    
    //******************************
    // PREDICTED MEAN AND COVARIANCE 
    //******************************
    // Predicted state mean
    x.fill(0.0);
    for (size_t i=0; i<2*n_aug+1; ++i)
        x += weights(i)*Xsig_pred.col(i);
        
    // Predicted state covariance matrix
    P.fill(0.0);
    for (size_t i=0; i<2*n_aug+1; ++i)
    {
        // State difference
        VectorXd x_diff = Xsig_pred.col(i) - x;
        
        // Angle normalization
        normalizeAngle(x_diff, 3);
        
        P += weights(i)*x_diff*x_diff.transpose();
    }
    
    std::cout << "Predicted state mean:\n" << x << std::endl;
    std::cout << "Predicted state covariance:\n" << P << std::endl;
    std::cout << "####  PREDICTION END  ####" << std::endl;
}

// Update position state values and respective covariance matrix values
void UKF::UpdateLidar(MeasurementPackage meas_package, bool use_lidar_ukf)
{
    std::cout << "####  UPDATE LIDAR  ####" << std::endl;
    n_z = 2;
    VectorXd x_diff, z_diff;
    
    // Mean of the predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    
    // LiDAR can use either simple Kalman filter or its Unscented variant
    if (use_lidar_ukf)
    {
        //################################
        // UNSCENTED KALMAN FILTER VARIANT
        //################################
        // Measurement covariance matrix S
        MatrixXd S = MatrixXd(n_z, n_z);
        S.fill(0.0);
        
        // Cross correlation matrix
        MatrixXd Tc = MatrixXd(n_x, n_z);
        Tc.fill(0.0);
        
        //****************************************************
        // SIGMA POINT TRANSFORMATION INTO A MEASUREMENT SPACE
        //****************************************************
        MatrixXd Zsig = MatrixXd(n_z, 2*n_aug + 1);
        
        for (size_t i=0; i<2*n_aug+1; ++i)
        {
          Zsig(0, i) = Xsig_pred(0, i);
          Zsig(1, i) = Xsig_pred(1, i);
        }
            
        //*****************************************
        // PREDICTED AND UPDATE MEAN AND COVARIANCE 
        //*****************************************
        // Calculate mean predicted measurement
        for (size_t i=0; i<2*n_aug+1; ++i)
            z_pred += weights(i)*Zsig.col(i);
        
        // Calculate innovation covariance matrix S and cross correlation matrix Tc
        for (size_t i=0; i<2*n_aug+1; ++i)
        {
            // Xsig_pred variance
            x_diff = Xsig_pred.col(i) - x;
            
            // Zsig variance
            z_diff = Zsig.col(i) - z_pred;
            
            // Angle normalization
            normalizeAngle(x_diff, 3);
            normalizeAngle(z_diff, 1);
            
            S += weights(i)*z_diff*z_diff.transpose();
            Tc += weights(i)*x_diff*z_diff.transpose();
        }
        
        // Measurement noise matrix R
        MatrixXd R = MatrixXd(n_z, n_z);
        R << pow(std_laspx, 2), 0, 0, pow(std_laspy, 2);
        S += R;
        
        // Kalman gain
        MatrixXd K = Tc*S.inverse();
        
        // Measurement-prediction residual
        z_diff = meas_package.raw_measurements_ - z_pred;
        
        // Angle normalization
        normalizeAngle(z_diff, 1);
        
        // Calculate NIS for LIDAR
        NIS_laser = z_diff.transpose() * S.inverse() * z_diff;
        
        // Update state mean and covariance matrix
        x += K*z_diff;
        P -= K*S*K.transpose();
    
    } else
    {
        //###############################
        // SIMPLE KALMAN FILTER VARIANT
        //###############################
        MatrixXd H = MatrixXd(n_z, n_x);
        H.fill(0.0);
        H(0, 0) = 1;
        H(1, 1) = 1;
        
        z_pred = H * x;
        VectorXd y = meas_package.raw_measurements_ - z_pred;
        MatrixXd Ht = H.transpose();
        MatrixXd R = MatrixXd(n_z, n_z);
        R << pow(std_laspx, 2), 0, 0, pow(std_laspy, 2);
        MatrixXd S = H * P * Ht + R;
        MatrixXd Si = S.inverse();
        MatrixXd PHt = P * Ht;
        MatrixXd K = PHt * Si;
        
        // Calculate NIS for LIDAR
        z_diff = meas_package.raw_measurements_ - z_pred;
        NIS_laser = z_diff.transpose() * S.inverse() * z_diff;
        
        // New estimate
        x += K*y;
        MatrixXd I = MatrixXd::Identity(n_x, n_x);
        P = (I - K*H)*P;
    }
    
    std::cout << "Predicted state:\n" << z_pred << "\nActual state:\n" << meas_package.raw_measurements_ << "\nError:\n" << z_diff << std::endl;
    std::cout << "Updated LiDAR state mean:\n" << x << std::endl;
    std::cout << "Updated LiDAR state covariance:\n" << P << std::endl;
    std::cout << "####  UPDATE LIDAR END  ####" << std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
    std::cout << "####  UPDATE RADAR  ####" << std::endl;
    n_z = 3;
    VectorXd x_diff = VectorXd(n_x);
    VectorXd z_diff = VectorXd(n_z);
    
    // Mean of the predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    
    // Measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    
    //****************************************************
    // SIGMA POINT TRANSFORMATION INTO A MEASUREMENT SPACE
    //****************************************************

    MatrixXd Zsig = MatrixXd(n_z, 2*n_aug + 1);
    VectorXd sigma_point = VectorXd(n_x);
    
    for (size_t i=0; i<2*n_aug+1; ++i)
    {
      sigma_point = Xsig_pred.col(i);
      Zsig(0, i) = sqrt(pow(sigma_point(0), 2) + pow(sigma_point(1), 2));
      Zsig(1, i) = atan2(sigma_point(1), sigma_point(0));
      Zsig(2, i) = (sigma_point(0)*cos(sigma_point(3))*sigma_point(2) + sigma_point(1)*sin(sigma_point(3))*sigma_point(2))/sqrt(pow(sigma_point(0), 2) + pow(sigma_point(1), 2));
    }
        
    //*****************************************
    // CALCULATE AND UPDATE MEAN AND COVARIANCE 
    //*****************************************
    
    // Cross correlation matrix
    MatrixXd Tc = MatrixXd(n_x, n_z);
    Tc.fill(0.0);
    
    // Calculate mean predicted measurement
    for (size_t i=0; i<2*n_aug+1; ++i)
        z_pred += weights(i)*Zsig.col(i);
    
    // Calculate innovation covariance matrix S and cross correlation matrix Tc
    for (size_t i=0; i<2*n_aug+1; ++i)
    {
        // Xsig_pred variance
        x_diff = Xsig_pred.col(i) - x;
        
        // Zsig variance
        z_diff = Zsig.col(i) - z_pred;

        // Angle normalization
        normalizeAngle(x_diff, 3);
        normalizeAngle(z_diff, 1);
        
        S += weights(i)*z_diff*z_diff.transpose();
        Tc += weights(i)*x_diff*z_diff.transpose();
    }
    
    // Measurement noise matrix R
    MatrixXd R = MatrixXd(n_z, n_z);
    R << pow(std_radr, 2), 0, 0, 0, pow(std_radphi, 2), 0, 0, 0, pow(std_radrd,2);
    S += R;
    
    // Kalman gain
    MatrixXd K = Tc*S.inverse();
    
    // Measurement-prediction residual
    z_diff = meas_package.raw_measurements_ - z_pred;
    
    // Angle normalization
    normalizeAngle(z_diff, 1);
    
    // Calculate the RADAR NIS
    NIS_radar = z_diff.transpose() * S.inverse() * z_diff;
    
    // Update state mean and covariance matrix
    x += K*z_diff;
    P -= K*S*K.transpose();
    
    std::cout << "Predicted state:\n" << z_pred << "\nActual state:\n" << meas_package.raw_measurements_ << "\nError:\n" << z_diff << std::endl;
    std::cout << "Updated RADAR state mean:\n" << x << std::endl;
    std::cout << "Updated RADAR state covariance:\n" << P << std::endl;
    std::cout << "####  UPDATE RADAR END  ####" << std::endl;
}