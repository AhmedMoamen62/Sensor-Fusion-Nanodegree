#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <string>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::string;
using std::endl;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5,5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1.5;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 2;

    /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;

    /**
   * End DO NOT MODIFY section for measurement noise values
   */

    /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
    n_x_ = 5;
    n_aug_ = 7;
    lambda_ = 3 - n_aug_;
    weights_ = VectorXd(2*n_aug_+1);
    weights_.fill(0.5/(lambda_+n_aug_));
    weights_(0.0) = lambda_/(lambda_+n_aug_);
    time_us_ = 0.0;
    Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1);
    Xsig_pred_.fill(0.0);
    is_initialized_ = false;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
    if(!is_initialized_)
    {
        string type;
        if(meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            x_(0) = meas_package.raw_measurements_(0);
            x_(1) = meas_package.raw_measurements_(1);
            type = "Lidar";
        }
        else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            while(meas_package.raw_measurements_(1)> M_PI) meas_package.raw_measurements_(1)-=2.*M_PI;
            while(meas_package.raw_measurements_(1)< -M_PI) meas_package.raw_measurements_(1)+=2.*M_PI;
            x_(0) = meas_package.raw_measurements_(0)*cos(meas_package.raw_measurements_(1));
            x_(1) = meas_package.raw_measurements_(0)*sin(meas_package.raw_measurements_(1));
            type = "Radar";
        }
        x_(2) = 0;
        x_(3) = 0;
        x_(4) = 0;
        P_ = MatrixXd(n_x_,n_x_);
        P_.fill(0);
        for(int i = 0 ; i < n_x_ ; i++)
            P_(i,i) = 1;
        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;

        cout << "Initialization From " + type << endl;
        cout << "________________________________" << endl;
    }
    Prediction((meas_package.timestamp_ - time_us_)*1e-6);
    time_us_ = meas_package.timestamp_;
    cout << "Prediction" << endl;
    cout << x_ << endl;
    cout << "________________________________" << endl;

    if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
        UpdateLidar(meas_package);
        cout << "Update From Lidar"<< endl;
        cout << "________________________________" << endl;
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
        UpdateRadar(meas_package);
        cout << "Update From Radar"<< endl;
        cout << "________________________________" << endl;
    }

    cout << x_ << endl;
    cout << "________________________________" << endl;
}

void UKF::Prediction(double delta_t) {
    /**
   * TODO: Complete this function! Estimate the object's location.
   * Modify the state vector, x_. Predict sigma points, the state,
   * and the state covariance matrix.
   */

    // prepare sigma points -----------------------------------------
    // using augumentation, because the noise term is unlinear
    // augumented state mean vector
    VectorXd x_aug_ = VectorXd(n_aug_);
    // augumented state covariance
    MatrixXd P_aug_ = MatrixXd(n_aug_,n_aug_);
    // augumented sigma point matrix
    MatrixXd Xsig_aug_ = MatrixXd(n_aug_,2*n_aug_+1);

    x_aug_.head(n_x_) = x_;
    x_aug_(n_x_) = 0; // velocity acceleration
    x_aug_(n_x_+1) = 0; // angle acceleration

    P_aug_.fill(0);
    P_aug_.topLeftCorner(n_x_,n_x_) = P_;
    P_aug_(n_x_,n_x_) = std_a_*std_a_;
    P_aug_(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;

    MatrixXd A = P_aug_.llt().matrixL();
    Xsig_aug_.col(0) = x_aug_;
    for (int i = 0 ; i < n_aug_ ; i++)
    {
        Xsig_aug_.col(i+1) = x_aug_ + sqrt(lambda_+n_aug_)*A.col(i);
        Xsig_aug_.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_+n_aug_)*A.col(i);
    }

    // sigma points prediction----------------------------
    for (int i = 0 ; i < 2*n_aug_+1 ; i++)
    {
        // extract state
        double p_x = Xsig_aug_(0,i);
        double p_y = Xsig_aug_(1,i);
        double v = Xsig_aug_(2,i);
        double yaw = Xsig_aug_(3,i);
        double yawd = Xsig_aug_(4,i);
        double nu_a = Xsig_aug_(5,i);
        double nu_yawdd = Xsig_aug_(6,i);

        // predict state
        double px_p, py_p, v_p, yaw_p, yawd_p;
        if (fabs(yawd) > 0.001)
        {
            // curve path
            px_p = p_x + v/yawd*(sin(yaw+yawd*delta_t)-sin(yaw)) + 0.5*delta_t*delta_t*cos(yaw)*nu_a;
            py_p = p_y + v/yawd*(-cos(yaw+yawd*delta_t)+cos(yaw)) + 0.5*delta_t*delta_t*sin(yaw)*nu_a;
            v_p = v + delta_t*nu_a;
            yaw_p = yaw + yawd*delta_t + 0.5*delta_t*delta_t*nu_yawdd;
            yawd_p = yawd + delta_t*nu_yawdd;
        }
        else
        {
            // straight path
            px_p = p_x + v*cos(yaw)*delta_t + 0.5*delta_t*delta_t*cos(yaw)*nu_a;
            py_p = p_y + v*sin(yaw)*delta_t + 0.5*delta_t*delta_t*sin(yaw)*nu_a;
            v_p = v + delta_t*nu_a;
            yaw_p = yaw + yawd*delta_t + 0.5*delta_t*delta_t*nu_yawdd;
            yawd_p = yawd + delta_t*nu_yawdd;
        }

        // update Xsig_pred
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }

    // predict state mean and covariance-----------------
    VectorXd final_x = VectorXd(n_x_);
    final_x.fill(0);
    MatrixXd final_P = MatrixXd(n_x_,n_x_);
    final_P.fill(0);

    for (int i = 0 ; i < 2*n_aug_+1 ; i++)
    {
        final_x += weights_(i) * Xsig_pred_.col(i);
    }

    for (int i = 0 ; i < 2*n_aug_+1 ; i++)
    {
        VectorXd x_diff = Xsig_pred_.col(i) - final_x;

        while (x_diff(3) > M_PI) x_diff(3)-=2.0*M_PI;
        while (x_diff(3) < -M_PI) x_diff(3)+=2.0*M_PI;

        final_P += weights_(i) * x_diff * x_diff.transpose();
    }


    // write result
    x_ = final_x;
    P_ = final_P;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
   * TODO: Complete this function! Use lidar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

    int n_z = 2;
    MatrixXd Zsig = MatrixXd(n_z,2*n_aug_+1);
    Zsig.fill(0);

    // predict measurements------------------------
    // transform sigma points into measurement space
    for (int i = 0 ; i < 2*n_aug_+1 ; i++)
    {
        // measurement model
        Zsig(0,i) = Xsig_pred_(0,i);   // p_x
        Zsig(1,i) = Xsig_pred_(1,i);   // p_y
    }


    // calculate predict mean and covariance in measurement space----------
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0);
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0);

    // measurement mean
    for (int i = 0 ; i < 2*n_aug_+1 ; i++)
    {
        z_pred += weights_(i)*Zsig.col(i);
    }

    // measurement covariance
    for (int i = 0; i < 2*n_aug_+1 ; i++)
    {
        VectorXd z_diff = Zsig.col(i) - z_pred;

        S += weights_(i)*z_diff*z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R << std_laspx_*std_laspx_, 0,
            0, std_laspy_*std_laspy_;
    S = S + R;


    // update state mean and covariance-------------------------------
    VectorXd z = meas_package.raw_measurements_; // true received measurement
    MatrixXd Tc = MatrixXd(n_x_,n_z); // cross correlation matrix
    Tc.fill(0);

    // calculate Tc
    for (int i = 0 ; i < 2*n_aug_+1 ; i++)
    {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        while (x_diff(3) > M_PI) x_diff(3)-=2.0*M_PI;
        while (x_diff(3) < -M_PI) x_diff(3)+=2.0*M_PI;

        // measurement difference
        VectorXd z_diff = Zsig.col(i) - z_pred;

        Tc += weights_(i)*x_diff*z_diff.transpose();
    }


    // calculate kalman gain K
    MatrixXd K = Tc * S.inverse();

    // residuals
    VectorXd z_diff = z - z_pred;

    // update state mean and covariance
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
   * TODO: Complete this function! Use radar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

    int n_z = 3;
    MatrixXd Zsig = MatrixXd(n_z,2*n_aug_+1);
    Zsig.fill(0);

    // predict measurements------------------------
    // transform sigma points into measurement space
    for (int i = 0 ; i < 2*n_aug_+1 ; i++)
    {
        // extract value
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);

        // measurement model
        double v1 = cos(yaw) * v;
        double v2 = sin(yaw) * v;
        Zsig(0,i) = sqrt(p_x * p_x + p_y * p_y);                    // r
        Zsig(1,i) = atan2(p_y, p_x);                                // phi
        Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);    // r_dot
    }


    // calculate predict mean and covariance in measurement space----------
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0);
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0);

    // measurement mean
    for (int i = 0 ; i < 2*n_aug_+1 ; i++)
    {
        z_pred += weights_(i) * Zsig.col(i);
    }

    // measurement covariance
    for (int i = 0 ; i < 2*n_aug_+1 ; i++)
    {
        VectorXd z_diff = Zsig.col(i) - z_pred;

        while (z_diff(1) > M_PI) z_diff(1)-=2.0*M_PI;
        while (z_diff(1) < -M_PI) z_diff(1)+=2.0*M_PI;

        S += weights_(i)*z_diff*z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_radr_*std_radr_, 0, 0,
            0, std_radphi_*std_radphi_, 0,
            0, 0, std_radrd_*std_radrd_;
    S = S + R;


    // update state mean and covariance-------------------------------
    VectorXd z = meas_package.raw_measurements_; // true received measurement
    MatrixXd Tc = MatrixXd(n_x_,n_z); // cross correlation matrix
    Tc.fill(0);

    // calculate Tc
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        while (x_diff(3) > M_PI) x_diff(3)-=2.0*M_PI;
        while (x_diff(3) < -M_PI) x_diff(3)+=2.0*M_PI;

        // measurement difference
        VectorXd z_diff = Zsig.col(i) - z_pred;

        while (z_diff(1) > M_PI) z_diff(1)-=2.0*M_PI;
        while (z_diff(1) < -M_PI) z_diff(1)+=2.0*M_PI;

        Tc += weights_(i)*x_diff*z_diff.transpose();
    }


    // calculate kalman gain K
    MatrixXd K = Tc * S.inverse();

    // residuals
    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1)-=2.0*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1)+=2.0*M_PI;

    // update state mean and covariance
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
}
