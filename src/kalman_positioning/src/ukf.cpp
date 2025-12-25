#include "kalman_positioning/ukf.hpp"
#include <iostream>
#include <map>

/**
 * STUDENT ASSIGNMENT: Unscented Kalman Filter Implementation
 * 
 * This file contains placeholder implementations for the UKF class methods.
 * Students should implement each method according to the UKF algorithm.
 * 
 * Reference: Wan, E. A., & Van Der Merwe, R. (2000). 
 * "The Unscented Kalman Filter for Nonlinear Estimation"
 */

// ============================================================================
// CONSTRUCTOR
// ============================================================================

/**
 * @brief Initialize the Unscented Kalman Filter
 * 
 * STUDENT TODO:
 * 1. Initialize filter parameters (alpha, beta, kappa, lambda)
 * 2. Initialize state vector x_ with zeros
 * 3. Initialize state covariance matrix P_ 
 * 4. Set process noise covariance Q_
 * 5. Set measurement noise covariance R_
 * 6. Calculate sigma point weights for mean and covariance
 */
UKF::UKF(double process_noise_xy, double process_noise_theta,
         double measurement_noise_xy, int num_landmarks)
    : nx_(5), nz_(2) {
    
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    x_ = Eigen::VectorXd::Zero(nx_); 
    P_ = Eigen::MatrixXd::Identity(nx_, nx_);
    
    Q_ = Eigen::MatrixXd::Zero(nx_, nx_);
    Q_(0,0) = process_noise_xy;
    Q_(1,1) = process_noise_xy;
    Q_(2,2) = process_noise_theta;
    

    R_ = Eigen::MatrixXd::Zero(nz_, nz_);
    R_(0,0) = measurement_noise_xy;
    R_(1,1) = measurement_noise_xy;


    lambda_ = ALPHA * ALPHA * (nx_ + KAPPA) - nx_;
    gamma_  = std::sqrt(nx_ + lambda_);

    int num_sigma = 2 * nx_ + 1;


    Wm_.resize(num_sigma);
    Wc_.resize(num_sigma);

    Wm_[0] = lambda_ / (nx_ + lambda_);
    Wc_[0] = lambda_ / (nx_ + lambda_) + (1.0 - ALPHA*ALPHA + BETA);

    double w = 1.0 / (2.0 * (nx_ + lambda_));
    for (int i = 1; i < num_sigma; ++i) {
        Wm_[i] = w;
        Wc_[i] = w;
    }
    
    
}

// ============================================================================
// SIGMA POINT GENERATION
// ============================================================================

/**
 * @brief Generate sigma points from mean and covariance
 * 
 * STUDENT TODO:
 * 1. Start with the mean as the first sigma point
 * 2. Compute Cholesky decomposition of covariance
 * 3. Generate 2*n symmetric sigma points around the mean
 */
std::vector<Eigen::VectorXd> UKF::generateSigmaPoints(const Eigen::VectorXd& mean,
                                                       const Eigen::MatrixXd& cov) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    const int n = nx_;    
    

    std::vector<Eigen::VectorXd> sigma_points;
    sigma_points.reserve(2 * n + 1);                                                    
    sigma_points.push_back(mean); 

    Eigen::LLT<Eigen::MatrixXd> llt(cov);
    Eigen::MatrixXd L = llt.matrixL();
    
    
    for(int i = 0; i < n; i++){
        Eigen::VectorXd offset = gamma_ * L.col(i);
        sigma_points.push_back(mean + offset);
        sigma_points.push_back(mean - offset);
    }

    return sigma_points;
}

// ============================================================================
// PROCESS MODEL
// ============================================================================

/**
 * @brief Apply motion model to a state vector
 * 
 * STUDENT TODO:
 * 1. Updates position: x' = x + dx, y' = y + dy
 * 2. Updates orientation: theta' = theta + dtheta (normalized)
 * 3. Updates velocities: vx' = dx/dt, vy' = dy/dt
 */
Eigen::VectorXd UKF::processModel(const Eigen::VectorXd& state, double dt,
                                  double dx, double dy, double dtheta) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    
    Eigen::VectorXd new_state = state;
    
    double x     = state(0);
    double y     = state(1);
    double theta = state(2);
    
    new_state(0) = x + dx;
    new_state(1) = y + dy;

    double new_theta = normalizeAngle(theta + dtheta);
    new_state(2) = new_theta;
    new_state(3) = dx / dt;   
    new_state(4) = dy / dt;                                  

    return new_state;
}

// ============================================================================
// MEASUREMENT MODEL
// ============================================================================

/**
 * @brief Predict measurement given current state and landmark
 * 
 * STUDENT TODO:
 * 1. Calculate relative position: landmark - robot position
 * 2. Transform to robot frame using robot orientation
 * 3. Return relative position in robot frame
 */
Eigen::Vector2d UKF::measurementModel(const Eigen::VectorXd& state, int landmark_id) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    
    if (landmarks_.find(landmark_id) == landmarks_.end()) {
        return Eigen::Vector2d::Zero();
    }
    std::pair<double, double> position = landmarks_.find(landmark_id)->second; 
    
    double lx = position.first;
    double ly = position.second;

    double x_robot = state(0);
    double y_robot = state(1);
    double theta   = state(2); 

    double dx = lx - x_robot;
    double dy = ly - y_robot;

    // Transform to robot frame (rotate by -theta)
    double c = std::cos(theta);
    double s = std::sin(theta);

    Eigen::Vector2d result;
    result(0) =  c * dx + s * dy;   // forward (x in robot frame)
    result(1) = -s * dx + c * dy; 

    return result;
}

// ============================================================================
// ANGLE NORMALIZATION
// ============================================================================

double UKF::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// ============================================================================
// PREDICTION STEP
// ============================================================================

/**
 * @brief Kalman Filter Prediction Step (Time Update)
 * 
 * STUDENT TODO:
 * 1. Generate sigma points from current state and covariance
 * 2. Propagate each sigma point through motion model
 * 3. Calculate mean and covariance of predicted sigma points
 * 4. Add process noise
 * 5. Update state and covariance estimates
 */
void UKF::predict(double dt, double dx, double dy, double dtheta) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    std::vector<Eigen::VectorXd> sigma_points = generateSigmaPoints(x_, P_);

    const int num_sigma = static_cast<int>(sigma_points.size());

    std::vector<Eigen::VectorXd> sigma_points_pred(num_sigma);
    for (int i = 0; i < num_sigma; ++i) {
        sigma_points_pred[i] = processModel(sigma_points[i], dt, dx, dy, dtheta);
    }

    // 3. Compute predicted mean state
    Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(nx_);
    for (int i = 0; i < num_sigma; ++i) {
        x_pred += Wm_[i] * sigma_points_pred[i];
    }

    // 4. Compute predicted covariance
    Eigen::MatrixXd P_pred = Eigen::MatrixXd::Zero(nx_, nx_);
    for (int i = 0; i < num_sigma; ++i) {
        Eigen::VectorXd diff = sigma_points_pred[i] - x_pred;
        // Make sure angle difference is wrapped to [-pi, pi]
        diff(2) = normalizeAngle(diff(2));
        P_pred += Wc_[i] * diff * diff.transpose();
    }

    // 5. Add process noise
    P_pred += Q_;

    // 6. Update internal state and covariance
    x_ = x_pred;
    P_ = P_pred;
    
}

// ============================================================================
// UPDATE STEP
// ============================================================================

/**
 * @brief Kalman Filter Update Step (Measurement Update)
 * 
 * STUDENT TODO:
 * 1. Generate sigma points
 * 2. Transform through measurement model
 * 3. Calculate predicted measurement mean
 * 4. Calculate measurement and cross-covariance
 * 5. Compute Kalman gain
 * 6. Update state with innovation
 * 7. Update covariance
 */
void UKF::update(const std::vector<std::tuple<int, double, double, double>>& landmark_observations) {
    std::cout << "UKF::update() called with "
          << landmark_observations.size()
          << " observations" << std::endl;

    if (landmark_observations.empty()) {
        return;
    }

    for (const auto& obs : landmark_observations) {
        int landmark_id;
        double z_x, z_y, noise_cov;
        std::tie(landmark_id, z_x, z_y, noise_cov) = obs;

        // Skip if we don't know this landmark's position
        if (!hasLandmark(landmark_id)) {
            continue;
        }

        // 1. Generate sigma points from current (already-updated) state and covariance
        std::vector<Eigen::VectorXd> sigma_points = generateSigmaPoints(x_, P_);
        const int num_sigma = static_cast<int>(sigma_points.size());

        // 2. Transform sigma points through measurement model z = h(x, landmark_id)
        std::vector<Eigen::Vector2d> z_sigma(num_sigma);
        for (int i = 0; i < num_sigma; ++i) {
            z_sigma[i] = measurementModel(sigma_points[i], landmark_id);
        }

        // 3. Predicted measurement mean
        Eigen::Vector2d z_pred = Eigen::Vector2d::Zero();
        for (int i = 0; i < num_sigma; ++i) {
            z_pred += Wm_[i] * z_sigma[i];
        }

        // 4. Measurement covariance S and cross-covariance Tc
        Eigen::Matrix2d S = Eigen::Matrix2d::Zero();
        Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(nx_, nz_); // cross-cov: state x measurement (5x2)

        for (int i = 0; i < num_sigma; ++i) {
            // innovation in measurement space
            Eigen::Vector2d z_diff = z_sigma[i] - z_pred;

            // innovation in state space
            Eigen::VectorXd x_diff = sigma_points[i] - x_;
            // wrap heading angle
            x_diff(2) = normalizeAngle(x_diff(2));

            S  += Wc_[i] * z_diff * z_diff.transpose();
            Tc += Wc_[i] * x_diff * z_diff.transpose();
        }

        // 4b. Add measurement noise for this observation
        Eigen::Matrix2d R_obs = Eigen::Matrix2d::Zero();
        if (noise_cov > 0.0) {
            R_obs(0,0) = noise_cov;
            R_obs(1,1) = noise_cov;
        } else {
            // fall back to default measurement noise
            R_obs = R_;
        }
        S += R_obs;

        // 5. Kalman gain
        Eigen::MatrixXd K = Tc * S.inverse();  // (5x2) * (2x2) = (5x2)

        // 6. Innovation (measurement residual)
        Eigen::Vector2d z_meas;
        z_meas << z_x, z_y;
        Eigen::Vector2d y = z_meas - z_pred;

        // 6b. State update
        x_ += K * y;
        x_(2) = normalizeAngle(x_(2));  // keep theta within [-pi, pi]

        // 7. Covariance update
        P_ -= K * S * K.transpose();
    }
    
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    
    
}

// ============================================================================
// LANDMARK MANAGEMENT
// ============================================================================

void UKF::setLandmarks(const std::map<int, std::pair<double, double>>& landmarks) {
    landmarks_ = landmarks;
}

bool UKF::hasLandmark(int id) const {
    return landmarks_.find(id) != landmarks_.end();
}

void UKF::setState(const Eigen::VectorXd &state)
{
    // assume correct size (5), or you can add a check
    x_ = state;
}
