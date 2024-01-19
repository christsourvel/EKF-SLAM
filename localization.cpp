#include <iostream>
//#include <Eigen/Dense>
#include "/Users/christos/Documents/headers/Eigen/Dense"

using namespace Eigen;

std::vector<std::pair<double, double> > landmark_distances;

VectorXd kinematic_update(const VectorXd& pose, const VectorXd& velocity) 
{
    double dt = 0.1;  
    VectorXd new_pose(3);
    double v_x = velocity(0);
    double v_y = velocity(1);  
    double omega = velocity(2);

    // Update the pose (x, y, theta)
    new_pose(0) = pose(0) + (v_x * std::cos(pose(2)) * dt) - (v_y * std::sin(pose(2)) * dt);    
    new_pose(1) = pose(1) + (v_x * std::sin(pose(2)) * dt) + (v_y * std::cos(pose(2)) * dt);  
    new_pose(2) = pose(2) + omega * dt;                                                   

    return new_pose;
}

MatrixXd motion_jacobian(const VectorXd& pose, const VectorXd& velocity) 
{
    double dt = 0.1;  
    double v_x = velocity(0);
    double v_y = velocity(1);   
    double omega = velocity(1);
    double theta = pose(2);

    Eigen::Matrix3d Gx;                                                 // change - and 1 in dt
        Gx << 1, 0, v_x * sin(theta) * dt - v_y * cos(theta) * dt,
              0, 1, v_x * cos(theta) * dt - v_y * sin(theta) * dt,
              0, 0, dt;
    
    return Gx;
}

MatrixXd noise_transformation(const VectorXd& pose, const VectorXd& velocity)
{
    double dt = 0.1;  
    double v_x = velocity(0);
    double v_y = velocity(1);
    double omega = velocity(1);
    double theta = pose(2);

    Eigen::Matrix3d Vx;
        Vx << cos(theta) * dt, - sin(theta) * dt, 0,
              sin(theta) * dt, cos(theta) * dt, 0,
              0, 0, 1;

    return Vx;
}

void predictionStep(VectorXd& state_vector, MatrixXd& Sigma, const VectorXd& velocity, const Matrix3d& Q) 
{
    // Calculating jacobian of motion model
    MatrixXd Gt = motion_jacobian(state_vector.head(3), velocity); 

    // Noise Transformation into State Space
    MatrixXd Vt = noise_transformation(state_vector.head(3), velocity);
    MatrixXd Qt = Vt * Q * Vt.transpose();
    
    // State Prediction
    state_vector.head(3) = kinematic_update(state_vector.head(3), velocity);
    std::cout<<"State after predict: "<<state_vector.transpose()<<std::endl;
    // Covariance Prediction
    Sigma = (Gt * Sigma * Gt.transpose()) + Q;
}


bool data_association(const VectorXd& state_vector, const VectorXd& measurements) 
{
    double x = state_vector(0);
    double y = state_vector(1);
    double theta = state_vector(2);
    double range = measurements(0);
    double bearing = measurements(1);

    double x_land = x + range * cos(theta + bearing);
    double y_land = y + range * sin(theta + bearing);

    double association_distance_threshold = 1.9; 
    double least_distance_square = std::pow(association_distance_threshold, 2);

    // Iterate through all of the cones in the current map
    for (size_t i = 0; i < landmark_distances.size(); ++i) 
    {
        const auto& pair = landmark_distances[i];
        double current_distance_square = std::pow(x_land - pair.first, 2) + std::pow(y_land - pair.second, 2);
    
        if(current_distance_square < least_distance_square) 
        {
            return true;  
        }
    }
    return false;  
}

// UPDATE STEP
void updateStep(VectorXd& state_vector, MatrixXd& Sigma, const VectorXd& measurements, const MatrixXd& Rt) 
{
    double x = state_vector(0);
    double y = state_vector(1);
    double theta = state_vector(2);
    
    // Data association
    data_association(state_vector, measurements);
    
    // Perception measurements
    double range = measurements(0);
    double bearing = measurements(1);
    MatrixXd zt(2,1);
        zt << range, 
              bearing;

    int matched_landmarks = 1;
    for(int i=0; i<matched_landmarks; ++i)
    {
            double x_land = x + range * cos(bearing + theta);
            double y_land = y + range * sin(bearing + theta);
            landmark_distances.push_back(std::make_pair(x_land, y_land));
            double dx = x_land - x;
            double dy = y_land - y;

            MatrixXd d(2,1);
                d << dx,
                     dy;

            double q = (d.transpose() * d).value();
            double q_sqrt = sqrt(q);

            // EXPECTED OBSERVATION
            MatrixXd zt_exp(2,1);
                zt_exp << q_sqrt,
                     atan2(dy,dx);
            
            MatrixXd Ht(2,3);
               Ht << - q_sqrt * dx, - q_sqrt * dy, 0,
                        dy, - dx, - q; 

        // KALMAN GAIN
        MatrixXd Kt = Sigma * Ht.transpose() * ((Ht * Sigma * Ht.transpose()) + Rt).inverse();
        
        // FINAL STATE VectorXd
        state_vector = state_vector + Kt * (zt - zt_exp); 
        std::cout<<"State after update: "<<state_vector.transpose()<<std::endl;
        // FINAL COV MATRIXXd
        Sigma = (MatrixXd::Identity(state_vector.size(), state_vector.size()) - Kt * Ht) * Sigma;
    }
}
 
 
// random velocity "measurements"
VectorXd generateRandomMotion() 
{
    VectorXd velocity(3);
    velocity << 1.0 + 0.2 * rand() / RAND_MAX,   // v_x
                1.0 + 0.2 * rand() / RAND_MAX,   // v_y
                0.1 * (2.0 * rand() / RAND_MAX - 1.0);  // omega

    return velocity;
}

// random range - bearing "measurements"
VectorXd generateRandomMeasurement() 
{
    VectorXd measurement(2);
        measurement << 5.0 * rand() / RAND_MAX,
                       5.0 * (2.0 * rand() / RAND_MAX - 1.0);
    return measurement;
}

int main() 
{
    // POSE, COV, NOISE INITIALIZATION
    VectorXd state_vector(3);                        // x,y,theta,landmarks,colors(?)
        state_vector << 0, 0, 0;      
   
    MatrixXd Sigma(3,3);                             // covariances
        Sigma << 0.5, 0, 0,
                 0, 0.1, 0,
                 0, 0, 0.1;   

    Matrix3d Q;                                      // model noise
        Q << 0.01, 0, 0,
             0, 0.01, 0,
             0, 0, 0.01;  
    
    Matrix2d Rt;                                      // sensor noise
        Rt << 0.1, 0,
             0, 0.1;
     
 int num_steps = 5;

    for (int step = 0; step < num_steps; ++step) {
        
        // Testing
        VectorXd velocity = generateRandomMotion();
        VectorXd measurements = generateRandomMeasurement();
        
        // Prediction step
        predictionStep(state_vector, Sigma, velocity, Q);

        // Update step
        updateStep(state_vector, Sigma, measurements, Rt);

        // Estimated State
        std::cout<<"End of state: "<<step+1<<std::endl;
        //std::cout << "Step: " << step << ", Estimated State: " << state_vector.transpose() << std::endl;
    }

    return 0;
}
