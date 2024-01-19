#include <iostream>
//#include <Eigen/Dense>
#include "/Users/christos/Documents/headers/Eigen/Dense"
#include <fstream>

using namespace Eigen;

// Global vector to store landmark distances
std::vector<std::pair<double, double> > landmark_distances;

// Function to perform kinematic update on pose
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

// Function to compute the motion model Jacobian
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

// Function to compute noise transformation into state space
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

// Function to update covariance matrix
MatrixXd covariance_update(MatrixXd& Sigma, const MatrixXd& Gt, int state_size)
{
    // Utilize block operations
    int N = state_size - 3;
    MatrixXd Sigma_vv = Sigma.topLeftCorner(3, 3);
    MatrixXd Sigma_mm = Sigma.bottomRightCorner(N, N);
    MatrixXd Sigma_vm = Sigma.topRightCorner(3, N);
    MatrixXd Sigma_mv = Sigma.bottomLeftCorner(N, 3);
  
    // Perform operations
    Eigen::MatrixXd Sigma_vv_new = Gt * Sigma_vv * Gt.transpose();
    Eigen::MatrixXd Sigma_mm_new = Sigma_mm;  
    Eigen::MatrixXd Sigma_vm_new = Gt * Sigma_vm;
    Eigen::MatrixXd Sigma_mv_new = Sigma_vm_new.transpose();  

    // Reconstruct Sigma
    Sigma.topLeftCorner(3, 3) = Sigma_vv_new;
    Sigma.bottomRightCorner(N, N) = Sigma_mm_new;
    Sigma.topRightCorner(3, N) = Sigma_vm_new;
    Sigma.bottomLeftCorner(N, 3) = Sigma_mv_new; 

    return Sigma;
}

// Function to perform the prediction step
void predictionStep(VectorXd& state_vector, MatrixXd& Sigma, const VectorXd& velocity, const Matrix3d& Q) 
{
    // Calculating jacobian of motion model
    MatrixXd Gt = motion_jacobian(state_vector.head(3), velocity); 

    // Noise Transformation into State Space
    MatrixXd Vt = noise_transformation(state_vector.head(3), velocity);
    MatrixXd Qt = Vt * Q * Vt.transpose();
    
    // State Prediction
    state_vector.head(3) = kinematic_update(state_vector.head(3), velocity);

    // Covariance Prediction
    Sigma = covariance_update(Sigma, Gt, state_vector.size());
}

// Function to perform data association
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

// Function to add new landmarks
void add_new_landmarks(VectorXd& state_vector, MatrixXd& Sigma, const VectorXd& measurements, int unmatched)
{
    double x = state_vector(0);
    double y = state_vector(1);
    double theta = state_vector(2);
    double range = measurements(0);
    double bearing = measurements(1);
    
    for(int i = 0; i < unmatched; ++i)
    {
        double x_land = x + range * cos(theta + bearing);
        double y_land = y + range * sin(theta + bearing);
        landmark_distances.push_back(std::make_pair(x_land, y_land));

        VectorXd new_state_vector(state_vector.size() + 2);
        new_state_vector << state_vector, x_land, y_land;

        MatrixXd Hu_inv(2,3);
            Hu_inv << 1, 0, -1 * range * sin(theta + bearing),
                      0, 1, range * cos(theta + bearing);

        MatrixXd H_inv = MatrixXd::Zero(2,state_vector.size());
        H_inv.block(0, 0, 2, 3) = Hu_inv;

        MatrixXd Hi_inv(2,2);
            Hi_inv << cos(theta + bearing), -1 * range * sin(theta + bearing),
                      sin(theta + bearing), range * cos(theta + bearing);

        Matrix2d Rt;                              
        Rt << 0.1, 0,
             0, 0.1;
        
        // Expand the covariance matrix
        MatrixXd new_Sigma = MatrixXd::Zero(Sigma.rows() + 2, Sigma.cols() + 2);
        new_Sigma.block(0, 0, Sigma.rows(), Sigma.cols()) = Sigma;

        new_Sigma.block(Sigma.rows(), 0, 2, Sigma.cols()) = H_inv * Sigma;
        new_Sigma.block(0, Sigma.cols(), Sigma.rows(), 2) = Sigma * H_inv.transpose();
        new_Sigma.block(Sigma.rows(), Sigma.cols(), 2, 2) = H_inv * Sigma * H_inv.transpose() + Hi_inv * Rt * Hi_inv.transpose();

        state_vector = new_state_vector;
        Sigma = new_Sigma;
    }
}

// UPDATE STEP
void updateStep(VectorXd& state_vector, MatrixXd& Sigma, const VectorXd& measurements, const MatrixXd& R) 
{
    double x = state_vector(0);
    double y = state_vector(1);
    double theta = state_vector(2);
    
    int measurements_num = 2;
    int unmatched_num = 1;

    // Initializing new landmarks
    if (!data_association(state_vector, measurements))
    {
        add_new_landmarks(state_vector, Sigma, measurements, unmatched_num);
    }

    // Perception measurements
    double range = measurements(0);
    double bearing = measurements(1);
    MatrixXd zt(2,1);
        zt << range, 
              bearing;

    MatrixXd Ht = MatrixXd::Zero(2 * measurements_num, state_vector.size());              //CHANGE
    MatrixXd Dzt = MatrixXd::Zero(2 * measurements_num, 1);
    MatrixXd Rt = MatrixXd::Zero(2 * measurements_num, 2 * measurements_num);
    Rt.diagonal().array() = 0.1;

    int matched_landmarks = 1;
    for(int i=0; i<matched_landmarks; ++i)
    {
            double x_land = x + range * cos(bearing + theta);
            double y_land = y + range * sin(bearing + theta);
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
            
            Dzt.block(2 * matched_landmarks, 0, 2, 1) = zt - zt_exp;
            
            MatrixXd Htu(2,3);
                Htu << - q_sqrt * dx, - q_sqrt * dy, 0,
                            dy, - dx, - q; 

            MatrixXd Htj(2,2);
                Htj << q_sqrt * dx, q_sqrt * dy,
                        - dy, dx;
            
            Ht.block(2 * matched_landmarks, 0, 2, 3) = Htu;
            Ht.block(2 * matched_landmarks, 2 * i + 3, 2, 2) = Htj;
    }

        // KALMAN GAIN
        MatrixXd Kt = Sigma * Ht.transpose() * ((Ht * Sigma * Ht.transpose()) + Rt).inverse();
        
        // FINAL STATE 
        state_vector = state_vector + Kt * Dzt; // or K * `Δzt

        // FINAL COV MATRIX
        Sigma = (MatrixXd::Identity(state_vector.size(), state_vector.size()) - Kt * Ht) * Sigma;
}
 
// Function to read a list of values from a line and store them in a vector
template<typename T>
void readList(std::istream& input, std::vector<T>& output) {
    std::string line;
    std::getline(input >> std::ws, line);
    std::istringstream stream(line);
    T value;
    while (stream >> value) {
        output.push_back(value);
    }
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

    Matrix3d Q;                                      // model noise add variance matrix from messages
        Q << 0.01, 0, 0,
             0, 0.01, 0,
             0, 0, 0.01;  
    
    Matrix2d R;                                      // sensor noise
        R << 0.1, 0,
             0, 0.1;                                // obs_noise << 0.0001, 0,
					                               //  0, 0.011*std::pow(landmark.range+1,2) - 0.082*(landmark.range+1) + 0.187;     change if x,y and not range-bearing
     

 std::ifstream velocityFile("good_velocityLog.txt");
 std::ifstream perceptionFile("good_perceptionLog.txt");

  if (!velocityFile.is_open() || !perceptionFile.is_open()) {
        std::cerr << "Error opening input files." << std::endl;
        return 1;
    }

    int step_cnt = 0;
    uint32_t globalIndexVelocity;
    uint32_t globalIndexPerception;
    std::string line;
    std::vector<int32_t> class_list;
    std::vector<float> theta_list;
    std::vector<float> range_list;
    
    while(velocityFile >> globalIndexVelocity &&
          perceptionFile >> globalIndexPerception)
    {
        std::cout<<globalIndexVelocity<<" goes with: "<<globalIndexPerception<<std::endl;
        // Read velocity from file
        VectorXd velocity(3);
        for (int i = 0; i < 3; ++i)
        {
            velocityFile >> velocity(i);
        }

        // Read variance matrix from velocity file
        Matrix<double, 3, 3> varianceMatrixVelocity;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                velocityFile >> varianceMatrixVelocity(i, j);
            }
        }
        // JUST FOR TESTING
        if(globalIndexVelocity >= 500){std::cout<<"Enough!"; break;}

        // Read perception measurements from file
        readList(perceptionFile, class_list);
        readList(perceptionFile, theta_list);
        readList(perceptionFile, range_list);


        VectorXd measurements(2);
            measurements << range_list[0], theta_list[0];

        // Prediction step
        predictionStep(state_vector, Sigma, velocity, Q);

        // Update step
        updateStep(state_vector, Sigma, measurements, R);

        // Estimated State
       // std::cout << "Step: " << step_cnt << ", Estimated State: " << state_vector.transpose() << std::endl;
        step_cnt++;
    }
    
    velocityFile.close();
    perceptionFile.close();

    return 0;
}



/*
    std::cout << "Class List: ";
    for (auto class_value : class_list) {
        std::cout << class_value << " ";
    }
    std::cout << std::endl;

    std::cout << "Theta List: ";
    for (auto theta_value : theta_list) {
        std::cout << theta_value << " ";
    }
    std::cout << std::endl;

    std::cout << "Range List: ";
    for (auto range_value : range_list) {
        std::cout << range_value << " ";
    }
    std::cout << std::endl;
*/
