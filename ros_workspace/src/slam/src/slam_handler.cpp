#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
/*
// Subscribe messages
#include <geometry_msgs/msg/twist.hpp>
#include <slam/msg/Perception2Slam.hpp>

// Publish message
#include "slam/msg/PoseMsg.hpp"


using namespace Eigen;
using namespace std;

class SlamNode : public rclcpp::Node {
public:
    SlamNode() : Node("Slam_node") {
        velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "velocity_topic", 10, std::bind(&SlamNode::velocityCallback, this, std::placeholders::_1));

        perception_subscriber_ = this->create_subscription<slam::msg::Perception2Slam>(
            "perception_topic", 10, std::bind(&SlamNode::PerceptionCallback, this, std::placeholders::_1));

        state_publisher_ = this->create_publisher<slam::msg::PoseMsg>(
            "state_topic", 10);    
    }

    void runSlamAlgorithm() {
        // Initialize state vector and covariance matrix
        VectorXd state_vector(3);
        state_vector << 0, 0, 0;

        MatrixXd Sigma(3,3);
        Sigma << 0.5, 0, 0,
                 0, 0.1, 0,
                 0, 0, 0.1;

        Matrix3d Q;
        Q << 0.01, 0, 0,
             0, 0.01, 0,
             0, 0, 0.01;

        Matrix2d Rt;
        Rt << 0.1, 0,
              0, 0.1;

         // Infinite loop - runs as long as the node is active
    while (rclcpp::ok()) {
        // Process any incoming messages
        rclcpp::spin_some(shared_from_this());

        // Use the updated velocity
        VectorXd velocity(3);
        velocity << current_velocity_[0], current_velocity_[1], current_velocity_[2];

        VectorXd measurements(2);
        measurements << current_perception_[3], current_perception_[2];


        predictionStep(state_vector, Sigma, velocity, Q);
        updateStep(state_vector, Sigma, measurements, Rt);

        // Create a Pose message
        slam::msg::PoseMsg pose_msg;
        pose_msg.x = state_vector(0); // x position
        pose_msg.y = state_vector(1); // y position
        pose_msg.theta = state_vector(2); // theta orientation

        // Publish the Pose message
        state_publisher_->publish(pose_msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        cout << "State Vector: " << state_vector.transpose() << endl;
    }
}
    

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
    double omega = velocity(2);
    double theta = pose(2);

    Eigen::Matrix3d Gx;                                                 // change - and 1 in dt
        Gx << 1, 0, v_x * sin(theta) * dt - v_y * cos(theta) * dt,
              0, 1, v_x * cos(theta) * dt - v_y * sin(theta) * dt,
              0, 0, 1;
    
    return Gx;
}

MatrixXd noise_transformation(const VectorXd& pose, const VectorXd& velocity)
{
    double dt = 0.1;  
    double v_x = velocity(0);
    double v_y = velocity(1);
    double omega = velocity(2);
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
 

private:
    vector<pair<double, double>> landmark_distances;
    Vector3d current_velocity_;
    
     struct PerceptionData {
        uint32_t global_index;
        std::vector<int32_t> class_list;
        std::vector<float> theta_list;
        std::vector<float> range_list;
    };

    PerceptionData current_perception_;
    
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    
        current_velocity_[0] = msg->linear.x;
        current_velocity_[1] = msg->linear.y;
        current_velocity_[2] = msg->angular.z;
    }

    void PerceptionCallback(const slam::msg::Perception2Slam msg) {
        
    current_perception_.global_index = msg->global_index;

    current_perception_.class_list = msg->class_list;
    current_perception_.theta_list = msg->theta_list;
    current_perception_.range_list = msg->range_list;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::Subscription<slam::msg::Perception2Slam>::SharedPtr perception_subscriber_;
    rclcpp::Publisher<slam::msg::PoseMsg>::SharedPtr state_publisher_;
};

*/
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    //auto node = make_shared<SlamNode>();
    //node->runSlamAlgorithm();
    //rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




/*
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

*/
