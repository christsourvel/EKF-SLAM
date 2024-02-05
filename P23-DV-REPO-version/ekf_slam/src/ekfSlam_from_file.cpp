#include <iostream>
#include <fstream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"

using namespace Eigen;
using namespace std;

// For testing
bool mapping = true;

class SlamNode : public rclcpp::Node {
public:
    SlamNode() : Node("Slam_node") {
       map_publisher_ = create_publisher<custom_msgs::msg::LocalMapMsg>("local_map", 10);
	   pose_publisher_ = create_publisher<custom_msgs::msg::PoseMsg>("pose", 10);    
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

        velocityFile.open("/home/chris/Desktop/P23-DV/ROS_Workspace/src/4.SLAM/ekf_slam/DataFiles/good_velocityLog.txt");
        perceptionFile.open("/home/chris/Desktop/P23-DV/ROS_Workspace/src/4.SLAM/ekf_slam/DataFiles/good_perceptionLog.txt");

        if (!velocityFile.is_open() || !perceptionFile.is_open()) {
            std::cerr << "Error opening input files." << std::endl;
            return;
        }
    
    int step_cnt = 0;
    uint32_t globalIndexVelocity;
    uint32_t globalIndexPerception;
    
    perceptionFile >> globalIndexPerception;
    velocityFile >> globalIndexVelocity;
    VectorXd velocity;

    while(rclcpp::ok() && !velocityFile.eof() && !perceptionFile.eof())
    {
        while(globalIndexVelocity != globalIndexPerception){
            velocity = readVelocity();
            predictionStep(state_vector, Sigma, velocity, Q);
            publishPoseMessage(state_vector, globalIndexVelocity, velocity);

            velocityFile >> globalIndexVelocity;  
            cout<<globalIndexVelocity<<endl;
        }

        velocity = readVelocity();
        while(globalIndexVelocity == globalIndexPerception){
            readOdometry();
            predictionStep(state_vector, Sigma, velocity, Q);
            publishPoseMessage(state_vector, globalIndexVelocity, velocity);

            updateStep(state_vector, Sigma, Rt);
            publishPoseMessage(state_vector, globalIndexVelocity, velocity);
            publishMapMessage(state_vector, globalIndexPerception, velocity);

            perceptionFile >> globalIndexPerception;
            cout<<globalIndexVelocity<<endl;
        }
        velocityFile >> globalIndexVelocity; 
        if(globalIndexPerception == 1640){break;}
        //std::cout<<globalIndexVelocity<<" Velocity goes with Perception: "<<globalIndexPerception<<std::endl;
        if(velocityFile.eof() || perceptionFile.eof()){
            velocityFile.close();
            perceptionFile.close();
            cout<<"THATS ALL FOLKS\n"; 
            break;}
        
        // Estimated State
        //std::cout << "Step: " << globalIndexPerception << ", Estimated State: " << state_vector.transpose() << std::endl;
        step_cnt++;
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
    double theta = pose(2);

    Eigen::Matrix3d Gx;                                                 // change - and 1 in dt
        Gx << 1, 0, v_x * sin(theta) * dt - v_y * cos(theta) * dt,
              0, 1, v_x * cos(theta) * dt - v_y * sin(theta) * dt,
              0, 0, 1;
    
    return Gx;
}

MatrixXd noise_transformation(const VectorXd& pose)
{
    double dt = 0.1;  
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
    MatrixXd Vt = noise_transformation(state_vector.head(3));
    MatrixXd Qt = Vt * Q * Vt.transpose();
    
    // State Prediction
    state_vector.head(3) = kinematic_update(state_vector.head(3), velocity);

    // Covariance Prediction
    if(mapping){
        MatrixXd expandedQ = MatrixXd::Zero(Sigma.rows(), Sigma.cols());
        expandedQ.topLeftCorner(3, 3) = Q;
        Sigma = covariance_update(Sigma, Gt, state_vector.size()) + expandedQ;
    }
    else{Sigma = (Gt * Sigma * Gt.transpose()) + Q;}
}


vector<pair<int, int>> data_association(const VectorXd& state_vector, vector<double> range,
                                        vector<double> bearing, vector<double> color, vector<int> &unmatched) 
{
    double x = state_vector(0);
    double y = state_vector(1);
    double theta = state_vector(2);
    double association_distance_threshold = 1.9;
    vector<pair<int, int>> proccessing;

    for(int j = 0; j < range.size(); ++j){
        double least_distance_square = std::pow(association_distance_threshold, 2);
        int best_match = -1;
        double x_land = x + range[j] * cos(theta + bearing[j]);
        double y_land = y + range[j] * sin(theta + bearing[j]);
        
        // Iterate through all of the cones in the current map
        for (size_t i = 0; i < landmark_distances.size(); ++i) 
        {   
          const auto& pair = landmark_distances[i];
          double current_distance_square = std::pow(x_land - pair.first, 2) + std::pow(y_land - pair.second, 2);//cout<<"state vector: "<<state_vector.transpose()<<endl;
         if(current_distance_square < least_distance_square && color[j] == landmark_colors[i]) 
          {
            least_distance_square = current_distance_square; 
            best_match = i;
          }//cout<<"PANIK"<<endl;
        }
        if(best_match >= 0){proccessing.push_back(std::make_pair(best_match, j));}
        else{unmatched.push_back(j);}
    }
    return proccessing;
}

// Function to add new landmarks
void add_new_landmarks(VectorXd& state_vector, MatrixXd& Sigma, vector<double> range,
                       vector<double> bearing, vector<double> color, vector<int> unmatched)
{
    double x = state_vector(0);
    double y = state_vector(1);
    double theta = state_vector(2);
 
    for(int i = 0; i < unmatched.size(); ++i)
    {   
        double x_land = x + range[unmatched[i]] * cos(theta + bearing[unmatched[i]]);
        double y_land = y + range[unmatched[i]] * sin(theta + bearing[unmatched[i]]);
        landmark_distances.push_back(std::make_pair(x_land, y_land));
        landmark_colors.push_back(color[unmatched[i]]);
        
        VectorXd new_state_vector(state_vector.size() + 2);
        new_state_vector << state_vector, x_land, y_land;

        MatrixXd Hu_inv(2,3);
            Hu_inv << 1, 0, -1 * range[unmatched[i]] * sin(theta + bearing[unmatched[i]]),
                      0, 1, range[unmatched[i]] * cos(theta + bearing[unmatched[i]]);
        
        MatrixXd H_inv = MatrixXd::Zero(2, state_vector.size());
        H_inv.block(0, 0, 2, 3) = Hu_inv;
        
        MatrixXd Hi_inv(2,2);
            Hi_inv << cos(theta + bearing[unmatched[i]]), -1 * range[unmatched[i]] * sin(theta + bearing[unmatched[i]]),
                      sin(theta + bearing[unmatched[i]]), range[unmatched[i]] * cos(theta + bearing[unmatched[i]]);

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
void updateStep(VectorXd& state_vector, MatrixXd& Sigma, const MatrixXd& R) 
{
    double x = state_vector(0);
    double y = state_vector(1);
    double theta = state_vector(2);
    vector<double> range = measurements.range_list;
    vector<double> bearing = measurements.theta_list;
    vector<double> color = measurements.class_list;
    vector<int> unmatched;
    
    // 1: matched_landmark, 2: measurement_index
    vector<pair<int, int>> matched = data_association(state_vector, range, bearing, color, unmatched);
    //cout<<"AFTER DA: "<<state_vector.head(3).transpose()<<endl;
    // add unmatched measurements on mapping mode
    if(mapping){add_new_landmarks(state_vector, Sigma, range, bearing, color, unmatched);}
      //  cout<<"AFTER NEW_LANDMARKS: "<<state_vector.head(3).transpose()<<endl;
    MatrixXd Ht = MatrixXd::Zero(2 * matched.size(), state_vector.size());
    MatrixXd Dzt = MatrixXd::Zero(2 * matched.size(), 1);
    MatrixXd Rt = MatrixXd::Zero(2 * matched.size(), 2 * matched.size());
    Rt.diagonal().array() = 0.1;
        
    for(int i=0; i<matched.size(); ++i)
    {   
            // Actual observation
            MatrixXd zt(2,1);
                zt << range[matched[i].second],
                      bearing[matched[i].second];
            
            double x_land = state_vector(2 * matched[i].first + 2);
            double y_land = state_vector(2 * matched[i].first + 3);
            double dx = x_land - x;
            double dy = y_land - y;

            MatrixXd d(2,1);
                d << dx,
                     dy;

            double q = (d.transpose() * d)(0,0);
            double q_sqrt = sqrt(q);

            // EXPECTED OBSERVATION
            MatrixXd zt_exp(2,1);
                zt_exp << q_sqrt,
                     atan2(dy,dx);
            
            MatrixXd Htu(2,3);
                Htu << - q_sqrt * dx, - q_sqrt * dy, 0,
                            dy, - dx, - q; 

           if(mapping){ 

            Dzt.block(2 * i, 0, 2, 1) = zt - zt_exp;
            MatrixXd Htj(2,2);
                Htj << q_sqrt * dx, q_sqrt * dy,
                        - dy, dx;
            
            Ht.block(2 * i, 0, 2, 3) = Htu;
            Ht.block(2 * i, 2 * matched[i].first + 3, 2, 2) = Htj;
           }
           else{
                MatrixXd Ht = Htu;
                MatrixXd Dzt = zt - zt_exp;
           }
    }   
        if(matched.size() > 0){// KALMAN GAIN
        MatrixXd Kt = Sigma * Ht.transpose() * ((Ht * Sigma * Ht.transpose()) + Rt).inverse();
        //cout<<"STATE AT END OF PREDICTION: "<<state_vector.transpose()<<endl;
        //cout<<"SIGMA AT END OF PREDICTION: "<<Sigma.transpose()<<endl;
        //cout<<"KT: "<<Kt.transpose()<<endl;
        //cout<<"HT*SIGMA: "<<Sigma * Ht.transpose()<<endl;
        //cout<<"Rt: "<<Rt.transpose()<<endl;
       // cout<<"Dzt: "<<Dzt.transpose()<<endl;
        // FINAL STATE 
        //cout<<"STATE AT END OF PREDICTION: "<<state_vector.transpose()<<endl;
        //cout<<"SIGMA AT END OF PREDICTION: "<<Sigma.transpose()<<endl;
        state_vector = state_vector + Kt * Dzt; // or K * `Δzt
       // cout<<"AFTER STATE UPDATE: "<<state_vector.head(3).transpose()<<endl;
        // FINAL COV MATRIX
        Sigma = (MatrixXd::Identity(state_vector.size(), state_vector.size()) - Kt * Ht) * Sigma;
        } 
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

VectorXd readVelocity(){
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
    //cout<<varianceMatrixVelocity.transpose()<<endl;
    //cout<<velocity.transpose()<<endl;
    return velocity;
}

void readOdometry(){
    measurements.class_list.clear();
    measurements.theta_list.clear();
    measurements.range_list.clear();
    // Read perception measurements from file
    readList(perceptionFile, measurements.class_list);
    readList(perceptionFile, measurements.theta_list);
    readList(perceptionFile, measurements.range_list);

    if(measurements.theta_list.size() != measurements.range_list.size()){
        cout<<" FUCKED UP TXT FORMAT!!!\n";
        return;
    }
}

void publishPoseMessage(const Eigen::VectorXd& state_vector, uint32_t globalIndexVelocity, const Eigen::VectorXd& velocity) {
    custom_msgs::msg::PoseMsg pose_msg;
    pose_msg.position.x = state_vector(0);
    pose_msg.position.y = state_vector(1);
    pose_msg.theta = state_vector(2);
    pose_msg.velocity_state.global_index = globalIndexVelocity;
    pose_msg.velocity_state.velocity_x = velocity(0);
    pose_msg.velocity_state.velocity_y = velocity(1);
    pose_msg.velocity_state.yaw_rate = velocity(2);
    pose_msg.lap_count = 69;

    pose_publisher_->publish(pose_msg);
}

void publishMapMessage(const Eigen::VectorXd& state_vector, uint32_t globalIndexPerception, const Eigen::VectorXd& velocity){
        // Publish map
		custom_msgs::msg::LocalMapMsg map_msg{};
		custom_msgs::msg::ConeStruct cone_msg{};

		map_msg.cones_count_actual = 69;
		map_msg.pose.position.x = state_vector(0);
		map_msg.pose.position.y = state_vector(1);
		map_msg.pose.theta = state_vector(2);
		map_msg.pose.velocity_state.global_index = globalIndexPerception;
		map_msg.pose.velocity_state.velocity_x = velocity(0);
		map_msg.pose.velocity_state.velocity_y = velocity(1);
		map_msg.pose.velocity_state.yaw_rate = velocity(2);
		map_msg.lap_count = 69;
		for (int i{ 0 }; i < 9;i++) { map_msg.pose.velocity_state.variance_matrix[i] = 0;}

			for (int i=3; i<state_vector.size()-3; i += 3)
			{
				cone_msg.coords.x = state_vector[i];
				cone_msg.coords.y = state_vector[i+1];
                cone_msg.color = state_vector[i+2];
				map_msg.local_map.push_back(cone_msg);
            }
		
        map_publisher_->publish(map_msg);
}

private:
    vector<pair<double, double>> landmark_distances;
    vector<double> landmark_colors;

    struct Measurements {
        std::vector<double> class_list;
        std::vector<double> theta_list;
        std::vector<double> range_list;
    };

    Measurements measurements; 

    std::ifstream velocityFile;
    std::ifstream perceptionFile;
    rclcpp::Publisher<custom_msgs::msg::LocalMapMsg>::SharedPtr map_publisher_;
    rclcpp::Publisher<custom_msgs::msg::PoseMsg>::SharedPtr pose_publisher_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<SlamNode>();
    node->runSlamAlgorithm();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



