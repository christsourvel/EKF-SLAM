#include <iostream>
//#include <Eigen/Dense>
#include "/Users/christos/Documents/headers/Eigen/Dense"
#include <fstream>

using namespace Eigen;
using namespace std;

bool mapping = true;

    vector<pair<double, double> > landmark_distances;
    vector<double> landmark_colors;

    struct Measurements {
        std::vector<double> class_list;
        std::vector<double> theta_list;
        std::vector<double> range_list;
    };

    Measurements measurements; 
    std::ifstream velocityFile("/Users/christos/Desktop/Experimental/EKF-SLAM/testing/good_velocityLog.txt");
    std::ifstream perceptionFile("/Users/christos/Desktop/Experimental/EKF-SLAM/testing/good_perceptionLog.txt");

// Function to perform kinematic update on pose
Vector3d kinematic_update(const VectorXd& pose, const VectorXd& velocity) 
{
    double dt = 0.1;  
    Vector3d new_pose(3);
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
Matrix3d motion_jacobian(const VectorXd& pose, const VectorXd& velocity) 
{
    double dt = 0.1;  
    double v_x = velocity(0);
    double v_y = velocity(1);   
    double theta = pose(2);

        Matrix3d Gx;                                                 // change - and 1 in dt
        Gx << 1, 0, v_x * sin(theta) * dt - v_y * cos(theta) * dt,
              0, 1, v_x * cos(theta) * dt - v_y * sin(theta) * dt,
              0, 0, 1;
    
    return Gx;
}

// Function to compute noise transformation into state space
Matrix3d noise_transformation(const VectorXd& pose)
{
    double dt = 0.1;  
    double theta = pose(2);

        Matrix3d Vx;
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
    MatrixXd Sigma_vv_new = Gt * Sigma_vv * Gt.transpose();
    MatrixXd Sigma_mm_new = Sigma_mm;  
    MatrixXd Sigma_vm_new = Gt * Sigma_vm;
    MatrixXd Sigma_mv_new = Sigma_vm_new.transpose();  

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

// Function to perform data association
vector<pair<int, int> > data_association(const VectorXd& state_vector, vector<double> range,
                                        vector<double> bearing, vector<double> color, vector<int> &unmatched) 
{
    double x = state_vector(0);
    double y = state_vector(1);
    double theta = state_vector(2);
    double association_distance_threshold = 1.9;
    vector<pair<int, int> > proccessing;

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
    vector<pair<int, int> > matched = data_association(state_vector, range, bearing, color, unmatched);
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
    
    Matrix2d Rt;                                      // sensor noise
        Rt << 0.1, 0,
             0, 0.1;                                // obs_noise << 0.0001, 0,
					                               //  0, 0.011*std::pow(landmark.range+1,2) - 0.082*(landmark.range+1) + 0.187;     change if x,y and not range-bearing
     

  if (!velocityFile.is_open() || !perceptionFile.is_open()) {
        std::cerr << "Error opening input files." << std::endl;
        return 1;
    }

       int step_cnt = 0;
    uint32_t globalIndexVelocity;
    uint32_t globalIndexPerception;
    
    perceptionFile >> globalIndexPerception;
    velocityFile >> globalIndexVelocity;
    VectorXd velocity;

    while(!velocityFile.eof() && !perceptionFile.eof())
    {
        while(globalIndexVelocity != globalIndexPerception){
            velocity = readVelocity();
            predictionStep(state_vector, Sigma, velocity, Q);

            velocityFile >> globalIndexVelocity;  
            cout<<globalIndexVelocity<<endl;
        }

        velocity = readVelocity();
        while(globalIndexVelocity == globalIndexPerception){
            readOdometry();
            predictionStep(state_vector, Sigma, velocity, Q);
            updateStep(state_vector, Sigma, Rt);
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
        cout<<globalIndexPerception<<endl;
        // Estimated State
        //std::cout << "Step: " << globalIndexPerception << ", Estimated State: " << state_vector.transpose() << std::endl;
        step_cnt++;
    }
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
