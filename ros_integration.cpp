#include <iostream>
//#include <Eigen/Dense>
#include "/Users/christos/Documents/headers/Eigen/Dense"

using namespace Eigen;
 
 class EkfSlam : public rclcpp::Node
 {
    public:
        EkfSlam() : Node("ros_integration"), count_ (0)
        {

        }

    private:

 };
 
 
 int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EkfSlam>());
    rclcpp::shutdown();
    return 0;
  }