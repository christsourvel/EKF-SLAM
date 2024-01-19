#include "/Users/christos/Documents/headers/Eigen/Dense"
#include <iostream>

using namespace Eigen;

int main() {
    VectorXd original_vector(3); // Initial vector with size 3
    original_vector << 1.0, 2.0, 3.0; // Example initial values

    // Print the original vector
    std::cout << "Original Vector: " << original_vector.transpose() << std::endl;

    // Increase the size of the vector by appending two new elements
    original_vector.conservativeResize(original_vector.size() + 2);

    // Print the updated vector (new elements are uninitialized)
    std::cout << "Updated Vector: " << original_vector.transpose() << std::endl;

    return 0;
}
