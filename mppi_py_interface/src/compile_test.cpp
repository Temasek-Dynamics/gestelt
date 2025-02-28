#include <iostream>

void say_hello() {
    std::cout << "Hello from non-ROS C++ code!" << std::endl;
}
// write a main
int main() {
    say_hello();
    return 0;
}