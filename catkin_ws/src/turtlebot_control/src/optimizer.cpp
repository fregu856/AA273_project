#include "ros/ros.h"
#include <stdlib.h> // (for system)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "optimizer");
  ros::NodeHandle n;

  ros::Rate loop_rate(0.2);

  while (ros::ok())
  {

    system("/home/fregu856/AA273/AA273_project/catkin_ws/src/SE-Sync/MATLAB/examples/test.sh");
    std::cout << "Optimal solution computed\n";

    loop_rate.sleep();
  }


  return 0;
}
