#include <iostream>
#include <fstream>
#include <stdlib.h>

int main ()
{

  std::ifstream infile("/home/fregu856/AA273/AA273_project/catkin_ws/src/SE-Sync/MATLAB/examples/test.txt");
  long double x, y, theta;
  int id = 0;
  while (infile >> x >> y >> theta)
  {
    std::cout << id << " " << x << " " << y << " " << theta << "\n";
    ++id;
  }

  return 0;
}
