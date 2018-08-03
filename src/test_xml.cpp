#include <iostream>
#include <cliffmap_ros/cliffmap.hpp>



int main(int argn, char *argv[]) {

  if (argn < 2) {
    std::cout << "Usage cliffmap_server <file_name>" << std::endl;
    return -1;
  }

  cliffmap_ros::CLiFFMap map(argv[1]);
  std::cout << map;

  map.organizeAsGrid();
  
  return 0;
}
