#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <iostream>

#include <cliffmap_ros/cliffmap.hpp>

int main(int argn, char *argv[]) {

  if (argn < 2) {
    std::cout << "usage test_xml <file_name>" << std::endl;
    return -1;
  }

  cliffmap_ros::CLiFFMap map(argv[1]);
  std::cout << map;

  map.organizeAsGrid();
  std::cout << map(12,12);
  return 0;
}
