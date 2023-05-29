#include <usv_navigation/usv_navigation.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "usv_navigation");

  ros::NodeHandle n;

  USVNav usvNavNode;

  ros::spin();

  return 0;
}
