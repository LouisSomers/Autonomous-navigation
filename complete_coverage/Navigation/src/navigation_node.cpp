#include <navigation/navigation.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "navigation");

  ros::NodeHandle n;

  navigation navigationNode;

  ros::spin();

  return 0;
}
