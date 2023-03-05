#include <Navigation/navigation.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "Navigation");

  ros::NodeHandle n;

  Navigation NavigationNode;

  ros::spin();

  return 0;
}
