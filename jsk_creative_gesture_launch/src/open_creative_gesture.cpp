#include <jsk_creative_gesture_launch/jsk_creative_gesture_ros.h>

using namespace jsk_creative_gesture;
int main(int argc, char* argv[]){
  ros::init(argc, argv, "test_creative");

  CreativeGestureRos creative_gesture;
  creative_gesture.Run();
}
