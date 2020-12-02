// #include <ros/ros.h>
#include <actionlib/client/terminal_state.h>
// #include <actionlib_tutorials/FibonacciAction.h>
// #include <actionlib_tutorials/FibonacciAction.h>

// ROS
#include <actionlib/client/simple_action_client.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// boost
#include <boost/thread.hpp>

// OpenCV2.
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Actions.
#include <darknet_ros_msgs/CheckForObjectsAction.h>
// #include <darknet_ros_msgs/FibonacciAction.h>
using CheckForObjectsActionClient = actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction>;
using CheckForObjectsActionClientPtr = std::shared_ptr<CheckForObjectsActionClient>;

// c++
#include <cmath>
#include <string>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;

#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

darknet_ros_msgs::BoundingBoxes boundingBoxesResults_;

void checkForObjectsResultCB(const actionlib::SimpleClientGoalState& state, const darknet_ros_msgs::CheckForObjectsResultConstPtr& result) {
  std::cout << "[ObjectDetectionTest] Received bounding boxes." << std::endl;

  // boundingBoxesResults_ = result->bounding_boxes;
  // if(boundingBoxesResults_.bounding_boxes)
    // std::cout<<result->bounding_boxes.bounding_boxes[0].id<<std::endl;
  // else
  //   std::cout<<"not found"<<std::endl;  
}

bool sendImageToYolo(ros::NodeHandle nh, const std::string& pathToTestImage) {
  //! Check for objects action client.
  CheckForObjectsActionClientPtr checkForObjectsActionClient;

  // Action clients.
  std::string checkForObjectsActionName;
  nh.param("/darknet_ros/camera_action", checkForObjectsActionName, std::string("/darknet_ros/check_for_objects"));
  checkForObjectsActionClient.reset(new CheckForObjectsActionClient(nh, checkForObjectsActionName, true));

  // Wait till action server launches.
  if (!checkForObjectsActionClient->waitForServer(ros::Duration(20.0))) {
    std::cout << "[ObjectDetectionTest] sendImageToYolo(): checkForObjects action server has not been advertised." << std::endl;
    return false;
  }

  // Get test image
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  cv_ptr->image = cv::imread(pathToTestImage, CV_LOAD_IMAGE_COLOR);
  cv_ptr->encoding = sensor_msgs::image_encodings::RGB8;
  sensor_msgs::ImagePtr image = cv_ptr->toImageMsg();

  // Generate goal.
  darknet_ros_msgs::CheckForObjectsGoal goal;
  goal.image = *image;

  // Send goal.
  ros::Time beginYolo = ros::Time::now();
  checkForObjectsActionClient->sendGoal(goal, boost::bind(&checkForObjectsResultCB, _1, _2),
                                        CheckForObjectsActionClient::SimpleActiveCallback(),
                                        CheckForObjectsActionClient::SimpleFeedbackCallback());

  if (!checkForObjectsActionClient->waitForResult(ros::Duration(100.0))) {
    std::cout << "[ObjectDetectionTest] sendImageToYolo(): checkForObjects action server took to long to send back result." << std::endl;
    return false;
  }
  ros::Time endYolo = ros::Time::now();
  std::cout << "[ObjectDetectionTest] Object detection for one image took " << endYolo - beginYolo << " seconds." << std::endl;
  return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "image_sender_test");
  ros::NodeHandle nodeHandle("~");
  std::string pathToTestImage = darknetFilePath_;
  pathToTestImage += "/data/";
  pathToTestImage += "dog";
  pathToTestImage += ".jpg";

  bool test123=false;
  test123=sendImageToYolo(nodeHandle, pathToTestImage);
  ROS_INFO("Action finished V1.0");
  // create the action client
  // true causes the client to spin its own thread
  // actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);

  // ROS_INFO("Waiting for action server to start.");
  // // wait for the action server to start
  // ac.waitForServer(); //will wait for infinite time

  // ROS_INFO("Action server started, sending goal.");
  // // send a goal to the action
  // actionlib_tutorials::FibonacciGoal goal;
  // goal.order = 20;
  // ac.sendGoal(goal);

  // //wait for the action to return
  // bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  // if (finished_before_timeout)
  // {
  //   actionlib::SimpleClientGoalState state = ac.getState();
  //   ROS_INFO("Action finished: %s",state.toString().c_str());
  // }
  // else
  //   ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
