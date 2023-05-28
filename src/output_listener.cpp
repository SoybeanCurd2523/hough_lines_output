#include <ros/ros.h>
#include <opencv_apps/LineArrayStamped.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <sensor_msgs/Image.h>
#include <deque>
#include <algorithm>
#include <numeric>
#include <cmath>
#include "std_msgs/Float64.h"

#define rad2deg(radians) ((radians) * 180.0 / M_PI)

// Maximum number of points to keep in memory
#define MAX_POINTS 100

// Deque to hold intersection points
std::deque<geometry_msgs::Point> intersectionPoints;

// Your desired range
// resolution: 1280x720
// middle point: (640, 360)
double xMin = 540;
double xMax = 740;
double yMin = 400;
double yMax = 600;

double min_angle = 10;
double max_angle = 80;

// Your desired standard deviation
double desiredStdDev = 1000;

float x_point = 0;
// Filter intersection
void filteredIntersection(const geometry_msgs::Point& point)
{
  // Your intersection calculation code goes here...
  // Let's say intersection point is (xIntersection, yIntersection)

  // Area based filtering
  // 이거 주의.
  if (point.x < xMin || point.x > xMax || point.y < yMin || point.y > yMax) {
    // ROS_WARN("Intersection point out of range. Skipping...");
    return;
  }

  // Create a geometry_msgs::Point for the intersection point
  geometry_msgs::Point intersection;
  intersection.x = point.x;
  intersection.y = point.y;
  intersection.z = 0.0;

  // Add the intersection point to the deque
  intersectionPoints.push_back(intersection);

  // Check if deque size exceeds MAX_POINTS
  if (intersectionPoints.size() > MAX_POINTS) {
    intersectionPoints.pop_front(); // Remove the oldest point
  }

  // Calculate mean
  double meanX = std::accumulate(intersectionPoints.begin(), intersectionPoints.end(), 0.0,
    [](double sum, const geometry_msgs::Point& pt) { return sum + pt.x; }) / intersectionPoints.size();
  double meanY = std::accumulate(intersectionPoints.begin(), intersectionPoints.end(), 0.0,
    [](double sum, const geometry_msgs::Point& pt) { return sum + pt.y; }) / intersectionPoints.size();

  // Calculate standard deviation
  double stdDevX = std::sqrt(std::accumulate(intersectionPoints.begin(), intersectionPoints.end(), 0.0,
    [meanX](double sum, const geometry_msgs::Point& pt) { return sum + (pt.x - meanX) * (pt.x - meanX); }) / intersectionPoints.size());
  double stdDevY = std::sqrt(std::accumulate(intersectionPoints.begin(), intersectionPoints.end(), 0.0,
    [meanY](double sum, const geometry_msgs::Point& pt) { return sum + (pt.y - meanY) * (pt.y - meanY); }) / intersectionPoints.size());

  if (stdDevX > desiredStdDev || stdDevY > desiredStdDev) {
    ROS_WARN("Standard deviation exceeds desired value. Skipping...");
    intersectionPoints.pop_back(); // Remove the latest point
    return;
  }

  // Print the intersection point
  ROS_INFO("Intersection: (%f, %f)", intersection.x, intersection.y);
  x_point = intersection.x;
}

// Calculate intersection of 4 points representing two lines
void calculateIntersection(const geometry_msgs::Point& pt1, const geometry_msgs::Point& pt2, const geometry_msgs::Point& pt3, const geometry_msgs::Point& pt4)
{
  // Check if the lines are vertical
  bool isVertical1 = (pt2.x - pt1.x) == 0;
  bool isVertical2 = (pt4.x - pt3.x) == 0;

  // Check if the lines are parallel
  bool isParallel = ((pt2.y - pt1.y) / (pt2.x - pt1.x)) == ((pt4.y - pt3.y) / (pt4.x - pt3.x));

  // Skip calculation if lines are vertical or parallel
  if (isVertical1 || isVertical2 || isParallel)
  {
    // ROS_WARN("Lines are vertical or parallel. Skipping intersection calculation.");
    return;
  }

  // Calculate the slopes of the two lines
  double slope1 = (pt2.y - pt1.y) / (pt2.x - pt1.x);
  double slope2 = (pt4.y - pt3.y) / (pt4.x - pt3.x);

  // Define the threshold angle for excluding lines
  double angleThreshold = 30.0; // Set your desired angle threshold here

  // Calculate the angles of the two lines
  double angle1 = atan(slope1) * 180.0 / M_PI;
  double angle2 = atan(slope2) * 180.0 / M_PI;

  if ( ( (angle1 > min_angle && angle1 < max_angle) || (angle1 < -min_angle && angle1 > -max_angle) ) && ( (angle2 > min_angle && angle2 < max_angle) || (angle2 < -min_angle && angle2 > -max_angle)) ) {
    // Calculate the y-intercepts of the two lines
    double yIntercept1 = pt1.y - slope1 * pt1.x;
    double yIntercept2 = pt3.y - slope2 * pt3.x; 

    // Calculate the x-coordinate of the intersection point
    double xIntersection = (yIntercept2 - yIntercept1) / (slope1 - slope2);

    // Calculate the y-coordinate of the intersection point
    double yIntersection = slope1 * xIntersection + yIntercept1;

    // Create a geometry_msgs::Point for the intersection point
    geometry_msgs::Point intersection;
    intersection.x = xIntersection;
    intersection.y = yIntersection;
    intersection.z = 0.0;

    // Print the intersection point
    // ROS_INFO("Intersection: (%f, %f)", intersection.x, intersection.y);

    // ROS_INFO("distance from center : (%lf, %lf)", intersection.x - 640, intersection.y - 360);
    filteredIntersection(intersection);
  }
}

void lineArrayCallback(const opencv_apps::LineArrayStamped::ConstPtr& msg)
{
  // Check if the lines array is not empty
  if (msg->lines.empty())
  {
    ROS_WARN("Empty lines array.");
    return;
  }

  // Initialize lists to store pt1 and pt2 coordinates of each line
  std::vector<geometry_msgs::Point> pt1_list;
  std::vector<geometry_msgs::Point> pt2_list;

  // Iterate over each line
  for (const auto& line : msg->lines)
  {
    // Convert pt1 from opencv_apps::Point2D to geometry_msgs::Point
    geometry_msgs::Point pt1;
    pt1.x = line.pt1.x;
    pt1.y = line.pt1.y;
    pt1.z = 0.0;  // Assuming z-coordinate is not provided in opencv_apps::Point2D

    // Convert pt2 from opencv_apps::Point2D to geometry_msgs::Point
    geometry_msgs::Point pt2;
    pt2.x = line.pt2.x;
    pt2.y = line.pt2.y;
    pt2.z = 0.0;  // Assuming z-coordinate is not provided in opencv_apps::Point2D

    pt1_list.push_back(pt1);
    pt2_list.push_back(pt2);
  }

  // Call calculateIntersection for each pair of lines
  for (size_t i = 0; i < pt1_list.size(); ++i)
  {
    for (size_t j = i + 1; j < pt1_list.size(); ++j)
    {
      calculateIntersection(pt1_list[i], pt2_list[i], pt1_list[j], pt2_list[j]);
    }
  }
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  int width = msg->width;
  int height = msg->height;
  ROS_INFO("Image resolution: %dx%d", width, height);
  // 1280x720
  // 중심점 : 640x360
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "output_listener");
  ros::NodeHandle nh;

  // Subscribe to the LineArr  // if (point.x < xMin || point.x > xMax || point.y < yMin || point.y > yMax) {
  //   // ROS_WARN("Intersection point out of range. Skipping...");
  //   return;
  // }ayStamped topic
  ros::Subscriber hough_line_sub = nh.subscribe("/hough_lines/lines", 1000, lineArrayCallback);
  // ros::Subscriber image_sub = nh.subscribe("/camera/color/image_raw", 1000, imageCallback);

  ros::Publisher x_point_pub = nh.advertise<std_msgs::Float64>("x_point_data", 1000);
  // Set the desired loop rate (e.g., 10 Hz)

  std_msgs::Float64 msg;

  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
    ros::spinOnce(); // Process callback functions

    msg.data = x_point;
    x_point_pub.publish(msg);

    loop_rate.sleep(); // Sleep to achieve the desired loop rate
  }
  
  return 0;
}
