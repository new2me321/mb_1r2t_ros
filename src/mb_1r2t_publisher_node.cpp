#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <mb_1r2t_ros/mb_1r2t.h>
#include <vector>

#define BAUDRATE 153600

float max_range = 10.0;
float min_range = 0.11;
double scan_time;
bool invalid_range_is_inf = true; // set invalid scan to infinity

ros::Publisher scan_pub;
sensor_msgs::LaserScan init_scan_msg;

sensor_msgs::LaserScan createScanMsg(std::string frame_id)
{
  sensor_msgs::LaserScan scan;
  scan.header.frame_id = frame_id;
  scan.angle_max = 2 * M_PI;
  scan.angle_min = 0;
  scan.range_min = min_range;
  scan.range_max = max_range;

  return scan;
}

void scanCallback(std::vector<Vector_2d> *vectors)
{
  if (vectors->size() == 0)
  {
    return;
  }

  sensor_msgs::LaserScan scan_msg = init_scan_msg;
  scan_msg.header.stamp = ros::Time::now();
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (vectors->size() - 1);
  scan_msg.time_increment = scan_time / (double)(vectors->size() - 1);
  scan_msg.scan_time = scan_time;

  int size = (scan_msg.angle_max - scan_msg.angle_min) /
             scan_msg.angle_increment;
  scan_msg.ranges.resize(size,
                         invalid_range_is_inf ? std::numeric_limits<float>::infinity() : 0.0);
  scan_msg.intensities.resize(size);

  for (size_t i = 0; i < vectors->size(); i++)
  {
    Vector_2d vector = vectors->at((vectors->size() - 1) - i);
    int index = std::ceil((vector.angle - scan_msg.angle_min) /
                          scan_msg.angle_increment);
    // printf("index %d!\n", index);
    if (index >= 0 && index < size)
    {
      if (vector.distance >= min_range && vector.distance <= max_range)
      {
        scan_msg.ranges[index] = vector.distance;
        scan_msg.intensities[index] = vector.intensity;
      }
    }
  }
  scan_pub.publish(scan_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mb_1r2t_publisher");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");
  std::string port;

  std::string frame_id;

  priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
  priv_nh.param("frame_id", frame_id, std::string("laser"));

  scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 10);

  ROS_INFO("Starting mb_1r2t_publisher (%s@%i)", port.c_str(), BAUDRATE);

  // dont need to redo this every time.

  init_scan_msg = createScanMsg(frame_id);

  Mb_1r2t lidar(port, BAUDRATE);
  lidar.setScanCallback(&scanCallback);

  // scan timer
  ros::Time start_scan_time;
  ros::Time end_scan_time;

  start_scan_time = ros::Time::now();

  while (ros::ok())
  {
    // read data from sensor
    lidar.parsePacket();
    end_scan_time = ros::Time::now();

    // time between scans
    scan_time = (end_scan_time - start_scan_time).toSec() * 1e-3;
    // reset timer
    start_scan_time = end_scan_time;

    // printf("scan_time %f!\n", scan_time);
  }
  lidar.close();

  return 0;
}
