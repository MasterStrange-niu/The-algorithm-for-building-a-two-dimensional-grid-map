#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <cstring>
#include <cmath>
#include <serial/serial.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include "livox_ros_driver/CustomMsg.h"

float current_distance;
float arctan_angle;
float angle;

constexpr double pi = 3.14159265358979323846;
using namespace std;
const double EARTH_RADIUS = 6371000.0; // 地球半径，单位为千米

// 将角度转换为弧度
double toRadians(double degrees) {
    return degrees * M_PI / 180;
}

// 将弧度转换为角度
double toDegrees(double radians) {
    return radians * 180 / M_PI;
}
// 使用 Haversine 公式计算两个经纬度坐标之间的距离，返回值单位为千米
void distance(double lat3, double lon3, double lat4, double lon4,double& dist) {
    double dLat = toRadians(lat4 - lat3);
    double dLon = toRadians(lon4 - lon3);
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(toRadians(lat3)) * cos(toRadians(lat4)) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
     dist= EARTH_RADIUS * c;
}
// 使用反余弦函数计算两个经纬度坐标之间的方位角，返回值单位为度
void bearing(double lat3, double lon3, double lat4, double lon4,double& bear) 
{
    double dLon = toRadians(lon4 - lon3);
    double y = sin(dLon) * cos(toRadians(lat4));
    double x = cos(toRadians(lat3)) * sin(toRadians(lat4)) -
               sin(toRadians(lat3)) * cos(toRadians(lat4)) * cos(dLon);
    double brng = atan2(y, x);
    bear=toDegrees(brng);
}
// 定义常量
constexpr double earth_radius = 6371000.0;

// 根据GPS信息和激光雷达数据计算障碍物所在的经纬度
void calculate_obstacle_location(double lat, double lon,
                                  double distance1, double angle1,
                                  double& lat2, double& lon2) {
    // 将激光雷达的极坐标转换为直角坐标
    double laser_x = distance1 * cos(angle1);
    double laser_y = distance1* sin(angle1);

    // 计算障碍物在地球上的相对位置
    double obstacle_x = lon + (laser_x / earth_radius) * (180.0 / pi) / cos(lat * pi / 180.0);
    double obstacle_y = lat + (laser_y / earth_radius) * (180.0 / pi);

    // 计算障碍物的经纬度
    lon2 = obstacle_x;
    lat2 = obstacle_y;
}


  void zhong_location(double lat, double lon,
                                  double ROWS, double COLS,
                                  double& lat2, double& lon2) {
    // 将激光雷达的极坐标转换为直角坐标
    double zhong_x = -^2^(0.5)ROWS/2;
    double zhong_y = ^2^(0.5)COLS/2;

    // 计算障碍物在地球上的相对位置
    double obstacle_x = lon + (zhong_x / earth_radius) * (180.0 / pi) / cos(lat * pi / 180.0);
    double obstacle_y = lat + (zhong_y/ earth_radius) * (180.0 / pi);

    // 计算障碍物的经纬度
    lon2 = obstacle_x;
    lat2 = obstacle_y;
}








void laserCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg){
    for(int i = 0; i < msg->point_num; i++){
       // if(msg->points[i].z==0){
        current_distance = sqrt(msg->points[i].x*msg->points[i].x+msg->points[i].y*msg->points[i].y);
        if(!std::isnan(current_distance)){
               arctan_angle = atan(msg->points[i].y/msg->points[i].x);
                angle = arctan_angle * pi / 180.0;    
                
      //  } 
        }
        }
}


double lat, lon;
double deg2rad(double deg) {
  return deg * pi / 180.0;}
double rad2deg(double rad) {
  return rad * 180.0 / pi;}

void calculateDistances(double dist, double bear,  int& north_south_dist, int& east_west_dist) {

    double bearing_rad = M_PI / 180.0 * bear;
    double ns_dist = dist * cos(bearing_rad);
    double ew_dist = dist * sin(bearing_rad);
     north_south_dist = static_cast<int>(std::floor(ns_dist));
     east_west_dist = static_cast<int>(std::floor(ew_dist));


}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "map_publisher");
    ros::NodeHandle nh;

    // 创建一个名为"map"的发布者，类型为nav_msgs::OccupancyGrid
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
  ros::Publisher pub = nh.advertise<sensor_msgs::NavSatFix>("gps_data", 10);
      ros::Subscriber sub = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar",10,laserCallback);
//  ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 10, laserCallback);
  serial::Serial ser("/dev/ttyUSB0", 9600, serial::Timeout::simpleTimeout(1000));
  

//定义00点经纬度坐标
    double lat3;
    double lon3;
    double RES;
int ROWS, COLS;
    cout << "请输入 ROWS , COLS , RES的值：" << endl;
    cin >> ROWS >> COLS>> RES ;
    

    //const int ROWS = 1000;
    //const int COLS = 1000;
    nav_msgs::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.width = COLS;
    map.info.height = ROWS;

    map.info.resolution = RES;  // 栅格尺寸为1米
    map.data.resize(ROWS*COLS);
int number=0;
  while (ros::ok()) {
    if (ser.available()) {
      string line = ser.readline();
      if (sscanf(line.c_str(), "$GNGLL,%lf,N,%lf,E", &lat, &lon) == 2 && lat != 0.0 && lon != 0.0) {
       
       //接收GPS经纬度定义为lat,lon并打印
        int lat_deg = (int)(lat / 100);
        double lat_min = lat - lat_deg * 100;
        lat = lat_deg + lat_min / 60;
        int lon_deg = (int)(lon / 100);
        double lon_min = lon - lon_deg * 100;
        lon = lon_deg + lon_min / 60;
//ROS_INFO("GPS location: (%f, %f)", lat, lon);
       
        	number++;
        }
    if (number==1){
lat3=lat;
lon3=lon;

}
        
    float distance1=current_distance;
    float angle1=arctan_angle;

//根据lat，lon与雷达信息distance1，angle1计算障碍物经纬度lat2，lon2
double lat2;
double lon2;
calculate_obstacle_location( lat,  lon,  distance1,  angle1,  lat2, lon2);
  //ROS_INFO("lat2, lon2: (%f, %f)", lat2, lon2);

   //根据障碍物经纬度lat2，lon2计算当前位置与目标障碍物的距离和角度dist与bear
    double lat4=lat2;
    double lon4=lon2;
    // 使用 Haversine 公式计算两个经纬度坐标之间的距离，返回值单位为千米
    double dist;
    distance( lat3,  lon3, lat4, lon4, dist) ;
   // ROS_INFO("dist: (%f)", dist);
    //使用反余弦函数计算两个经纬度坐标之间的方位角，返回值单位为度
    double bear;
    bearing(lat3,  lon3,  lat4,  lon4, bear) ;
   // ROS_INFO("bear: (%f)", bear);
    //坐标变换
int north_south_dist, east_west_dist;
calculateDistances(dist, bear, north_south_dist, east_west_dist);
      //ROS_INFO("x: %d", north_south_dist);
      //ROS_INFO("y: %d", east_west_dist);
     //打印所有信息 
   if(!std::isnan(current_distance)&&current_distance!=0){
        //打印GPS信息
        ROS_INFO("ROWS 和 COLS 的值: (%d, %d)", ROWS,COLS);
        ROS_INFO("lat3, lon3: (%f, %f)", lat3, lon3);
        ROS_INFO("GPS location: (%f, %f)", lat, lon);
       //打印雷达信息
        ROS_INFO("currrent_distance: (%f)", current_distance);
        ROS_INFO("angle1: (%f)", arctan_angle);
        ROS_INFO("agnle2: (%f)", angle);
        //打印经纬度距离，角度
        ROS_INFO("dist: (%f)", dist);
        ROS_INFO("bear: (%f)", bear);
        //打印x,y坐标
        ROS_INFO("x: %d", north_south_dist);
        ROS_INFO("y: %d", east_west_dist);
        }   


 for(int i = 0; i < ROWS; i++) {
      for(int j = 0; j < COLS; j++) {
            if(i == north_south_dist  && j == east_west_dist) {  // 将像素点设为障碍物
               map.data[i*COLS+j] = 100;
           }
       }
   }

        // 设置时间戳
        map.header.stamp = ros::Time::now();

        // 发布地图消息
        map_pub.publish(map);

        // 等待下一次循环
        sensor_msgs::NavSatFix msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "gps";
        msg.latitude = lat;
        msg.longitude = lon;
        pub.publish(msg);
            ros::Rate loop_rate(50);
            loop_rate.sleep();
      
    }
    ros::spinOnce();
  }

    return 0;
}
