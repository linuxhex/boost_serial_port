#include "SPort.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "chassis.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>

tf::TransformBroadcaster odom_broadcaster;
ros::Publisher  odom_pub;

double g_odom_x   = 0.0;
double g_odom_y   = 0.0;
double g_odom_tha = 0.0;
double g_odom_v   = 0.0;
double g_odom_w   = 0.0;

float H     = 0.0;
float Dia_F = 0.0;
float Dia_B = 0.0;
float Axle  = 0.0;
int Counts  = 0;

void publishOdom(void){
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();;

  odom.header.frame_id = "base_odom";
  // set the position
  odom.pose.pose.position.x = g_odom_x;
  odom.pose.pose.position.y = g_odom_y;
  odom.pose.pose.position.z = 0.0;
  int i;
  for (i = 0; i < 36; i++) odom.pose.covariance.elems[i] = 0.0;
  odom.pose.covariance.elems[0]  = 1.0;
  odom.pose.covariance.elems[7]  = 1.0;
  odom.pose.covariance.elems[14] = 1.0;
  odom.pose.covariance.elems[21] = 1.0;
  odom.pose.covariance.elems[28] = 1.0;
  odom.pose.covariance.elems[35] = 1.0;

  for (i = 0; i < 36; i++) odom.twist.covariance.elems[i] = 0.0;
  odom.twist.covariance.elems[0]  = 1.0;
  odom.twist.covariance.elems[7]  = 1.0;
  odom.twist.covariance.elems[14] = 1.0;
  odom.twist.covariance.elems[21] = 1.0;
  odom.twist.covariance.elems[28] = 1.0;
  odom.twist.covariance.elems[35] = 1.0;

  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(g_odom_tha);

  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = g_odom_v;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = g_odom_w;
  odom_pub.publish(odom);

  tf::Quaternion q;
  tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
  tf::Transform odom_meas(q, tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0));
  tf::StampedTransform odom_transform(odom_meas, ros::Time::now(), "base_odom", "base_link");
  odom_broadcaster.sendTransform(odom_transform);
}

void initParam(void)
{
    ros::NodeHandle nh("~");
    odom_pub  = nh.advertise<nav_msgs::Odometry>("odom", 50);
    nh.param("F_DIA", Dia_F, static_cast<float>(0.223));
    nh.param("B_DIA", Dia_B, static_cast<float>(0.3));
    nh.param("H", H, static_cast<float>(0.92));
    nh.param("AXLE", Axle, static_cast<float>(0.715));
    nh.param("counts", Counts, 4000);
}

int main(int argc, char **argv)
{

 ros::init(argc, argv, "wc_chassis");
 ros::Rate loop(10);
 initParam();
 Chassis_mcu g_chassis_mcu;
 g_chassis_mcu.Init(H,Dia_F,Dia_B,Axle,Counts);

 while (ros::ok()) {

     g_chassis_mcu.getOdo(g_odom_x, g_odom_y, g_odom_tha);
     g_chassis_mcu.getCSpeed(g_odom_v, g_odom_w);

     publishOdom();

     ros::spinOnce();
     loop.sleep();
 }
  return 0;
}




//测试
//SerialPort* transfer_l_;
//int main()
//{
//    transfer_l_ = new SerialPort();
//    transfer_l_->Init(115200);

//    unsigned char ss[5]={'1','2','3','4'};

//    while(1){
//        sleep(1);
//        transfer_l_->Send_data(ss,4);
//        std::cout << transfer_l_ << std::endl;
//    }

//}
