#include "chassis.h"
#include "SPort.h"
#include "protocol.h"
#include <ros/ros.h>
extern SerialPort *transfer;
extern unsigned char data[1024];
extern int data_len;

Chassis_mcu::Chassis_mcu()
{
}

Chassis_mcu::~Chassis_mcu()
{
}

void Chassis_mcu::Init(float H,float Dia_F, float Dia_B, float Axle,int RCounts,float DeltaT)
{

  if ((H > 0) && (H < 2.0)) {
    H_ = H;
  } else {
    std::cout << "H err value:" <<H<< std::endl;
  }

  if ((Dia_F > 0) && (Dia_F < 0.5)) {
    Dia_F_ = Dia_F;
  } else {
    std::cout << "Dia_F err value:" <<Dia_F<< std::endl;
  }

  if ((Dia_B > 0) && (Dia_B < 0.5)) {
    Dia_B_ = Dia_B;
  } else {
    std::cout << "Dia_B err value:" <<Dia_B<< std::endl;
  }

  if ((Axle > 0) && (Axle < 1.0)) {
    Axle_ = Axle;
  } else {
    std::cout << "Axle err value:" <<Axle<< std::endl;
  }


  if ((RCounts > 0) && (RCounts < 11000)) {
    RCounts_ = RCounts;
  } else {
    std::cout << "RCounts err value:" << RCounts<< std::endl;
  }

  DeltaT_ = DeltaT;

}



bool Chassis_mcu::getOdo(double &x, double &y, double &a,double &v, double &w) {


    float getTheta;
   // ROS_INFO("[wc_chassis] cc data_len = %d",data_len);

    if (data_len == 30) {
         data_len = 0;
         //char getThetaArray[4] = {0};
         //memcpy(getThetaArray,&data[17],4);
          getTheta = *((float *)(&data[17]));

         //std::cout <<" "<< getTheta << std::endl;
         //ROS_INFO("[wc_chassis] cc getTheta = %lf",getTheta);

         long long getCounts = *((long long *)(&data[21]));
         //std::cout <<"cc getCounts = "<< getCounts << std::endl;
         //ROS_INFO("[wc_chassis] cc getCounts = %d",getCounts);
         counts_rear_ = getCounts;
    }else {
        return false;
    }

    if (first_odo_) {
      odom_x_ = 0;
      odom_y_ = 0;
      odom_a_ = 0;

      last_counts_rear_ = counts_rear_;
      if ((abs(last_counts_rear_) > 0)) {
        first_odo_ = false;
      }
      return false;
    }

    int delta_counts_rear = (counts_rear_ - last_counts_rear_);

    if (delta_counts_rear > 20000) {
      delta_counts_rear -= 65536;
    } else if (delta_counts_rear < -20000) {
      delta_counts_rear += 65536;
    }


    if (abs(delta_counts_rear) > 20000) {
      std::cout << "err delta_counts_rear: " << delta_counts_rear << std::endl;
    }

    //std::cout << "delta_counts_rear: " << delta_counts_rear << std::endl;

    double angle = 0.0;
    angle = static_cast<double>(getTheta * M_PI_2) / 90;
    std::cout <<"angle = %lf"<< angle << std::endl;

    v = static_cast<double>(delta_counts_rear * M_PI * Dia_B_ ) / RCounts_ / 4.0 / DeltaT_;
    w = tan(angle) * v / H_;


    double dd = DeltaT_ * v;
    double da = DeltaT_ * w;

    odom_a_ += da;
    odom_x_ += dd * cos(odom_a_);
    odom_y_ += dd * sin(odom_a_);


    while (odom_a_ <= - M_PI) {
      odom_a_ += M_PI*2;
    }
    while (odom_a_ > M_PI) {
      odom_a_ -= M_PI*2;
    }

    x = odom_x_;
    y = odom_y_;
    a = odom_a_;

    last_counts_rear_ = counts_rear_;

    return true;
}

