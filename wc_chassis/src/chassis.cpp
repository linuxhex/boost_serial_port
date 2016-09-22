#include "chassis.h"
#include "SPort.h"
#include <ros/ros.h>
extern SerialPort *transfer;
extern unsigned char data[1024];
extern int data_len;

Chassis_mcu::Chassis_mcu()
{
    this->first_odo_ = true;
    this->first_gps_ = true;
}

Chassis_mcu::~Chassis_mcu(){}

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



bool Chassis_mcu::getOdo(double &x, double &y, double &a,double &v, double &w)
{

    float getTheta;

    if (data_len == SERIAL_BYTE_LEN) {
         data_len = 0;
          getTheta = *((float *)(&data[17]));
          long long getCounts = *((long long *)(&data[21]));
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

    double angle = static_cast<double>(getTheta * M_PI_2) / 90;
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

bool Chassis_mcu::getGps(double &north,double &east, bool &valid,float &compass)
{
   static double m_deg_to_rad=0.01745329251994;
   static double constants_radius_of_earth=6371000;

   static double Ref_lat_rad;
   static double Ref_lon_rad;
   static double Ref_sin_lat;
   static double Ref_cos_lat;

   compass = *((float *)(&data[29])) / 180 * M_PI;

   double lat = *((double *)(&data[1]));
   double lon = *((double *)(&data[9]));

   std::cout << "gps lat = " << lat << std::endl;
   std::cout << "gps lon = " << lon << std::endl;

   if((lat < DBL_EPSILON) && (lon < DBL_EPSILON)){
       valid = false;
       return false;
   }else{
       valid = true;
   }

   if(first_gps_){
       Ref_lat_rad = lat * m_deg_to_rad;
       Ref_lon_rad=  lon * m_deg_to_rad;
       Ref_sin_lat= sin(Ref_lat_rad);
       Ref_cos_lat= cos(Ref_lat_rad);
       first_gps_ = false;
       return false;
   }

   double lat_rad =  lat * m_deg_to_rad;
   double lon_rad =  lon * m_deg_to_rad;

   double sin_lat   = sin(lat_rad);
   double cos_lat   = cos(lat_rad);
   double cos_d_lon = cos(lon_rad - Ref_lon_rad);
   double c = acos(Ref_sin_lat * sin_lat + Ref_cos_lat * cos_lat * cos_d_lon);

   double k;
   if(fabs(c) < DBL_EPSILON){
       k=1.0;
   }else {
       k=(c/sin(c));
   }

   north = k * (Ref_cos_lat * sin_lat - Ref_sin_lat * cos_lat * cos_d_lon) * constants_radius_of_earth;
   east  = k * cos_lat * sin(lon_rad - Ref_lon_rad) * constants_radius_of_earth;

   std::cout << "gps north = " << north << std::endl;
   std::cout << "gps east = " << east << std::endl;

   return true;
}
