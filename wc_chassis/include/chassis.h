#ifndef CHASSIS_H
#define CHASSIS_H

#include <string>
#include <map>

class Chassis_mcu{
public:
    Chassis_mcu();
    ~Chassis_mcu();

    void Init(float,float,float,float,int,float);
    bool getOdo(double &x,double &y,double &a,double &v, double &w);
    bool getGps(double &north,double &east,bool &valid,float &compass);

private:
    bool is_auto_;
    int delta_counts_rear_;

    float tha_zero_;

    float H_;
    float Dia_F_;
    float Dia_B_;
    float Axle_;
    int RCounts_;
    float DeltaT_;
    int reduction_ratio_;

    bool first_odo_;
    bool first_gps_;
    double odom_x_;
    double odom_y_;
    double odom_a_;
    double odom_v;
    double odom_w;

    long long last_counts_rear_;
    int last_delta_counts_rear_;
    long long counts_rear_;

    int last_angle_;
    int current_angle_;

};

#endif // CHASSIS_H
