#ifndef CHASSIS_H
#define CHASSIS_H

#include <string>
#include <map>

class SerialPort;

class Chassis_mcu{
public:
    Chassis_mcu();
    ~Chassis_mcu();

    void Init(float,float,float,float,int,int);
    bool getOdo(double &x,double &y,double &a);
    void comunication(void);
    bool getCSpeed(double &v, double &w);

private:
    bool is_auto_;

    int getFPos();
    int getRPos();
    int delta_counts_front_;
    int delta_counts_rear_;

    float tha_zero_;

    float H_;
    float Dia_F_;
    float Dia_B_;
    float Axle_;
    int FCounts_;
    int RCounts_;
    int reduction_ratio_;

    SerialPort* transfer = NULL;

    bool first_odo_;
    double odom_x_;
    double odom_y_;
    double odom_a_;
    double odom_v;
    double odom_w;

    int last_counts_front_;
    int last_counts_rear_;
    int last_delta_counts_front_;
    int last_delta_counts_rear_;
    int counts_front_;
    int counts_rear_;

    unsigned char send_[10];
    unsigned char rec_[20];

    int direction;

    int last_angle_;
    int current_angle_;

};

#endif // CHASSIS_H
