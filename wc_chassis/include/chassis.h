#ifndef _CHASSIS_MCU_
#define _CHASSIS_MCU_

#include <string>
#include <map>

class SerialPort;

class Chassis_mcu{
public:
    Chassis_mcu();
    ~Chassis_mcu();

    void Init(float H,float Dia_F,float Dia_B,float Axle,int Counts);
    bool getOdo(double &x,double &y,double &a);
    void comunication(void);
    bool getCSpeed(double &v, double &w);

private:
    bool is_auto_;

    int getLPos();
    int getRPos();
    int delta_counts_left_;
    int delta_counts_right_;

    float tha_zero_;

    float H_;
    float Dia_F_;
    float Dia_B_;
    float Axle_;
    int Counts_;
    int reduction_ratio_;

    SerialPort* transfer = NULL;

    bool first_odo_;
    double odom_x_;
    double odom_y_;
    double odom_a_;
    double odom_v;
    double odom_w;

    int last_counts_left_;
    int last_counts_right_;
    int counts_left_;
    int counts_right_;

    unsigned char send_[10];
    unsigned char rec_[20];

    int direction;

    int last_angle_;
    int current_angle_;

};

#endif
