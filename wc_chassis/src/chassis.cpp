#include "chassis.h"
#include "SPort.h"
#include "protocol.h"

Chassis_mcu::Chassis_mcu()
{
    if(this->transfer == NULL){
        this->transfer = new SerialPort();
    }
}

Chassis_mcu::~Chassis_mcu()
{
   if(this->transfer != NULL){
       delete this->transfer;
       this->transfer = NULL;
   }
}

void Chassis_mcu::Init(float H,float Dia_F, float Dia_B, float Axle, int Counts)
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

  if ((Counts > 0) && (Counts < 11000)) {
    Counts_ = Counts;
  } else {
    std::cout << "Counts err value:" <<Counts<< std::endl;
  }

  this->transfer->Init(115200);

}

bool Chassis_mcu::getCSpeed(double &v, double &w)
{
  if (abs(delta_counts_front_) > 2000) {
    std::cout << "err delta_counts_front_:" << delta_counts_front_ << std::endl;
    delta_counts_front_ = last_delta_counts_front_;
  } else {
    last_delta_counts_front_ = delta_counts_front_;
  }
  if (abs(delta_counts_rear_) > 2000) {
    std::cout << "err delta_counts_rear_:" << delta_counts_rear_ << std::endl;
    delta_counts_rear_ = last_delta_counts_rear_;
  } else {
    last_delta_counts_rear_ = delta_counts_rear_;
  }

  const double t = 0.05;
  double l_wheel_pos = static_cast<double>(Dia_B_ * delta_counts_front_ * M_PI) / Counts_;  // 200000;  // 81920
  double r_wheel_pos = static_cast<double>(Dia_B_ * delta_counts_rear_ * M_PI) / Counts_;  // 200000;  // 81920

  double dx = (r_wheel_pos + l_wheel_pos) * 0.5;
  double da = (r_wheel_pos - l_wheel_pos) / Axle_;

  if ((fabs(t) > 10e-3) && (fabs(t) < 10e3)) {
    v = dx / t;
    w = da / t;
  } else {
    v = 0;
    w = 0;
  }
  return true;
}

bool Chassis_mcu::getOdo(double &x, double &y, double &a) {

    comunication();
    if (first_odo_) {
      odom_x_ = 0;
      odom_y_ = 0;
      odom_a_ = 0;

      last_counts_front_ = counts_front_;
      last_counts_rear_ = counts_rear_;
      if ((abs(last_counts_front_) > 0) && (abs(last_counts_rear_) > 0)) {
        first_odo_ = false;
      }
      return false;
    }

    int delta_counts_front = (counts_front_ - last_counts_front_);
    int delta_counts_rear = (counts_rear_ - last_counts_rear_);

    if (delta_counts_front > 10000) {
      delta_counts_front -= 65536;
    } else if (delta_counts_front < -10000) {
      delta_counts_front += 65536;
    }
    if (delta_counts_rear > 10000) {
      delta_counts_rear -= 65536;
    } else if (delta_counts_rear < -10000) {
      delta_counts_rear += 65536;
    }


    if (abs(delta_counts_rear) > 2000) {
      std::cout << "err delta_counts_rear: " << delta_counts_rear << std::endl;
    }
    if (abs(delta_counts_front) > 2000) {
      std::cout << "err delta_counts_front: " << delta_counts_front << std::endl;
    }

    double l_wheel_pos = static_cast<double>(Dia_B_ * delta_counts_front * M_PI) / Counts_;  // 200000;  // 81920
    double r_wheel_pos = static_cast<double>(Dia_B_ * delta_counts_rear * M_PI) / Counts_;  // 200000;  // 81920

    double dx = (r_wheel_pos + l_wheel_pos) * 0.5;
    double da = (r_wheel_pos - l_wheel_pos) / Axle_;

    odom_x_ += dx * cos(odom_a_);
    odom_y_ += dx * sin(odom_a_);
    odom_a_ += da;

    while (odom_a_ <= - M_PI) {
      odom_a_ += M_PI*2;
    }
    while (odom_a_ > M_PI) {
      odom_a_ -= M_PI*2;
    }

    x = odom_x_;
    y = odom_y_;
    a = odom_a_;

    last_counts_front_ = counts_front_;
    last_counts_rear_ = counts_rear_;

    return true;
}
int Chassis_mcu::getFPos()
{
  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  CreateRPos(send, &len, 0);

  if (transfer) {
    transfer->Send_data(send, len);
    transfer->Read_data(rec, rlen, 23, 500);
  }

  if (rlen == 23) {
    for (int i = 0; i < rlen; ++i) {
      if (IRQ_CH(rec[i])) {
        counts_front_ = GetPos(0);
        return GetDelta(0);
      }
    }
  }
  return delta_counts_front_;
}

int Chassis_mcu::getRPos()
{
  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  CreateRPos(send, &len, 1);

  if (transfer) {
    transfer->Send_data(send, len);
    transfer->Read_data(rec, rlen, 23, 500);
  }

  if (rlen == 23) {
    for (int i = 0; i < rlen; ++i) {
      if (IRQ_CH(rec[i])) {
        counts_rear_ = GetPos(1);
        return GetDelta(1);
      }
    }
  }
  return delta_counts_rear_;
}
void Chassis_mcu::comunication(void) {
  delta_counts_front_ = getFPos();
  usleep(1000);
  delta_counts_rear_ = getRPos();
  usleep(1000);
}
