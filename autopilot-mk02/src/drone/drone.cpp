#include "./drone.h"

Drone::Drone()
{
}

void Drone::set_receiver_channel(uint16_t *receiver_channel)
{
    this->receiver_channel = receiver_channel;
}

void Drone::update()
{
    // this->calculatePID();   // pid calculation

    this->propeller.speed = ((double)this->receiver_channel[2] - 1000) / 10; // from 1000 to 100

    // 1us is 0.18 deg and convert it into -90 to +90
    this->fin1.angle = (int8_t) ((this->receiver_channel[0] - 1500) * 0.18);
    this->fin2.angle = (int8_t) ((this->receiver_channel[0] - 1500) * 0.18);
    this->fin3.angle = (int8_t) ((this->receiver_channel[1] - 1500) * 0.18);
    this->fin4.angle = (int8_t) ((this->receiver_channel[1] - 1500) * 0.18);

    this->propeller.update();
    this->fin1.update();
    this->fin2.update();
    this->fin3.update();
    this->fin4.update();
}

// calculate the pid for drone
void Drone::calculate_pid()
{
    // calculate errors
    this->error_ypr[0] = this->target_angle[0] - this->current_angle[0];
    this->error_ypr[1] = this->target_angle[1] - this->current_angle[1];
    this->error_ypr[2] = this->target_angle[2] - this->current_angle[2];

    // delta error
    this->delta_error_ypr[0] = this->previous_error_ypr[0] - this->error_ypr[0];
    this->delta_error_ypr[1] = this->previous_error_ypr[1] - this->error_ypr[1];
    this->delta_error_ypr[2] = this->previous_error_ypr[2] - this->error_ypr[2];

    // pid Proportion
    this->pid_proportion_ypr[0] = this->error_ypr[0] * this->pid_k_yaw.kp;
    this->pid_proportion_ypr[1] = this->error_ypr[1] * this->pid_k_pitch_roll.kp;
    this->pid_proportion_ypr[2] = this->error_ypr[2] * this->pid_k_pitch_roll.kp;

    // pid Integral
    if ((this->error_ypr[0] * (this->error_ypr[0] < 0 ? -1 : 1)) < 0.1)
        this->pid_integral_ypr[0] += this->error_ypr[0] * this->pid_k_yaw.ki;
    else
        this->pid_integral_ypr[0] = 0;
    if ((this->error_ypr[1] * (this->error_ypr[1] < 0 ? -1 : 1)) < 0.1)
        this->pid_integral_ypr[1] += this->error_ypr[1] * this->pid_k_pitch_roll.ki;
    else
        this->pid_integral_ypr[1] = 0;
    if ((this->error_ypr[2] * (this->error_ypr[2] < 0 ? -1 : 1)) < 0.1)
        this->pid_integral_ypr[2] += this->error_ypr[2] * this->pid_k_pitch_roll.ki;
    else
        this->pid_integral_ypr[2] = 0;

    // pid derevative
    this->pid_derevative_ypr[0] = this->delta_error_ypr[0] * this->pid_k_yaw.kd;
    this->pid_derevative_ypr[1] = this->delta_error_ypr[1] * this->pid_k_pitch_roll.kd;
    this->pid_derevative_ypr[2] = this->delta_error_ypr[2] * this->pid_k_pitch_roll.kd;

    // set error to previous error
    this->previous_error_ypr[0] = this->error_ypr[0];
    this->previous_error_ypr[1] = this->error_ypr[1];
    this->previous_error_ypr[2] = this->error_ypr[2];
}
