/* 
 * File:    pid.cpp
 * version: ver01.00 (2019/07/18)
 * Author:  SIVA
 *
 * Created on 2019/07/11
 */

#include "pid.hpp"
#include "main.h"

Pid::Pid(double Kp, double Ki, double Kd){
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}

Pid::Pid(double Kp, double Ki, double Kd, double loop_time){
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    loop_time_ = loop_time;
}

Pid::~Pid() {
}

void Pid::set_enc(int enc){
    enc_ = enc;
}

void Pid::set_control_data(int control_data){
    control_data_ = control_data;
}

void Pid::update_errors(){
    int diff_data = control_data_ - enc_;
    d_error_ = diff_data - prev_diff_data_;
    p_error_ = diff_data;
    i_error_ += diff_data;

    prev_diff_data_ = diff_data;
}

double Pid::pid_calc(int enc, int control_data){
    double total_error;
    set_enc(enc);
    set_control_data(control_data);
    update_errors();
    
    total_error = Kp_*p_error_ + Ki_*i_error_*loop_time_ + Kd_*d_error_;
    velocity_ = velocity_ + total_error*loop_time_;
    return velocity_;
}

void Pid::reset_internal_state() {
    p_error_ = 0;
    i_error_ = 0;
    d_error_ = 0;
    prev_diff_data_ = 0;
    velocity_ = 0;
}