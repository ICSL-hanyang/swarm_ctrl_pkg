#ifndef _PID_H_
#define _PID_H_

template <typename T>
class PIDController
{
private:
    double dt_;   //loop interval time
    double kp_;   //proportional gain
    double kd_;   //derivative gain
    double ki_;   //Integral gain
    T pre_error_;
    T integral_;

public:
    PIDController();
    ~PIDController();
    T calculate(T setpoint, T pv);
    // Returns the manipulated variable given a setpoint and current process value

    bool setKp(double);
    double getKp() const;
    bool setKd(double);
    double getKd() const;
    bool setKi(double);
    double getKi() const;
    bool setDt(double);
    double getDt() const;
};

template <typename T>
PIDController<T>::PIDController() : dt_(0),
                                 kp_(0),
                                 kd_(0),
                                 ki_(0)
{
}

template <typename T>
PIDController<T>::~PIDController()
{
}

template <typename T>
T PIDController<T>::calculate(T target, T current_value)
{
    // Calculate error
    T error = target - current_value;

    // Proportional term
    T p_out = error * kp_;

    // Integral term
    integral_ += error * dt_;
    T i_out = integral_ * ki_;

    // Derivative term
    T derivative = (error - pre_error_) / dt_;
    T d_out = derivative * kd_;

    // Calculate total output
    T output = p_out + i_out + d_out;

    // Save error to previous error
    pre_error_ = error;

    return output;
}

template <typename T>
bool PIDController<T>::setKp(double kp){
    kp_ = kp;
    return true;
}

template <typename T>
double PIDController<T>::getKp() const{
    return kp_;
}

template <typename T>
bool PIDController<T>::setKd(double kd){
    kd_ = kd;
    return true;
}

template <typename T>
double PIDController<T>::getKd() const{
    return kd_;
}

template <typename T>
bool PIDController<T>::setKi(double ki){
    ki_ = ki;
    return true;
}

template <typename T>
double PIDController<T>::getKi() const{
    return ki_;
}

template <typename T>
bool PIDController<T>::setDt(double dt){
    dt_ = dt;
    return true;
}

template <typename T>
double PIDController<T>::getDt() const{
    return dt_;
}

#endif