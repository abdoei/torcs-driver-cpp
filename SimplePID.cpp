/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

class PID
{
public:
    PID(double kp, double ki, double kd, double sp, double dt);
    PID() = default;
    void setSetPoint(double setPoint);
    void setInput(double input);
    void setKp(double kp);
    void setKi(double ki);
    void setKd(double kd);
    void setSampleTime(double sampleTime);
    double compute(double input);

    double getError();
    double getOutput();

private:
    double kp;
    double ki;
    double kd;
    double error;
    double last_e;
    double integral;
    double derivative;
    double input;
    double output;
    double setPoint;
    double dt;
    double sampleTime;
};

PID::PID(double kp, double ki, double kd, double sp, double dt)
    : kp(kp), ki(ki), kd(kd), setPoint(sp), dt(dt)
{

}
void PID::setSetPoint(double setPoint){
    this->setPoint = setPoint;
}
void PID::setInput(double input){
    this->input = input;
}
void PID::setKp(double kp){
    this->kp = kp;
}
void PID::setKi(double ki){
    this->ki = ki;
}
void PID::setKd(double kd){
    this->kd = kd;
}
void PID::setSampleTime(double sampleTime){
    this->sampleTime = sampleTime;
}
double PID::compute(double input){
    error = setPoint - input;
    integral = integral + error * dt;
    derivative = (error - last_e) / dt;
    output = kp * error + ki * integral + kd * derivative;
    last_e = error;
    return output;

}

double PID::getError(){
    return error;
}
double PID::getOutput(){
    return output;
}