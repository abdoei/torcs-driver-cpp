// Description: A simple PID controller class

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef SIMPLEPID_H_
#define SIMPLEPID_H_



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
    double maxI=1;
    double maxD=1;
};

#endif /* SIMPLEPID_H_ */