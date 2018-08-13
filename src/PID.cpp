#include "PID.h"
#include <iostream>

PID::PID()
:p_error(0.0),
 i_error(0.0),
 d_error(0.0),
 Kp(0.0),
 Ki(0.0),
 Kd(0.0),
 gain_set(false),
 initialised(false),
 last_timestamp(0),
 last_cte(0.0)
{
}

PID::~PID() {}

void PID::Init(double in_Kp, double in_Ki, double in_Kd)
{
    Kp = in_Kp;
    Ki = in_Ki;
    Kd = in_Kd;

    gain_set = ((0.0 != Kp) || (0.0 != Ki) || (0.0 != Kd));

    std::cout << "PID Gains"
              << ", Kp: "       << Kp
              << ", Ki: "       << Ki
              << ", Kd: "       << Kd
              << ", Gains set " << (gain_set ? "OK":"Error!")
              << std::endl;
}

//@TODO: Take current speed as input?
void PID::UpdateError(double cte)
{
    //@TODO: Set timestamp to system time.
    unsigned long timestamp = 0.0;

    if(initialised)
    {
        //@TODO: Calculate P_error, I_error and D_error from current cte, last cte and delta time
    }
    else
    {
        if(gain_set)
        {
            initialised = true;
        }
        else
        {
            std::cout << "Error! PID Gains not set yet!" << std::endl;
        }
    }

    last_cte = cte;
    last_timestamp = timestamp;
}

double PID::TotalError() const
{
    if( (0.0 == Kp) && (0.0 == Ki) && (0.0 == Kd) )
    {
        std::cout << "Warning! All PID gains still 0.0!" << std::endl;
    }

    //@TODO: Scale by speed and between -1.0 and 1.0
    return (Kp * p_error + Ki * i_error * Kd * d_error);
}

bool PID::IsInitialised() const
{
    return initialised;
}

