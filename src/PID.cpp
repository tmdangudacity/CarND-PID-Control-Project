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
 last_clock_ticks(0),
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

void PID::UpdateError(double cte)
{
    clock_t clock_ticks = clock();

    if(initialised)
    {
        double dt = (double)(clock_ticks - last_clock_ticks) / CLOCKS_PER_SEC;

        if(dt > 0.0)
        {
            //Calculate P_error, I_error and D_error from current cte, last cte and delta time
            p_error = cte;

            i_error += cte * dt;
            if(i_error > 10.0)
            {
                i_error = 10.0;
            }
            else if(i_error < -10.0)
            {
                i_error = -10.0;
            }

            d_error = (cte - last_cte) / dt;

            std::cout << "-- Cte: "             << cte
                      << ", Clock ticks: "      << clock_ticks
                      << ", Last clock ticks: " << last_clock_ticks
                      << ", Dt: "               << dt
                      << ", Perror: "           << p_error
                      << ", Ierror: "           << i_error
                      << ", Derror: "           << d_error
                      << std::endl;
        }
        else
        {
            std::cout << "ERROR! Current clock_ticks: " << clock_ticks
                      << " < Last clock_ticks: "        << last_clock_ticks
                      << ", NOT update error terms!"  << std::endl;
        }
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
    last_clock_ticks = clock_ticks;
}

double PID::TotalError() const
{
    if( (0.0 == Kp) && (0.0 == Ki) && (0.0 == Kd) )
    {
        std::cout << "Warning! All PID gains still 0.0!" << std::endl;
    }

    return -(Kp * p_error + Ki * i_error * Kd * d_error);
}

bool PID::IsInitialised() const
{
    return initialised;
}

