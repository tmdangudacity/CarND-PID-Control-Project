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
 last_cte(0.0),
 run_counter(0),
 avg_sqr_cte(0.0)
{
}

PID::~PID() {}

bool PID::Init(double in_Kp, double in_Ki, double in_Kd)
{
    bool ret = false;

    if((0.0 != in_Kp) || (0.0 != in_Ki) || (0.0 != in_Kd))
    {
        //Update gains
        Kp = in_Kp;
        Ki = in_Ki;
        Kd = in_Kd;

        //Reset error terms
        p_error = 0.0;
        i_error = 0.0;
        d_error = 0.0;

        //Run counter
        run_counter = 0;
        avg_sqr_cte = 0.0;

        ret = true;

        if(!gain_set)
        {
            gain_set = true;
        }

        std::cout << "PID Gains set to"
                  << ": Kp: "       << Kp
                  << ", Ki: "       << Ki
                  << ", Kd: "       << Kd
                  << std::endl;
    }
    else
    {
        std::cout << "Input gains Error!"
                  << " Kp: "  << in_Kp
                  << ", Ki: " << in_Ki
                  << ", Kd: " << in_Kd
                  << ", Update gains FAILED!"
                  << std::endl;
    }

    return ret;
}

double PID::UpdateError(double cte)
{
    static double avg_dt = 0.0;

    clock_t clock_ticks = clock();
    double dt = 0.0;

    if(initialised)
    {
        dt = (double)(clock_ticks - last_clock_ticks) / CLOCKS_PER_SEC;

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

            avg_dt = (run_counter * avg_dt + dt)/(run_counter + 1);
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
            std::cout << "PID initialised!" << std::endl;
        }
        else
        {
            std::cout << "Error! PID Gains not set yet!" << std::endl;
        }
    }

    last_cte = cte;
    last_clock_ticks = clock_ticks;

    avg_sqr_cte = (run_counter * avg_sqr_cte + (cte * cte)) / (run_counter + 1);
    ++run_counter;

    double pid_output = -( (Kp * p_error) + (Ki * i_error) + (Kd * d_error) );

    std::cout << "-- Cte: "             << cte
              << ", Clock ticks: "      << clock_ticks
              << ", Last clock ticks: " << last_clock_ticks
              << ", Dt: "               << dt
              << ", Perror: "           << p_error
              << ", Ierror: "           << i_error
              << ", Derror: "           << d_error
              << ", AvgDt: "            << avg_dt
              << ", AvgSqrCte: "        << avg_sqr_cte
              << ", Run counter: "      << run_counter
              << ", PID output: "       << pid_output
              << std::endl;

    return pid_output;
}

/*
double PID::TotalError() const
{
    if(!gain_set)
    {
        std::cout << "Warning! All PID gains still 0.0!" << std::endl;
    }

    return -(Kp * p_error + Ki * i_error * Kd * d_error);
}
*/

bool PID::IsInitialised() const
{
    return initialised;
}

