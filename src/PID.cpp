#include "PID.h"
#include <iostream>
#include <cstring>

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
 run_optimisation(false),
 sqr_cte_limit(0.0),
 avg_sqr_cte(0.0),
 run_counter(0)
{
    memset(pid_gains, 0.0, 3);
    memset(gain_steps, 0.0, 3);
}

PID::~PID() {}

bool PID::Init(double in_Kp, double in_Ki, double in_Kd)
{
    bool ret = false;

    //Gains must not be negative and must not all zero
    if( (0.0 <= in_Kp) &&
        (0.0 <= in_Ki) &&
        (0.0 <= in_Kd) &&
        (0.0 < (in_Kp + in_Ki + in_Kd)) )
    {
        //Update gains
        Kp = in_Kp;
        Ki = in_Ki;
        Kd = in_Kd;

        //Reset error terms
        p_error = 0.0;
        i_error = 0.0;
        d_error = 0.0;

        //Reset run counter and average squared crosstrack error
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

bool PID::InitOptimisation(double init_pid_gains[],
                           double init_gain_steps[],
                           double in_sqr_cte_limit,
                           unsigned int in_max_run_counter)
{
    //Array size must be 3
    //Parameters must not be negative and must not all zero
    //Parameter steps must be positive
    bool ret = (0.0 <= init_pid_gains[0]) &&
               (0.0 <= init_pid_gains[1]) &&
               (0.0 <= init_pid_gains[2]) &&
               (0.0 < (init_pid_gains[0] + init_pid_gains[1] + init_pid_gains[2])) &&
               (0.0 < init_gain_steps[0]) &&
               (0.0 < init_gain_steps[1]) &&
               (0.0 < init_gain_steps[2]) &&
               (0.0 < in_sqr_cte_limit)   &&
               (0   < in_max_run_counter);

    if(ret)
    {
        memcpy(pid_gains,  init_pid_gains,  sizeof(pid_gains) );
        memcpy(gain_steps, init_gain_steps, sizeof(gain_steps));
        sqr_cte_limit    = in_sqr_cte_limit;
        max_run_counter  = in_max_run_counter;
        run_optimisation = true;

        std::cout << "Optimisation"
                  << ", Parameters ("      << pid_gains[0]  << ", " << pid_gains[1]  << ", " << pid_gains[2]  << ")"
                  << ", Parameter Steps (" << gain_steps[0] << ", " << gain_steps[1] << ", " << gain_steps[2] << ")"
                  << ", SqrCteLimit: "     << sqr_cte_limit
                  << ", MaxRunCounter: "   << max_run_counter
                  << std::endl;

        //Initialise PID controller using initial parameters
        Init(pid_gains[0], pid_gains[1], pid_gains[2]);
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

    avg_sqr_cte = (run_counter * avg_sqr_cte + (cte * cte)) / (run_counter + 1);
    ++run_counter;

    double pid_output = -( (Kp * p_error) + (Ki * i_error) + (Kd * d_error) );

    std::cout << " -- dT: "         << dt
              << ", avg_dT: "       << avg_dt
              << ", Kp: "           << Kp
              << ", Ki: "           << Ki
              << ", Kd: "           << Kd
              << ", Perr: "         << p_error
              << ", Ierr: "         << i_error
              << ", Derr: "         << d_error
              << ", PID: "          << pid_output
              << ", Optimisation: " << (run_optimisation ? "YES":"NO")
              << ", AvgSqrCte: "    << avg_sqr_cte
              << ", Counter: "      << run_counter
              << std::endl;

    last_cte = cte;
    last_clock_ticks = clock_ticks;

    RunOptimisation();

    return pid_output;
}

bool PID::IsInitialised() const
{
    return initialised;
}

void PID::RunOptimisation()
{
    if(run_optimisation)
    {
        if(avg_sqr_cte > sqr_cte_limit)

        {
            if(run_counter == max_run_counter)
            {
                //@TODO: Adjusting pid_gains at current index or update index of optimisation
                std::cout << "Run Optimisation! Adjusting Gains and Steps ..." << std::endl;

                // Right not just restart the whole run again
                Init(pid_gains[0], pid_gains[1], pid_gains[2]);
            }
        }
        else
        {
            run_optimisation = false;

            std::cout << "Optimisation completed! "
                      << " Kp: "  << Kp
                      << ", Ki: " << Ki
                      << ", Kd: " << Kd
                      << ", AvgSqrCte: " << avg_sqr_cte
                      << std::endl;
        }
    }
}
