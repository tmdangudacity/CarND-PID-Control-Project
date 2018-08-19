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
 gain_step_tol(0.0),
 sqr_err(0.0),
 best_sqr_err(-1.0),
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
        sqr_err = 0.0;

        ret = true;

        if(!gain_set)
        {
            gain_set = true;
        }

        std::cout << "PID Gains set to"
                  << ": Kp: " << Kp
                  << ", Ki: " << Ki
                  << ", Kd: " << Kd
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
                           double in_gain_step_tol,
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
               (0.0 < in_gain_step_tol)   &&
               (0   < in_max_run_counter);

    if(ret)
    {
        memcpy(pid_gains,  init_pid_gains,  sizeof(pid_gains) );
        memcpy(gain_steps, init_gain_steps, sizeof(gain_steps));
        gain_step_tol    = in_gain_step_tol;
        max_run_counter  = in_max_run_counter;
        best_sqr_err  = -1.0;
        run_optimisation = true;

        std::cout << "Optimisation"
                  << ", PID Gains ("           << pid_gains[0]  << ", " << pid_gains[1]  << ", " << pid_gains[2]  << ")"
                  << ", Gain Steps ("          << gain_steps[0] << ", " << gain_steps[1] << ", " << gain_steps[2] << ")"
                  << ", Gain Step Tolerance: " << gain_step_tol
                  << ", MaxRunCounter: "       << max_run_counter
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

    sqr_err = (run_counter * sqr_err + (cte * cte)) / (run_counter + 1);
    ++run_counter;

    double pid_output = -( (Kp * p_error) + (Ki * i_error) + (Kd * d_error) );

    if(!run_optimisation)
    {
        std::cout << " -- dT: "         << dt
                  << ", avg_dT: "       << avg_dt
                  << ", Kp: "           << Kp
                  << ", Ki: "           << Ki
                  << ", Kd: "           << Kd
                  << ", Perr: "         << p_error
                  << ", Ierr: "         << i_error
                  << ", Derr: "         << d_error
                  << ", PID: "          << pid_output
                  << ", Optimisation: " << (run_optimisation ? "ON":"OFF")
                  << ", SqrError: "     << sqr_err
                  << ", Counter: "      << run_counter
                  << std::endl;
    }

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
    static unsigned int iter  = 0;
    static unsigned int index = 0;
    static unsigned int step_index = 0;

    if(run_optimisation)
    {
        //One lap completed
        if(run_counter >= max_run_counter)
        {
            if( gain_step_tol < (gain_steps[0] + gain_steps[1] + gain_steps[2]) )
            {
                if(-1.0 == best_sqr_err)
                {
                    //Set best error the first time
                    best_sqr_err = sqr_err;

                    //First run
                    std::cout << " - Optimisation, First run"
                              << ", Kp: "          << pid_gains[0]
                              << ", Ki: "          << pid_gains[1]
                              << ", Kd: "          << pid_gains[2]
                              << ", Best SqrErr: " << best_sqr_err
                              << std::endl;

                    //Add gain step to the gain at index 0
                    pid_gains[index] += gain_steps[index];

                    std::cout << " -- Step " << step_index
                              << ", Gain["   << index << "] = " << pid_gains[index]
                              << ", Step["   << index << "] = " << gain_steps[index]
                              << std::endl;

                    Init(pid_gains[0], pid_gains[1], pid_gains[2]);
                    step_index = 1;
                }
                else if(1 == step_index) //Step 1
                {
                    if(sqr_err < best_sqr_err)
                    {
                        //Update best error
                        best_sqr_err = sqr_err;

                        //Increase current gain step by 10%
                        gain_steps[index] *= 1.1;

                        unsigned int temp_index = index;

                        //Move on to the next PID gain
                        index = (index + 1) % 3;

                        std::cout << " -- Step "                   << step_index
                                  << ", Index: "                   << temp_index
                                  << ", SqrErr (better): "         << best_sqr_err
                                  << ", Keep Gain["                << temp_index
                                  << "] at: "                      << pid_gains[temp_index]
                                  << ", Increased Step["           << temp_index
                                  << "] to: "                      << gain_steps[temp_index]
                                  << ", Move to Next Gain index: " << index
                                  << std::endl;

                        if(0 == index)
                        {   //If return back to PID gain index 0, increase iterator
                            std::cout << " - Optimisation"
                                      << " Iteration: "    << iter
                                      << ", Kp: "          << pid_gains[0]
                                      << ", Ki: "          << pid_gains[1]
                                      << ", Kd: "          << pid_gains[2]
                                      << ", Best SqrErr: " << best_sqr_err
                                      << std::endl;
                            ++iter;
                        }

                        //Add gain step to the gain at next index
                        pid_gains[index] += gain_steps[index];

                        std::cout << " -- Step " << step_index
                                  << ", Gain["   << index << "] = " << pid_gains[index]
                                  << ", Step["   << index << "] = " << gain_steps[index]
                                << std::endl;

                        Init(pid_gains[0], pid_gains[1], pid_gains[2]);
                        step_index = 1;
                    }
                    else
                    {
                        //Go back two steps for current gain
                        pid_gains[index] -= 2.0 * gain_steps[index];

                        std::cout << " -- Step "               << step_index
                                  << ", Index: "               << index
                                  << ", SqrErr (not better): " << sqr_err
                                  << ", Decreased Gain["       << index
                                  << "] to: "                  << pid_gains[index]
                                  << std::endl;

                        Init(pid_gains[0], pid_gains[1], pid_gains[2]);
                        step_index = 2;
                    }
                }
                else if(2 == step_index) //Step 2
                {
                    if(sqr_err < best_sqr_err)
                    {
                        //Update best error
                        best_sqr_err = sqr_err;

                        //Increase current gain step by 10%
                        gain_steps[index] *= 1.1;

                        std::cout << " -- Step "           << step_index
                                  << ", Index: "           << index
                                  << ", SqrErr (better): " << sqr_err
                                  << ", Keep Gain["        << index
                                  << "] at: "              << pid_gains[index]
                                  << ", Increased Step["   << index
                                  << "] to: "              << gain_steps[index];
                    }
                    else
                    {
                        //Return current gain to the original value
                        pid_gains[index] += gain_steps[index];

                        //Decrease current gain step by 10%
                        gain_steps[index] *= 0.9;

                        std::cout << " -- Step "               << step_index
                                  << ", Index: "               << index
                                  << ", SqrErr (not better): " << sqr_err
                                  << ", Set Gain["             << index
                                  << "] back to "              << pid_gains[index]
                                  << ", Reduced Step["         << index
                                  << "] to: "                  << gain_steps[index];
                    }

                    //Move on to the next PID gain
                    index = (index + 1) % 3;
                    std::cout << ", Move to next Gain Index " << index << std::endl;

                    if(0 == index)
                    {   //If return back to PID gain index 0, increase iterator
                        std::cout << " - Optimisation"
                                  << ", New Iteration: " << iter
                                  << ", Kp: "            << pid_gains[0]
                                  << ", Ki: "            << pid_gains[1]
                                  << ", Kd: "            << pid_gains[2]
                                  << ", Best SqrErr: "   << best_sqr_err
                                  << std::endl;
                        ++iter;
                    }

                    //Add gain step to the gain at next index
                    pid_gains[index] += gain_steps[index];

                    std::cout << " -- Step " << step_index
                              << ", Gain["   << index << "] = " << pid_gains[index]
                              << ", Step["   << index << "] = " << gain_steps[index]
                              << std::endl;

                    Init(pid_gains[0], pid_gains[1], pid_gains[2]);
                    step_index = 1;
                }
            }
            else
            {
                run_optimisation = false;

                std::cout << " - Optimisation Completed!"
                          << " Kp: "          << pid_gains[0]
                          << ", Ki: "         << pid_gains[1]
                          << ", Kd: "         << pid_gains[2]
                          << ", BestSqrErr: " << best_sqr_err
                          << std::endl;
            }
        }
    }
}
