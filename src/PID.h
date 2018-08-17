#ifndef PID_H
#define PID_H
#include <ctime>
#include <vector>

class PID
{
    public:

        /*
         * Constructor
         */
        PID();

        /*
         * Destructor.
         */
        virtual ~PID();

        /*
         * Initialize PID.
         */
        bool Init(double in_Kp, double in_Ki, double in_Kd);

        /*
         * Initialize parameter optimisation
         */
        bool InitOptimisation(double init_pid_gains[],
                              double init_gain_steps[],
                              double in_gain_step_tol,
                              unsigned int in_max_run_counter);

        /*
         * Update the PID error variables given cross track error.
         */
        double UpdateError(double cte);

        bool IsInitialised() const;

    private:

        /*
         * Errors
         */

        double p_error;
        double i_error;
        double d_error;

        /*
         * Coefficients
         */
        double Kp;
        double Ki;
        double Kd;

        bool gain_set;
        bool initialised;

        clock_t last_clock_ticks;
        double last_cte;

        //Optimisation
        bool run_optimisation;
        double pid_gains[3];
        double gain_steps[3];
        double gain_step_tol;
        unsigned int max_run_counter;

        double sqr_err;
        double best_sqr_err;
        unsigned int run_counter;

        void RunOptimisation();
};

#endif /* PID_H */
