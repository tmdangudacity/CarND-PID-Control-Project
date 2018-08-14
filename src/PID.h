#ifndef PID_H
#define PID_H
#include <ctime>

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
        void Init(double in_Kp, double in_Ki, double in_Kd);

        /*
         * Update the PID error variables given cross track error.
         */
        void UpdateError(double cte);

        /*
         * Calculate the total PID error.
         */
        double TotalError() const;

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
};

#endif /* PID_H */
