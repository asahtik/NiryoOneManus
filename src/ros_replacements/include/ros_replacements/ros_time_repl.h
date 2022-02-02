#include <thread>
#include <chrono>

#ifndef REPL_TIME_H
#define REPL_TIME_H

#define clk std::chrono::steady_clock

using namespace std::chrono;

namespace repl {
    typedef time_point<clk> Time;

    extern microseconds sec_to_usec(double);
    extern Time time_now();
    extern void sleep(double);
    extern void sleep_until(Time);
    extern Time next_loop_start(Time, double);

    class Rate {
        public:
        /**
         * @brief Construct a new Rate object just before starting loop
         * 
         * @param rate max loops per second
         */
        Rate(double);

        /**
         * @brief Sleeps until next loop should be started. Call at the end of loop
         * 
         */
        void sleep();
        private:
        microseconds timeout;
        Time loop_calc_start;
    };
}
#endif