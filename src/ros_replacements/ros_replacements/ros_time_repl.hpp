#pragma once

// ros replacements
#include <thread>
#include <chrono>

#define clk std::chrono::steady_clock

using namespace std::chrono;

namespace repl {
    typedef time_point<clk> Time;

    inline microseconds sec_to_usec(double sec) { return microseconds((int)(1000000 * sec)); }
    inline Time time_now() {
        return clk::now();
    }
    inline void sleep(double x) {
        std::this_thread::sleep_for(sec_to_usec(x));
    }
    inline void sleep_until(Time t) {
        std::this_thread::sleep_until(t);
    }
    inline Time next_loop_start(Time prev_loop_start, double rate) {
            return prev_loop_start + sec_to_usec(1.0 / rate);
    }

    class Rate {
        public:
        /**
         * @brief Construct a new Rate object just before starting loop
         * 
         * @param rate max loops per second
         */
        Rate(double rate) {
            this->timeout = sec_to_usec(1.0 / rate);
            this->loop_calc_start = time_now();
        }

        /**
         * @brief Sleeps until next loop should be started. Call at the end of loop
         * 
         */
        void sleep() {
            sleep_until(this->loop_calc_start + this->timeout);
            this->loop_calc_start = time_now();
        }
        private:
        microseconds timeout;
        Time loop_calc_start;
    };
}