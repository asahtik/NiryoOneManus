#include "ros_replacements/ros_time_repl.h"

namespace repl {
    microseconds sec_to_usec(double sec) { return microseconds((int)(1000000 * sec)); }
    Time time_now() {
        return clk::now();
    }
    void sleep(double x) {
        std::this_thread::sleep_for(sec_to_usec(x));
    }
    void sleep_until(Time t) {
        std::this_thread::sleep_until(t);
    }
    Time next_loop_start(Time prev_loop_start, double rate) {
            return prev_loop_start + sec_to_usec(1.0 / rate);
    }

    Rate::Rate(double rate) {
        this->timeout = sec_to_usec(1.0 / rate);
        this->loop_calc_start = time_now();
    }
    void Rate::sleep() {
        sleep_until(this->loop_calc_start + this->timeout);
        this->loop_calc_start = time_now();
    }
}