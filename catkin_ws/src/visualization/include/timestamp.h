#ifndef _TIMESTAMP_H_
#define _TIMESTAMP_H_
#include <ctime>
#include <stdint.h>
#include <chrono>

using Duration = std::chrono::nanoseconds;

using Timestamp = std::chrono::time_point<std::chrono::system_clock, Duration>;

using nanos = std::chrono::nanoseconds;
using micros = std::chrono::microseconds;
using millis = std::chrono::milliseconds;
using seconds = std::chrono::seconds;
using minutes = std::chrono::minutes;
using hours = std::chrono::hours;

namespace planning {
    std::time_t getTimeStamp();
    std::tm* gettm(int64_t timestamp);


    static Timestamp SystemNow() {
        return std::chrono::time_point_cast<Duration>(
            std::chrono::system_clock::now());
    }

    template <typename PrecisionDuration>
    int64_t AsInt64(const Duration &duration) {
        return std::chrono::duration_cast<PrecisionDuration>(duration).count();
    }

    inline double ToSecond(const Timestamp &timestamp) {
        return static_cast<double>(AsInt64<nanos>(timestamp.time_since_epoch())) *
            1e-9;
    }
    static double NowInSeconds() { return ToSecond(SystemNow()); }

}

#endif
