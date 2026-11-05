#pragma once

#include <cmath>
#include <cstdint>

namespace pb_minco {

// No motion estimation or PVA reconstruction. The owner supplies locking.
class StalledWarmStartGate {
 public:
    struct State {
        bool inhibited{false};
        std::uint64_t epoch{0};
    };

    void update(bool stalled, double receipt_time, double timeout) {
        if (!validTime(receipt_time) || !std::isfinite(timeout) || timeout <= 0.0) {return;}
        const bool interrupted = !have_sample_ || receipt_time < receipt_time_ ||
            receipt_time - receipt_time_ > timeout;
        if (stalled && (!stalled_ || interrupted)) {++epoch_;}
        stalled_ = stalled;
        receipt_time_ = receipt_time;
        have_sample_ = true;
    }

    State state(double now, double timeout) const {
        const bool fresh = have_sample_ && validTime(now) && std::isfinite(timeout) &&
            timeout > 0.0 && now >= receipt_time_ && now - receipt_time_ <= timeout;
        return {fresh && stalled_, epoch_};
    }

 private:
    static bool validTime(double value) {return std::isfinite(value) && value >= 0.0;}
    bool have_sample_{false}, stalled_{false};
    double receipt_time_{0.0};
    std::uint64_t epoch_{0};
};

}  // namespace pb_minco
