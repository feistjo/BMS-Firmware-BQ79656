#pragma once
#include <Arduino.h>

class CoulombCounting
{
private:
    float state_of_charge_;
    float full_capacity_ = 4.2 * 4;
    int remaining_;
    uint32_t previous_time_;

public:
    void Initialize(float SOC, uint32_t current_time_)
    {
        state_of_charge_ = SOC;
        remaining_ = SOC * full_capacity_ * 60;
        previous_time_ = current_time_;
    }

    int CountCoulombs(float current, uint32_t current_time_)
    {
        remaining_ -= current * (current_time_ - previous_time_) / 1000;
        previous_time_ = current_time_;

        return remaining_;
    }

    float getSOC()
    {
        state_of_charge_ = remaining_ * 1000 / (60 * full_capacity_);
        return state_of_charge_;
    }
};