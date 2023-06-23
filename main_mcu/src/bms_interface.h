#pragma once

#include <vector>

class IBMS
{
public:
    enum class BMSFault : bool
    {
        kNotFaulted = 0,
        kFaulted = 1,
    };

    enum class BMSState
    {
        kShutdown = 0,
        kPrecharge = 1,
        kActive = 2,
        kCharging = 3,
        kFault = 4
    };

    virtual const std::vector<float> &GetVoltages() = 0;
    virtual const std::vector<float> &GetTemperatures() = 0;
    virtual const std::vector<float> &GetCurrent() = 0;

    virtual BMSState GetState() = 0;
    virtual float GetMaxCellTemperature() = 0;
    virtual float GetMinCellTemperature() = 0;
    virtual float GetAverageCellTemperature() = 0;
    virtual float GetMaxCellVoltage() = 0;
    virtual float GetMinCellVoltage() = 0;
    virtual float GetSOC() = 0;

    virtual float GetMaxDischargeCurrent() = 0;
    virtual float GetMaxRegenCurrent() = 0;
    virtual float GetPackVoltage() = 0;

    virtual BMSFault GetFaultSummary() = 0;
    virtual BMSFault GetUnderVoltageFault() = 0;
    virtual BMSFault GetOverVoltageFault() = 0;
    virtual BMSFault GetUnderTemperatureFault() = 0;
    virtual BMSFault GetOverTemperatureFault() = 0;
    virtual BMSFault GetOverCurrentFault() = 0;
    virtual BMSFault GetExternalKillFault() = 0;
    virtual BMSFault GetOpenWireFault() = 0;
};