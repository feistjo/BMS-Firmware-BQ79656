#pragma once
#include <Arduino.h>

#include <algorithm>

#include "I-Charger.h"
#include "Watchdog_t4.h"
#include "bms_interface.h"
#include "bms_telemetry.h"
#include "bq_comm.h"
#include "can_interface.h"
#include "cellinfo.h"
#include "coulomb_counting.h"
#include "teensy_pin_defs.h"

template <typename T>
T clamp(const T& n, const T& lower, const T& upper)
{
    return std::max(lower, std::min(n, upper));
}

class ShutdownInput
{
public:
    ShutdownInput(uint8_t pin, float resistor_ratio, float min_lv_voltage, float min_charger_voltage)
        : pin_{pin},
          resistor_ratio_{resistor_ratio},
          min_lv_voltage_{min_lv_voltage},
          min_charger_voltage_{min_charger_voltage}
    {
        pinMode(pin_, INPUT);
    }

    enum class InputState
    {
        kShutdown,
        kCharging,
        kActive,
    };

    InputState GetStatus()
    {
        float voltage = analogRead(pin_) * 3.3 / 1023 / resistor_ratio_;
        if (voltage > min_lv_voltage_)
        {
            return InputState::kActive;
        }
        else if (voltage > min_charger_voltage_)
        {
            return InputState::kCharging;
        }
        else
        {
            return InputState::kShutdown;
        }
    }

private:
    uint8_t pin_;
    float resistor_ratio_;
    float min_lv_voltage_;
    float min_charger_voltage_;
};

class BMS : public IBMS
{
public:
    enum class Command : uint8_t
    {
        kNoAction = 0,
        kPrechargeAndCloseContactors = 1,
        kShutdown = 2,
        kClearFaults = 3
    };

    BMS(BQ79656 bq /*  = BQ79656{Serial8, 35} */,
        int num_cells_series,
        int num_thermistors,
        ICharger& charger,
        VirtualTimerGroup& timer_group,
        ICAN& hp_can,
        ICAN& lp_can,
        ICAN& vb_can,
        ShutdownInput& shutdown_input)
        : bq_{bq},
          kNumCellsSeries{num_cells_series},
          kNumThermistors{num_thermistors},
          charger_{charger},
          timer_group_{timer_group},
          hp_can_{hp_can},
          lp_can_{lp_can},
          vb_can_{vb_can},
          shutdown_input_{shutdown_input},
          voltages_{std::vector<float>(kNumCellsSeries)},
          temperatures_{std::vector<float>(kNumThermistors)},
          current_{std::vector<float>(1)}
    {
        command_signal_ = Command::kNoAction;
    }

    void Initialize()
    {
        // attach fault interrupts
        /* for (int i = 0; i < num_kill_pins; i++)
        {
            // TODO: fix kill pins activating on contactor
            //  pinMode(kill_pins[i], INPUT);
            //  attachInterrupt(digitalPinToInterrupt(kill_pins[i]), FaultInterrupt, FALLING);
        } */

        pinMode(charger_sense, INPUT_PULLDOWN);

        pinMode(coolant_ctrl, OUTPUT);
        pinMode(contactorprecharge_ctrl, OUTPUT);
        pinMode(contactorp_ctrl, OUTPUT);
        pinMode(contactorn_ctrl, OUTPUT);

        // initialize the BQ chip driver
        // bq_.SetStackSize(2);  // TODO: temporary
        bq_.Initialize();

        // initialize BMS telemetry
        telemetry.InitializeCAN();
        hp_can_.RegisterRXMessage(command_message_hp_);
        vb_can_.RegisterRXMessage(command_message_vb_);

        // initialize the watchdog timer to shutdown if the dog isn't fed for 1 second, reset if the dog isn't fed for 2
        // seconds
        WDT_timings_t config;
        config.trigger = 1; /* in seconds, 0->128 */
        config.timeout = 2; /* in seconds, 0->128 */
        config.callback = [this]() { this->ChangeState(BMSState::kFault); };
        watchdog_timer_.begin(config);

        timer_group_.AddTimer(
            15000, [this]() { this->open_wire_fault_ = static_cast<BMSFault>(this->bq_.RunOpenWireCheck()); });
    }

    void Tick();

    void CalculateSOE();

    const std::vector<float>& GetVoltages() override { return voltages_; }
    const std::vector<float>& GetTemperatures() override { return temperatures_; }
    const std::vector<float>& GetCurrent() override { return current_; }

    BMSState GetState() override { return current_state_; }
    float GetMaxCellTemperature() override { return max_cell_temperature_; }
    float GetAverageCellTemperature() override { return average_cell_temperature_; }
    float GetMinCellTemperature() override { return min_cell_temperature_; }
    float GetMaxCellVoltage() override { return max_cell_voltage_; }
    float GetMinCellVoltage() override { return min_cell_voltage_; }
    float GetSOC() override
    {
        if (BMSState::kActive == current_state_)
        {
            return coulomb_count_.getSOC() * 100;
        }
        else
            return cell.VoltageToSOC(min_cell_voltage_);
    }

    float GetMaxDischargeCurrent() override { return max_allowed_discharge_current_; }
    float GetMaxRegenCurrent() override { return max_allowed_regen_current_; }
    float GetPackVoltage() override { return pack_voltage_; }

    BMSFault GetFaultSummary() override { return static_cast<BMSFault>(current_state_ == BMSState::kFault); }
    BMSFault GetUnderVoltageFault() override { return undervoltage_fault_; }
    BMSFault GetOverVoltageFault() override { return overvoltage_fault_; }
    BMSFault GetUnderTemperatureFault() override { return undertemperature_fault_; }
    BMSFault GetOverTemperatureFault() override { return overtemperature_fault_; }
    BMSFault GetOverCurrentFault() override { return overcurrent_fault_; }
    BMSFault GetExternalKillFault() override { return external_kill_fault_; }
    BMSFault GetOpenWireFault() override { return open_wire_fault_; }

private:
    INR21700P42A cell;

    CoulombCounting coulomb_count_;

    BQ79656 bq_;

    WDT_T4<WDT1> watchdog_timer_;

    const int kNumCellsSeries;
    const int kNumThermistors;

    ICharger& charger_;
    VirtualTimerGroup& timer_group_;

    ICAN& hp_can_;
    ICAN& lp_can_;
    ICAN& vb_can_;

    ShutdownInput& shutdown_input_;

    // Consts for SoE calculation + Fault Detection
    const int kNumCellsParallel{4};
    const float kDischargeCurrent{45.0f * kNumCellsParallel};
    const float kRegenCurrent{45.0f * kNumCellsParallel};
    const float kMaxPowerOutput{80000.0f};
    const float kCellUndervoltage{2.5f};
    const float kCellOvervoltage{4.2f};
    const float kInternalResistance{0.016f};
    const float kOvercurrent{180.0f};
    const float kOvertemp{60.0f};
    const float kUndertemp{-40.0f};
    const uint32_t kPrechargeTime{2000};

    MakeUnsignedCANSignal(Command, 0, 8, 1, 0) command_signal_{};
    MakeUnsignedCANSignal(bool, 8, 1, 1, 0) high_current_charging_{};
    CANRXMessage<1> command_message_hp_{hp_can_, 0x242, command_signal_};
    CANRXMessage<2> command_message_vb_{vb_can_, 0x242, command_signal_, high_current_charging_};

    std::vector<float> voltages_;
    std::vector<float> temperatures_;
    std::vector<float> current_;

    float max_cell_voltage_;
    float min_cell_voltage_;
    float pack_voltage_;
    float max_cell_temperature_;
    float min_cell_temperature_;
    float average_cell_temperature_;
    float max_allowed_discharge_current_;
    float max_allowed_regen_current_;
    float state_of_charge_;

    BMSFault undervoltage_fault_{BMSFault::kNotFaulted};
    BMSFault overvoltage_fault_{BMSFault::kNotFaulted};
    BMSFault undertemperature_fault_{BMSFault::kNotFaulted};
    BMSFault overtemperature_fault_{BMSFault::kNotFaulted};
    BMSFault overcurrent_fault_{BMSFault::kNotFaulted};
    BMSFault external_kill_fault_{BMSFault::kNotFaulted};
    BMSFault open_wire_fault_{BMSFault::kNotFaulted};

    static int fault_pin_;
    BMSFault fault_{BMSFault::kNotFaulted};

    BMSState current_state_{BMSState::kShutdown};

    BMSTelemetry telemetry{hp_can_, vb_can_, lp_can_, timer_group_, *this};
    uint32_t state_entry_time_{0};

    void ProcessState();
    void ChangeState(BMSState new_state);

    void UpdateValues();

    void ProcessCooling();

    void CheckFaults();

    static void ShutdownCar()
    {
        // kill the car
        digitalWrite(contactorn_ctrl, LOW);
        digitalWrite(contactorp_ctrl, LOW);
        digitalWrite(contactorprecharge_ctrl, LOW);
        return;
    }

    /* static void FaultInterrupt()
    {
        ShutdownCar();

        bool foundFault = 0;
        for (int i = 0; i < num_kill_pins; i++)
        {
            if (!digitalRead(kill_pins[i]))
            {
                fault_pin_ = kill_pins[i];
                break;
            }
        }
        if (!foundFault)
        {
            fault_pin_ = -1;
        }
    } */

    void GetMaxMinAvgTot(double* arr,
                         int arrSize,
                         double* res)  // may be unused/deleted or replaced with a different function/implementation
    {
        double currMax = arr[0];
        double currMin = arr[0];
        double currTot = arr[0];
        for (int i = 1; i < arrSize; i++)
        {
            currMax = max(currMax, arr[i]);
            currMin = min(currMin, arr[i]);
            currTot += arr[i];
        }
        res[0] = currMax;
        res[1] = currMin;
        res[2] = currTot / arrSize;
        res[3] = currTot;
    }
};
