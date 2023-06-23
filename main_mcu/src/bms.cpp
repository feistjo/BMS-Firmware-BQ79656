#include "bms.h"

#include <algorithm>
#include <numeric>

#include "I-Charger.h"
#include "can_interface.h"

int BMS::fault_pin_{-1};

void BMS::CheckFaults()
{
    overvoltage_fault_ = static_cast<BMSFault>(max_cell_voltage_ >= kCellOvervoltage);
    undervoltage_fault_ = static_cast<BMSFault>(min_cell_voltage_ <= kCellUndervoltage);
    overcurrent_fault_ = static_cast<BMSFault>(current_[0] >= kOvercurrent);
    overtemperature_fault_ = static_cast<BMSFault>(max_cell_temperature_ >= kOvertemp);
    undertemperature_fault_ = static_cast<BMSFault>(min_cell_temperature_ <= kUndertemp);
    external_kill_fault_ = static_cast<BMSFault>(shutdown_input_.GetStatus() == ShutdownInput::InputState::kShutdown);

    fault_ =
        static_cast<BMSFault>(static_cast<bool>(overvoltage_fault_) || static_cast<bool>(undervoltage_fault_)
                              || static_cast<bool>(overcurrent_fault_) || static_cast<bool>(overtemperature_fault_)
                              || static_cast<bool>(undertemperature_fault_) || static_cast<bool>(open_wire_fault_)
                              || (static_cast<bool>(external_kill_fault_) && current_state_ != BMSState::kShutdown));
}

void BMS::Tick()
{
    watchdog_timer_.feed();  // so we don't reboot
    // check fault status
    if (fault_ != BMSFault::kNotFaulted && current_state_ != BMSState::kFault)
    {
        /* #if serialdebug
                Serial.println("Faults:");
                if (overvoltage_fault_)
                {
                    Serial.println("  Overvoltage");
                }
                else if (undervoltage_fault_)
                {
                    Serial.println("  Undervoltage");
                }
                if (overtemperature_fault_)
                {
                    Serial.println("  Overtemperature");
                }
                else if (undertemperature_fault_)
                {
                    Serial.println("  Undertemperature");
                }
                if (overcurrent_fault_)
                {
                    Serial.println("  Overcurrent");
                }
                Serial.println("");
        #endif */

        ChangeState(BMSState::kFault);
    }

    ProcessState();

    // log to SD, send to ESP, send to CAN
    // todo
}

// Find maximum discharge and regen current
void BMS::CalculateSOE()
{
    float current_per_cell = current_[0] / kNumCellsParallel;
    float internal_resistance_per_series_element = kInternalResistance / kNumCellsParallel;

    // Find highest and lowest open circuit voltage
    float min_open_circuit_voltage = min_cell_voltage_ + (current_per_cell * kInternalResistance);
    float max_open_circuit_voltage = max_cell_voltage_ + (current_per_cell * kInternalResistance);

    // Limit at V_open + I * (R_internal / numCellsParallel) = V_boundary
    //  => I = (numCellsParallel / R_internal) * (V_boundary - V_open)
    float max_discharge_voltage_delta = min_open_circuit_voltage - kCellUndervoltage;
    float max_regen_voltage_delta = kCellOvervoltage - max_open_circuit_voltage;
    float uncapped_discharge_current = max_discharge_voltage_delta / internal_resistance_per_series_element;
    float uncapped_regen_current = max_regen_voltage_delta / internal_resistance_per_series_element;

    // I = P / V
    pack_voltage_ = std::accumulate(voltages_.begin(), voltages_.end(), 0);
    float power_capped_current = kMaxPowerOutput / pack_voltage_;

    max_allowed_discharge_current_ = std::min({uncapped_discharge_current, power_capped_current, kDischargeCurrent});
    // Serial.println(max_allowed_discharge_current_);
    max_allowed_regen_current_ = std::min(uncapped_regen_current, kRegenCurrent);
}

void BMS::ProcessCooling()
{
    // check temperatures
    bq_.GetTemps(temperatures_);
    max_cell_temperature_ = *std::max_element(temperatures_.begin(), temperatures_.end());
    min_cell_temperature_ = *std::min_element(temperatures_.begin(), temperatures_.end());
    average_cell_temperature_ = std::accumulate(temperatures_.begin(), temperatures_.end(), 0) / temperatures_.size();
    analogWrite(
        coolant_ctrl,
        clamp<uint8_t>(
            map(max_cell_temperature_, 20, 50, 0, 255), 0, 255));  // Current pin is NOT pwm capable - rev board or
                                                                   // use softpwm, also map ranges may be suboptimal
}

void BMS::UpdateValues()
{
    ProcessCooling();
    bq_.GetCurrent(current_);
    bq_.GetVoltages(voltages_);
    max_cell_voltage_ = *std::max_element(voltages_.begin(), voltages_.end());
    min_cell_voltage_ = *std::min_element(voltages_.begin(), voltages_.end());
    CalculateSOE();
    if (!coulomb_count_.Initialized())
    {
        coulomb_count_.Initialize(cell.VoltageToSOC(min_cell_voltage_), millis());
    }
    else
    {
        coulomb_count_.CountCoulombs(current_[0], millis());
    }
#if serialdebug
    Serial.print("Current: ");
    Serial.print(current_[0]);
    Serial.print("\tMax voltage: ");
    Serial.println(max_cell_voltage_);
#endif
}

void BMS::ProcessState()
{
    UpdateValues();
    // check faults
    CheckFaults();
    switch (current_state_)
    {
        case BMSState::kShutdown:
            // check for command to go to active
            if (command_signal_ == Command::kPrechargeAndCloseContactors)
            {
                ChangeState(BMSState::kPrecharge);
            }
            else if (charger_.IsConnected() && !(command_signal_ == Command::kShutdown))
            {
                Serial.println("Detected charger");
                ChangeState(BMSState::kPrecharge);
            }
            break;
        case BMSState::kPrecharge:
            // do a time-based precharge
            if (command_signal_ == Command::kShutdown)
            {
                ChangeState(BMSState::kShutdown);
            }
            if (millis() >= state_entry_time_ + kPrechargeTime)
            {
                if (charger_.IsConnected())
                {
                    ChangeState(BMSState::kCharging);
                }
                else if (command_signal_ == Command::kPrechargeAndCloseContactors)
                {
                    ChangeState(BMSState::kActive);
                }
                else
                {
                    ChangeState(BMSState::kShutdown);
                }
            }

            break;
        case BMSState::kActive:
            if (command_signal_ == Command::kShutdown)
            {
                ChangeState(BMSState::kShutdown);
            }
            else if (charger_.IsConnected())
            {
                ChangeState(BMSState::kCharging);
            }
            break;
        case BMSState::kCharging:
            static constexpr float kMaxChargeVoltage{4.19f};

            charger_.Tick(millis());

            if (!charger_.IsConnected() || command_signal_ == Command::kShutdown)
            {
                charger_.Disable();
                ChangeState(BMSState::kShutdown);
                break;
            }
            // cell balancing if charging
            bq_.ProcessBalancing(voltages_, kMaxChargeVoltage);
            // pause charging if danger of overvoltage
            if (max_cell_voltage_ > kMaxChargeVoltage)
            {
                charger_.SetVoltageCurrent(kMaxChargeVoltage * kNumCellsSeries, 0);
                // pause charging, set current to 0
            }
            else
            {
                if (high_current_charging_)
                {
                    // charger_.SetMaxCurrent(14);
                    charger_.SetMaxPower(240 * 20);
                }
                else
                {
                    // charger_.SetMaxCurrent(0.1);
                    charger_.SetMaxPower(120 * 15);
                }
                charger_.SetVoltageCurrent(kMaxChargeVoltage * kNumCellsSeries, max_allowed_regen_current_);
            }
            // todo
            break;
        case BMSState::kFault:
            // check for clear faults command
            if (command_signal_ == Command::kClearFaults)
            {
                external_kill_fault_ = BMSFault::kNotFaulted;
                ChangeState(BMSState::kShutdown);
            }
            break;
    }
}

void BMS::ChangeState(BMSState new_state)
{
#if serialdebug
    Serial.print("Changing state: ");
    Serial.println(new_state == BMSState::kShutdown    ? "Shutdown"
                   : new_state == BMSState::kPrecharge ? "Precharge"
                   : new_state == BMSState::kFault     ? "Fault"
                                                       : "Other");
#endif
    state_entry_time_ = millis();
    switch (new_state)
    {
        case BMSState::kShutdown:
            ShutdownCar();
            current_state_ = BMSState::kShutdown;
            break;
        case BMSState::kPrecharge:
            digitalWrite(contactorn_ctrl, HIGH);
            delay(1);
            digitalWrite(contactorprecharge_ctrl, HIGH);  // precharge, but don't turn on car yet
            current_state_ = BMSState::kPrecharge;
            break;
        case BMSState::kActive:
            digitalWrite(contactorp_ctrl, HIGH);         // turn on car
            digitalWrite(contactorprecharge_ctrl, LOW);  // disable precharge when car is running
            coulomb_count_.Initialize(cell.VoltageToSOC(min_cell_voltage_), state_entry_time_);
            current_state_ = BMSState::kActive;
            break;
        case BMSState::kCharging:
            // enable charger?
            {
                digitalWrite(contactorp_ctrl, HIGH);         // turn on car
                digitalWrite(contactorprecharge_ctrl, LOW);  // disable precharge when car is running
                coulomb_count_.Initialize(cell.VoltageToSOC(min_cell_voltage_), state_entry_time_);
                charger_.Enable();
                current_state_ = BMSState::kCharging;
                break;
            }
        case BMSState::kFault:
            // open contactors
            ShutdownCar();
            current_state_ = BMSState::kFault;
            break;
    }
}
