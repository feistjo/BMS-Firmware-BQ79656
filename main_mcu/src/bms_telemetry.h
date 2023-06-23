#pragma once

#include "bms_interface.h"
#include "teensy_can.h"
#include "virtualTimer.h"

class BMSTelemetry
{
public:
    BMSTelemetry(ICAN &hp_can_bus, ICAN &vb_can_bus, ICAN &lp_can_bus, VirtualTimerGroup &timer_group, IBMS &bms)
        : hp_can_bus_{hp_can_bus},
          vb_can_bus_{vb_can_bus},
          lp_can_bus_{lp_can_bus},
          timer_group_{timer_group},
          bms_{bms}
    {
    }

    void InitializeCAN();

private:
    static const uint16_t kSignalsPerMessage = 7;
    static const uint16_t kNumVoltageMessages = 20;
    static const uint16_t kNumTemperatureMessages = 16;

    ICAN &hp_can_bus_;
    ICAN &vb_can_bus_;
    ICAN &lp_can_bus_;

    VirtualTimerGroup &timer_group_;

    IBMS &bms_;

    static const uint32_t kVoltageMessageTransmitPeriod{100};
    std::array<ITypedCANSignal<float> *, kNumVoltageMessages * kSignalsPerMessage> voltage_signals_;
    std::array<CANTXMessage<kSignalsPerMessage> *, kNumVoltageMessages> voltage_messages_;

    static const uint32_t kTemperatureMessageTransmitPeriod{100};
    std::array<ITypedCANSignal<float> *, kNumTemperatureMessages * kSignalsPerMessage> temperature_signals_;
    std::array<CANTXMessage<kSignalsPerMessage> *, kNumTemperatureMessages> temperature_messages_;

    static const uint32_t kSOEMessageTransmitPeriod{10};
    MakeUnsignedCANSignal(float, 0, 12, 0.1, 0) max_discharge_signal_;
    MakeUnsignedCANSignal(float, 12, 12, 0.1, 0) max_regen_signal_;
    MakeUnsignedCANSignal(float, 24, 16, 0.01, 0) battery_voltage_signal_;
    MakeUnsignedCANSignal(float, 40, 8, 1, -40) battery_temperature_signal_;
    MakeSignedCANSignal(float, 48, 16, 0.01, 0) battery_current_signal_;
    CANTXMessage<5> hp_soe_message_{hp_can_bus_,
                                    576,
                                    8,
                                    kSOEMessageTransmitPeriod,
                                    timer_group_,
                                    max_discharge_signal_,
                                    max_regen_signal_,
                                    battery_voltage_signal_,
                                    battery_temperature_signal_,
                                    battery_current_signal_};
    CANTXMessage<5> vb_soe_message_{vb_can_bus_,
                                    576,
                                    8,
                                    kSOEMessageTransmitPeriod,
                                    timer_group_,
                                    max_discharge_signal_,
                                    max_regen_signal_,
                                    battery_voltage_signal_,
                                    battery_temperature_signal_,
                                    battery_current_signal_};

    static const uint32_t kFaultMessageTransmitPeriod{100};
    MakeUnsignedCANSignal(IBMS::BMSFault, 0, 1, 1, 0) fault_summary_signal_;
    MakeUnsignedCANSignal(IBMS::BMSFault, 2, 1, 1, 0) undervoltage_signal_;
    MakeUnsignedCANSignal(IBMS::BMSFault, 3, 1, 1, 0) overvoltage_signal_;
    MakeUnsignedCANSignal(IBMS::BMSFault, 3, 1, 1, 0) undertemperature_signal_;
    MakeUnsignedCANSignal(IBMS::BMSFault, 4, 1, 1, 0) overtemperature_signal_;
    MakeUnsignedCANSignal(IBMS::BMSFault, 5, 1, 1, 0) overcurrent_signal_;
    MakeUnsignedCANSignal(IBMS::BMSFault, 6, 1, 1, 0) external_kill_signal_;
    MakeUnsignedCANSignal(IBMS::BMSFault, 7, 1, 1, 0) open_wire_signal_;
    CANTXMessage<8> vb_fault_message_{vb_can_bus_,
                                      592,
                                      1,
                                      kFaultMessageTransmitPeriod,
                                      timer_group_,
                                      fault_summary_signal_,
                                      undervoltage_signal_,
                                      overvoltage_signal_,
                                      undertemperature_signal_,
                                      overtemperature_signal_,
                                      overcurrent_signal_,
                                      external_kill_signal_,
                                      open_wire_signal_};
    CANTXMessage<7> lp_fault_message_{lp_can_bus_,
                                      592,
                                      1,
                                      kFaultMessageTransmitPeriod,
                                      timer_group_,
                                      fault_summary_signal_,
                                      undervoltage_signal_,
                                      overvoltage_signal_,
                                      undertemperature_signal_,
                                      overtemperature_signal_,
                                      overcurrent_signal_,
                                      external_kill_signal_};

    static const uint32_t kStatusMessageTransmitPeriod{100};
    MakeUnsignedCANSignal(IBMS::BMSState, 0, 8, 1, 0) state_signal_;
    MakeUnsignedCANSignal(float, 8, 8, 1, -40) max_cell_temperature_signal_;
    MakeUnsignedCANSignal(float, 16, 8, 1, -40) min_cell_temperature_signal_;
    MakeUnsignedCANSignal(float, 24, 8, 0.012, 2) max_cell_voltage_signal_;
    MakeUnsignedCANSignal(float, 32, 8, 0.012, 2) min_cell_voltage_signal_;
    MakeUnsignedCANSignal(float, 40, 8, 0.5, 0) soc_signal_;
    CANTXMessage<6> hp_status_message_{hp_can_bus_,
                                       577,
                                       6,
                                       kStatusMessageTransmitPeriod,
                                       timer_group_,
                                       state_signal_,
                                       max_cell_temperature_signal_,
                                       min_cell_temperature_signal_,
                                       max_cell_voltage_signal_,
                                       min_cell_voltage_signal_,
                                       soc_signal_};
    CANTXMessage<6> vb_status_message_{vb_can_bus_,
                                       577,
                                       6,
                                       kStatusMessageTransmitPeriod,
                                       timer_group_,
                                       state_signal_,
                                       max_cell_temperature_signal_,
                                       min_cell_temperature_signal_,
                                       max_cell_voltage_signal_,
                                       min_cell_voltage_signal_,
                                       soc_signal_};

    void AttachVoltageMessages(ICAN &bus,
                               std::array<CANTXMessage<7> *, kNumVoltageMessages> &message_ptr_array,
                               uint32_t transmit_period);

    void AttachTemperatureMessages(ICAN &bus,
                                   std::array<CANTXMessage<7> *, kNumTemperatureMessages> &message_ptr_array,
                                   uint32_t transmit_period);

    void UpdateSOESignals();
    void UpdateFaultSignals();
    void UpdateStatusSignals();

    void InitializeVoltageAndTemperatureMessages();

    void TickHPCAN();
    void TickVBCAN();
    void TickLPCAN();
};
