/*
 * LTC6811.cpp
 *
 *  Created on: 12 Mar 2020
 *      Author: Joshua
 */

#include "LTC6811.h"
#include "dwt_delay.h"

LTC6811::LTC6811(SPI_HandleTypeDef& hspi, Mode mode, DCP dcp, CellCh cell, AuxCh aux, STSCh sts)
: hspi{ hspi } {
    uint8_t md_bits = (mode & 0x02) >> 1;
    uint16_t PEC{ 0 };

    ADCV[0]   = 0x02 | md_bits;
    ADAX[0]   = 0x04 | md_bits;
    ADSTAT[0] = 0x04 | md_bits;

    md_bits   = (mode & 0x01) << 7;
    ADCV[1]   = md_bits | 0x60 | dcp << 4 | cell;
    ADAX[1]   = md_bits | 0x60 | aux;
    ADSTAT[1] = md_bits | 0x68 | sts;

    PEC = PEC15Calc(ADCV, 2);
    ADCV[2] = static_cast<uint8_t>(PEC >> 8);
    ADCV[3] = static_cast<uint8_t>(PEC);

    PEC = PEC15Calc(ADAX, 2);
    ADAX[2] = static_cast<uint8_t>(PEC >> 8);
    ADAX[3] = static_cast<uint8_t>(PEC);

    PEC = PEC15Calc(ADSTAT, 2);
    ADSTAT[2] = static_cast<uint8_t>(PEC >> 8);
    ADSTAT[3] = static_cast<uint8_t>(PEC);

    slave_cfg_tx.register_group.fill({ 0xFE, 0, 0, 0, 0, 0 });

    DWT_Init();
    WakeFromSleep(); // TODO Takes 2.2s to fall asleep so if this has to be called after this, we have problems
}

void LTC6811::WakeFromSleep(void) {
    for (size_t i = 0; i < kDaisyChainLength; ++i) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        DWT_Delay(T_WAKE_MAX); // Guarantees the LTC6811 will be in standby
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        DWT_Delay(10);
    }
}

void LTC6811::WakeFromIdle(void) {
    uint8_t const data = 0xFF;

    for (size_t i = 0; i < kDaisyChainLength; ++i) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi, &data, 1, HAL_MAX_DELAY); //Guarantees the isoSPI will be in ready mode
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }

}

/* Read a cell voltage register group of an LTC6811 daisy chain.
 * Returns 0 on success, 1 if either PEC or SPI error.
 */
bool LTC6811::ReadVoltageRegisterGroup(Group const group) {
    return ReadRegisterGroup(cell_data[group]);
}

/* Read an auxiliary register group of an LTC6811 daisy chain.
 * Returns 0 on success, 1 if either PEC or SPI error.
 */
bool LTC6811::ReadAuxRegisterGroup(Group const group) {
    return ReadRegisterGroup(cell_data[group]);
}

/* Read a status register group of an LTC6811 daisy chain. */
bool LTC6811::ReadStatusRegisterGroup(Group const group) {
    return ReadRegisterGroup(status_registers[group]);
}

/* Read the configuration register group of an LTC6811 daisy chain */
bool LTC6811::ReadConfigRegisterGroup(void) {
    return ReadRegisterGroup(slave_cfg_rx);
}

/* Write to the configuration register group of an LTC6811 daisy chain. */
bool LTC6811::WriteConfigRegisterGroup(void) {
    return WriteRegisterGroup(slave_cfg_tx);
}

/* Clear the LTC6811 cell voltage registers. */
void LTC6811::ClearVoltageRegisters(void) {
    constexpr static LTC6811Command command{ 7, 17, 201, 192 };

    WakeFromIdle();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi, command.data(), kCommandLength, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/* Clear the LTC6811 Auxiliary registers. */
void LTC6811::ClearAuxRegisters(void) {
    constexpr static LTC6811Command command{ 7, 18, 223, 164 };

    WakeFromIdle();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi, command.data(), kCommandLength, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/* Generate a status report of the cell voltage register groups.
 * Returns an LTC6811VoltageStatus on success, nullopt if error
 */
std::optional<LTC6811VoltageStatus> LTC6811::GetVoltageStatus(void) {
    LTC6811VoltageStatus status{};
    size_t count{ 0 };

    StartConversion(ADCV);

    for (size_t group = A; group <= D; ++group)
        if (!ReadVoltageRegisterGroup(static_cast<Group>(group)))
            return std::nullopt;

    for (const auto& register_group : cell_data) {
        for (const auto& Register : register_group.register_group) {
            for (const auto voltage : Register.data) {
                status.sum += voltage;

                if (voltage < status.min) {
                    status.min = voltage;
                    status.min_id = count;
                } else if (voltage > status.max) {
                    status.max = voltage;
                    status.max_id = count;
                }
                ++count;
            }
        }
    }
    return status;
}

/* Generate a status report of the current temperatures from aux voltage register groups.
 * Returns an LTC6811TempStatus on success, nullopt if error
 */
std::optional<LTC6811TempStatus> LTC6811::GetTemperatureStatus() {
    LTC6811TempStatus status{};
    size_t count{ 0 };
    auto steinharthart = [](int16_t const NTC_voltage) noexcept {
        constexpr auto Vin = 30000.0f; // 3[V], or 30000[V * 10-5]
        constexpr auto KtoC = 27315; // centiKelvin to centiDegCelsius
        constexpr auto A = 0.003354016f;
        constexpr auto B = 0.000256524f;
        constexpr auto C = 0.00000260597f;
        constexpr auto D = 0.0000000632926f;
        auto log = -logf(Vin / NTC_voltage - 1);

        return static_cast<int16_t>(100.0f / (A + log * ( B + log * (C + D * log))) - KtoC);
    };

    StartConversion(ADAX);

    for (size_t group = A; group <= D; ++group)
        if (!ReadAuxRegisterGroup(static_cast<Group>(group)))
            return std::nullopt;

    for (const auto& register_group : cell_data) {
        for (const auto& Register : register_group.register_group) {
            for (auto temperature : Register.data) {
                temperature = steinharthart(temperature);

                if (temperature < status.min) {
                    status.min = temperature;
                    status.min_id = count;
                } else if (temperature > status.max) {
                    status.max = temperature;
                    status.max_id = count;
                }

                ++count;
            }
        }
    }
    return status;
}

void LTC6811::BuildDischargeConfig(const LTC6811VoltageStatus& voltage_status) {
    uint16_t DCCx{ 0 };
    uint8_t current_cell{ 0 }, current_ic{ kDaisyChainLength - 1 };

    switch (discharge_mode) {
    case GTMinPlusDelta:
        for (auto& cfg_register : slave_cfg_tx.register_group) {
            DCCx = 0;
            current_cell = 0;

            for (const auto& register_group : cell_data) { // 4 voltage register groups
                for (const auto voltage : register_group.register_group[current_ic--].data) { // 3 voltages per IC
                    if (voltage > voltage_status.min + kDelta)
                        DCCx |= 1 << current_cell;
                    ++current_cell;
                } // 4 * 3 = 12 voltages associated with each LTC6811 in the daisy chain
            }

            cfg_register.data[4] |= DCCx & 0xFF;
            cfg_register.data[5] |= DCCx >> 8 & 0xF;
            cfg_register.PEC = PEC15Calc(cfg_register.data);
        }
        break;

    case MaxOnly:
        if (voltage_status.max - voltage_status.min > kDelta) {
            current_ic = voltage_status.max_id / 3 % 12;
            DCCx |= 1 << voltage_status.max_id % 11;
            slave_cfg_tx.register_group[current_ic].data[4] = DCCx & 0xFF;
            slave_cfg_tx.register_group[current_ic].data[5] = DCCx >> 8 & 0xF;
            slave_cfg_tx.register_group[current_ic].PEC = PEC15Calc(slave_cfg_tx.register_group[current_ic].data);
        }
        break;

    case GTMeanPlusDelta: {
        size_t average_voltage{ voltage_status.sum / (12 * kDaisyChainLength) };

        for (auto& cfg_register : slave_cfg_tx.register_group) {
            DCCx = 0;
            current_cell = 0;

            for (const auto& register_group : cell_data) { // 4 voltage register groups
                for (const auto voltage : register_group.register_group[current_ic--].data) { // 3 voltages per IC
                    if (voltage > average_voltage + kDelta)
                        DCCx |= 1 << current_cell;
                    ++current_cell;
                } // 4 * 3 = 12 voltages associated with each LTC6811 in the daisy chain
            }

            cfg_register.data[4] |= DCCx & 0xFF;
            cfg_register.data[5] |= DCCx >> 8 & 0xF;
            cfg_register.PEC = PEC15Calc(cfg_register.data);
        }
    }
    break;
    }

    WriteConfigRegisterGroup();
    DWT_Delay(500); // TODO take this out. Just read when we need the data to send over CAN or whatever
    ReadConfigRegisterGroup();
}


/* Start a conversion */
void LTC6811::StartConversion(const LTC6811Command& command) {
    WakeFromIdle(); // It's possible all of these can be removed

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi, command.data(), kCommandLength, HAL_MAX_DELAY);        // Start cell voltage conversion.
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

    DWT_Delay(T_REFUP_MAX + T_CYCLE_FAST_MAX); // TODO we aren't in fast conversion mode??? Also these delays aren't in the Linduino library
}
