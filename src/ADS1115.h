#ifndef _ADS1115_H_
#define _ADS1115_H_

#include <inttypes.h>

// -----------------------------------------------------------------------------
// Arduino-style "Serial.print" debug constant (uncomment to enable)
// -----------------------------------------------------------------------------
//#define ADS1115_SERIAL_DEBUG

#define ADS1115_ADDRESS_ADDR_GND    0x48 // address pin low (GND)
#define ADS1115_ADDRESS_ADDR_VDD    0x49 // address pin high (VCC)
#define ADS1115_ADDRESS_ADDR_SDA    0x4A // address pin tied to SDA pin
#define ADS1115_ADDRESS_ADDR_SCL    0x4B // address pin tied to SCL pin
#define ADS1115_DEFAULT_ADDRESS     ADS1115_ADDRESS_ADDR_GND

#define ADS1115_RA_CONVERSION       0x00
#define ADS1115_RA_CONFIG           0x01
#define ADS1115_RA_LO_THRESH        0x02
#define ADS1115_RA_HI_THRESH        0x03

#define ADS1115_CFG_OS_BIT          _BV(15)
#define ADS1115_CFG_MUX_MASK        (_BV(14) | _BV(13) | _BV(12))
#define ADS1115_CFG_MUX_SHIFT       12
#define ADS1115_CFG_PGA_MASK        (_BV(11) | _BV(10) | _BV(9))
#define ADS1115_CFG_PGA_SHIFT       9
#define ADS1115_CFG_MODE_BIT        _BV(8)
#define ADS1115_CFG_DR_MASK         (_BV(7) | _BV(6) | _BV(5))
#define ADS1115_CFG_DR_SHIFT        5
#define ADS1115_CFG_COMP_MODE_BIT   _BV(4)
#define ADS1115_CFG_COMP_POL_BIT    _BV(3)
#define ADS1115_CFG_COMP_LAT_BIT    _BV(2)
#define ADS1115_CFG_COMP_QUE_MASK   (_BV(1) | _BV(0))
#define ADS1115_CFG_COMP_QUE_SHIFT  0


#define ADS1115_MUX_P0_N1           0x00 // default
#define ADS1115_MUX_P0_N3           0x01
#define ADS1115_MUX_P1_N3           0x02
#define ADS1115_MUX_P2_N3           0x03
#define ADS1115_MUX_P0_NG           0x04
#define ADS1115_MUX_P1_NG           0x05
#define ADS1115_MUX_P2_NG           0x06
#define ADS1115_MUX_P3_NG           0x07

#define ADS1115_PGA_6P144           0x00
#define ADS1115_PGA_4P096           0x01
#define ADS1115_PGA_2P048           0x02 // default
#define ADS1115_PGA_1P024           0x03
#define ADS1115_PGA_0P512           0x04
#define ADS1115_PGA_0P256           0x05
#define ADS1115_PGA_0P256B          0x06
#define ADS1115_PGA_0P256C          0x07

#define ADS1115_MV_6P144            0.187500
#define ADS1115_MV_4P096            0.125000
#define ADS1115_MV_2P048            0.062500 // default
#define ADS1115_MV_1P024            0.031250
#define ADS1115_MV_0P512            0.015625
#define ADS1115_MV_0P256            0.007813
#define ADS1115_MV_0P256B           0.007813
#define ADS1115_MV_0P256C           0.007813

#define ADS1115_FSR_6P144            6144
#define ADS1115_FSR_4P096            4096
#define ADS1115_FSR_2P048            2048
#define ADS1115_FSR_1P024            1024
#define ADS1115_FSR_0P512            512
#define ADS1115_FSR_0P256            256

#define ADS1115_MODE_CONTINUOUS     0x00
#define ADS1115_MODE_SINGLESHOT     0x01 // default

#define ADS1115_RATE_8              0x00
#define ADS1115_RATE_16             0x01
#define ADS1115_RATE_32             0x02
#define ADS1115_RATE_64             0x03
#define ADS1115_RATE_128            0x04 // default
#define ADS1115_RATE_250            0x05
#define ADS1115_RATE_475            0x06
#define ADS1115_RATE_860            0x07

#define ADS1115_COMP_MODE_HYSTERESIS    0x00 // default
#define ADS1115_COMP_MODE_WINDOW        0x01

#define ADS1115_COMP_POL_ACTIVE_LOW     0x00 // default
#define ADS1115_COMP_POL_ACTIVE_HIGH    0x01

#define ADS1115_COMP_LAT_NON_LATCHING   0x00 // default
#define ADS1115_COMP_LAT_LATCHING       0x01

#define ADS1115_COMP_QUE_ASSERT1    0x00
#define ADS1115_COMP_QUE_ASSERT2    0x01
#define ADS1115_COMP_QUE_ASSERT4    0x02
#define ADS1115_COMP_QUE_DISABLE    0x03 // default

// -----------------------------------------------------------------------------
// Arduino-style "Serial.print" debug constant (uncomment to enable)
// -----------------------------------------------------------------------------
//#define ADS1115_SERIAL_DEBUG


class ADS1115 {
    public:
        ADS1115(uint8_t address = ADS1115_DEFAULT_ADDRESS);

        void initialize();
        bool testConnection();

        // SINGLE SHOT utilities
        bool pollConversion(uint16_t max_retries);
        void triggerConversion();

        // Read the current CONVERSION register
        int16_t getConversion(bool triggerAndPoll=true);

        // Differential
        int16_t getConversionP0N1();
        int16_t getConversionP0N3();
        int16_t getConversionP1N3();
        int16_t getConversionP2N3();

        // Single-ended
        int16_t getConversionP0GND();
        int16_t getConversionP1GND();
        int16_t getConversionP2GND();
        int16_t getConversionP3GND();

        // Utility
        float getMilliVolts(bool triggerAndPoll=true);
        float getMvPerCount();
        uint16_t getFullScale(uint8_t pga);

        // CONFIG register
        bool isConversionReady();
        uint8_t getMultiplexer();
        void setMultiplexer(uint8_t mux);
        uint8_t getGain();
        void setGain(uint8_t gain);
        uint8_t getMode();
        void setMode(uint8_t mode);
        uint8_t getRate();
        void setRate(uint8_t rate);
        uint8_t getComparatorMode();
        void setComparatorMode(uint8_t mode);
        uint8_t getComparatorPolarity();
        void setComparatorPolarity(uint8_t polarity);
        uint8_t getComparatorLatchEnabled();
        void setComparatorLatchEnabled(uint8_t enabled);
        uint8_t getComparatorQueueMode();
        void setComparatorQueueMode(uint8_t mode);
        void setConversionReadyPinMode();

        // *_THRESH registers
        int16_t getLowThreshold();
        void setLowThreshold(int16_t threshold);
        int16_t getHighThreshold();
        void setHighThreshold(int16_t threshold);

        // DEBUG
        void showConfigRegister();

    protected:
        uint16_t readRegister(uint8_t regaddr);
        void writeRegister(uint8_t regAddr, uint16_t value);

    private:
        uint8_t  devAddr;
        uint8_t  devMode;
        uint8_t  muxMode;
        uint8_t  pgaMode;
        uint16_t configValue;
};

#endif /* _ADS1115_H_ */

// vim:ts=4:sw=4:ai:et:si:sts=4
