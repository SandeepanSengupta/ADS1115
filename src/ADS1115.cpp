#include "Arduino.h"

#ifndef _BV
#define _BV(x)  (1<<(x))
#endif

#include <Wire.h>
#include "ADS1115.h"


/** Specific address constructor.
 * @param address I2C address
 * @see ADS1115_DEFAULT_ADDRESS
 * @see ADS1115_ADDRESS_ADDR_GND
 * @see ADS1115_ADDRESS_ADDR_VDD
 * @see ADS1115_ADDRESS_ADDR_SDA
 * @see ADS1115_ADDRESS_ADDR_SDL
 */
ADS1115::ADS1115(uint8_t address) {
    Wire.begin();
    devAddr = address;
}

/** Power on and prepare for general usage.
 * This device is ready to use automatically upon power-up. It defaults to
 * single-shot read mode, P0/N1 mux, 2.048v gain, 128 samples/sec, default
 * comparator with hysterysis, active-low polarity, non-latching comparator,
 * and comparater-disabled operation.
 */
void ADS1115::initialize()
{
    configValue = 0;
    setMultiplexer(ADS1115_MUX_P0_N1);
    setGain(ADS1115_PGA_2P048);
    setMode(ADS1115_MODE_SINGLESHOT);
    setRate(ADS1115_RATE_128);
    setComparatorMode(ADS1115_COMP_MODE_HYSTERESIS);
    setComparatorPolarity(ADS1115_COMP_POL_ACTIVE_LOW);
    setComparatorLatchEnabled(ADS1115_COMP_LAT_NON_LATCHING);
    setComparatorQueueMode(ADS1115_COMP_QUE_DISABLE);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool ADS1115::testConnection()
{
    Wire.beginTransmission(devAddr);
    int status = Wire.endTransmission();
    return status == 0;
}

/** Poll the operational status bit until the conversion is finished
 * Retry at most 'max_retries' times
 * conversion is finished, then return true;
 * @see ADS1115_CFG_OS_BIT
 * @return True if data is available, false otherwise
 */
bool ADS1115::pollConversion(uint16_t max_retries)
{
    for (uint16_t i = 0; i < max_retries; i++) {
        if (isConversionReady()) {
            return true;
        }
    }
    return false;
}

uint16_t ADS1115::readRegister(uint8_t regAddr)
{
    Wire.beginTransmission(devAddr);
    Wire.write(regAddr);
    Wire.endTransmission();

    delay(1);

    Wire.requestFrom(devAddr, (uint8_t)2);
    return ((Wire.read() << 8) | Wire.read());
}

void ADS1115::writeRegister(uint8_t regAddr, uint16_t value)
{
    Wire.beginTransmission(devAddr);
    Wire.write(regAddr);
    Wire.write((value & 0xFF00) >> 8);
    Wire.write(value & 0x00FF);
    Wire.endTransmission();
}

/** Read differential value based on current MUX configuration.
 * The default MUX setting sets the device to get the differential between the
 * AIN0 and AIN1 pins. There are 8 possible MUX settings, but if you are using
 * all four input pins as single-end voltage sensors, then the default option is
 * not what you want; instead you will need to set the MUX to compare the
 * desired AIN* pin with GND. There are shortcut methods (getConversion*) to do
 * this conveniently, but you can also do it manually with setMultiplexer()
 * followed by this method.
 *
 * In single-shot mode, this register may not have fresh data. You need to write
 * a 1 bit to the MSB of the CONFIG register to trigger a single read/conversion
 * before this will be populated with fresh data. This technique is not as
 * effortless, but it has enormous potential to save power by only running the
 * comparison circuitry when needed.
 *
 * @param triggerAndPoll If true (and only in singleshot mode) the conversion trigger
 *        will be executed and the conversion results will be polled.
 * @return 16-bit signed differential value
 * @see getConversionP0N1();
 * @see getConversionPON3();
 * @see getConversionP1N3();
 * @see getConversionP2N3();
 * @see getConversionP0GND();
 * @see getConversionP1GND();
 * @see getConversionP2GND();
 * @see getConversionP3GND);
 * @see setMultiplexer();
 * @see ADS1115_RA_CONVERSION
 * @see ADS1115_MUX_P0_N1
 * @see ADS1115_MUX_P0_N3
 * @see ADS1115_MUX_P1_N3
 * @see ADS1115_MUX_P2_N3
 * @see ADS1115_MUX_P0_NG
 * @see ADS1115_MUX_P1_NG
 * @see ADS1115_MUX_P2_NG
 * @see ADS1115_MUX_P3_NG
 */
int16_t ADS1115::getConversion(bool triggerAndPoll)
{
    if (triggerAndPoll && devMode == ADS1115_MODE_SINGLESHOT) {
        triggerConversion();
        pollConversion(1000);
    }

    return (int16_t)(readRegister(ADS1115_RA_CONVERSION));
}

/** Get AIN0/N1 differential.
 * This changes the MUX setting to AIN0/N1 if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP0N1()
{
    if (muxMode != ADS1115_MUX_P0_N1) {
        setMultiplexer(ADS1115_MUX_P0_N1);
    }
    return getConversion();
}

/** Get AIN0/N3 differential.
 * This changes the MUX setting to AIN0/N3 if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP0N3()
{
    if (muxMode != ADS1115_MUX_P0_N3) {
        setMultiplexer(ADS1115_MUX_P0_N3);
    }
    return getConversion();
}

/** Get AIN1/N3 differential.
 * This changes the MUX setting to AIN1/N3 if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP1N3()
{
    if (muxMode != ADS1115_MUX_P1_N3) {
        setMultiplexer(ADS1115_MUX_P1_N3);
    }
    return getConversion();
}

/** Get AIN2/N3 differential.
 * This changes the MUX setting to AIN2/N3 if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP2N3()
{
    if (muxMode != ADS1115_MUX_P2_N3) {
        setMultiplexer(ADS1115_MUX_P2_N3);
    }
    return getConversion();
}

/** Get AIN0/GND differential.
 * This changes the MUX setting to AIN0/GND if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP0GND()
{
    if (muxMode != ADS1115_MUX_P0_NG) {
        setMultiplexer(ADS1115_MUX_P0_NG);
    }
    return getConversion();
}

/** Get AIN1/GND differential.
 * This changes the MUX setting to AIN1/GND if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP1GND()
{
    if (muxMode != ADS1115_MUX_P1_NG) {
        setMultiplexer(ADS1115_MUX_P1_NG);
    }
    return getConversion();
}

/** Get AIN2/GND differential.
 * This changes the MUX setting to AIN2/GND if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP2GND()
{
    if (muxMode != ADS1115_MUX_P2_NG) {
        setMultiplexer(ADS1115_MUX_P2_NG);
    }
    return getConversion();
}

/** Get AIN3/GND differential.
 * This changes the MUX setting to AIN3/GND if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP3GND()
{
    if (muxMode != ADS1115_MUX_P3_NG) {
        setMultiplexer(ADS1115_MUX_P3_NG);
    }
    return getConversion();
}

/** Get the current voltage reading
 * Read the current differential and return it multiplied
 * by the constant for the current gain.  mV is returned to
 * increase the precision of the voltage
 * @param triggerAndPoll If true (and only in singleshot mode) the conversion trigger
 *        will be executed and the conversion results will be polled.
 */
float ADS1115::getMilliVolts(bool triggerAndPoll) {
    int16_t reading = getConversion(triggerAndPoll);
    float factor = getMvPerCount();
    return (float)reading * factor;
}

/**
 * Return the current multiplier for the PGA setting.
 *
 * This may be directly retreived by using getMilliVolts(),
 * but this causes an independent read.  This function could
 * be used to average a number of reads from the getConversion()
 * getConversionx() functions and cut downon the number of
 * floating-point calculations needed.
 *
 */

float ADS1115::getMvPerCount() {
    switch (pgaMode) {
        case ADS1115_PGA_6P144:
            return ADS1115_MV_6P144;
        case ADS1115_PGA_4P096:
            return  ADS1115_MV_4P096;
        case ADS1115_PGA_2P048:
            return ADS1115_MV_2P048;
        case ADS1115_PGA_1P024:
            return ADS1115_MV_1P024;
        case ADS1115_PGA_0P512:
            return ADS1115_MV_0P512;
        case ADS1115_PGA_0P256:
        case ADS1115_PGA_0P256B:
        case ADS1115_PGA_0P256C:
            return ADS1115_MV_0P256;
        default:
            return 0.0;
    }
}

uint16_t ADS1115::getFullScale(uint8_t pga)
{
    switch (pga) {
        case ADS1115_PGA_6P144:
            return ADS1115_FSR_6P144;
        case ADS1115_PGA_4P096:
            return ADS1115_FSR_4P096;
        case ADS1115_PGA_2P048:
            return ADS1115_FSR_2P048;
        case ADS1115_PGA_1P024:
            return ADS1115_FSR_1P024;
        case ADS1115_PGA_0P512:
            return ADS1115_FSR_0P512;
        case ADS1115_PGA_0P256:
        case ADS1115_PGA_0P256B:
        case ADS1115_PGA_0P256C:
            return ADS1115_FSR_0P256;
        default:
            return 0;
    }
}

// CONFIG register

/** Get operational status.
 * @return Current operational status (false for active conversion, true for inactive)
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_OS_BIT
 */
bool ADS1115::isConversionReady()
{
    uint16_t value = readRegister(ADS1115_RA_CONFIG);
    return !(!(value & ADS1115_CFG_OS_BIT));
}

/** Trigger a new conversion.
 * Writing to this bit will only have effect while in power-down mode (no conversions active).
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_OS_BIT
 */
void ADS1115::triggerConversion()
{
    configValue |= ADS1115_CFG_OS_BIT;
    writeRegister(ADS1115_RA_CONFIG, configValue);
}

/** Get multiplexer connection.
 * @return Current multiplexer connection setting
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_MUX_MASK
 * @see ADS1115_CFG_MUX_SHIFT
 */
uint8_t ADS1115::getMultiplexer()
{
    uint16_t value = readRegister(ADS1115_RA_CONFIG);
    muxMode = (uint8_t)((value & ADS1115_CFG_MUX_MASK) >>
                         ADS1115_CFG_MUX_SHIFT);
    return muxMode;
}

/** Set multiplexer connection.  Continous mode may fill the conversion register
 * with data before the MUX setting has taken effect.  A stop/start of the conversion
 * is done to reset the values.
 * @param mux New multiplexer connection setting
 * @see ADS1115_MUX_P0_N1
 * @see ADS1115_MUX_P0_N3
 * @see ADS1115_MUX_P1_N3
 * @see ADS1115_MUX_P2_N3
 * @see ADS1115_MUX_P0_NG
 * @see ADS1115_MUX_P1_NG
 * @see ADS1115_MUX_P2_NG
 * @see ADS1115_MUX_P3_NG
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_MUX_MASK
 * @see ADS1115_CFG_MUX_SHIFT
 */
void ADS1115::setMultiplexer(uint8_t mux)
{
    configValue &= ~ADS1115_CFG_MUX_MASK;
    configValue |= (mux << ADS1115_CFG_MUX_SHIFT) & ADS1115_CFG_MUX_MASK;
    writeRegister(ADS1115_RA_CONFIG, configValue);
    muxMode = mux;
    if (devMode == ADS1115_MODE_CONTINUOUS) {
        // Force a stop/start
        setMode(ADS1115_MODE_SINGLESHOT);
        getConversion();
        setMode(ADS1115_MODE_CONTINUOUS);
    }
}

/** Get programmable gain amplifier level.
 * @return Current programmable gain amplifier level
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_PGA_MASK
 * @see ADS1115_CFG_PGA_SHIFT
 */
uint8_t ADS1115::getGain()
{
    uint16_t value = readRegister(ADS1115_RA_CONFIG);
    pgaMode = (uint8_t)((value & ADS1115_CFG_PGA_MASK) >>
                         ADS1115_CFG_PGA_SHIFT);
    return pgaMode;
}

/** Set programmable gain amplifier level.
 * Continous mode may fill the conversion register
 * with data before the gain setting has taken effect.  A stop/start of the conversion
 * is done to reset the values.
 * @param gain New programmable gain amplifier level
 * @see ADS1115_PGA_6P144
 * @see ADS1115_PGA_4P096
 * @see ADS1115_PGA_2P048
 * @see ADS1115_PGA_1P024
 * @see ADS1115_PGA_0P512
 * @see ADS1115_PGA_0P256
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_PGA_MASK
 * @see ADS1115_CFG_PGA_SHIFT
 */
void ADS1115::setGain(uint8_t gain)
{
    configValue &= ~ADS1115_CFG_PGA_MASK;
    configValue |= (gain << ADS1115_CFG_PGA_SHIFT) & ADS1115_CFG_PGA_MASK;
    writeRegister(ADS1115_RA_CONFIG, configValue);
    pgaMode = gain;
    if (devMode == ADS1115_MODE_CONTINUOUS) {
        // Force a stop/start
        setMode(ADS1115_MODE_SINGLESHOT);
        getConversion();
        setMode(ADS1115_MODE_CONTINUOUS);
    }
}

/** Get device mode.
 * @return Current device mode
 * @see ADS1115_MODE_CONTINUOUS
 * @see ADS1115_MODE_SINGLESHOT
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_MODE_BIT
 */
uint8_t ADS1115::getMode()
{
    uint16_t value = readRegister(ADS1115_RA_CONFIG);
    devMode = (uint8_t)!(!(value & ADS1115_CFG_MODE_BIT));
    return devMode;
}

/** Set device mode.
 * @param mode New device mode
 * @see ADS1115_MODE_CONTINUOUS
 * @see ADS1115_MODE_SINGLESHOT
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_MODE_BIT
 */
void ADS1115::setMode(uint8_t mode)
{
    configValue &= ~ADS1115_CFG_MODE_BIT;
    if (mode) {
        configValue |= ADS1115_CFG_MODE_BIT;
    }
    writeRegister(ADS1115_RA_CONFIG, configValue);
    devMode = mode;
}

/** Get data rate.
 * @return Current data rate
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_DR_MASK
 * @see ADS1115_CFG_DR_SHIFT
 */
uint8_t ADS1115::getRate()
{
    uint16_t value = readRegister(ADS1115_RA_CONFIG);
    return (uint8_t)((value & ADS1115_CFG_DR_MASK) >> ADS1115_CFG_DR_SHIFT);
}

/** Set data rate.
 * @param rate New data rate
 * @see ADS1115_RATE_8
 * @see ADS1115_RATE_16
 * @see ADS1115_RATE_32
 * @see ADS1115_RATE_64
 * @see ADS1115_RATE_128
 * @see ADS1115_RATE_250
 * @see ADS1115_RATE_475
 * @see ADS1115_RATE_860
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_DR_MASK
 * @see ADS1115_CFG_DR_SHIFT
 */
void ADS1115::setRate(uint8_t rate)
{
    configValue &= ~ADS1115_CFG_DR_MASK;
    configValue |= (rate << ADS1115_CFG_DR_SHIFT) & ADS1115_CFG_DR_MASK;
    writeRegister(ADS1115_RA_CONFIG, configValue);
}

/** Get comparator mode.
 * @return Current comparator mode
 * @see ADS1115_COMP_MODE_HYSTERESIS
 * @see ADS1115_COMP_MODE_WINDOW
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_MODE_BIT
 */
uint8_t ADS1115::getComparatorMode()
{
    uint16_t value = readRegister(ADS1115_RA_CONFIG);
    return (uint8_t)!(!(value & ADS1115_CFG_COMP_MODE_BIT));
}

/** Set comparator mode.
 * @param mode New comparator mode
 * @see ADS1115_COMP_MODE_HYSTERESIS
 * @see ADS1115_COMP_MODE_WINDOW
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_MODE_BIT
 */
void ADS1115::setComparatorMode(uint8_t mode)
{
    configValue &= ~ADS1115_CFG_COMP_MODE_BIT;
    if (mode) {
        configValue |= ADS1115_CFG_COMP_MODE_BIT;
    }
}

/** Get comparator polarity setting.
 * @return Current comparator polarity setting
 * @see ADS1115_COMP_POL_ACTIVE_LOW
 * @see ADS1115_COMP_POL_ACTIVE_HIGH
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_POL_BIT
 */
uint8_t ADS1115::getComparatorPolarity()
{
    uint16_t value = readRegister(ADS1115_RA_CONFIG);
    return (uint8_t)!(!(value & ADS1115_CFG_COMP_POL_BIT));
}

/** Set comparator polarity setting.
 * @param polarity New comparator polarity setting
 * @see ADS1115_COMP_POL_ACTIVE_LOW
 * @see ADS1115_COMP_POL_ACTIVE_HIGH
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_POL_BIT
 */
void ADS1115::setComparatorPolarity(uint8_t polarity)
{
    configValue &= ~ADS1115_CFG_COMP_POL_BIT;
    if (polarity) {
        configValue |= ADS1115_CFG_COMP_POL_BIT;
    }
}

/** Get comparator latch enabled value.
 * @return Current comparator latch enabled value
 * @see ADS1115_COMP_LAT_NON_LATCHING
 * @see ADS1115_COMP_LAT_LATCHING
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_LAT_BIT
 */
uint8_t ADS1115::getComparatorLatchEnabled()
{
    uint16_t value = readRegister(ADS1115_RA_CONFIG);
    return (uint8_t)!(!(value & ADS1115_CFG_COMP_LAT_BIT));
}

/** Set comparator latch enabled value.
 * @param enabled New comparator latch enabled value
 * @see ADS1115_COMP_LAT_NON_LATCHING
 * @see ADS1115_COMP_LAT_LATCHING
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_LAT_BIT
 */
void ADS1115::setComparatorLatchEnabled(uint8_t enabled)
{
    configValue &= ~ADS1115_CFG_COMP_LAT_BIT;
    if (enabled) {
        configValue |= ADS1115_CFG_COMP_LAT_BIT;
    }
}

/** Get comparator queue mode.
 * @return Current comparator queue mode
 * @see ADS1115_COMP_QUE_ASSERT1
 * @see ADS1115_COMP_QUE_ASSERT2
 * @see ADS1115_COMP_QUE_ASSERT4
 * @see ADS1115_COMP_QUE_DISABLE
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_QUE_MASK
 * @see ADS1115_CFG_COMP_QUE_SHIFT
 */
uint8_t ADS1115::getComparatorQueueMode()
{
    uint16_t value = readRegister(ADS1115_RA_CONFIG);
    return (uint8_t)((value & ADS1115_CFG_COMP_QUE_MASK) >>
                      ADS1115_CFG_COMP_QUE_SHIFT);
}

/** Set comparator queue mode.
 * @param mode New comparator queue mode
 * @see ADS1115_COMP_QUE_ASSERT1
 * @see ADS1115_COMP_QUE_ASSERT2
 * @see ADS1115_COMP_QUE_ASSERT4
 * @see ADS1115_COMP_QUE_DISABLE
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_QUE_MASK
 * @see ADS1115_CFG_COMP_QUE_SHIFT
 */
void ADS1115::setComparatorQueueMode(uint8_t mode)
{
    configValue &= ~ADS1115_CFG_COMP_QUE_MASK;
    configValue |= (mode << ADS1115_CFG_COMP_QUE_SHIFT) &
                   ADS1115_CFG_COMP_QUE_MASK;
    writeRegister(ADS1115_RA_CONFIG, configValue);
}

// *_THRESH registers

/** Get low threshold value.
 * @return Current low threshold value
 * @see ADS1115_RA_LO_THRESH
 */
int16_t ADS1115::getLowThreshold()
{
    return (int16_t)readRegister(ADS1115_RA_LO_THRESH);
}

/** Set low threshold value.
 * @param threshold New low threshold value
 * @see ADS1115_RA_LO_THRESH
 */
void ADS1115::setLowThreshold(int16_t threshold)
{
    writeRegister(ADS1115_RA_LO_THRESH, (uint16_t)threshold);
}

/** Get high threshold value.
 * @return Current high threshold value
 * @see ADS1115_RA_HI_THRESH
 */
int16_t ADS1115::getHighThreshold()
{
    return (int16_t)readRegister(ADS1115_RA_HI_THRESH);
}

/** Set high threshold value.
 * @param threshold New high threshold value
 * @see ADS1115_RA_HI_THRESH
 */
void ADS1115::setHighThreshold(int16_t threshold)
{
    writeRegister(ADS1115_RA_HI_THRESH, (uint16_t)threshold);
}

/** Configures ALERT/RDY pin as a conversion ready pin.
 *  It does this by setting the MSB of the high threshold register to '1' and the MSB
 *  of the low threshold register to '0'. COMP_POL and COMP_QUE bits will be set to '0'.
 *  Note: ALERT/RDY pin requires a pull up resistor.
 */
void ADS1115::setConversionReadyPinMode()
{
    setHighThreshold(-1);
    setLowThreshold(0);
    setComparatorPolarity(0);
    setComparatorQueueMode(0);
}

/** Show all the config register settings
 */
void ADS1115::showConfigRegister()
{
    uint16_t value = readRegister(ADS1115_RA_CONFIG);

#ifdef ADS1115_SERIAL_DEBUG
    Serial.print("Register is:");
    Serial.println(configRegister, BIN);

    Serial.print("OS:\t");
    Serial.println(!(!(value & ADS1115_CFG_OS_BIT)));

    Serial.print("MUX:\t");
    Serial.println((value & ADS1115_CFG_MUX_MASK) >> ADS1115_CFG_MUX_SHIFT,
                   BIN);

    Serial.print("PGA:\t");
    Serial.println((value & ADS1115_CFG_PGA_MASK) >> ADS1115_CFG_PGA_SHIFT,
                   BIN);

    Serial.print("MODE:\t");
    Serial.println(!(!(value & ADS1115_CFG_MODE_BIT)));

    Serial.print("DR:\t");
    Serial.println((value & ADS1115_CFG_DR_MASK) >> ADS1115_CFG_DR_SHIFT,
                   BIN);

    Serial.print("CMP_MODE:\t");
    Serial.println(!(!(value & ADS1115_CFG_COMP_MODE_BIT)));

    Serial.print("CMP_POL:\t");
    Serial.println(!(!(value & ADS1115_CFG_COMP_POL_BIT)));

    Serial.print("CMP_LAT:\t");
    Serial.println(!(!(value & ADS1115_CFG_COMP_LAT_BIT)));
    Serial.println(getValueFromBits(configRegister,
        ADS1115_CFG_COMP_LAT_BIT,1), BIN);

    Serial.print("CMP_QUE:\t");
    Serial.println((value & ADS1115_CFG_COMP_QUE_MASK) >>
                   ADS1115_CFG_COMP_QUE_SHIFT, BIN);
#endif
}

// vim:ts=4:sw=4:ai:et:si:sts=4
