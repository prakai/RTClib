#include "RTClib.h"

#define PCF85363A_ADDRESS 0x51 ///< I2C address for PCF85363A

/* See https://www.nxp.com/docs/en/data-sheet/PCF85363A.pdf for a
 * description of the registers */

/*
 * Date/Time registers
 */
#define PCF85363A_DT_100THS 0x00
#define PCF85363A_DT_SECS 0x01
#define PCF85363A_DT_MINUTES 0x02
#define PCF85363A_DT_HOURS 0x03
#define PCF85363A_DT_DAYS 0x04
#define PCF85363A_DT_WEEKDAYS 0x05
#define PCF85363A_DT_MONTHS 0x06
#define PCF85363A_DT_YEARS 0x07
/*
 * Alarm registers
 */
#define PCF85363A_DT_SECOND_ALM1 0x08
#define PCF85363A_DT_MINUTE_ALM1 0x09
#define PCF85363A_DT_HOUR_ALM1 0x0A
#define PCF85363A_DT_DAY_ALM1 0x0B
#define PCF85363A_DT_MONTH_ALM1 0x0C
#define PCF85363A_DT_MINUTE_ALM2 0x0D
#define PCF85363A_DT_HOUR_ALM2 0x0E
#define PCF85363A_DT_WEEKDAY_ALM2 0x0F
#define PCF85363A_DT_ALARM_EN 0x10
/*
 * Time stamp registers
 */
#define PCF85363A_DT_TIMESTAMP1 0x11
#define PCF85363A_DT_TIMESTAMP2 0x17
#define PCF85363A_DT_TIMESTAMP3 0x1D
#define PCF85363A_DT_TS_MODE 0x23
/*
 * control registers
 */
#define PCF85363A_CTRL_OFFSET 0x24
#define PCF85363A_CTRL_OSCILLATOR 0x25
#define PCF85363A_CTRL_BATTERY 0x26
#define PCF85363A_CTRL_PIN_IO 0x27
#define PCF85363A_CTRL_FUNCTION 0x28
#define PCF85363A_CTRL_INTA_EN 0x29
#define PCF85363A_CTRL_INTB_EN 0x2A
#define PCF85363A_CTRL_FLAGS 0x2B
#define PCF85363A_CTRL_RAMBYTE 0x2C
#define PCF85363A_CTRL_WDOG 0x2D
#define PCF85363A_CTRL_STOP_EN 0x2E
#define PCF85363A_CTRL_RESETS 0x2F
#define PCF85363A_CTRL_RAM 0x40

#define PCF85363A_STOP_EN_STOP BIT(0)
#define PCF85363A_CLEAE_EN_STOP 0
#define PCF85363A_RESET_SR 0x2C
#define PCF85363A_RESET_CPR 0xA4
#define PCF85363A_RESET_CTS 0x25

#define PCF85363A_MARK_AS_INITIALIZED_ADDR 0x7F
#define PCF85363A_MARK_AS_INITIALIZED_VALE 0xAA
/**************************************************************************/
/*!
    @brief  Start I2C for the PCF85363A and test succesful connection
    @param  wireInstance pointer to the I2C bus
    @return True if Wire can find PCF85363A or false otherwise.
*/
/**************************************************************************/
bool RTC_PCF85363A::begin(TwoWire *wireInstance)
{
  if (i2c_dev)
    delete i2c_dev;
  i2c_dev = new Adafruit_I2CDevice(PCF85363A_ADDRESS, wireInstance);
  if (!i2c_dev->begin())
    return false;
  return true;
}

/**************************************************************************/
/*!
    @brief  Check the saved value of INITIALIZED in the last bye of RAM.
    @details The PCF85363A has no on-chip voltage-low detector. So, we save
     INITIALIZED value (0xAA) in to PCF85363A at the last byte of RAM. If PCF85363A
     lost power, the value that read from the last byte of RAM will be 0x00.
    @return True if value from the last byte of RAM is not 0xAA,indicating that
     the device is lost power.
*/
/**************************************************************************/
bool RTC_PCF85363A::lostPower(void)
{
  return (bool)(read_register(PCF85363A_MARK_AS_INITIALIZED_ADDR) != PCF85363A_MARK_AS_INITIALIZED_VALE);
}

/**************************************************************************/
/*!
    @brief  Set the date and time
    @param dt DateTime to set
*/
/**************************************************************************/
void RTC_PCF85363A::adjust(const DateTime &dt)
{
  // set STOP and then CPR
  this->stop();
  this->clearPrescaler();

  // Setting time
  uint8_t buffer[9] = {PCF85363A_DT_100THS, // 100th register address
                       0,                   // 100th seconds
                       bin2bcd(dt.second()), bin2bcd(dt.minute()),
                       bin2bcd(dt.hour()), bin2bcd(dt.day()),
                       bin2bcd(0), // skip weekdays
                       bin2bcd(dt.month()), bin2bcd(dt.year() - 2000U)};
  i2c_dev->write(buffer, 9);

  // clear STOP
  this->start();

  // Mark as initialized in RAM
  uint8_t init[2] = {PCF85363A_MARK_AS_INITIALIZED_ADDR, // Last byte of RAM address (7Fh)
                     PCF85363A_MARK_AS_INITIALIZED_VALE};
  i2c_dev->write(init, 2);
}

/**************************************************************************/
/*!
    @brief  Get the current date/time
    @return DateTime object containing the current date/time
*/
/**************************************************************************/
DateTime RTC_PCF85363A::now(void)
{
  uint8_t buffer[9];
  buffer[0] = PCF85363A_DT_100THS; // start at location 2, VL_SECONDS
  i2c_dev->write_then_read(buffer, 1, buffer, 8);

  return DateTime(bcd2bin(buffer[7]) + 2000U, bcd2bin(buffer[6] & ~0xE0),
                  bcd2bin(buffer[4] & ~0xC0), bcd2bin(buffer[3] & ~0xC0),
                  bcd2bin(buffer[2] & ~0x80), bcd2bin(buffer[1] & ~0x80));
}

/**************************************************************************/
/*!
    @brief  Resets the STOP bit (00h) in register Stop_enable (2Eh)
*/
/**************************************************************************/
void RTC_PCF85363A::start(void)
{
  write_register(PCF85363A_CTRL_STOP_EN, PCF85363A_CLEAE_EN_STOP);
}

/**************************************************************************/
/*!
    @brief  Sets the STOP bit (01h) in register Stop_enable (2Eh)
*/
/**************************************************************************/
void RTC_PCF85363A::stop(void)
{
  write_register(PCF85363A_CTRL_STOP_EN, PCF85363A_STOP_EN_STOP);
}

/**************************************************************************/
/*!
    @brief  Is the PCF85363A running? Check the STOP bit in register
            Stop_enable (2Eh)
    @return 1 if the RTC is running, 0 if not
*/
/**************************************************************************/
uint8_t RTC_PCF85363A::isRunning()
{
  return !((read_register(PCF85363A_CTRL_STOP_EN) & 0x01) == 0x01);
}

/**************************************************************************/
/*!
    @brief  Sent SR (software reset/2Ch) to Reset address (2Fh)
*/
/**************************************************************************/
void RTC_PCF85363A::softwareReset(void)
{
  write_register(PCF85363A_CTRL_RESETS, PCF85363A_RESET_SR);
}

/**************************************************************************/
/*!
    @brief  Sent CPR (clear prescaler/A4h) to Reset address (2Fh)
*/
/**************************************************************************/
void RTC_PCF85363A::clearPrescaler(void)
{
  write_register(PCF85363A_CTRL_RESETS, PCF85363A_RESET_CPR);
}

/**************************************************************************/
/*!
    @brief  Sent CTS (clear timestamp/25h) to Reset address (2Fh)
*/
/**************************************************************************/
void RTC_PCF85363A::clearTimestamp(void)
{
  write_register(PCF85363A_CTRL_RESETS, PCF85363A_RESET_CTS);
}

/**************************************************************************/
/*!
    @brief  Read the mode of the CLKOUT pin on the PCF85363A
    @return CLKOUT pin mode as a #Pcf8563SqwPinMode enum
*/
/**************************************************************************/
// Pcf85363ASqwPinMode RTC_PCF85363A::readSqwPinMode() {
//   int mode = read_register(PCF85363A_CLKOUTCONTROL);
//   return static_cast<Pcf85363ASqwPinMode>(mode & PCF85363A_CLKOUT_MASK);
// }

/**************************************************************************/
/*!
    @brief  Set the CLKOUT pin mode on the PCF85363A
    @param mode The mode to set, see the #Pcf85363ASqwPinMode enum for options
*/
/**************************************************************************/
// void RTC_PCF85363A::writeSqwPinMode(Pcf85363ASqwPinMode mode) {
//   write_register(PCF85363A_CLKOUTCONTROL, mode);
// }

/**************************************************************************/
/*!
    @brief  Set alarm 1 for PCF85363A
        @param 	dt DateTime object
        @param 	alarm_map Desired map, see Pcf85363aAlarm1Map enum
    @return True only
*/
/**************************************************************************/
bool RTC_PCF85363A::setAlarm1(const DateTime &dt, uint8_t alarm_map)
{
  /*
    uint8_t ctrl;

    uint8_t A1M1 = (alarm_mode & 0x01) << 7; // Seconds bit 7.
    uint8_t A1M2 = (alarm_mode & 0x02) << 6; // Minutes bit 7.
    uint8_t A1M3 = (alarm_mode & 0x04) << 5; // Hour bit 7.
    uint8_t A1M4 = (alarm_mode & 0x08) << 4; // Day/Date bit 7.
    uint8_t DY_DT = (alarm_mode & 0x10)
                    << 2; // Day/Date bit 6. Date when 0, day of week when 1.
    uint8_t day = (DY_DT) ? dowToPCF85363A(dt.dayOfTheWeek()) : dt.day();

    uint8_t buffer[5] = {PCF85363A_ALARM1, uint8_t(bin2bcd(dt.second()) | A1M1),
                         uint8_t(bin2bcd(dt.minute()) | A1M2),
                         uint8_t(bin2bcd(dt.hour()) | A1M3),
                         uint8_t(bin2bcd(day) | A1M4 | DY_DT)};
    i2c_dev->write(buffer, 5);

    write_register(PCF85363A_CONTROL, ctrl | 0x01); // AI1E
  */
  return true;
}

/**************************************************************************/
/*!
    @brief  Set alarm 2 for PCF85363A
        @param 	dt DateTime object
        @param 	alarm_map Desired map, see Pcf85363aAlarm2Map enum
    @return True only
*/
/**************************************************************************/
bool RTC_PCF85363A::setAlarm2(const DateTime &dt, uint8_t alarm_map)
{
  /*
    uint8_t ctrl;

    uint8_t A2M2 = (alarm_mode & 0x01) << 7; // Minutes bit 7.
    uint8_t A2M3 = (alarm_mode & 0x02) << 6; // Hour bit 7.
    uint8_t A2M4 = (alarm_mode & 0x04) << 5; // Day/Date bit 7.
    uint8_t DY_DT = (alarm_mode & 0x08)
                    << 3; // Day/Date bit 6. Date when 0, day of week when 1.
    uint8_t day = (DY_DT) ? dowToPCF85363A(dt.dayOfTheWeek()) : dt.day();

    uint8_t buffer[4] = {PCF85363A_ALARM2, uint8_t(bin2bcd(dt.minute()) | A2M2),
                         uint8_t(bin2bcd(dt.hour()) | A2M3),
                         uint8_t(bin2bcd(day) | A2M4 | DY_DT)};
    i2c_dev->write(buffer, 4);

    write_register(PCF85363A_CONTROL, ctrl | 0x02); // AI2E
  */
  return true;
}

/**************************************************************************/
/*!
    @brief  Get the date/time value of Alarm1
    @return DateTime object with the Alarm1 data set in the
            day, hour, minutes, and seconds fields
*/
/**************************************************************************/
DateTime RTC_PCF85363A::getAlarm1()
{
  /*
    uint8_t buffer[5] = {PCF85363A_ALARM1, 0, 0, 0, 0};
    i2c_dev->write_then_read(buffer, 1, buffer, 5);

    uint8_t seconds = bcd2bin(buffer[0] & 0x7F);
    uint8_t minutes = bcd2bin(buffer[1] & 0x7F);
    // Fetching the hour assumes 24 hour time (never 12)
    // because this library exclusively stores the time
    // in 24 hour format. Note that the PCF85363A supports
    // 12 hour storage, and sets bits to indicate the type
    // that is stored.
    uint8_t hour = bcd2bin(buffer[2] & 0x3F);

    // Determine if the alarm is set to fire based on the
    // day of the week, or an explicit date match.
    bool isDayOfWeek = (buffer[3] & 0x40) >> 6;
    uint8_t day;
    if (isDayOfWeek) {
      // Alarm set to match on day of the week
      day = bcd2bin(buffer[3] & 0x0F);
    } else {
      // Alarm set to match on day of the month
      day = bcd2bin(buffer[3] & 0x3F);
    }

    // On the first week of May 2000, the day-of-the-week number
    // matches the date number.
    return DateTime(2000, 5, day, hour, minutes, seconds);
  */
  return DateTime(2000, 1, 1, 0, 0, 0);
}

/**************************************************************************/
/*!
    @brief  Get the date/time value of Alarm2
    @return DateTime object with the Alarm2 data set in the
            day, hour, and minutes fields
*/
/**************************************************************************/
DateTime RTC_PCF85363A::getAlarm2()
{
  /*
    uint8_t buffer[4] = {PCF85363A_ALARM2, 0, 0, 0};
    i2c_dev->write_then_read(buffer, 1, buffer, 4);

    uint8_t minutes = bcd2bin(buffer[0] & 0x7F);
    // Fetching the hour assumes 24 hour time (never 12)
    // because this library exclusively stores the time
    // in 24 hour format. Note that the PCF85363A supports
    // 12 hour storage, and sets bits to indicate the type
    // that is stored.
    uint8_t hour = bcd2bin(buffer[1] & 0x3F);

    // Determine if the alarm is set to fire based on the
    // day of the week, or an explicit date match.
    bool isDayOfWeek = (buffer[2] & 0x40) >> 6;
    uint8_t day;
    if (isDayOfWeek) {
      // Alarm set to match on day of the week
      day = bcd2bin(buffer[2] & 0x0F);
    } else {
      // Alarm set to match on day of the month
      day = bcd2bin(buffer[2] & 0x3F);
    }

    // On the first week of May 2000, the day-of-the-week number
    // matches the date number.
    return DateTime(2000, 5, day, hour, minutes, 0);
  */
  return DateTime(2000, 1, 1, 0, 0, 0);
}

/**************************************************************************/
/*!
    @brief  Get the enables for Alarm1
    @return Map of the current Alarm1 enables
*/
/**************************************************************************/
uint8_t RTC_PCF85363A::getAlarm1Enables()
{
  return read_register(PCF85363A_DT_ALARM_EN) & 0x1F;
}

/**************************************************************************/
/*!
    @brief  Get the mode for Alarm2
    @return Map of the current Alarm2 enables
*/
/**************************************************************************/
uint8_t RTC_PCF85363A::getAlarm2Enables()
{
  return read_register(PCF85363A_DT_ALARM_EN) & 0xE0;
}

/**************************************************************************/
/*!
    @brief  Disable alarm
        @param 	alarm_num Alarm number to disable
*/
/**************************************************************************/
void RTC_PCF85363A::disableAlarm(uint8_t alarm_num)
{
  uint8_t ctrl = read_register(PCF85363A_DT_ALARM_EN);
  if (alarm_num == 1)
    ctrl &= 0xE0;
  else if (alarm_num == 2)
    ctrl &= 0x1F;
  write_register(PCF85363A_DT_ALARM_EN, ctrl);
}

/**************************************************************************/
/*!
    @brief  Clear status of alarm
        @param 	alarm_num Alarm number to clear
*/
/**************************************************************************/
void RTC_PCF85363A::clearAlarm(uint8_t alarm_num)
{
  if (alarm_num < 1 || alarm_num > 2)
    return;
  uint8_t flags = read_register(PCF85363A_CTRL_FLAGS);
  flags &= ~(0x20 << (alarm_num - 1));
  write_register(PCF85363A_CTRL_FLAGS, flags);
}

/**************************************************************************/
/*!
    @brief  Get status of alarm
        @param 	alarm_num Alarm number to check status of
        @return True if alarm has been fired otherwise false
*/
/**************************************************************************/
bool RTC_PCF85363A::alarmFired(uint8_t alarm_num)
{
  if (alarm_num < 1 || alarm_num > 2)
    return false;
  return (read_register(PCF85363A_CTRL_FLAGS) & (0x20 << (alarm_num - 1)));
}