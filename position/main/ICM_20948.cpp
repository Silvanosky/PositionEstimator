#include "ICM_20948.h"

#include "util/ICM_20948_REGISTERS.h"
#include "util/AK09916_REGISTERS.h"

// Forward Declarations
ICM_20948_Status_e ICM_20948_write_I2C(uint8_t reg, uint8_t *data, uint32_t len, void *user);
ICM_20948_Status_e ICM_20948_read_I2C(uint8_t reg, uint8_t *buff, uint32_t len, void *user);

static void delay(size_t ms)
{
	vTaskDelay(ms / portTICK_RATE_MS);
}

// Base
ICM_20948::ICM_20948()
{
}

ICM_20948_AGMT_t ICM_20948::getAGMT(void)
{
    status = ICM_20948_get_agmt(&_device, &agmt);

    return agmt;
}

float ICM_20948::magX(void)
{
    return getMagUT(agmt.mag.axes.x);
}

float ICM_20948::magY(void)
{
    return getMagUT(agmt.mag.axes.y);
}

float ICM_20948::magZ(void)
{
    return getMagUT(agmt.mag.axes.z);
}

float ICM_20948::getMagUT(int16_t axis_val)
{
    return (((float)axis_val) * 0.15);
}

float ICM_20948::accX(void)
{
    return getAccMG(agmt.acc.axes.x);
}

float ICM_20948::accY(void)
{
    return getAccMG(agmt.acc.axes.y);
}

float ICM_20948::accZ(void)
{
    return getAccMG(agmt.acc.axes.z);
}

float ICM_20948::getAccMG(int16_t axis_val)
{
    switch (agmt.fss.a)
    {
    case 0:
        return (((float)axis_val) / 16.384);
        break;
    case 1:
        return (((float)axis_val) / 8.192);
        break;
    case 2:
        return (((float)axis_val) / 4.096);
        break;
    case 3:
        return (((float)axis_val) / 2.048);
        break;
    default:
        return 0;
        break;
    }
}

float ICM_20948::gyrX(void)
{
    return getGyrDPS(agmt.gyr.axes.x);
}

float ICM_20948::gyrY(void)
{
    return getGyrDPS(agmt.gyr.axes.y);
}

float ICM_20948::gyrZ(void)
{
    return getGyrDPS(agmt.gyr.axes.z);
}

float ICM_20948::getGyrDPS(int16_t axis_val)
{
    switch (agmt.fss.g)
    {
    case 0:
        return (((float)axis_val) / 131);
        break;
    case 1:
        return (((float)axis_val) / 65.5);
        break;
    case 2:
        return (((float)axis_val) / 32.8);
        break;
    case 3:
        return (((float)axis_val) / 16.4);
        break;
    default:
        return 0;
        break;
    }
}

float ICM_20948::temp(void)
{
    return getTempC(agmt.tmp.val);
}

float ICM_20948::getTempC(int16_t val)
{
    return (((float)val) / 333.87) + 21;
}

const char *ICM_20948::statusString(ICM_20948_Status_e stat)
{
    ICM_20948_Status_e val;
    if (stat == ICM_20948_Stat_NUM)
    {
        val = status;
    }
    else
    {
        val = stat;
    }

    switch (val)
    {
    case ICM_20948_Stat_Ok:
        return "All is well.";
        break;
    case ICM_20948_Stat_Err:
        return "General Error";
        break;
    case ICM_20948_Stat_NotImpl:
        return "Not Implemented";
        break;
    case ICM_20948_Stat_ParamErr:
        return "Parameter Error";
        break;
    case ICM_20948_Stat_WrongID:
        return "Wrong ID";
        break;
    case ICM_20948_Stat_InvalSensor:
        return "Invalid Sensor";
        break;
    case ICM_20948_Stat_NoData:
        return "Data Underflow";
        break;
    case ICM_20948_Stat_SensorNotSupported:
        return "Sensor Not Supported";
        break;
    default:
        return "Unknown Status";
        break;
    }
    return "None";
}

// Device Level
ICM_20948_Status_e ICM_20948::setBank(uint8_t bank)
{
    status = ICM_20948_set_bank(&_device, bank);
    return status;
}

ICM_20948_Status_e ICM_20948::swReset(void)
{
    status = ICM_20948_sw_reset(&_device);
    return status;
}

ICM_20948_Status_e ICM_20948::sleep(bool on)
{
    status = ICM_20948_sleep(&_device, on);
    return status;
}

ICM_20948_Status_e ICM_20948::lowPower(bool on)
{
    status = ICM_20948_low_power(&_device, on);
    return status;
}

ICM_20948_Status_e ICM_20948::setClockSource(ICM_20948_PWR_MGMT_1_CLKSEL_e source)
{
    status = ICM_20948_set_clock_source(&_device, source);
    return status;
}

ICM_20948_Status_e ICM_20948::checkID(void)
{
    status = ICM_20948_check_id(&_device);
    return status;
}

bool ICM_20948::dataReady(void)
{
    status = ICM_20948_data_ready(&_device);
    if (status == ICM_20948_Stat_Ok)
    {
        return true;
    }
    return false;
}

uint8_t ICM_20948::getWhoAmI(void)
{
    uint8_t retval = 0x00;
    status = ICM_20948_get_who_am_i(&_device, &retval);
    return retval;
}

bool ICM_20948::isConnected(void)
{
    status = checkID();
    if (status == ICM_20948_Stat_Ok)
    {
        return true;
    }
    return false;
}

// Internal Sensor Options
ICM_20948_Status_e ICM_20948::setSampleMode(uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode)
{
    status = ICM_20948_set_sample_mode(&_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, (ICM_20948_LP_CONFIG_CYCLE_e)lp_config_cycle_mode);
    return status;
}

ICM_20948_Status_e ICM_20948::setFullScale(uint8_t sensor_id_bm, ICM_20948_fss_t fss)
{
    status = ICM_20948_set_full_scale(&_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, fss);
    return status;
}

ICM_20948_Status_e ICM_20948::setDLPFcfg(uint8_t sensor_id_bm, ICM_20948_dlpcfg_t cfg)
{
    status = ICM_20948_set_dlpf_cfg(&_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, cfg);
    return status;
}

ICM_20948_Status_e ICM_20948::enableDLPF(uint8_t sensor_id_bm, bool enable)
{
    status = ICM_20948_enable_dlpf(&_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, enable);
    return status;
}

ICM_20948_Status_e ICM_20948::setSampleRate(uint8_t sensor_id_bm, ICM_20948_smplrt_t smplrt)
{
    status = ICM_20948_set_sample_rate(&_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, smplrt);
    return status;
}

// Interrupts on INT Pin
ICM_20948_Status_e ICM_20948::clearInterrupts(void)
{
    ICM_20948_INT_STATUS_t int_stat;
    ICM_20948_INT_STATUS_1_t int_stat_1;

    // read to clear interrupts
    status = ICM_20948_set_bank(&_device, 0);
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    status = ICM_20948_execute_r(&_device, AGB0_REG_INT_STATUS, (uint8_t *)&int_stat, sizeof(ICM_20948_INT_STATUS_t));
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    status = ICM_20948_execute_r(&_device, AGB0_REG_INT_STATUS_1, (uint8_t *)&int_stat_1, sizeof(ICM_20948_INT_STATUS_1_t));
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }

    // todo: there may be additional interrupts that need to be cleared, like FIFO overflow/watermark

    return status;
}

ICM_20948_Status_e ICM_20948::cfgIntActiveLow(bool active_low)
{
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg(&_device, NULL, &reg); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    reg.INT1_ACTL = active_low;                           // set the setting
    status = ICM_20948_int_pin_cfg(&_device, &reg, NULL); // write phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::cfgIntOpenDrain(bool open_drain)
{
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg(&_device, NULL, &reg); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    reg.INT1_OPEN = open_drain;                           // set the setting
    status = ICM_20948_int_pin_cfg(&_device, &reg, NULL); // write phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::cfgIntLatch(bool latching)
{
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg(&_device, NULL, &reg); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    reg.INT1_LATCH_EN = latching;                         // set the setting
    status = ICM_20948_int_pin_cfg(&_device, &reg, NULL); // write phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::cfgIntAnyReadToClear(bool enabled)
{
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg(&_device, NULL, &reg); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    reg.INT_ANYRD_2CLEAR = enabled;                       // set the setting
    status = ICM_20948_int_pin_cfg(&_device, &reg, NULL); // write phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::cfgFsyncActiveLow(bool active_low)
{
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg(&_device, NULL, &reg); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    reg.ACTL_FSYNC = active_low;                          // set the setting
    status = ICM_20948_int_pin_cfg(&_device, &reg, NULL); // write phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::cfgFsyncIntMode(bool interrupt_mode)
{
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg(&_device, NULL, &reg); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    reg.FSYNC_INT_MODE_EN = interrupt_mode;               // set the setting
    status = ICM_20948_int_pin_cfg(&_device, &reg, NULL); // write phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

//      All these individual functions will use a read->set->write method to leave other settings untouched
ICM_20948_Status_e ICM_20948::intEnableI2C(bool enable)
{
    ICM_20948_INT_enable_t en;                          // storage
    status = ICM_20948_int_enable(&_device, NULL, &en); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    en.I2C_MST_INT_EN = enable;                        // change the setting
    status = ICM_20948_int_enable(&_device, &en, &en); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    if (en.I2C_MST_INT_EN != enable)
    {
        status = ICM_20948_Stat_Err;
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::intEnableDMP(bool enable)
{
    ICM_20948_INT_enable_t en;                          // storage
    status = ICM_20948_int_enable(&_device, NULL, &en); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    en.DMP_INT1_EN = enable;                           // change the setting
    status = ICM_20948_int_enable(&_device, &en, &en); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    if (en.DMP_INT1_EN != enable)
    {
        status = ICM_20948_Stat_Err;
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::intEnablePLL(bool enable)
{
    ICM_20948_INT_enable_t en;                          // storage
    status = ICM_20948_int_enable(&_device, NULL, &en); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    en.PLL_RDY_EN = enable;                            // change the setting
    status = ICM_20948_int_enable(&_device, &en, &en); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    if (en.PLL_RDY_EN != enable)
    {
        status = ICM_20948_Stat_Err;
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::intEnableWOM(bool enable)
{
    ICM_20948_INT_enable_t en;                          // storage
    status = ICM_20948_int_enable(&_device, NULL, &en); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    en.WOM_INT_EN = enable;                            // change the setting
    status = ICM_20948_int_enable(&_device, &en, &en); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    if (en.WOM_INT_EN != enable)
    {
        status = ICM_20948_Stat_Err;
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::intEnableWOF(bool enable)
{
    ICM_20948_INT_enable_t en;                          // storage
    status = ICM_20948_int_enable(&_device, NULL, &en); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    en.REG_WOF_EN = enable;                            // change the setting
    status = ICM_20948_int_enable(&_device, &en, &en); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    if (en.REG_WOF_EN != enable)
    {
        status = ICM_20948_Stat_Err;
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::intEnableRawDataReady(bool enable)
{
    ICM_20948_INT_enable_t en;                          // storage
    status = ICM_20948_int_enable(&_device, NULL, &en); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    en.RAW_DATA_0_RDY_EN = enable;                     // change the setting
    status = ICM_20948_int_enable(&_device, &en, &en); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    if (en.RAW_DATA_0_RDY_EN != enable)
    {
        status = ICM_20948_Stat_Err;
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::intEnableOverflowFIFO(uint8_t bm_enable)
{
    ICM_20948_INT_enable_t en;                          // storage
    status = ICM_20948_int_enable(&_device, NULL, &en); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    en.FIFO_OVERFLOW_EN_0 = ((bm_enable >> 0) & 0x01); // change the settings
    en.FIFO_OVERFLOW_EN_1 = ((bm_enable >> 1) & 0x01);
    en.FIFO_OVERFLOW_EN_2 = ((bm_enable >> 2) & 0x01);
    en.FIFO_OVERFLOW_EN_3 = ((bm_enable >> 3) & 0x01);
    en.FIFO_OVERFLOW_EN_4 = ((bm_enable >> 4) & 0x01);
    status = ICM_20948_int_enable(&_device, &en, &en); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::intEnableWatermarkFIFO(uint8_t bm_enable)
{
    ICM_20948_INT_enable_t en;                          // storage
    status = ICM_20948_int_enable(&_device, NULL, &en); // read phase
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    en.FIFO_WM_EN_0 = ((bm_enable >> 0) & 0x01); // change the settings
    en.FIFO_WM_EN_1 = ((bm_enable >> 1) & 0x01);
    en.FIFO_WM_EN_2 = ((bm_enable >> 2) & 0x01);
    en.FIFO_WM_EN_3 = ((bm_enable >> 3) & 0x01);
    en.FIFO_WM_EN_4 = ((bm_enable >> 4) & 0x01);
    status = ICM_20948_int_enable(&_device, &en, &en); // write phase w/ readback
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

// Interface Options
ICM_20948_Status_e ICM_20948::i2cMasterPassthrough(bool passthrough)
{
    status = ICM_20948_i2c_master_passthrough(&_device, passthrough);
    return status;
}

ICM_20948_Status_e ICM_20948::i2cMasterEnable(bool enable)
{
    status = ICM_20948_i2c_master_enable(&_device, enable);
    return status;
}

ICM_20948_Status_e ICM_20948::i2cMasterReset()
{
    status = ICM_20948_i2c_master_reset(&_device);
    return status;
}

ICM_20948_Status_e ICM_20948::i2cMasterConfigureSlave(uint8_t slave, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap)
{
    status = ICM_20948_i2c_master_configure_slave(&_device, slave, addr, reg, len, Rw, enable, data_only, grp, swap);
    return status;
}

ICM_20948_Status_e ICM_20948::i2cMasterSLV4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr)
{
    status = ICM_20948_i2c_master_slv4_txn(&_device, addr, reg, data, len, Rw, send_reg_addr);
    return status;
}

ICM_20948_Status_e ICM_20948::i2cMasterSingleW(uint8_t addr, uint8_t reg, uint8_t data)
{
    status = ICM_20948_i2c_master_single_w(&_device, addr, reg, &data);
    return status;
}
uint8_t ICM_20948::i2cMasterSingleR(uint8_t addr, uint8_t reg)
{
    uint8_t data;
    status = ICM_20948_i2c_master_single_r(&_device, addr, reg, &data);
    return data;
}

ICM_20948_Status_e ICM_20948::startupDefault(void)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

    retval = checkID();
    if (retval != ICM_20948_Stat_Ok)
    {
        status = retval;
        return status;
    }

    retval = swReset();
    if (retval != ICM_20948_Stat_Ok)
    {
        status = retval;
        return status;
    }
    delay(50);

    retval = sleep(false);
    if (retval != ICM_20948_Stat_Ok)
    {
        status = retval;
        return status;
    }

    retval = lowPower(false);
    if (retval != ICM_20948_Stat_Ok)
    {
        status = retval;
        return status;
    }

    retval = setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous); // options: ICM_20948_Sample_Mode_Continuous or ICM_20948_Sample_Mode_Cycled
    if (retval != ICM_20948_Stat_Ok)
    {
        status = retval;
        return status;
    } // sensors: 	ICM_20948_Internal_Acc, ICM_20948_Internal_Gyr, ICM_20948_Internal_Mst

    ICM_20948_fss_t FSS;
    FSS.a = gpm2;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
    FSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
    retval = setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS);
    if (retval != ICM_20948_Stat_Ok)
    {
        status = retval;
        return status;
    }

    ICM_20948_dlpcfg_t dlpcfg;
    dlpcfg.a = acc_d473bw_n499bw;
    dlpcfg.g = gyr_d361bw4_n376bw5;
    retval = setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
    if (retval != ICM_20948_Stat_Ok)
    {
        status = retval;
        return status;
    }

    retval = enableDLPF(ICM_20948_Internal_Acc, false);
    if (retval != ICM_20948_Stat_Ok)
    {
        status = retval;
        return status;
    }
    retval = enableDLPF(ICM_20948_Internal_Gyr, false);
    if (retval != ICM_20948_Stat_Ok)
    {
        status = retval;
        return status;
    }
    retval = startupMagnetometer();
    if (retval != ICM_20948_Stat_Ok)
    {
        status = retval;
        return status;
    }

    return status;
}

// direct read/write
ICM_20948_Status_e ICM_20948::read(uint8_t reg, uint8_t *pdata, uint32_t len)
{
    status = ICM_20948_execute_r(&_device, reg, pdata, len);
    return (status);
}

ICM_20948_Status_e ICM_20948::write(uint8_t reg, uint8_t *pdata, uint32_t len)
{
    status = ICM_20948_execute_w(&_device, reg, pdata, len);
    return (status);
}

uint8_t ICM_20948::readMag(AK09916_Reg_Addr_e reg)
{
    uint8_t data = i2cMasterSingleR(MAG_AK09916_I2C_ADDR, reg);
    return data;
}

ICM_20948_Status_e ICM_20948::writeMag(AK09916_Reg_Addr_e reg, uint8_t *pdata)
{
    status = i2cMasterSingleW(MAG_AK09916_I2C_ADDR, reg, *pdata);
    return status;
}

// I2C
ICM_20948_I2C::ICM_20948_I2C()
{
}

#define KHZ (1000U)

#define I2C_MASTER_SCL_IO GPIO_NUM_18
#define I2C_MASTER_SDA_IO GPIO_NUM_5

#define I2C_MASTER_NUM I2C_NUM_0

#define I2C_MASTER_FREQ_HZ (400*KHZ)
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL (i2c_ack_type_t)0x1                            /*!< I2C nack value */


ICM_20948_Status_e ICM_20948_I2C::begin(bool ad0val, uint8_t ad0pin)
{
	int i2c_master_port = I2C_MASTER_NUM;
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_MASTER_SDA_IO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = I2C_MASTER_SCL_IO;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	i2c_param_config((i2c_port_t) i2c_master_port, &conf);
	i2c_driver_install((i2c_port_t) i2c_master_port, conf.mode,
			I2C_MASTER_RX_BUF_DISABLE,
			I2C_MASTER_TX_BUF_DISABLE, 0);

    // Associate
    _addr = ICM_20948_I2C_ADDR_AD0;

    // _i2c->begin(); // Moved into user's sketch

    // Set up the serif
    _serif.write = ICM_20948_write_I2C;
    _serif.read = ICM_20948_read_I2C;
    _serif.user = (void *)this; // refer to yourself in the user field

    // Link the serif
    _device._serif = &_serif;

    // Perform default startup
    status = startupDefault();
    if (status != ICM_20948_Stat_Ok)
    {
        return status;
    }
    return status;
}

ICM_20948_Status_e ICM_20948::startupMagnetometer(void)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

    i2cMasterPassthrough(false); //Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
    i2cMasterEnable(true);

    //After a ICM reset the Mag sensor may stop responding over the I2C master
    //Reset the Master I2C until it responds
    uint8_t tries = 0;
    uint8_t maxTries = 5;
    while (tries < maxTries)
    {
        //See if we can read the WhoIAm register correctly
        retval = magWhoIAm();
        if (retval == ICM_20948_Stat_Ok)
            break; //WIA matched!

        i2cMasterReset(); //Otherwise, reset the master I2C and try again
        tries++;
    }

    if (tries == maxTries)
    {
        status = ICM_20948_Stat_WrongID;
        return status;
    }

    //Serial.printf("Mag connected tries: %d\n", tries);

    //Set up magnetometer
    AK09916_CNTL2_Reg_t reg;
    reg.MODE = AK09916_mode_cont_100hz;
    retval = writeMag(AK09916_REG_CNTL2, (uint8_t *)&reg);
    if (retval != ICM_20948_Stat_Ok)
    {
        status = retval;
        return status;
    }

    retval = i2cMasterConfigureSlave(0, MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 9, true, true, false, false, false);
    if (retval != ICM_20948_Stat_Ok)
    {
        status = retval;
        return status;
    }

    return status;
}

ICM_20948_Status_e ICM_20948::magWhoIAm(void)
{
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

    uint8_t whoiam1, whoiam2;
    whoiam1 = readMag(AK09916_REG_WIA1);
    whoiam2 = readMag(AK09916_REG_WIA2);
    status = retval;
    if (retval != ICM_20948_Stat_Ok)
    {
        return retval;
    }

    if ((whoiam1 == (MAG_AK09916_WHO_AM_I >> 8)) && (whoiam2 == (MAG_AK09916_WHO_AM_I & 0xFF)))
    {
        retval = ICM_20948_Stat_Ok;
        status = retval;
        return status;
    }
    retval = ICM_20948_Stat_WrongID;
    status = retval;
    return status;
}

// serif functions for the I2C and SPI classes
ICM_20948_Status_e ICM_20948_write_I2C(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
    if (user == NULL)
    {
        return ICM_20948_Stat_ParamErr;
    }
    uint8_t addr = ((ICM_20948_I2C *)user)->_addr;

	i2c_port_t i2c_master_port = I2C_MASTER_NUM;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (addr << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
	i2c_master_write(cmd, data, len, ACK_CHECK_EN);
	i2c_master_stop(cmd);

	bool result = false;
    for (int i = 0; !result && i < 3; i++) {
        result = (i2c_master_cmd_begin(i2c_master_port, cmd, 100/portTICK_RATE_MS) == ESP_OK);
        if (!result) {
            i2c_reset_tx_fifo(i2c_master_port);
            i2c_reset_rx_fifo(i2c_master_port);
        }
    }
	i2c_cmd_link_delete(cmd);

    return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e ICM_20948_read_I2C(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
    if (user == NULL)
    {
        return ICM_20948_Stat_ParamErr;
    }
    uint8_t addr = ((ICM_20948_I2C *)user)->_addr;

	i2c_port_t i2c_master_port = I2C_MASTER_NUM;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write(cmd, &reg, 1, true);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
	i2c_master_read(cmd, buff, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

	bool result = false;
    for (int i = 0; !result && i < 3; i++) {
        result = (i2c_master_cmd_begin(i2c_master_port, cmd, 100/portTICK_RATE_MS) == ESP_OK);
        if (!result) {
            i2c_reset_tx_fifo(i2c_master_port);
            i2c_reset_rx_fifo(i2c_master_port);
        }
    }

    i2c_cmd_link_delete(cmd);

	return (result) ?  ICM_20948_Stat_Ok : ICM_20948_Stat_NoData;
}
