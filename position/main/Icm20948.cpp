#include "Icm20948.h"

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

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

IMU_ST_SENSOR_DATA gstGyroOffset = {0,0,0};

#ifdef __cplusplus
extern "C" {
#endif

	void icm20948init(void);
	bool icm20948Check(void);
	void icm20948GyroRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z);
	void icm20948AccelRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z);
	void icm20948MagRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z);
	bool icm20948MagCheck(void);
	void icm20948CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal);
	void icm20948GyroOffset(void);
	void icm20948ReadSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8Len, uint8_t *pu8data);
	void icm20948WriteSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8data);
	bool icm20948Check(void);

	/******************************************************************************
	 * interface driver                                                           *
	 ******************************************************************************/
	void delay(size_t ms)
	{
		vTaskDelay(ms / portTICK_RATE_MS);
	}

	esp_err_t I2C_InitBus(void)
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
		return i2c_driver_install((i2c_port_t) i2c_master_port, conf.mode,
				I2C_MASTER_RX_BUF_DISABLE,
				I2C_MASTER_TX_BUF_DISABLE, 0);

	}

	uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr)
	{
		uint8_t value;

		int i2c_master_port = I2C_MASTER_NUM;

		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (DevAddr << 1) | READ_BIT, ACK_CHECK_EN);
		i2c_master_read_byte(cmd, &value, NACK_VAL);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin((i2c_port_t) i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
		i2c_cmd_link_delete(cmd);

		return value;
	}

	void I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t value)
	{
		int i2c_master_port = I2C_MASTER_NUM;

		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (DevAddr << 1) | WRITE_BIT, ACK_CHECK_EN);
		i2c_master_write_byte(cmd, RegAddr, ACK_CHECK_EN);
		i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin((i2c_port_t) i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
		i2c_cmd_link_delete(cmd);
	}

	/******************************************************************************
	 * IMU module                                                                 *
	 ******************************************************************************/
#define Kp 4.50f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 1.0f    // integral gain governs rate of convergence of gyroscope biases

	float angles[3];
	float q0, q1, q2, q3;

	void imuInit(IMU_EN_SENSOR_TYPE *penMotionSensorType)
	{
		bool bRet = false;
		I2C_InitBus();

		bRet = icm20948Check();
		if( true == bRet)
		{
			*penMotionSensorType = IMU_EN_SENSOR_TYPE_ICM20948;
			icm20948init();
			printf("[IMU] Good IMU\n");
		}
		else
		{
			*penMotionSensorType = IMU_EN_SENSOR_TYPE_NULL;
			printf("[IMU] Error no IMU\n");
		}

		q0 = 1.0f;
		q1 = 0.0f;
		q2 = 0.0f;
		q3 = 0.0f;

		return;
	}

	void imuDataGet(IMU_ST_SENSOR_DATA *pstGyroRawData,
			IMU_ST_SENSOR_DATA *pstAccelRawData,
			IMU_ST_SENSOR_DATA *pstMagnRawData)
	{
		int16_t s16Gyro[3], s16Accel[3], s16Magn[3];

		icm20948AccelRead(&s16Accel[0], &s16Accel[1], &s16Accel[2]);
		icm20948GyroRead(&s16Gyro[0], &s16Gyro[1], &s16Gyro[2]);
		icm20948MagRead(&s16Magn[0], &s16Magn[1], &s16Magn[2]);

		pstGyroRawData->s16X = s16Gyro[0];
		pstGyroRawData->s16Y = s16Gyro[1];
		pstGyroRawData->s16Z = s16Gyro[2];

		pstAccelRawData->s16X = s16Accel[0];
		pstAccelRawData->s16Y = s16Accel[1];
		pstAccelRawData->s16Z  = s16Accel[2];

		pstMagnRawData->s16X = s16Magn[0];
		pstMagnRawData->s16Y = s16Magn[1];
		pstMagnRawData->s16Z = s16Magn[2];
	}

	/******************************************************************************
	 * icm20948 sensor device                                                     *
	 ******************************************************************************/
	void icm20948init(void)
	{
		/* user bank 0 register */
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_PWR_MIGMT_1,  REG_VAL_ALL_RGE_RESET);
		delay(10);
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_PWR_MIGMT_1,  REG_VAL_RUN_MODE);

		/* user bank 2 register */
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2);
		I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_GYRO_SMPLRT_DIV, 0x07);
		I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_GYRO_CONFIG_1,
				REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_1000DPS | REG_VAL_BIT_GYRO_DLPF);
		I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_ACCEL_SMPLRT_DIV_2,  0x07);
		I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_ACCEL_CONFIG,
				REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF);

		/* user bank 0 register */
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);

		delay(100);
		/* offset */
		icm20948GyroOffset();

		icm20948MagCheck();

		icm20948WriteSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_WRITE,
				REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_20HZ);
		return;
	}

	bool icm20948Check(void)
	{
		bool bRet = false;
		if(REG_VAL_WIA == I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_WIA))
		{
			bRet = true;
		}
		return bRet;
	}
	void icm20948GyroRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
	{
		uint8_t u8Buf[6];
		int16_t s16Buf[3] = {0};
		uint8_t i;
		int32_t s32OutBuf[3] = {0};
		static ICM20948_ST_AVG_DATA sstAvgBuf[3];
		static int16_t ss16c = 0;
		ss16c++;

		u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_XOUT_L);
		u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_XOUT_H);
		s16Buf[0]=  (u8Buf[1]<<8)|u8Buf[0];

		u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_YOUT_L);
		u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_YOUT_H);
		s16Buf[1]=  (u8Buf[1]<<8)|u8Buf[0];

		u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_ZOUT_L);
		u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_ZOUT_H);
		s16Buf[2]=  (u8Buf[1]<<8)|u8Buf[0];

		for(i = 0; i < 3; i ++)
		{
			icm20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
		}
		*ps16X = s32OutBuf[0] - gstGyroOffset.s16X;
		*ps16Y = s32OutBuf[1] - gstGyroOffset.s16Y;
		*ps16Z = s32OutBuf[2] - gstGyroOffset.s16Z;

		return;
	}
	void icm20948AccelRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
	{
		uint8_t u8Buf[2];
		int16_t s16Buf[3] = {0};
		uint8_t i;
		int32_t s32OutBuf[3] = {0};
		static ICM20948_ST_AVG_DATA sstAvgBuf[3];

		u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_XOUT_L);
		u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_XOUT_H);
		s16Buf[0]=  (u8Buf[1]<<8)|u8Buf[0];

		u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_YOUT_L);
		u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_YOUT_H);
		s16Buf[1]=  (u8Buf[1]<<8)|u8Buf[0];

		u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_ZOUT_L);
		u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_ZOUT_H);
		s16Buf[2]=  (u8Buf[1]<<8)|u8Buf[0];

		for(i = 0; i < 3; i ++)
		{
			icm20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
		}
		*ps16X = s32OutBuf[0];
		*ps16Y = s32OutBuf[1];
		*ps16Z = s32OutBuf[2];

		return;

	}
	void icm20948MagRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
	{
		uint8_t counter = 20;
		uint8_t u8Data[MAG_DATA_LEN];
		int16_t s16Buf[3] = {0};
		uint8_t i;
		int32_t s32OutBuf[3] = {0};
		static ICM20948_ST_AVG_DATA sstAvgBuf[3];
		while( counter>0 )
		{
			delay(10);
			icm20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,
					REG_ADD_MAG_ST2, 1, u8Data);

			if ((u8Data[0] & 0x01) != 0)
				break;

			counter--;
		}

		if(counter != 0)
		{
			icm20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,
					REG_ADD_MAG_DATA,
					MAG_DATA_LEN,
					u8Data);
			s16Buf[0] = ((int16_t)u8Data[1]<<8) | u8Data[0];
			s16Buf[1] = ((int16_t)u8Data[3]<<8) | u8Data[2];
			s16Buf[2] = ((int16_t)u8Data[5]<<8) | u8Data[4];
		}

		for(i = 0; i < 3; i ++)
		{
			icm20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
		}

		*ps16X =  s32OutBuf[0];
		*ps16Y = -s32OutBuf[1];
		*ps16Z = -s32OutBuf[2];
		return;
	}

	void icm20948ReadSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8Len, uint8_t *pu8data)
	{
		uint8_t i;
		uint8_t u8Temp;

		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3); //swtich bank3
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_ADDR, u8I2CAddr);
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_REG,  u8RegAddr);
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN|u8Len);

		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

		u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_USER_CTRL);
		u8Temp |= REG_VAL_BIT_I2C_MST_EN;
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);
		delay(5);
		u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);

		for(i=0; i<u8Len; i++)
		{
			*(pu8data+i) = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_EXT_SENS_DATA_00+i);

		}
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); //swtich bank3

		u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_I2C_SLV0_CTRL);
		u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL,  u8Temp);

		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

	}

	void icm20948WriteSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8data)
	{
		uint8_t u8Temp;
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3); //swtich bank3
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_ADDR, u8I2CAddr);
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_REG,  u8RegAddr);
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_DO,   u8data);
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN|1);

		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

		u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_USER_CTRL);
		u8Temp |= REG_VAL_BIT_I2C_MST_EN;
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);
		delay(5);
		u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);

		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); //swtich bank3

		u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_I2C_SLV0_CTRL);
		u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));
		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL,  u8Temp);

		I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

		return;
	}

	void icm20948CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
	{
		uint8_t i;

		*(pAvgBuffer + ((*pIndex) ++)) = InVal;
		*pIndex &= 0x07;

		*pOutVal = 0;
		for(i = 0; i < 8; i ++)
		{
			*pOutVal += *(pAvgBuffer + i);
		}
		*pOutVal >>= 3;
	}

	void icm20948GyroOffset(void)
	{
		uint8_t i;
		int16_t s16Gx = 0, s16Gy = 0, s16Gz = 0;
		int32_t s32TempGx = 0, s32TempGy = 0, s32TempGz = 0;
		for(i = 0; i < 32; i ++)
		{
			icm20948GyroRead(&s16Gx, &s16Gy, &s16Gz);
			s32TempGx += s16Gx;
			s32TempGy += s16Gy;
			s32TempGz += s16Gz;
			delay(10);
		}
		gstGyroOffset.s16X = s32TempGx >> 5;
		gstGyroOffset.s16Y = s32TempGy >> 5;
		gstGyroOffset.s16Z = s32TempGz >> 5;
		return;
	}

	bool icm20948MagCheck(void)
	{
		bool bRet = false;
		uint8_t u8Ret[2];

		icm20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,
				REG_ADD_MAG_WIA1, 2,u8Ret);
		if( (u8Ret[0] == REG_VAL_MAG_WIA1) && ( u8Ret[1] == REG_VAL_MAG_WIA2) )
		{
			bRet = true;
		}

		return bRet;
	}

#ifdef __cplusplus
}
#endif
