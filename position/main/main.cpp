#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "ICM_20948.h"
#include "Fusion.h"

FusionBias fusionBias;
FusionAhrs fusionAhrs;

//float samplePeriod = 0.01f; // replace this value with actual sample period in seconds
float samplePeriod = 1.f; // replace this value with actual sample period in seconds
const float G = 9.807f;


FusionVector3 gyroscopeSensitivity = {
	.axis = { 2000.0f/32767.5f * 3.14159265359f/180.0f, 2000.0f/32767.5f * 3.14159265359f/180.0f, 2000.0f/32767.5f * 3.14159265359f/180.0f}
}; // replace these values with actual sensitivity in degrees per second per lsb as specified in gyroscope datasheet

FusionVector3 accelerometerSensitivity = {
	.axis = { G * 16.0f/32767.5f, G * 16.0f/32767.5f, G * 16.0f/32767.5f} //TODO check value
}; // replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet

FusionVector3 accelerometerBias = {
	.axis = { 0.f, 0.f, 0.f}
}; // replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet

FusionVector3 hardIronBias = {
	.axis = { 0.0f, 0.0f, 0.0f}
}; // replace these values with actual hard-iron bias in uT if known

static void delay(size_t ms)
{
	vTaskDelay(ms / portTICK_RATE_MS);
}

//TODO gérer temps

void delay_microseconds(uint16_t us)
{
    if (us <= 100) {
        ets_delay_us(us);
    } else {
        uint32_t tick = portTICK_PERIOD_MS * 1000;
        vTaskDelay((us+tick-1)/tick);
    }
}

ICM_20948_I2C myICM;

static void periodic_timer_callback(void* arg);

extern "C" void app_main()
{
	int r = myICM.begin();
	if (r < 0)
		printf("No connected %d\n", r);

	// Set Gyro and Accelerometer to a particular sample mode
	// options: ICM_20948_Sample_Mode_Continuous
	//          ICM_20948_Sample_Mode_Cycled
	myICM.setSampleMode( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
	// Set full scale ranges for both acc and gyr
	ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

	myFSS.a = gpm16;         // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
	// gpm2
	// gpm4
	// gpm8
	// gpm16

	myFSS.g = dps2000;       // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
	// dps250
	// dps500
	// dps1000
	// dps2000

	myICM.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS );

	// Set up Digital Low-Pass Filter configuration
	ICM_20948_dlpcfg_t myDLPcfg;            // Similar to FSS, this uses a configuration structure for the desired sensors
	myDLPcfg.a = acc_d246bw_n265bw;         // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
	// acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
	// acc_d111bw4_n136bw
	// acc_d50bw4_n68bw8
	// acc_d23bw9_n34bw4
	// acc_d11bw5_n17bw
	// acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
	// acc_d473bw_n499bw

	myDLPcfg.g = gyr_d361bw4_n376bw5;       // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
	// gyr_d196bw6_n229bw8
	// gyr_d151bw8_n187bw6
	// gyr_d119bw5_n154bw3
	// gyr_d51bw2_n73bw3
	// gyr_d23bw9_n35bw9
	// gyr_d11bw6_n17bw8
	// gyr_d5bw7_n8bw9
	// gyr_d361bw4_n376bw5

	myICM.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg );

	// Choose whether or not to use DLPF
	myICM.enableDLPF( ICM_20948_Internal_Acc, true );
	myICM.enableDLPF( ICM_20948_Internal_Gyr, true );

	delay(100);

	const esp_timer_create_args_t periodic_timer_args = { &periodic_timer_callback, NULL, ESP_TIMER_TASK, "periodic" };

    esp_timer_handle_t periodic_timer;
    esp_timer_create(&periodic_timer_args, &periodic_timer);
    /* The timer has been created but is not running yet */

    /* Start the timers */
    esp_timer_start_periodic(periodic_timer, 3000);

	// Initialise gyroscope bias correction algorithm
	FusionBiasInitialise(&fusionBias, 1.5f, 0.003f); // stationary threshold = 0.5 degrees per second

    // Initialise AHRS algorithm
    FusionAhrsInitialise(&fusionAhrs, 10.f); // gain = 0.5

    // Set optional magnetic field limits
    FusionAhrsSetMagneticField(&fusionAhrs, 20.0f, 70.0f); // valid magnetic field range = 20 uT to 70 uT

	// The contents of this do while loop should be called for each time new sensor measurements are available
	// put your main code here, to run repeatedly:

	//Groupe 3
	do {
		delay(10000);
	} while(true);
    /* Clean up and finish the example */
    esp_timer_stop(periodic_timer);
    esp_timer_delete(periodic_timer);


}

FusionVector3 velocity;
FusionVector3 position;

static void periodic_timer_callback(void* arg)
{
	//printf("\f");
	static int n = 0;
	static int64_t last_time = esp_timer_get_time();


	myICM.getAGMT();

	// Calibrate gyroscope
	FusionVector3 uncalibratedGyroscope = {
		.axis = { myICM.gyrX(), myICM.gyrY(), myICM.gyrZ() } /* replace this value with actual gyroscope x,y,z axis measurement in lsb */
	};
	FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

	// Calibrate accelerometer
	FusionVector3 uncalibratedAccelerometer = {
		.axis = { myICM.accX(), myICM.accY(), myICM.accZ()}/* replace this value with actual accelerometer x,y,z axis measurement in lsb */
	};
	FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

	// Calibrate magnetometer
	FusionVector3 uncalibratedMagnetometer = {
		.axis = { myICM.magX(), myICM.magY(), myICM.magZ()} /* replace this value with actual magnetometer x axis measurement in uT */
	};

	/*
	if (n >= 0)
	{
		printf("%f;%f;%f;", uncalibratedGyroscope.axis.x,      uncalibratedGyroscope.axis.y,      uncalibratedGyroscope.axis.z);
		printf("%f;%f;%f;", uncalibratedAccelerometer.axis.x,  uncalibratedAccelerometer.axis.y,  uncalibratedAccelerometer.axis.z);
		printf("%f;%f;%f\n", uncalibratedMagnetometer.axis.x,   uncalibratedMagnetometer.axis.y,   uncalibratedMagnetometer.axis.z);
		n = 0;
	}
	n++;
	*/

	FusionVector3 calibratedMagnetometer = FusionCalibrationMagnetic(uncalibratedMagnetometer, FUSION_ROTATION_MATRIX_IDENTITY, hardIronBias);

	// Update gyroscope bias correction algorithm
	calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);

    int64_t time_since_boot = esp_timer_get_time();

	double time = (time_since_boot - last_time) / 1000000.;
	last_time = time_since_boot;
	// Update AHRS algorithm
	FusionAhrsUpdate(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, calibratedMagnetometer, time);

	// Print Euler angles
	FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));

	//Compute Velocity and Posiiton
	FusionVector3 acc = FusionAhrsGetEarthAcceleration(&fusionAhrs);

	FusionVector3 at = FusionVectorMultiplyScalar(acc, time);
	velocity = FusionVectorAdd(velocity, at);


	FusionVector3 att = FusionVectorMultiplyScalar(at, time);
	FusionVector3 att2 = FusionVectorMultiplyScalar(att, 0.5f);

	position = FusionVectorAdd(position, att2);

	FusionVector3 vt = FusionVectorMultiplyScalar(velocity, time);
	position = FusionVectorAdd(position, vt);

	if (n >= 10)
	{
		n = 0;
		//printf("%f\n", time);
		//printf("Roll = %0.1f, Pitch = %0.1f, Yaw = %0.1f\r\n", eulerAngles.angle.roll, eulerAngles.angle.pitch, eulerAngles.angle.yaw);
		//printf("ax = %0.1f, ay = %0.1f, Yaw = %f\r\n", acc.axis.x, acc.axis.y, eulerAngles.angle.yaw);
		//printf("vx = %f, vy = %f, Yaw = %f\r\n", velocity.axis.x, velocity.axis.y, eulerAngles.angle.yaw);
		//printf("x = %f, y = %f, Yaw = %f\r\n", position.axis.x, position.axis.y, eulerAngles.angle.yaw);
		printf("%f;%f;%f\n", position.axis.x, position.axis.y, eulerAngles.angle.yaw);
	}
	n++;


}

