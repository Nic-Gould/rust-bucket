//use esp_idf_hal::delay::{FreeRtos, BLOCK};
//use esp_idf_hal::i2c::*;
//use esp_idf_hal::peripherals::Peripherals;
//use esp_idf_hal::prelude::*;


fn main(){
	
 //We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 

//Magnetometer Registers
//const AK8963_ADDRESS   :u8 = 0x0C; // addresses defined below
const AK8963_WHO_AM_I  :u8 = 0x00; // should return :u8 = 0x48
const AK8963_INFO      :u8 = 0x01;
const AK8963_ST1       :u8 = 0x02 ; // data ready status bit 0
const AK8963_XOUT_L	 :u8 = 0x03;  // data
const AK8963_XOUT_H	 :u8 = 0x04;
const AK8963_YOUT_L	 :u8 = 0x05;
const AK8963_YOUT_H	 :u8 = 0x06;
const AK8963_ZOUT_L	 :u8 = 0x07;
const AK8963_ZOUT_H	 :u8 = 0x08;
const AK8963_ST2       :u8 = 0x09;  // Data overflow bit 3 and data read error status bit 2
const AK8963_CNTL      :u8 = 0x0A;  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
const AK8963_ASTC      :u8 = 0x0C;  // Self test control
const AK8963_I2CDIS    :u8 = 0x0F;  // I2C disable
const AK8963_ASAX      :u8 = 0x10;  // Fuse ROM x-axis sensitivity adjustment value
const AK8963_ASAY      :u8 = 0x11;  // Fuse ROM y-axis sensitivity adjustment value
const AK8963_ASAZ      :u8 = 0x12;  // Fuse ROM z-axis sensitivity adjustment value

const SELF_TEST_X_GYRO :u8 = 0x00 ;                 
const SELF_TEST_Y_GYRO :u8 = 0x01  ;                                                                        
const SELF_TEST_Z_GYRO :u8 = 0x02;

/*const X_FINE_GAIN      :u8 = 0x03 // [7:0] fine gain
const Y_FINE_GAIN      :u8 = 0x04
const Z_FINE_GAIN      :u8 = 0x05
const XA_OFFSET_H      :u8 = 0x06 // User-defined trim values for accelerometer
const XA_OFFSET_L_TC   :u8 = 0x07
const YA_OFFSET_H      :u8 = 0x08
const YA_OFFSET_L_TC   :u8 = 0x09
const ZA_OFFSET_H      :u8 = 0x0A
const ZA_OFFSET_L_TC   :u8 = 0x0B */

const SELF_TEST_X_ACCEL :u8 = 0x0D;
const SELF_TEST_Y_ACCEL :u8 = 0x0E ;   
const SELF_TEST_Z_ACCEL :u8 = 0x0F;

const SELF_TEST_A      :u8 = 0x10;

const XG_OFFSET_H      :u8 = 0x13 ; // User-defined trim values for gyroscope
const XG_OFFSET_L      :u8 = 0x14;
const YG_OFFSET_H      :u8 = 0x15;
const YG_OFFSET_L      :u8 = 0x16;
const ZG_OFFSET_H      :u8 = 0x17;
const ZG_OFFSET_L      :u8 = 0x18;
const SMPLRT_DIV       :u8 = 0x19;
const CONFIG           :u8 = 0x1A;
const GYRO_CONFIG      :u8 = 0x1B;
const ACCEL_CONFIG     :u8 = 0x1C;
const ACCEL_CONFIG2    :u8 = 0x1D;
const LP_ACCEL_ODR     :u8 = 0x1E;  
const WOM_THR          :u8 = 0x1F; 

const MOT_DUR          :u8 = 0x20;  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
const ZMOT_THR         :u8 = 0x21;  // Zero-motion detection threshold bits [7:0]
const ZRMOT_DUR        :u8 = 0x22;  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

const FIFO_EN          :u8 = 0x23;
const I2C_MST_CTRL     :u8 = 0x24;  
const I2C_SLV0_ADDR    :u8 = 0x25;
const I2C_SLV0_REG     :u8 = 0x26;
const I2C_SLV0_CTRL    :u8 = 0x27;
const I2C_SLV1_ADDR    :u8 = 0x28;
const I2C_SLV1_REG     :u8 = 0x29;
const I2C_SLV1_CTRL    :u8 = 0x2A;
const I2C_SLV2_ADDR    :u8 = 0x2B;
const I2C_SLV2_REG     :u8 = 0x2C;
const I2C_SLV2_CTRL    :u8 = 0x2D;
const I2C_SLV3_ADDR    :u8 = 0x2E;
const I2C_SLV3_REG     :u8 = 0x2F;
const I2C_SLV3_CTRL    :u8 = 0x30;
const I2C_SLV4_ADDR    :u8 = 0x31;
const I2C_SLV4_REG     :u8 = 0x32;
const I2C_SLV4_DO      :u8 = 0x33;
const I2C_SLV4_CTRL    :u8 = 0x34;
const I2C_SLV4_DI      :u8 = 0x35;
const I2C_MST_STATUS   :u8 = 0x36;
const INT_PIN_CFG      :u8 = 0x37;
const INT_ENABLE       :u8 = 0x38;
const DMP_INT_STATUS   :u8 = 0x39;  // Check DMP interrupt
const INT_STATUS       :u8 = 0x3A;
const ACCEL_XOUT_H     :u8 = 0x3B;
const ACCEL_XOUT_L     :u8 = 0x3C;
const ACCEL_YOUT_H     :u8 = 0x3D;
const ACCEL_YOUT_L     :u8 = 0x3E;
const ACCEL_ZOUT_H     :u8 = 0x3F;
const ACCEL_ZOUT_L     :u8 = 0x40;
const TEMP_OUT_H       :u8 = 0x41;
const TEMP_OUT_L       :u8 = 0x42;
const GYRO_XOUT_H      :u8 = 0x43;
const GYRO_XOUT_L      :u8 = 0x44;
const GYRO_YOUT_H      :u8 = 0x45;
const GYRO_YOUT_L      :u8 = 0x46;
const GYRO_ZOUT_H      :u8 = 0x47;
const GYRO_ZOUT_L      :u8 = 0x48;
const EXT_SENS_DATA_00 :u8 = 0x49;
const EXT_SENS_DATA_01 :u8 = 0x4A;
const EXT_SENS_DATA_02 :u8 = 0x4B;
const EXT_SENS_DATA_03 :u8 = 0x4C;
const EXT_SENS_DATA_04 :u8 = 0x4D;
const EXT_SENS_DATA_05 :u8 = 0x4E;
const EXT_SENS_DATA_06 :u8 = 0x4F;
const EXT_SENS_DATA_07 :u8 = 0x50;
const EXT_SENS_DATA_08 :u8 = 0x51;
const EXT_SENS_DATA_09 :u8 = 0x52;
const EXT_SENS_DATA_10 :u8 = 0x53;
const EXT_SENS_DATA_11 :u8 = 0x54;
const EXT_SENS_DATA_12 :u8 = 0x55;
const EXT_SENS_DATA_13 :u8 = 0x56;
const EXT_SENS_DATA_14 :u8 = 0x57;
const EXT_SENS_DATA_15 :u8 = 0x58;
const EXT_SENS_DATA_16 :u8 = 0x59;
const EXT_SENS_DATA_17 :u8 = 0x5A;
const EXT_SENS_DATA_18 :u8 = 0x5B;
const EXT_SENS_DATA_19 :u8 = 0x5C;
const EXT_SENS_DATA_20 :u8 = 0x5D;
const EXT_SENS_DATA_21 :u8 = 0x5E;
const EXT_SENS_DATA_22 :u8 = 0x5F;
const EXT_SENS_DATA_23 :u8 = 0x60;
const MOT_DETECT_STATUS :u8 = 0x61;
const I2C_SLV0_DO      :u8 = 0x63;
const I2C_SLV1_DO      :u8 = 0x64;
const I2C_SLV2_DO      :u8 = 0x65;
const I2C_SLV3_DO      :u8 = 0x66;
const I2C_MST_DELAY_CTRL :u8 = 0x67;
const SIGNAL_PATH_RESET  :u8 = 0x68;
const MOT_DETECT_CTRL  :u8 = 0x69;
const USER_CTRL        :u8 = 0x6A;  // Bit 7 enable DMP, bit 3 reset DMP
const PWR_MGMT_1       :u8 = 0x6B; // Device defaults to the SLEEP mode
const PWR_MGMT_2       :u8 = 0x6C;
const DMP_BANK         :u8 = 0x6D; // Activates a specific bank in the DMP
const DMP_RW_PNT       :u8 = 0x6E;  // Set read/write pointer to a specific start address in specified DMP bank
const DMP_REG          :u8 = 0x6F; // Register in DMP from which to read or to which to write
const DMP_REG_1        :u8 = 0x70;
const DMP_REG_2        :u8 = 0x71;
const FIFO_COUNTH      :u8 = 0x72;
const FIFO_COUNTL      :u8 = 0x73;
const FIFO_R_W         :u8 = 0x74;
const WHO_AM_I_MPU9250 :u8 = 0x75; 

const XA_OFFS_USRH     :u8 = 0x77;	//These are mislabeled in the HW Offset datasheet.
const XA_OFFS_USRL      :u8 = 0x78;
const YA_OFFS_USRH      :u8 = 0x7A;
const YA_OFFS_USRL      :u8 = 0x7B;
const ZA_OFFS_USRH      :u8 = 0x7D;
const ZA_OFFS_USRL      :u8 = 0x7E;

const MPU9250_ADDRESS :u8 = 0x68;  // Device address when ADO = 0
const AK8963_ADDRESS :u8 = 0x0C;  //  Address of magnetometer
//	const MS5637_ADDRESS :u8 = 076;   // Address of altimeter
  

const SerialDebug:bool = true;  // set to true to get Serial output for debugging


const ADC_256  :u8 = 0x00; // define pressure and temperature conversion rates
const ADC_512  :u8 = 0x02;
const ADC_1024 :u8 = 0x04;
const ADC_2048 :u8 = 0x06;
const ADC_4096 :u8 = 0x08;
const ADC_8192 :u8 = 0x0A;
const ADC_D1   :u8 = 0x40;
const ADC_D2   :u8 = 0x50;
const PI:f32= std::f32::consts::PI;	
const RAD:f32	= PI/180.0;
const DEG:f32	= 180.0/PI;
enum AccScale {
	AFS_2G,
	AFS_4G,
	AFS_8G,
	AFS_16G,
}

enum GyroScale {
	GFS_250DPS,
	GFS_500DPS,
	GFS_1000DPS,
	GFS_2000DPS,
}

enum MagScale {
	MFS_14BITS,// 0.6 mG per LSB
	MFS_16BITS,     // 0.15 mG per LSB
}




// Specify sensor full scale
let gybar = GyroScale::GFS_250DPS;
let accbar = AccScale::AFS_2G;
let magmar = MagScale::MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
let mag_mode :u8 = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
let acc_res = get_acc_res(accbar);
let gyro_res =  get_gyro_res(gybar);
let mag_res  = get_mag_res(magmar);  
  
// Pin definitions
let interupt_pin:i32 = 8;  
let led_pin :i32= 13;


let new_data:bool = false;
let new_mag_data:bool = false;
let mpu_data:[i16;7]=[0;7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
let mag_data:[i16;3] = [0;3];    // Stores the 16-bit signed magnetometer sensor output
let q:[f32;4]=[1.0, 0.0, 0.0, 0.0];
let mag_calibration:[f32;3] = [0.0;3];  // Factory mag calibration and mag bias
let mag_scale:[f32;3] = [0.0;3]; 

let gyro_bias:[f32;3] = [0.0;3]; 
let acc_bias:[f32;3] = [0.0;3];
let mag_bias:[f32;3] = [0.0;3];
     
let tempCount:i16=0;            // temperature raw count output
let temperature:f32=0.0;          // Stores the MPU9250 gyro internal chip temperature in degrees Celsius
//double Temperature, Pressure; // stores MS5637 pressures sensor pressure and temperature
let self_test_data:[f32;6]= [0.0;6];            // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
let gyro_err:f32 = RAD* 4.0;   // gyroscope measurement error in rads/s (start at 40 deg/s)
let gyro_drift:f32 = RAD*0.0;   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)


	
let beta = f32::sqrt(3.0_f32 / 4.0_f32) * gyro_err;   
let zeta = f32::sqrt(3.0_f32 / 4.0_f32) * gyro_drift;   
const Kp:f32 =2.0_f32 * 5.0_f32; 
const Ki:f32 =0.0_f32;


fn setup(){

  
  
/*   Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(4000);
  Serial.begin(38400);
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(interupt_pin, INPUT);
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH); */
  
  
  	self_test();
  	calibrate_mpu(acc_bias, gyro_bias); 
  	init_mpu(); 
	init_mag(mag_calibration); 
	calibrate_mag(mag_bias, mag_scale);

	

  //attachInterrupt(interupt_pin, myinthandler, RISING);  // define interrupt for INT pin output of MPU9250

 
}

loop {
	
	  // If interupt_pin goes high, all data registers have new data
   	if new_data {  
     	new_data = false;  // reset new_data flag
    	read_mpu_data(mpu_data, acc_bias, acc_res, gyro_res); // INT cleared on any read
	}
	if new_mag_data { 
		new_mag_data = false;
		read_mag_data(mag_data, mag_bias, mag_calibration, mag_res, mag_scale, mag_mode);
	}	  
	now = micros();
	deltat = (now - last_update)/1000000.0; // set integration time by time elapsed since last filter update
	last_update = now;

	sum += deltat; // sum for averaging filter update rate
	sum_count+=1;
  
	mahoney(mpu_data, mag_data);
    let location = to_ref_frame(q);

	  
    digitalWrite(led_pin, !digitalRead(led_pin));
    count = millis(); 
    sum_count = 0;
    sum = 0; 

}

//===================================================================================================================
//====== Function definitions
//===================================================================================================================
fn to_ref_frame(q:[f32;4])-> &[f32;6]{
	let a12 =   2.0_f32 * (q[1] * q[2] + q[0] * q[3]);
	let a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	let a31 =   2.0_f32 * (q[0] * q[1] + q[2] * q[3]);
	let a32 =   2.0_f32 * (q[1] * q[3] - q[0] * q[2]);
	let a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
	let pitch = -f32::asin(a32);
	let roll  = f32::atan2(a31, a33);
	let yaw   = f32::atan2(a12, a22);
	pitch *= DEG;
	yaw   *= DEG;
	yaw   += declination; // 
	if yaw < 0.0 {yaw   += 360.0}; // Ensure yaw stays between 0 and 360
	roll  *= DEG;
	
	let lin_ax = ax + a31;
	let lin_ay = ay + a32;
	let lin_az = az - a33;
	&[pitch, yaw, roll, lin_ax, lin_ay, lin_az]
}


fn print_outputs(){
	/* if(SerialDebug) {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);

    Serial.print("Grav_x, Grav_y, Grav_z: ");
    Serial.print(-a31*1000, 2);
    Serial.print(", ");
    Serial.print(-a32*1000, 2);
    Serial.print(", ");
    Serial.print(a33*1000, 2);  Serial.println(" mg");
    Serial.print("Lin_ax, Lin_ay, Lin_az: ");
    Serial.print(lin_ax*1000, 2);
    Serial.print(", ");
    Serial.print(lin_ay*1000, 2);
    Serial.print(", ");
    Serial.print(lin_az*1000, 2);  Serial.println(" mg");
    
    Serial.print("rate = "); Serial.print((float)sum_count/sum, 2); Serial.println(" Hz");
	}
	if(SerialDebug) {
		Serial.print("ax = "); Serial.print((int)1000*ax);  
		Serial.print(" ay = "); Serial.print((int)1000*ay); 
		Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
		Serial.print("gx = "); Serial.print( gx, 2); 
		Serial.print(" gy = "); Serial.print( gy, 2); 
		Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
		Serial.print("mx = "); Serial.print( (int)mx ); 
		Serial.print(" my = "); Serial.print( (int)my ); 
		Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
		
		Serial.print("q0 = "); Serial.print(q[0]);
		Serial.print(" qx = "); Serial.print(q[1]); 
		Serial.print(" qy = "); Serial.print(q[2]); 
		Serial.print(" qz = "); Serial.println(q[3]); 
		Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C	
	}               
 */
}

fn myinthandler(){
	new_data = true;
	new_mag_data = i2c.read(AK8963_ADDRESS, AK8963_ST1) & 0x01;
}

fn get_mag_res(magmar: MagScale)->f32 {
	match magmar{
		// Possible magnetometer scales (and their register bit settings) are:
		// 14 bit resolution (0) and 16 bit resolution (1)
		MFS_14BITS => 10.*4912./8190.0, // Proper scale to return milliGauss
		MFS_16BITS => 10.*4912./32760.0, // Proper scale to return milliGauss
	}
}

fn get_gyro_res(gybar: GyroScale)->f32 {
  match gybar {
	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    GFS_250DPS=>250.0/32768.0,
    GFS_500DPS=>500.0/32768.0,
    GFS_1000DPS=> 1000.0/32768.0,
    GFS_2000DPS=>2000.0/32768.0,
  }
}

fn get_acc_res(accbar: AccScale) -> f32 {
  	match accbar {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    AFS_2G => 2.0/32768.0,
    AFS_4G => 4.0/32768.0,
    AFS_8G => 8.0/32768.0,          
	AFS_16G => 16.0/32768.0,
	}
}

fn read_mpu_data(mpu_data: [i16; 7], acc_bias: [f32; 3],acc_res: f64, gyro_res: f64) {
  let raw_data:[u8;14];  // x/y/z accel register data stored here
  i2c.read(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, &raw_data);  // Read the 14 raw data registers into data array
  mpu_data[1] = i16::from_le_bytes([raw_data[2] ,raw_data[3] ]);  
  mpu_data[0] = i16::from_le_bytes([raw_data[0] ,raw_data[1] ]);  // Turn the MSB and LSB into a signed 16-bit value
  mpu_data[2] = i16::from_le_bytes([raw_data[4] ,raw_data[5] ]);
  mpu_data[3] = i16::from_le_bytes([raw_data[6] ,raw_data[7] ]);  
  mpu_data[4] = i16::from_le_bytes([raw_data[8] ,raw_data[9] ]); 
  mpu_data[5] = i16::from_le_bytes([raw_data[10],raw_data[11]]);  
  mpu_data[6] = i16::from_le_bytes([raw_data[12],raw_data[13]]); 

  mpu_data[0] = mpu_data[0] * (acc_res - acc_bias[0]) as i16; 
  mpu_data[1] = mpu_data[1] * (acc_res - acc_bias[1]) as i16;   
  mpu_data[2] = mpu_data[2] * (acc_res - acc_bias[2]) as i16;  
  mpu_data[4] = mpu_data[4] * (gyro_res) as i16;  
  mpu_data[5] = mpu_data[5] * (gyro_res) as i16;  
  mpu_data[6] = mpu_data[6] * (gyro_res) as i16;   
  let temperature:f32 = read_temp_data() as f32 / 333.87 + 21.0; // Gyro chip temperature in degrees Centigrade
}

/*
fn read_acc_data(buffer:[T])
{
  let raw_data:[u8;6];  // x/y/z accel register data stored here
  i2c.read(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &raw_data[0]);  // Read the six raw data registers into data array
  buffer[0] = ((int16_t)raw_data[0] << 8) | raw_data[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  buffer[1] = ((int16_t)raw_data[2] << 8) | raw_data[3] ;  
  buffer[2] = ((int16_t)raw_data[4] << 8) | raw_data[5] ; 
}


fn read_gyro_data(buffer:[T])
{
  let raw_data:[u8;6];  // x/y/z gyro register data stored here
  i2c.read(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &raw_data[0]);  // Read the six raw data registers sequentially into data array
  buffer[0] = ((int16_t)raw_data[0] << 8) | raw_data[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  buffer[1] = ((int16_t)raw_data[2] << 8) | raw_data[3] ;  
  buffer[2] = ((int16_t)raw_data[4] << 8) | raw_data[5] ; 
}
*/

fn read_mag_data(mag_data: [i16; 3], mag_bias: [f32; 3], mag_calibration: [f32; 3], mag_res: f64, mag_scale: [f32; 3], mag_mode: u8)
{
	let raw_data:[u8;7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	i2c.read(AK8963_ADDRESS, AK8963_XOUT_L, 7, &raw_data);  // Read the six raw data and ST2 registers sequentially into data array
	let st2reg = raw_data[6]; // End data read by reading ST2 register
   
	if st2reg & 0x08!= 0x08 { // Check if magnetic sensor overflow set, if not then report data
    	mag_data[0] = i16::from_le_bytes([raw_data[1], raw_data[0]]) ;  // Turn the MSB and LSB into a signed 16-bit value
    	mag_data[1] = i16::from_le_bytes([raw_data[3], raw_data[2]]);  // Data stored as little Endian
    	mag_data[2] = i16::from_le_bytes([raw_data[5], raw_data[4]]); 
   	}

	mag_data[0] *= (mag_scale[0] * (mag_res*mag_calibration[0] - mag_bias[0])) as i16;  // get actual magnetometer value, this depends on scale being set
	mag_data[1] *= (mag_scale[1] * (mag_res*mag_calibration[1] - mag_bias[1])) as i16;  
	mag_data[2] *= (mag_scale[2] * (mag_res*mag_calibration[2] - mag_bias[2])) as i16;  

}

fn read_temp_data()->i16{
	let raw_data:[u8;2];  // x/y/z gyro register data stored here
	i2c.read(MPU9250_ADDRESS, TEMP_OUT_H, 2, &raw_data);  // Read the two raw data registers sequentially into data array 
	return ((raw_data[0]as i16) << 8) | raw_data[1] as i16;  // Turn the MSB and LSB into a 16-bit value
}
       
fn init_mag(mag_calibration: [f32; 3], mag_scale: [f32; 3], mag_mode: u8) {
	let who2 = i2c.read(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
	Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

	// First extract the factory calibration for each magnetometer axis
	let raw_data:[u8;3];  // x/y/z gyro calibration data stored here
	i2c.write(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
	delay(10);
	i2c.write(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	delay(10);
	i2c.read(AK8963_ADDRESS, AK8963_ASAX, 3, &raw_data);  // Read the x-, y-, and z-axis calibration values
	mag_calibration[0] =  ((raw_data[0] - 128)/256 + 1).into();   // Return x-axis sensitivity adjustment values, etc.
	mag_calibration[1] =  ((raw_data[1] - 128)/256 + 1).into();  
	mag_calibration[2] =  ((raw_data[2] - 128)/256 + 1).into(); 
	i2c.write(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
	delay(10);
	// Configure the magnetometer for continuous read and highest resolution
	// set mag_scale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition mag_mode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	i2c.write(AK8963_ADDRESS, AK8963_CNTL, mag_scale << 4 | mag_mode); // Set magnetometer data resolution and sample ODR
	delay(10);
	Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
}


fn init_mpu(){  
	// wake up device
	i2c.write(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
	delay(100); // Wait for all registers to reset 

	// get stable time source
	i2c.write(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
	delay(200); 
	
	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	i2c.write(MPU9250_ADDRESS, CONFIG, 0x03);  

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	i2c.write(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
										// determined inset in CONFIG above
	
	// Set gyroscope full scale range
	// Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	let gyro_config:u8 = i2c.read(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
	//gyro_config=gyro_config& ~0xE0; // Clear self-test bits [7:5] 
	gyro_config=gyro_config & !0x03; // Clear Fchoice bits [1:0] 
	gyro_config=gyro_config & !0x18; // Clear GFS bits [4:3]
	gyro_config=gyro_config| gyro_scale << 3; // Set full scale range for the gyro
	//gyro_config=| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	i2c.write(MPU9250_ADDRESS, GYRO_CONFIG,gyro_config); // Write new GYRO_CONFIG value to register
	
	// Set accelerometer full-scale range configuration
	let acc_config:u8 = i2c.read(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
	//acc_config=acc_config& ~0xE0; // Clear self-test bits [7:5] 
	acc_config =acc_config& !0x18;  // Clear AFS bits [4:3]
	acc_config =acc_config| Acc_Scale << 3; // Set full scale range for the accelerometer 
	i2c.write(MPU9250_ADDRESS, ACCEL_CONFIG, acc_config); // Write new ACCEL_CONFIG register value

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	acc_config = i2c.read(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	acc_config =acc_config& !0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	acc_config =acc_config| 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	i2c.write(MPU9250_ADDRESS, ACCEL_CONFIG2, acc_config); // Write new ACCEL_CONFIG2 register value
	
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
	// can join the I2C bus and all can be controlled by the Arduino as master
	//   i2c.write(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
	i2c.write(MPU9250_ADDRESS, INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear  
	i2c.write(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	delay(100);

	Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.


fn calibrate_mpu(acc_bias: [f32; 3], gyro_bias: [f32; 3]){  
	   // get sensor resolutions, only need to do this once
	   // scale resolutions per LSB for the sensors 

	Serial.println(" Calibrate gyro and accel");
    
	// reset device
	i2c.write(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);
	
	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
	// else use the internal oscillator, bits 2:0 = 001
	i2c.write(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  
	i2c.write(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
	delay(200);                                    

	// Configure device for bias calculation
	i2c.write(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
	i2c.write(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
	i2c.write(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	i2c.write(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
	i2c.write(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	i2c.write(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay(15);
  
	// Configure MPU6050 gyro and accelerometer for bias calculation
	i2c.write(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	i2c.write(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	i2c.write(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	i2c.write(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
	
  	let  gyro_sens:i16  = 131;   // = 131 LSB/degrees/sec
  	let  acc_sens:i16 = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
  	i2c.write(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
  	i2c.write(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  	delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	let fifo_temp:[u8;2]=[0;2];
	i2c.write(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	i2c.read(MPU9250_ADDRESS, FIFO_COUNTH, 2, &fifo_temp); // read FIFO sample count
	
	let fifo_count = i16::from_le_bytes([fifo_temp[0], fifo_temp [1]]) ;
	let packet_count = fifo_count /12;// How many sets of full gyro and accelerometer data for averaging
	let accel_temp:[i16;3] = [0;3];
	let gyro_temp:[i16;3] = [0;3];
  	let data:[u8;12] = [0;12];
	for _ in 0..packet_count {
		i2c.read(MPU9250_ADDRESS, FIFO_R_W, 12, &data); // read data for averaging
		accel_temp[0] = i16::from_le_bytes([data[0], data[1]]);  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = i16::from_le_bytes([data[2], data[3]]);
		accel_temp[2] = i16::from_le_bytes([data[4], data[5]]);    
		gyro_temp[0]  = i16::from_le_bytes([data[6], data[7]]);
		gyro_temp[1]  = i16::from_le_bytes([data[8], data[9]]);
		gyro_temp[2]  = i16::from_le_bytes([data[10], data[11]]);
		
		acc_bias[0] +=  accel_temp[0].into(); // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		acc_bias[1] +=  accel_temp[1].into();
		acc_bias[2] +=  accel_temp[2].into();
		gyro_bias[0]  +=  gyro_temp[0].into();
		gyro_bias[1]  +=  gyro_temp[1].into();
		gyro_bias[2]  +=  gyro_temp[2].into();
				
	}
    acc_bias[0] /= packet_count as f32; // Normalize sums to get average count biases
    acc_bias[1] /= packet_count as f32;
    acc_bias[2] /= packet_count as f32;
    gyro_bias[0]  /= packet_count as f32;
    gyro_bias[1]  /= packet_count as f32;
    gyro_bias[2]  /= packet_count as f32;
    
	if acc_bias[2] > 0 {acc_bias[2] -= acc_sens as f32}  // Remove gravity from the z-axis accelerometer bias calculation
	else {acc_bias[2] += acc_sens as f32};
   
	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = ((-gyro_bias[0]/4.0) as u32 >> 8)  as u8; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = ((-gyro_bias[0]/4.0) as u32)        as u8; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = ((-gyro_bias[1]/4.0) as u32 >> 8)  as u8;
	data[3] = ((-gyro_bias[1]/4.0) as u32)        as u8;
	data[4] = ((-gyro_bias[2]/4.0) as u32 >> 8)  as u8;
	data[5] = ((-gyro_bias[2]/4.0) as u32)        as u8;
	
	// Push gyro biases to hardware registers
	i2c.write(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
	i2c.write(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
	i2c.write(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
	i2c.write(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
	i2c.write(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
	i2c.write(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
	
	// Output scaled gyro biases for display in the main program
	dest1[0] = gyro_bias[0]as f32/ gyro_sens as f32;  
	dest1[1] = gyro_bias[1]as f32/ gyro_sens as f32;
	dest1[2] = gyro_bias[2]as f32/ gyro_sens as f32;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	let acc_bias_reg:[i16;3] = [0;3]; // A place to hold the factory accelerometer trim biases
	i2c.read(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data); // Read factory accelerometer trim values
	i2c.read(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data);
	i2c.read(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data);
	acc_bias_reg[0] = i16::from_le_bytes([data[0], data[1]]);
	acc_bias_reg[1] = i16::from_le_bytes([data[0], data[1]]);
	acc_bias_reg[2] = i16::from_le_bytes([data[0], data[1]]);

	
	
	let mask:i16 = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	let mask_bit:[i16;3] = [0;3]; // Define array to hold mask bit for each accelerometer bias axis
	
	for i in 0..mask_bit.len(){
			mask_bit[i] = acc_bias_reg[i] & mask; // If temperature compensation bit is set, record that fact in mask_bit
	}
	
	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	acc_bias_reg[0] -= (acc_bias[0]/8.0).into(); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	acc_bias_reg[1] -= (acc_bias[1]/8.0).into();
	acc_bias_reg[2] -= (acc_bias[2]/8.0).into();
	
	data[0] = (acc_bias_reg[0] >> 8)		  as u8;
	data[1] = (acc_bias_reg[0]|mask_bit[0])	  as u8;    // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (acc_bias_reg[1] >> 8)		  as u8;
	data[3] = (acc_bias_reg[1]| mask_bit[1])  as u8; 
	data[4] = (acc_bias_reg[2] >> 8)		  as u8;
	data[5] = (acc_bias_reg[2]| mask_bit[2])  as u8;
	
	
	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	/*  i2c.write(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
	i2c.write(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
	i2c.write(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
	i2c.write(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
	i2c.write(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
	i2c.write(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
	*/
	
	
	// Output scaled accelerometer biases for display in the main program
	acc_bias[0] = acc_bias[0]/acc_sens as f32; 
	acc_bias[1] = acc_bias[1]/acc_sens as f32;
	acc_bias[2] = acc_bias[2]/acc_sens as f32;
	
	
	//Serial.println("accel biases (mg)"); Serial.println(1000.*acc_bias[0]); Serial.println(1000.*acc_bias[1]); Serial.println(1000.*acc_bias[2]);
	//Serial.println("gyro biases (dps)"); Serial.println(gyro_bias[0]); Serial.println(gyro_bias[1]); Serial.println(gyro_bias[2]);
}


fn calibrate_mag(bias_out: [f32; 3], scale_out:[f32; 3], mag_res: f32, mag_calibration: [f32; 3], mag_mode: u8){
	let sample_count:u16 = 0;
	let mag_bias:[i32;3] = [0, 0, 0]; 
	let mag_scale:[i32;3] = [0, 0, 0];
	let mag_max:[i16;3] = [-32767, -32767, -32767];  //why are these values hardcoded all the way down here
	let mag_min:[i16;3] = [32767, 32767, 32767]; 
	let mag_temp:[i16;3] = [0;3];

	Serial.println("Mag Calibration: Wave device in a figure eight until done!");
	delay(4000);
  
    // shoot for ~fifteen seconds of mag data
    if mag_mode == 0x02 {sample_count = 128};  // at 8 Hz ODR, new mag data is available every 125 ms
    if mag_mode == 0x06 {sample_count = 1500};  // at 100 Hz ODR, new mag data is available every 10 ms
   	for i in 0..sample_count{
    	read_mag_data(mag_temp, mag_bias,mag_calibration, mag_res, mag_scale, mag_mode);  // Read the mag data   
    	for j in 0..3{
      		if mag_temp[j] > mag_max[j] {mag_max[j] = mag_temp[j]};
      		if mag_temp[j] < mag_min[j] {mag_min[j] = mag_temp[j]};
    	}
    	if mag_mode == 0x02 {delay(135)};  // at 8 Hz ODR, new mag data is available every 125 ms
    	if mag_mode == 0x06 {delay(12)};  // at 100 Hz ODR, new mag data is available every 10 ms
    }

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = ((mag_max[0] + mag_min[0])/2) as i32;  // get average x mag bias in counts
    mag_bias[1]  = ((mag_max[1] + mag_min[1])/2) as i32;  // get average y mag bias in counts
    mag_bias[2]  = ((mag_max[2] + mag_min[2])/2) as i32;  // get average z mag bias in counts
    
    bias_out[0] = (mag_bias[0]as f32 * mag_res * mag_calibration[0]) ;  // save mag biases in G for main program
    bias_out[1] = (mag_bias[1]as f32 * mag_res * mag_calibration[1]) ;   
    bias_out[2] = (mag_bias[2]as f32 * mag_res * mag_calibration[2]) ;  
       
    // Get soft iron correction estimate
    mag_scale[0]  = ((mag_max[0] - mag_min[0])/2) as i32;  // get average x axis max chord length in counts
    mag_scale[1]  = ((mag_max[1] - mag_min[1])/2) as i32;  // get average y axis max chord length in counts
    mag_scale[2]  = ((mag_max[2] - mag_min[2])/2) as i32;  // get average z axis max chord length in counts

    let avg_rad = (mag_scale[0] + mag_scale[1] + mag_scale[2])/3;
    

    scale_out[0] = (avg_rad/(mag_scale[0]))as f32;
    scale_out[1] = (avg_rad/(mag_scale[1]))as f32;
    scale_out[2] = (avg_rad/(mag_scale[2]))as f32;
  
	/* Serial.println("Mag Calibration done!");
	Serial.println("AK8963 mag biases (mG)"); Serial.println(mag_bias[0]); Serial.println(mag_bias[1]); Serial.println(mag_bias[2]); 
	Serial.println("AK8963 mag scale (mG)"); Serial.println(mag_scale[0]); Serial.println(mag_scale[1]); Serial.println(mag_scale[2]); 
	delay(2000); // add delay to see results before serial spew of data
	
	if SerialDebug {
	//  Serial.println("Calibration values: ");
	Serial.print("X-Axis sensitivity adjustment value "); Serial.println(mag_calibration[0], 2);
	Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(mag_calibration[1], 2);
	Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(mag_calibration[2], 2); }
	*/
	
}


		

fn self_test(data_out:[f32; 6]){ 
	// Accelerometer and gyroscope self test; check calibration wrt factory settings
	// Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass

    // Read the WHO_AM_I register, this is a good test of communication
	Serial.println("MPU9250 9-axis motion sensor...");
	let who:u8 = i2c.read(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
	Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
			
	

	if who == 0x68{ // WHO_AM_I should always be 0x68
    
    	Serial.println("MPU9250 is online...");
	}
	else
	{
		Serial.print("Could not connect to MPU9250: 0x");
		Serial.println(c, HEX);
		loop {}; // Loop forever if communication doesn't happen
	}  
    
    delay(1000);
		
	let raw_data:[u8;6] = [0;6];
	let self_test_data:[u8;6] = [0;6];		//stupid local shaddowing is so dumb.
	let gyro_avg:[i32;3] = [0;3];
	let acc_avg:[i32;3] = [0;3];
	let factory_trim:[f32;6]=[0.0;6];
	let FS:i32 = 0;
	let num_samples = 200;
	
	i2c.write(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
	i2c.write(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	i2c.write(MPU9250_ADDRESS, GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
	i2c.write(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	i2c.write(MPU9250_ADDRESS, ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g

	for _ in 0..num_samples {  // get average current values of gyro and acclerometer
		
		i2c.read(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &raw_data);        // Read the six raw data registers into data array
		acc_avg[0] += i16::from_le_bytes([raw_data[0], raw_data[1]]).into();  // Turn the MSB and LSB into a signed 16-bit value
		acc_avg[1] += i16::from_le_bytes([raw_data[2], raw_data[3]]).into();  
		acc_avg[2] += i16::from_le_bytes([raw_data[4], raw_data[5]]).into(); 
	
		i2c.read(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &raw_data);       // Read the six raw data registers sequentially into data array
		gyro_avg[0] += i16::from_le_bytes([raw_data[0], raw_data[1]]).into();  // Turn the MSB and LSB into a signed 16-bit value
		gyro_avg[1] += i16::from_le_bytes([raw_data[2], raw_data[3]]).into();  
		gyro_avg[2] += i16::from_le_bytes([raw_data[4], raw_data[5]]).into(); 
  	}
  
	for i in 0..acc_avg.len() {  // Get average of 200 values and store as average current readings
		acc_avg[i] /= 200;
		gyro_avg[i] /= 200;
	}
  
	// Configure the accelerometer for self-test
	i2c.write(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	i2c.write(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	delay(25);  // Delay a while to let the device stabilize
	let a_st_avg:[i32;3] = [0;3]; 
	let g_st_avg:[i32;3] = [0;3];
  	for _ in 0..200 {  // get average self-test values of gyro and acclerometer
  
		i2c.read(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &raw_data);  // Read the six raw data registers into data array
		a_st_avg[0] += i16::from_le_bytes([raw_data[0], raw_data[1]]).into() ;  // Turn the MSB and LSB into a signed 16-bit value
		a_st_avg[1] += i16::from_le_bytes([raw_data[2], raw_data[3]]).into() ;  
		a_st_avg[2] += i16::from_le_bytes([raw_data[4], raw_data[5]]).into() ; 
		
		i2c.read(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &raw_data);  // Read the six raw data registers sequentially into data array
		g_st_avg[0] += i16::from_le_bytes([raw_data[0], raw_data[1]]).into() ;  // Turn the MSB and LSB into a signed 16-bit value
		g_st_avg[1] += i16::from_le_bytes([raw_data[2], raw_data[3]]).into() ;  
		g_st_avg[2] += i16::from_le_bytes([raw_data[4], raw_data[5]]).into() ; 
  	}
  
	for i in 0..3 {  // Get average of 200 values and store as average self-test readings
		a_st_avg[i] /= 200;
		g_st_avg[i] /= 200;
	}   
  
 	// Configure the gyro and accelerometer for normal operation
	i2c.write(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);  
	i2c.write(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);  
	delay(25);  // Delay a while to let the device stabilize
	
	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	self_test_data[0] = i2c.read(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); 
	self_test_data[1] = i2c.read(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); 
	self_test_data[2] = i2c.read(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); 
	self_test_data[3] = i2c.read(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  
	self_test_data[4] = i2c.read(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  
	self_test_data[5] = i2c.read(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  

	// Retrieve factory self-test value from self-test code reads
	factory_trim[0] = (2620/1<<FS)as f32 *f32::powf( 1.01 , (self_test_data[0] - 1)as f32); 
	factory_trim[1] = (2620/1<<FS)as f32 *f32::powf( 1.01 , (self_test_data[1] - 1)as f32); 
	factory_trim[2] = (2620/1<<FS)as f32 *f32::powf( 1.01 , (self_test_data[2] - 1)as f32); 
	factory_trim[3] = (2620/1<<FS)as f32 *f32::powf( 1.01 , (self_test_data[3] - 1)as f32); 
	factory_trim[4] = (2620/1<<FS)as f32 *f32::powf( 1.01 , (self_test_data[4] - 1)as f32); 
	factory_trim[5] = (2620/1<<FS)as f32 *f32::powf( 1.01 , (self_test_data[5] - 1)as f32); 
	
	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for i in 0..3 {
		data_out[i]   = 100.0*(a_st_avg[i] - acc_avg[i]) as f32 /factory_trim[i] - 100.;   // Report percent differences
		data_out[i+3] = 100.0*(g_st_avg[i] - gyro_avg[i]) as f32 /factory_trim[i+3] - 100.; // Report percent differences
	}
   
			/* Serial.print("x-axis self test: acceleration trim within : "); Serial.print(self_test_data[0],1); Serial.println("% of factory value");
			Serial.print("y-axis self test: acceleration trim within : "); Serial.print(self_test_data[1],1); Serial.println("% of factory value");
			Serial.print("z-axis self test: acceleration trim within : "); Serial.print(self_test_data[2],1); Serial.println("% of factory value");
			Serial.print("x-axis self test: gyration trim within : "); Serial.print(self_test_data[3],1); Serial.println("% of factory value");
			Serial.print("y-axis self test: gyration trim within : "); Serial.print(self_test_data[4],1); Serial.println("% of factory value");
			Serial.print("z-axis self test: gyration trim within : "); Serial.print(self_test_data[5],1); Serial.println("% of factory value"); */
}

fn mahony(mpu_data: [i16; 7], mag_data: [i16; 3], q:[f32;4]) {

                
	let ax = -mpu_data[0];
	let ay = mpu_data[1];
	let az = mpu_data[2];
	let gx = mpu_data[3]*RAD;
	let gy = -mpu_data[4]*RAD;
	let gz = -mpu_data[5]*RAD;
	let my = mag_data[0];
	let mx = mag_data[1];
	let mz = mag_data[2];

	/*         float norm;
	float vx, vy, vz;
	float ex, ey, ez;  //error terms
	float qa, qb, qc;
	static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
	float tmp; */

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	let tmp = ax * ax + ay * ay + az * az;
	if tmp !=0.0 {
		// Normalise accelerometer (assumed to measure the direction of gravity in body frame)
		let norm = fisr(tmp);
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Estimated direction of gravity in the body frame (factor of two divided out)
		let vx = q[1] * q[3] - q[0] * q[2];
		let vy = q[0] * q[1] + q[2] * q[3];
		let vz = q[0] * q[0] - 0.5 + q[3] * q[3];

		// Error is cross product between estimated and measured direction of gravity in body frame
		// (half the actual magnitude)
		let ex = ay * vz - az * vy;
		let ey = az * vx - ax * vz;
		let ez = ax * vy - ay * vx;

		// Compute and apply to gyro term the integral feedback, if enabled
		if Ki > 0.0 {
			let ix += Ki * ex * deltaT;  // integral error scaled by Ki
			let iy += Ki * ey * deltaT;
			let iz += Ki * ez * deltaT;
			gx += ix;  // apply integral feedback
			gy += iy;
			gz += iz;
		}

		// Apply proportional feedback to gyro term
		gx += Kp * ex;
		gy += Kp * ey;
		gz += Kp * ez;
	}

	// Integrate rate of change of quaternion, q cross gyro term
	deltaT = 0.5 * deltaT;
	gx *= deltaT;  // pre-multiply common factors
	gy *= deltaT;
	gz *= deltaT;
	let qa = q[0];
	let qb = q[1];
	let qc = q[2];
	q[0] += -qb * gx - qc * gy - q[3] * gz;
	q[1] += qa * gx + qc * gz - q[3] * gy;
	q[2] += qa * gy - qb * gz + q[3] * gx;
	q[3] += qa * gz + qb * gy - qc * gx;

	// renormalise quaternion
	let qnorm = fisr(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] = q[0] * qnorm;
	q[1] = q[1] * qnorm;
	q[2] = q[2] * qnorm;
	q[3] = q[3] * qnorm;
}

}