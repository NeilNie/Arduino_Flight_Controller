#include <Wire.h>

//////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
//////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int PID_MAX_ROLL = 400;                    //Maximum output of the PID-controller (+/-)
//-------------------------------------------------------------------------------------------
float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = PID_MAX_ROLL;          //Maximum output of the PID-controller (+/-)
//-------------------------------------------------------------------------------------------
float pid_p_gain_yaw = 4.0;                //Gain setting for the yaw P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the yaw I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the yaw D-controller.
int PID_MAX_YAW = 400;                     //Maximum output of the PID-controller (+/-)

////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
byte highByte, lowByte;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage, gyro_address; //some stuff
int cal_int, start;
int receiver_input[5];
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;
//-------------------------------------------------------------------------------------------
long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  //----------------------------------------------calibration--------------------------------
  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                          //Take 2000 readings for calibration.
    if (cal_int % 15 == 0)digitalWrite(12, !digitalRead(12));               //Change the led status to indicate calibration.
    gyro_signalen();                                                        //Read the gyro output.
    gyro_axis_cal[1] += gyro_axis[1];                                       //Ad roll value to gyro_roll_cal.
    gyro_axis_cal[2] += gyro_axis[2];                                       //Ad pitch value to gyro_pitch_cal.
    gyro_axis_cal[3] += gyro_axis[3];                                       //Ad yaw value to gyro_yaw_cal.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_axis_cal[1] /= 2000;                                                 //Divide the roll total by 2000.
  gyro_axis_cal[2] /= 2000;                                                 //Divide the pitch total by 2000.
  gyro_axis_cal[3] /= 2000;                                                 //Divide the yaw total by 2000.

  //-------------------------------------------------------------------------------------------
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void set_gyro_registers() {

  //Setup the MPU-6050
  if (eeprom_data[31] == 1) {
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1B);                                                          //Start reading @ register 0x1B
    Wire.endTransmission();                                                    //End the transmission
    Wire.requestFrom(gyro_address, 1);                                         //Request 1 bytes from the gyro
    while (Wire.available() < 1);                                              //Wait until the 6 bytes are received
    if (Wire.read() != 0x08) {                                                 //Check if the value is 0x08
      digitalWrite(12, HIGH);                                                  //Turn on the warning led
      while (1)delay(10);                                                      //Stay in this loop for ever
    }

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro

  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen() {
  //Read the MPU-6050
  if (eeprom_data[31] == 1) {
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(gyro_address, 14);                                     //Request 14 bytes from the gyro.

    receiver_input_channel_1 = convert_receiver_channel(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    receiver_input_channel_2 = convert_receiver_channel(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

    while (Wire.available() < 14);                                          //Wait until the 14 bytes are received.
    acc_axis[1] = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_x variable.
    acc_axis[2] = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_y variable.
    acc_axis[3] = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_z variable.
    temperature = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the temperature variable.
    gyro_axis[1] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
    gyro_axis[2] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
    gyro_axis[3] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
  }

  if (cal_int == 2000) {
    gyro_axis[1] -= gyro_axis_cal[1];                                       //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                                       //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                                       //Only compensate after the calibration.
  }
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];                      //Set gyro_roll to the correct axis that was stored in the EEPROM.
  if (eeprom_data[28] & 0b10000000)gyro_roll *= -1;                         //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  if (eeprom_data[29] & 0b10000000)gyro_pitch *= -1;                        //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];                       //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  if (eeprom_data[30] & 0b10000000)gyro_yaw *= -1;                          //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];                           //Set acc_x to the correct axis that was stored in the EEPROM.
  if (eeprom_data[29] & 0b10000000)acc_x *= -1;                             //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];                           //Set acc_y to the correct axis that was stored in the EEPROM.
  if (eeprom_data[28] & 0b10000000)acc_y *= -1;                             //Invert acc_y if the MSB of EEPROM bit 28 is set.
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];                           //Set acc_z to the correct axis that was stored in the EEPROM.
  if (eeprom_data[30] & 0b10000000)acc_z *= -1;                             //Invert acc_z if the MSB of EEPROM bit 30 is set.
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
int convert_receiver_channel(byte function) {

  byte channel, reverse;                       //First we declare some local variables
  int low, center, high, actual;
  int difference;

  channel = eeprom_data[function + 23] & 0b00000111;                           //What channel corresponds with the specific function
  if (eeprom_data[function + 23] & 0b10000000)reverse = 1;                     //Reverse channel when most significant bit is set
  else reverse = 0;                                                            //If the most significant is not set there is no reverse

  actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];   //Store the high value for the specific receiver input channel

  if (actual < center) {                                                       //The actual receiver value is lower than the center value
    if (actual < low)actual = low;                                             //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
    if (reverse == 1)return 1500 + difference;                                 //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if (actual > center) {                                                  //The actual receiver value is higher than the center value
    if (actual > high)actual = high;                                           //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
    if (reverse == 1)return 1500 - difference;                                 //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
//https://youtu.be/JBvnB0279-Q
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid() {

  //Roll calculations--------------------------------------------------------------
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > PID_MAX_ROLL)
    pid_i_mem_roll = PID_MAX_ROLL;
  else if (pid_i_mem_roll < PID_MAX_ROLL * -1)
    pid_i_mem_roll = PID_MAX_ROLL * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > PID_MAX_ROLL)
    pid_output_roll = PID_MAX_ROLL;
  else if (pid_output_roll < PID_MAX_ROLL * -1)
    pid_output_roll = PID_MAX_ROLL * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations--------------------------------------------------------------
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)
    pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)
    pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)
    pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)
    pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations--------------------------------------------------------------
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > PID_MAX_YAW)
    pid_i_mem_yaw = PID_MAX_YAW;
  else if (pid_i_mem_yaw < PID_MAX_YAW * -1)
    pid_i_mem_yaw = PID_MAX_YAW * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > PID_MAX_YAW)
    pid_output_yaw = PID_MAX_YAW;
  else if (pid_output_yaw < PID_MAX_YAW * -1)
    pid_output_yaw = PID_MAX_YAW * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

