// by yui_maker
// a level pong game that uses MPU-6050 to know it's board pitch angle.
// the code for MPU-6050 pitch is taken from drone-bot workshop's code https://dronebotworkshop.com/?s=mpu

// including wire library for i2c communication
#include <Wire.h>

// Define I2C Address for MPU6050
const int i2c_addr = 0x3F;

// Libraries for matrix display
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
Adafruit_8x8matrix matrix = Adafruit_8x8matrix();

// Variables for MPU-6050 sensor values
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

// Setup temp variables
int temp;

// for adding a low-pass filter on pitch angle output values
float angle_pitch_output_new;
float angle_pitch_old;

// flag for going from game mode to waiting mode or game over mode
boolean game_on = false;

// level pong bottom bar variable
const byte pong_bar_V = 7; // vertical position of the bar
byte pong_bar_H_x;         // horizontal x position of the bar
byte pong_bar_H_y;         // horizontal y position of the bar

// variable for knowing assigning different pitch angles to different display bottom bar positions
float stable_x_p = 1.5;
float stable_x_n = -1.5;
const byte angle_value = 6;

// creating a random ball on the matrix display.
byte ball_v_pos = 0;
byte ball_h_pos = 0;

// variable for ball position update time
unsigned long ball_move_speed;

// variable for keeping score and defining game difficulty
int score = 0;
float game_difficulty;
const byte constant_game_difficulty = 12; // Once the score reaches 12 the game difficulty won't increasse anymore and stay constant

// variable for how long it takes the ball to drop down
const int total_drop_time = 180;             // when the game starts and score is zero it'll take 180ms far ball to go from one spot to next
const int total_drop_time_reduce_factor = 7; // we will reduce the time it takes for ball to drop one spot by 7ms every time score increases by one

// variable for start mode loop period
unsigned long start_loop_time;

// defining game start push button pin.
#define START_BTN 2

void setup()
{

  // Start I2C
  Wire.begin();

  // Setup the registers of the MPU-6050
  setup_mpu_6050_registers();

  // Read the raw acc and gyro data from the MPU-6050 1000 times
  for (int cal_int = 0; cal_int < 1000; cal_int++)
  {
    read_mpu_6050_data();
    // Add the gyro x offset to the gyro_x_cal variable
    gyro_x_cal += gyro_x;
    // Add the gyro y offset to the gyro_y_cal variable
    gyro_y_cal += gyro_y;
    // Add the gyro z offset to the gyro_z_cal variable
    gyro_z_cal += gyro_z;
    // Delay 3us to have 250Hz for-loop
    delay(3);
  }

  // Divide all results by 1000 to get average offset
  gyro_x_cal /= 1000;
  gyro_y_cal /= 1000;
  gyro_z_cal /= 1000;

  // setting up the matrix display
  matrix.begin(0x70); // pass in the address
  matrix.setRotation(1);

  // seeding the random so, that the ball creation is random
  randomSeed(analogRead(millis()));

  // creating the first dropping ball
  ball_h_pos = random(0, 8);

  // ball position update time
  ball_move_speed = millis();

  // start mode display update time
  start_loop_time = millis();

  // Setting Push button pins as INPUT pins with internal pullup resistors
  pinMode(START_BTN, INPUT_PULLUP);

} // closing the setup function

// Setup the registers of the MPU-6050
void setup_mpu_6050_registers()
{
  // Activate the MPU-6050

  // Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  // Send the requested starting register
  Wire.write(0x6B);
  // Set the requested starting register
  Wire.write(0x00);
  // End the transmission
  Wire.endTransmission();

  // Configure the accelerometer (+/-8g)

  // Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  // Send the requested starting register
  Wire.write(0x1C);
  // Set the requested starting register
  Wire.write(0x10);
  // End the transmission
  Wire.endTransmission();

  // Configure the gyro (500dps full scale)

  // Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  // Send the requested starting register
  Wire.write(0x1B);
  // Set the requested starting register
  Wire.write(0x08);
  // End the transmission
  Wire.endTransmission();
} // closing the setup_mpu_6050_registers function

// constants for the smiley, neutral and frown faces that will be shown on display.
// the matrix display related codes are either taken or based on Adafruit backpack library matrix 8x8 display example code
static const uint8_t PROGMEM
    smile_bmp[] =
        {B00111100,
         B01000010,
         B10100101,
         B10000001,
         B10100101,
         B10011001,
         B01000010,
         B00111100},
    neutral_bmp[] =
        {B00111100,
         B01000010,
         B10100101,
         B10000001,
         B10111101,
         B10000001,
         B01000010,
         B00111100},
    frown_bmp[] =
        {B00111100,
         B01000010,
         B10100101,
         B10000001,
         B10011001,
         B10100101,
         B01000010,
         B00111100};

// following function returns the pitch angle of the MPU-6050
float pitch_output(void)
{
  // Get data from MPU-6050
  read_mpu_6050_data();

  // Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  // Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_pitch += gyro_x * 0.0000611;
  // Calculate the traveled roll angle and add this to the angle_roll variable

  // If the IMU has yawed transfer the roll angle to the pitch angle
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);

  // Accelerometer angle calculations

  // Calculate the total accelerometer vector
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

  // 57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  // Calculate the pitch angle
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;

  // Accelerometer calibration value for pitch
  angle_pitch_acc -= 0.0;

  if (set_gyro_angles)
  {
    // If the IMU has been running
    // Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
  }
  else
  {
    // IMU has just started
    // Set the gyro pitch angle equal to the accelerometer pitch angle
    angle_pitch = angle_pitch_acc;
  }

  // To dampen the pitch and roll angles a complementary filter is used
  // Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
  angle_pitch_output_new = angle_pitch_old * 0.9 + angle_pitch_output * 0.1; // adding a low pass filter to the pitch output data we get from MPU-6050
  angle_pitch_old = angle_pitch_output_new;
  return angle_pitch_output_new;

} // closing the pitch_output function

// reading the data from MPU-6050 sensor
void read_mpu_6050_data()
{
  // Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  // Send the requested starting register
  Wire.write(0x3B);
  // End the transmission
  Wire.endTransmission();
  // Request 14 bytes from the MPU-6050
  Wire.requestFrom(0x68, 14);
  // Wait until all the bytes are received
  while (Wire.available() < 14)
    ;

  // Following statements left shift 8 bits, then bitwise OR.
  // Turns two 8-bit values into one 16-bit value
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temp = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
} // closing the read_mpu_6050_data function

// the following function will be running when we are waiting for the player to press the button and start the game
void game_start()
{
  bool face_change = true; // for going from smiley to neutral face

  while (digitalRead(START_BTN) == HIGH) // until player presses the button we will be looping here
  {
    if (millis() - start_loop_time >= 1000) // every one second (1000) we will be moving from smiley to neutral
    {
      if (face_change) // showing the smiley face
      {
        matrix.clear();
        matrix.drawBitmap(0, 0, smile_bmp, 8, 8, LED_ON);
        matrix.writeDisplay();

        face_change = false; // changing the flag to false so that next time in loop we go to else statement
      }
      else // showing the neutral face
      {
        matrix.clear();
        matrix.drawBitmap(0, 0, neutral_bmp, 8, 8, LED_ON);
        matrix.writeDisplay();

        face_change = true;
      }
      start_loop_time = millis(); // updating the time so that we go back to the loop after one second
    }
  } // closing the while loop

  // once the button will be pressed we will go to the following for loop showing a countdown from 3 to 1.
  for (int i = 3; i >= 1; i--)
  {
    // showing the start count down
    matrix.clear();
    matrix.setCursor(1, 0);
    matrix.print(i);
    matrix.writeDisplay();
    delay(600);
  }

  // after the count down we create our first ball, set the score to zero and put the game_on flage to true.
  random_ball_creator();
  score = 0;
  game_on = true;
} // closing the game_start function

// following function creates the random ball that drop down.
byte random_ball_creator()
{
  ball_v_pos = 0; // setting the ball's vertical position to zero so that we start from the top of the display
  byte old_dot_h_pos = ball_h_pos;
  // we want the new ball to be at least 2 spots away from the last ball
  while (abs(ball_h_pos - old_dot_h_pos) <= 1)
  {
    randomSeed(analogRead(millis()));
    ball_h_pos = random(0, 8);
  }
} // closing the random_ball_creator function

// following function shows the dropping ball position and the bottom bar position on the display
void matrix_display(byte pong_bar_H_x, byte pong_bar_H_y, byte pong_bar_V, byte ball_place, byte ball_move)
{
  matrix.clear();                                                              // clear whatever was on the display
  matrix.drawLine(pong_bar_H_x, pong_bar_V, pong_bar_H_y, pong_bar_V, LED_ON); // this will the bottom bar

  matrix.drawPixel(ball_place, ball_move, LED_ON); // this will display the dropping ball
  matrix.writeDisplay();                           // write the changes we just made to the display
} // closing the matrix_display function

// this function returns certain values based on the pitch angle of the MPU-6050 the return value is used
// to place the bottom bar on the display
int pong_angle()
{
  // Making the pong_pad move with the angle
  if (pitch_output() <= stable_x_n - angle_value * 2)
  {
    return 6;
  }
  else if ((pitch_output() > stable_x_n - angle_value * 2) && (pitch_output() <= stable_x_n - angle_value))
  {
    return 5;
  }
  else if ((pitch_output() > stable_x_n - angle_value) && (pitch_output() <= stable_x_n))
  {
    return 4;
  }
  else if ((pitch_output() < stable_x_p) && (pitch_output() > stable_x_n))
  {
    return 3;
  }
  else if ((pitch_output() >= stable_x_p) && (pitch_output() < stable_x_p + angle_value))
  {
    return 2;
  }
  else if ((pitch_output() >= stable_x_p + angle_value) && (pitch_output() < stable_x_p + angle_value * 2))
  {
    return 1;
  }
  else if (pitch_output() >= stable_x_p + angle_value * 2)
  {
    return 0;
  }
} // closing the pong_angle function

// this game checks if the player caught the ball or not by checking the position of the ball and the position of the bottom bar
//  if the bar is where it should be for the ball position then the score is increased by one and another ball is created
void game_check()
{
  // following if statements match the ball's horizontal position with the value returned by the pong_angle function
  if (ball_h_pos == 0 && pong_angle() == 0)
  {
    score += 1;
    random_ball_creator();
  }
  else if (ball_h_pos == 1 && (pong_angle() == 0 || pong_angle() == 1))
  {
    score += 1;
    random_ball_creator();
  }
  else if (ball_h_pos == 2 && (pong_angle() == 1 || pong_angle() == 2))
  {
    score += 1;
    random_ball_creator();
  }
  else if (ball_h_pos == 3 && (pong_angle() == 2 || pong_angle() == 3))
  {
    score += 1;
    random_ball_creator();
  }
  else if (ball_h_pos == 4 && (pong_angle() == 3 || pong_angle() == 4))
  {
    score += 1;
    random_ball_creator();
  }
  else if (ball_h_pos == 5 && (pong_angle() == 4 || pong_angle() == 5))
  {
    score += 1;
    random_ball_creator();
  }
  else if (ball_h_pos == 6 && (pong_angle() == 5 || pong_angle() == 6))
  {
    score += 1;
    random_ball_creator();
  }
  else if (ball_h_pos == 7 && pong_angle() == 6)
  {
    score += 1;
    random_ball_creator();
  }

  // if the ball's position and bottom bar position don't match the ball was not caught and game ends
  else
  {
    game_over();
  }
} // closing the game_check function

// if the ball was not caught by the bottom bar the game will end
void game_over(void)
{
  matrix.clear();
  matrix.drawBitmap(0, 0, frown_bmp, 8, 8, LED_ON);
  matrix.writeDisplay();
  delay(500);

  // showing the final score
  if (score < 10) //if the score is a one digit number
  {
    // if the score is one digit number we will just display it on the screen
    matrix.clear();
    matrix.setCursor(2, 0);
    matrix.print(score);
    matrix.writeDisplay();
    delay(1500);
  }
  else if (score < 100) // if the score is a two digit number
  {
    // if score is a double digit number it will scroll to left.
    for (int8_t x = 3; x >= -8; x--)
    {
      matrix.clear();
      matrix.setCursor(x, 0);
      matrix.print(score);
      matrix.writeDisplay();
      delay(175);
    }
    delay(100);
  }
    else  // if the score is more than two digit number. (Hopefully nobody will play long enough to get a three digit score)
  {
    // if score is a double digit number it will scroll to left.
    for (int8_t x = 3; x >= -16; x--)
    {
      matrix.clear();
      matrix.setCursor(x, 0);
      matrix.print(score);
      matrix.writeDisplay();
      delay(175);
    }
    delay(100);
  }
  game_on = false; // changing te game_on flag to false so that we go into the start mode.
} // closing game over function

void loop()
{
  if (game_on) // if this flag return true that means we are playing the game
  {
    matrix_display(pong_angle(), pong_angle() + 1, pong_bar_V, ball_h_pos, ball_v_pos); // updating the display

    // for the first time it will take 180 miliseconds for the ball to drop down. After that time will go down by 7 as score increases by one.
    if (score >= constant_game_difficulty) // the game difficulty will stay the same after player get's 12. Why 12 because 17 was my highest score...
    {
      game_difficulty = total_drop_time - constant_game_difficulty * total_drop_time_reduce_factor;
    }
    else
    {
      game_difficulty = total_drop_time - score * total_drop_time_reduce_factor;
    }

    // changing the ball position based on the game_difficulty
    if (millis() - ball_move_speed >= game_difficulty)
    {
      ball_v_pos++;
      if (ball_v_pos >= 7)
      {
        game_check();
      }
      ball_move_speed = millis();
    }
  }

  // if we are not in the playing mode we will go to the start mode
  else
  {
    game_start();
  }
} // closing the loop function
