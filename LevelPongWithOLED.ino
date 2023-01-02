// by yui_maker
// this code works just fine with OLED display.

// Include Wire Library for I2C
#include <Wire.h>

// Define I2C Address for MPU6050
const int i2c_addr = 0x3F;

// Libraries for matrix display
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

// Libraries for OLED display
Adafruit_8x8matrix matrix = Adafruit_8x8matrix();
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 32    // OLED display height, in pixels
#define OLED_RESET 4        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Variables for Gyroscope
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

// Setup timers and temp variables
int temp;

// pitch angle outputs trying to soften
float angle_pitch_output_new;
float angle_pitch_old;

// game variable
boolean game_on = false;

// pong bar variable
const byte pong_bar_V = 7; // vertical position of the pong bar
byte pong_bar_H_x;         // horizontal x position of the pong bar
byte pong_bar_H_y;         // horizontal y position of the pong bar

// pong balanced variable
float stable_x_p = 1.5;
float stable_x_n = -1.5;
byte tilt_value = 6;

// creating a random dot
byte dot_v_pos = 0;
byte dot_h_pos = 0;

// variable for dot position update time
unsigned long dot_move_speed;

// variable for keep score
int score = 0;

// variable for start mode loop
unsigned long start_loop_time;

// defining start push button pin.
#define START_BTN 2

void setup()
{

  // Start I2C
  Wire.begin();

  // waiting for oled display to begin
  //  SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  display.setTextColor(SSD1306_WHITE);

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

  // Start Serial Monitor
  Serial.begin(115200);

  // Display setup
  matrix.begin(0x70); // pass in the address
  matrix.setRotation(1);

  // seeding the random so, that it is not always the same pattern
  randomSeed(millis);

  // creating first dot
  randomSeed(analogRead(millis()));
  dot_h_pos = random(0, 8);

  // dot position update time
  dot_move_speed = millis();

  // start mode display update time
  start_loop_time = millis();

  // Setting Push button pins as INPUT pins with internal pullup resistors
  pinMode(START_BTN, INPUT_PULLUP);
}

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

float pitch_output(void)
{
  // Get data from MPU-6050
  read_mpu_6050_data();

  // Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  // Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)

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
  angle_pitch_output_new = angle_pitch_old * 0.9 + angle_pitch_output * 0.1;
  angle_pitch_old = angle_pitch_output_new;
  return angle_pitch_output_new;
}

void matrix_display(byte pong_bar_H_x, byte pong_bar_H_y, byte pong_bar_V, byte dot_place, byte move_on)
{
  matrix.clear();
  matrix.drawLine(pong_bar_H_x, pong_bar_V, pong_bar_H_y, pong_bar_V, LED_ON); // this will display the ball

  matrix.drawPixel(dot_place, move_on, LED_ON); // this will display the pad
  matrix.writeDisplay();                        // write the changes we just made to the display
}

void game_over(void)
{
  Serial.println(score);
  matrix.clear();
  matrix.drawBitmap(0, 0, frown_bmp, 8, 8, LED_ON);
  matrix.writeDisplay();

  // showing the start count down
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(15, 14);
  display.println("Score: " + String(score));
  display.display();
  delay(2 * 1000);
  game_on = false;
}

void updateScore(void)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 18);
  display.println("Your score: ");
  display.setTextSize(2);
  display.setCursor(80, 14);
  display.println(score);
  display.display();
} // closing the updateScore function

void game_check()
{
  if (dot_h_pos == 0 && pong_angle() == 0)
  {
    Serial.println(String(dot_h_pos) + " pitch should have been beyond 8");

    score += 1;
    updateScore();
    random_dot_creater();
  }
  else if (dot_h_pos == 1 && (pong_angle() == 0 || pong_angle() == 1))
  {
    Serial.println(String(dot_h_pos) + " pitch should have been beyond 4.5");
    score += 1;
    updateScore();
    random_dot_creater();
  }
  else if (dot_h_pos == 2 && (pong_angle() == 1 || pong_angle() == 2))
  {
    Serial.println(String(dot_h_pos) + " pitch should have been between 1.5 and 8");
    score += 1;
    updateScore();
    random_dot_creater();
  }
  else if (dot_h_pos == 3 && (pong_angle() == 2 || pong_angle() == 3))
  {
    Serial.println(String(dot_h_pos) + " pitch should have been between -1.5 and 4.5");
    score += 1;
    updateScore();
    random_dot_creater();
  }
  else if (dot_h_pos == 4 && (pong_angle() == 3 || pong_angle() == 4))
  {
    Serial.println(String(dot_h_pos) + " pitch should have been between -1.5 and -4.5");
    score += 1;
    updateScore();
    random_dot_creater();
  }
  else if (dot_h_pos == 5 && (pong_angle() == 4 || pong_angle() == 5))
  {
    Serial.println(String(dot_h_pos) + " pitch should have been between -1.5 and -8");
    score += 1;
    updateScore();
    random_dot_creater();
  }
  else if (dot_h_pos == 6 && (pong_angle() == 5 || pong_angle() == 6))
  {
    Serial.println(String(dot_h_pos) + " pitch should have been beyond -4.5");
    score += 1;
    updateScore();
    random_dot_creater();
  }
  else if (dot_h_pos == 7 && pong_angle() == 6)
  {
    Serial.println(String(dot_h_pos) + " pitch should have been beyond -8");
    score += 1;
    updateScore();
    random_dot_creater();
  }

  else
  {
    game_over();
  }
}

byte random_dot_creater()
{
  dot_v_pos = 0;
  byte old_dot_h_pos = dot_h_pos;
  Serial.println("we are tryting to create a new dot");

  while (abs(dot_h_pos - old_dot_h_pos) <= 1)
  {
    randomSeed(analogRead(millis()));
    dot_h_pos = random(0, 8);
  }
}

void game_start()
{
  bool face_change = true;
  while (digitalRead(START_BTN) == HIGH)
  {
    if (millis() - start_loop_time >= 1000)
    {
      if (face_change)
      {
        matrix.clear();
        matrix.drawBitmap(0, 0, smile_bmp, 8, 8, LED_ON);
        matrix.writeDisplay();

        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.println("Press BTN to play!");
        display.display();
        face_change = false;
      }
      else
      {
        matrix.clear();
        matrix.drawBitmap(0, 0, neutral_bmp, 8, 8, LED_ON);
        matrix.writeDisplay();

        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 15);
        display.println("Let's Play");
        display.display();
        face_change = true;
      }
      start_loop_time = millis();
    }
  }
  for (int i = 3; i >= 1; i--)
  {
    // showing the start count down
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(25, 4);
    display.println(F("Let's start!"));
    display.setTextSize(2);
    display.setCursor(55, 17);
    display.println(i);
    display.display();
    delay(500);
  }
  random_dot_creater();
  score = 0;
  game_on = true;
}

void loop()
{
  if (game_on)
  {
    matrix_display(pong_angle(), pong_angle() + 1, pong_bar_V, dot_h_pos, dot_v_pos);

    if (millis() - dot_move_speed >= 250 - score * 5)
    {
      dot_v_pos++;
      if (dot_v_pos >= 7)
      {
        game_check();
      }
      dot_move_speed = millis();
    }
  }
  else
  {
    game_start();
  }
}

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
}

void read_mpu_6050_data()
{
  // Read the raw gyro and accelerometer data

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
}

int pong_angle()
{
  // Making the pong_pad move with the angle
  if (pitch_output() <= stable_x_n - tilt_value * 2)
  {
    return 6;
  }
  else if ((pitch_output() > stable_x_n - tilt_value * 2) && (pitch_output() <= stable_x_n - tilt_value))
  {
    return 5;
  }
  else if ((pitch_output() > stable_x_n - tilt_value) && (pitch_output() <= stable_x_n))
  {
    return 4;
  }
  else if ((pitch_output() < stable_x_p) && (pitch_output() > stable_x_n))
  {
    return 3;
  }
  else if ((pitch_output() >= stable_x_p) && (pitch_output() < stable_x_p + tilt_value))
  {
    return 2;
  }
  else if ((pitch_output() >= stable_x_p + tilt_value) && (pitch_output() < stable_x_p + tilt_value * 2))
  {
    return 1;
  }
  else if (pitch_output() >= stable_x_p + tilt_value * 2)
  {
    return 0;
  }
}
