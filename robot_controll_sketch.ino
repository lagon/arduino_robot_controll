#include <LCD_I2C.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_BMP085_U.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>

#include <PsxControllerHwSpi.h>
#include <PsxNewLib.h>

#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>

class SFR02_i2c {
private:
  int address;
  int reading_in_progress;
  unsigned long measurement_start;

  void initiate_measuring(int units);
  uint16_t read_results();
  int check_correct_results_available(int units);

public:
  int begin(int address);

  int start_measuring_cm();
  int start_measuring_inches();
  int start_measuring_us();

  int32_t get_result_cm();
  int32_t get_result_inches();
  int32_t get_result_us();
};

#define PS2_BTN_NONE (0x0000)
#define PS2_BTN_SELECT (0x0001)
#define PS2_BTN_L3 (0x0002)
#define PS2_BTN_R3 (0x0004)
#define PS2_BTN_START (0x0008)
#define PS2_BTN_PAD_UP (0x0010)
#define PS2_BTN_PAD_RIGHT (0x0020)
#define PS2_BTN_PAD_DOWN (0x0040)
#define PS2_BTN_PAD_LEFT (0x0080)
#define PS2_BTN_L2 (0x0100)
#define PS2_BTN_R2 (0x0200)
#define PS2_BTN_L1 (0x0400)
#define PS2_BTN_R1 (0x0800)
#define PS2_BTN_TRIANGLE (0x1000)
#define PS2_BTN_CIRCLE (0x2000)
#define PS2_BTN_CROSS (0x4000)
#define PS2_BTN_SQUARE (0x8000)

#define LCD_CTRL_BAT_V    (0)
#define LCD_MOTOR_BAT_V   (1)
#define LCD_MOTOR_BAT_AMP (2)
#define LCD_TEMPERATURE   (3)
#define LCD_PRESSURE      (4)
#define LCD_HEIGHT_GAIN   (5)
#define LCD_DISTANCE      (6)


// #define MOTOR_MAX_SPEED (0x1000)
#define MOTOR_SPEED_INCREMENT (0x0200)


Adafruit_INA219 ina219 = Adafruit_INA219(0x44);                // Address 0x44
Adafruit_ADS1015 ads;                                          // Address 0x48
SFR02_i2c dist;                                                // Address 0x71
LCD_I2C lcd(0x27, 16, 2);                                      // Address 0x27
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);  //Address 0x77
PsxControllerHwSpi<10> ps2_ctrl;                               //Address - N/A
Adafruit_PWMServoDriver motor_pwm(0x40);                       //Address 0x40
Adafruit_PWMServoDriver led_pwm(0x45);                         //Address 0x45


void setup_display_message(const char *msg, LCD_I2C *lcd) {
  String serial_msg = "Initialising: ";
  serial_msg = serial_msg + msg;
  Serial.println(serial_msg);
  lcd->clear();
  lcd->setCursor(0, 0);
  lcd->print("Initialising");
  lcd->setCursor(0, 1);
  lcd->print(msg);
}

void setup_display_fail(const char *msg, LCD_I2C *lcd) {
  String serial_msg = "FAILED AT: ";
  serial_msg = serial_msg + msg;
  serial_msg = serial_msg + " STOPPING FOR GOOD.";
  Serial.println(serial_msg);
  lcd->clear();
  lcd->setCursor(0, 0);
  lcd->print("!!! FAILED");
  lcd->setCursor(0, 1);
  lcd->print(msg);
}

int SFR02_i2c::begin(int address) {
  this->address = address;
  this->reading_in_progress = -1;

  Wire.beginTransmission(this->address);
  Wire.write(byte(0x00));
  Wire.endTransmission();

  Wire.requestFrom(this->address, 1);
  if (Wire.available() == 1) {
    uint8_t revision = Wire.read();
    if (revision != 0x6) {
      return 0;
    }
  } else {
    return 0;
  }

  Wire.beginTransmission(this->address);
  Wire.write(byte(0x01));
  Wire.endTransmission();
  Wire.requestFrom(this->address, 1);
  if (Wire.available() == 1) {
    uint8_t const_val = Wire.read();
    if (const_val != 0x18) {
      return 0;
    }
  } else {
    return 0;
  }

  return 1;
}

void SFR02_i2c::initiate_measuring(int units) {
  this->measurement_start = millis();
  this->reading_in_progress = units;

  Wire.beginTransmission(this->address);
  Wire.write(byte(0x00));
  Wire.write(byte(units));
  Wire.endTransmission();
}

int SFR02_i2c::start_measuring_cm() {
  if (this->reading_in_progress != -1) {
    return -1;
  }
  this->initiate_measuring(0x51);
  return this->reading_in_progress;
}

int SFR02_i2c::start_measuring_inches() {
  if (this->reading_in_progress != -1) {
    return -1;
  }
  this->initiate_measuring(0x50);
  return this->reading_in_progress;
}

int SFR02_i2c::start_measuring_us() {
  if (this->reading_in_progress != -1) {
    return -1;
  }
  this->initiate_measuring(0x52);
  return this->reading_in_progress;
}

uint16_t SFR02_i2c::read_results() {
  Wire.beginTransmission(this->address);
  Wire.write(byte(0x02));
  Wire.endTransmission();

  Wire.requestFrom(address, 2);
  uint16_t dist = 0;
  if (2 <= Wire.available())  // if two bytes were received
  {
    uint8_t v1 = Wire.read();
    uint8_t v2 = Wire.read();
    dist = (v1 << 8) + v2;
    // Serial.println(" -> " + String(dist) + " (0x" + String(dist,16) + ") = 0x" + String(v1, 16) + " 0x" + String(v2, 16));
  }
  return dist;
}

int SFR02_i2c::check_correct_results_available(int units) {
  if (this->reading_in_progress != units) {
    return -2;
  }
  if (millis() - this->measurement_start < 71) {
    return -1;
  }
  return 0;
}

int32_t SFR02_i2c::get_result_cm() {
  int avail = this->check_correct_results_available(0x51);
  if (avail < 0) {
    return avail;
  } else {
    this->reading_in_progress = -1;
    return this->read_results();
  }
}

int32_t SFR02_i2c::get_result_inches() {
  int avail = this->check_correct_results_available(0x50);
  if (avail < 0) {
    return avail;
  } else {
    this->reading_in_progress = -1;
    return this->read_results();
  }
}

int32_t SFR02_i2c::get_result_us() {
  int avail = this->check_correct_results_available(0x52);
  if (avail < 0) {
    return avail;
  } else {
    this->reading_in_progress = -1;
    return this->read_results();
  }
}

long loop_start;

long ps2_read_last_time = 0;
PsxButtons ps2_buttons;
uint16_t max_speed_factor = 1;
#define PS2_READ_INTERVAL (50)

long motor_control_last_time = 0;
uint16_t left_motor_speed = 0;
uint16_t right_motor_speed = 0;
int8_t left_motor_direction = 0;
int8_t right_motor_direction = 0;
#define MOTOR_CONTROL_INTERVAL (50)

long hw_button_controll_last_time = 0;
uint8_t hw_button_state = 0;
#define HW_BUTTON_CONTROL_INTERVAL (100)

long display_controll_last_time = 0;
uint8_t display_info_shown = 0;
#define DISPLAY_CONTROL_INTERVAL (250)

long ina219_read_last_time = 0;
float motor_battery_voltage_V;
float motor_battery_current_mA;
#define INA219_READ_INTERVAL (200)

long ads1015_read_last_time = 0;
float ctrl_battery_voltage_V;
#define ADS1015_READ_INTERVAL (1000)

long bmp180_read_last_time = 0;
float bmp180_initial_pressure = 0;
float bmp180_current_pressure = 0;
float bmp180_current_temperature = 0;
float bmp180_altitude_change = 0;
#define BMP180_READ_INTERVAL (1000)

long battery_level_update_last_time = 0;
uint8_t controll_battery_voltage_level = 0;
uint8_t motor_battery_voltage_level = 0;
uint8_t controll_battery_previous_voltage_level = -1;
uint8_t motor_battery_previous_voltage_level = -1;
#define BATTERY_LEVEL_UPDATE_INTERVAL (1000)

long dist_measure_update_last_time = 0;
uint8_t dist_measure_is_measuring = 0;
int dist_measure_distance_cm = 0;
#define DIST_MEASURE_AVGERAGING_LEN (3)
int dist_measure_past_distances[DIST_MEASURE_AVGERAGING_LEN];
int dist_measure_past_dist_idx = 0;
int dist_measure_averaged_distance = 0;
#define DIST_MEASURE_UPDATE_INTERVAL (100)


uint8_t led_brakes = 0;
uint8_t led_headlights = 0;


uint16_t motor_max_speeds[3] = {0x1000, 0x800, 0x500};
int8_t motor_final_max_spd_idx = 0;
int8_t motor_max_spd_idx = 0;
int8_t motor_dist_limit_max_spd_idx = 0;
int8_t motor_dist_limit_max_spd_idx_previous_level = -1;

uint8_t battery_level_characters[6][8] = {{
      0b01110,
      0b11111,
      0b11111,
      0b11111,
      0b11111,
      0b11111,
      0b11111,
      0b11111
  }, {
      0b01110,
      0b11111,
      0b10001,
      0b11111,
      0b11111,
      0b11111,
      0b11111,
      0b11111
  }, {
      0b01110,
      0b11111,
      0b10001,
      0b10001,
      0b11111,
      0b11111,
      0b11111,
      0b11111
  }, {
      0b01110,
      0b11111,
      0b10001,
      0b10001,
      0b10001,
      0b11111,
      0b11111,
      0b11111
  }, {
      0b01110,
      0b11111,
      0b10001,
      0b10001,
      0b10001,
      0b10001,
      0b11111,
      0b11111
  }, {
      0b01110,
      0b11111,
      0b10001,
      0b10001,
      0b10001,
      0b10001,
      0b10001,
      0b11111
}};

uint8_t motor_dist_max_spd_char[3][8] = {{
  0b00000,
  0b00100,
  0b01010,
  0b10101,
  0b01010,
  0b10101,
  0b01010,
  0b10001
}, {
  0b00000,
  0b00000,
  0b00000,
  0b00100,
  0b01010,
  0b10101,
  0b01010,
  0b10001
  }, {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00100,
  0b01010,
  0b10001
}};


uint16_t led_headlights_intensity[5] = {
  0x1000, 0x0DDD, 0x0AAA, 0x0555, 0x0000
};


//RGB - Inversed (0x1000 OFF; 0x000 ON)
uint16_t led_batery_levels[6][3] = {
  {0x1000, 0x1000, 0x0000},
  {0x1000, 0x0000, 0x1000},
  {0x0A00, 0x0000, 0x1000},
  {0x0000, 0x0000, 0x1000},
  {0x0000, 0x0A00, 0x1000},
  {0x0000, 0x1000, 0x1000}
};

void setup() {
  // First Initialise serial line
  Serial.begin(115200);
  Serial.println("Serial line initialised");
  delay(100);

  //Start LCD Display and show message
  lcd.begin();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("LCD Initialised");
  delay(100);

  //Initialise ADS1015
  setup_display_message("ADS1015", &lcd);
  ads.begin();
  if (!ads.begin()) {
    setup_display_fail("ADS1015", &lcd);
    while (1)
      ;
  }
  ads.setGain(GAIN_TWOTHIRDS);
  delay(100);

  //Initislise BMP180
  setup_display_message("BMP180", &lcd);
  if (!bmp.begin()) {
    setup_display_fail("BMP180", &lcd);
    while (1)
      ;
  }
  bmp.getPressure(&bmp180_initial_pressure);
  delay(100);

  setup_display_message("INA21", &lcd);
  ina219.begin();
  delay(100);

  //Initialise SFR02 (Distance finder)
  setup_display_message("distance", &lcd);
  Wire.begin();
  if (!dist.begin(0x71)) {
    setup_display_fail("distance", &lcd);
    while (1)
      ;
  }
  dist.start_measuring_cm();
  delay(100);
  Serial.println(" Dist: " + String(dist.get_result_cm()) + "cm");
  delay(100);

  setup_display_message("Controller", &lcd);
  uint8_t fail_idx = 0;
  uint8_t fail_wait_chars [4] = {' ', '.', 'o', 'O'};
  while(!ps2_ctrl.begin()) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("CTRL FAIL!");
    lcd.setCursor(0, 1); lcd.print("RETRY IN A SEC" + String(fail_wait_chars[fail_idx]));
    Serial.print("!");
    fail_idx = (fail_idx + 1) % 4;
    delay(1000);
  }

  if (!ps2_ctrl.enterConfigMode()) {
    Serial.println(F("Cannot enter config mode"));
  } else {
    PsxControllerType ctype = ps2_ctrl.getControllerType();
    Serial.println(ctype);
    // PGM_BYTES_P cname = reinterpret_cast<PGM_BYTES_P> (pgm_read_ptr (&(controllerTypeStrings[ctype < PSCTRL_MAX ? static_cast<byte> (ctype) : PSCTRL_MAX])));
    // Serial.print (F("Controller Type is: "));
    // Serial.println (PSTR_TO_F (cname));

    if (!ps2_ctrl.enableAnalogSticks()) {
      Serial.println(F("Cannot enable analog sticks"));
    }

    if (!ps2_ctrl.enableAnalogButtons()) {
      Serial.println(F("Cannot enable analog buttons"));
    }

    if (!ps2_ctrl.exitConfigMode()) {
      Serial.println(F("Cannot exit config mode"));
    }
  }
  delay(100);


  //Moter + LED PWM
  setup_display_message("LEDs+Motor", &lcd);
  if (!motor_pwm.begin()) {
    setup_display_fail("Motor", &lcd);
    while (1)
      ;
  }
  if (!led_pwm.begin()) {
    setup_display_fail("LED", &lcd);
    while (1)
      ;
  }
  motor_pwm.setOscillatorFrequency(27000000);
  motor_pwm.setPWMFreq(1600);
  led_pwm.setOscillatorFrequency(27000000);
  led_pwm.setPWMFreq(1600);
  led_pwm.setOutputMode(false);

  delay(100);

  setup_display_message("Lights", &lcd);
  for (int lvl = 0; lvl < 6; lvl++) {
    Serial.println("13: 0x" + String(led_batery_levels[lvl][0], 16));
    led_pwm.setPin(13, led_batery_levels[lvl][0]); //RED
    led_pwm.setPin(14, led_batery_levels[lvl][1]); //GREEN
    led_pwm.setPin(15, led_batery_levels[lvl][2]); //BLUE

    led_pwm.setPin(10, led_batery_levels[lvl][0]);
    led_pwm.setPin(11, led_batery_levels[lvl][1]);
    led_pwm.setPin(12, led_batery_levels[lvl][2]);
    delay(250);
  }

  led_pwm.setPin(4, 0x1000);
  led_pwm.setPin(7, 0x1000);
  led_pwm.setPin(8, 0x1000);
  delay(250);
  led_pwm.setPin(4, 0x0000);
  led_pwm.setPin(7, 0x0000);
  led_pwm.setPin(8, 0x0000);
  delay(250);
  led_pwm.setPin(4, 0x1000);
  led_pwm.setPin(7, 0x1000);
  led_pwm.setPin(8, 0x1000);
  delay(250);
  led_pwm.setPin(4, 0x0000);
  led_pwm.setPin(7, 0x0000);
  led_pwm.setPin(8, 0x0000);
  delay(250);
  setup_display_message("Finished OK", &lcd);
  
  delay(1000);
}

uint8_t *get_battery_level_char(uint8_t battery_level) {
  battery_level = battery_level % 6;
  return battery_level_characters[battery_level];
}


void advance_display_info() {
  display_info_shown = display_info_shown + 1;
  if (display_info_shown > 6) {
    display_info_shown = 0;
  }
}


void copy_line_data(const char *title, String value, const char *units, char *line1, char *line2) {
  uint8_t l1_len = min(strlen(title), 12);
  memcpy(line1, title, l1_len);

  String l2_str = value + " " + String(units);
  uint8_t l2_len = min(l2_str.length(), 12);
  memcpy(line2 + (12 - l2_len), l2_str.c_str(), l2_len);
}

void display_information() {
  char l1[17], l2[17];
  strncpy(l1, "                ", 16);
  strncpy(l2, "                ", 16);

  if (display_info_shown == LCD_CTRL_BAT_V) {
    copy_line_data("BATTERY CTL", String(ctrl_battery_voltage_V, 1), "V", l1, l2);
  } else if (display_info_shown == LCD_MOTOR_BAT_V) {
    copy_line_data("BATTERY MOT", String(motor_battery_voltage_V, 1), "V", l1, l2);
  } else if (display_info_shown == LCD_MOTOR_BAT_AMP) {
    copy_line_data("CURRENT MOT", String(motor_battery_current_mA, 1), "mA", l1, l2);
  } else if (display_info_shown == LCD_TEMPERATURE) {
    copy_line_data("TEMPERATURE", String(bmp180_current_temperature, 1), "C", l1, l2);
  } else if (display_info_shown == LCD_PRESSURE) {
    copy_line_data("PRESSURE", String(bmp180_current_pressure, 1), "hPa", l1, l2);
  } else if (display_info_shown == LCD_HEIGHT_GAIN) {
    copy_line_data("HEIGHT GAIN", String(bmp180_altitude_change, 1), "m", l1, l2);
  } else if (display_info_shown == LCD_DISTANCE) {
    copy_line_data("HEAD DIST", String(dist_measure_distance_cm), " cm", l1, l2);
  } else {
    copy_line_data("NO DIST", "??", "", l1, l2);
  }

  if (controll_battery_voltage_level != controll_battery_previous_voltage_level) {
    lcd.createChar(1, get_battery_level_char(controll_battery_voltage_level));
    controll_battery_previous_voltage_level = controll_battery_voltage_level;
    Serial.println("CTRL Battery update");
  }
  if (motor_battery_voltage_level != motor_battery_previous_voltage_level) {
    lcd.createChar(2, get_battery_level_char(motor_battery_voltage_level));
    motor_battery_previous_voltage_level = motor_battery_voltage_level;
    Serial.println("Motor Battery update");
  }

  if (motor_final_max_spd_idx != motor_dist_limit_max_spd_idx_previous_level) {
    lcd.createChar(3, motor_dist_max_spd_char[motor_final_max_spd_idx]);
    motor_dist_limit_max_spd_idx_previous_level = motor_final_max_spd_idx;
    Serial.println("Motor Battery update");
  }

  l1[15] = 0x1;
  l2[15] = 0x2;
  l1[14] = 0x3;

  lcd.setCursor(0, 0);
  lcd.print(l1);
  lcd.setCursor(0, 1);
  lcd.print(l2);

  // lcd.setCursor(15, 0);
  // lcd.write(0);
  // lcd.setCursor(15, 1);
  // lcd.write(1);
}


void update_motor_speed(int8_t *motor_direction, uint16_t *motor_speed, int8_t desired_direction, uint16_t max_speed, uint16_t speed_change) {
  // Serial.println(" ---> " + String(*motor_direction) + " >> " + String(*motor_speed) + ":" + String(max_speed - speed_change - 1) + "    + " + String(desired_direction) + " | " + String(max_speed) + " | " + String(speed_change));
  if (*motor_direction == 0) {
    *motor_direction = desired_direction;
    *motor_speed = 0x0000;
  } else if (*motor_direction == desired_direction) {
    if (*motor_speed > max_speed - speed_change - 1) {
      *motor_speed = max_speed;
    } else {
      *motor_speed += speed_change;
    }
  } else {
    if (*motor_speed < speed_change) {
      *motor_speed = 0;
      *motor_direction = 0;
    } else {
      *motor_speed -= speed_change;
    }
  }
  // Serial.println(""); //" ======> " + String(*motor_direction) + " >> " + String(*motor_speed) + "    + " + String(desired_direction) + " | " + String(max_speed) + " | " + String(speed_change));
}

void set_motor_speed(int8_t motor_direction, uint16_t motor_speed, uint8_t fwd_pin, uint8_t bck_pin, uint8_t spd_pin) {
  if (motor_direction == 1) {
    motor_pwm.setPin(fwd_pin, 0xFFFF);
    motor_pwm.setPin(bck_pin, 0);
    motor_pwm.setPin(spd_pin, motor_speed);
  } else if (motor_direction == -1) {
    motor_pwm.setPin(fwd_pin, 0);
    motor_pwm.setPin(bck_pin, 0xFFFF);
    motor_pwm.setPin(spd_pin, motor_speed);
  } else {
    motor_pwm.setPin(fwd_pin, 0);
    motor_pwm.setPin(bck_pin, 0);
    motor_pwm.setPin(spd_pin, 0);
  }
}

void updated_leds(uint8_t controll_battery_voltage_level, uint8_t motor_battery_voltage_level, uint8_t led_headlights, uint8_t led_brakes) {
  led_pwm.setPin(13, led_batery_levels[controll_battery_voltage_level][0]);
  led_pwm.setPin(14, led_batery_levels[controll_battery_voltage_level][1]);
  led_pwm.setPin(15, led_batery_levels[controll_battery_voltage_level][2]);

  led_pwm.setPin(10, led_batery_levels[motor_battery_voltage_level][0]);
  led_pwm.setPin(11, led_batery_levels[motor_battery_voltage_level][1]);
  led_pwm.setPin(12, led_batery_levels[motor_battery_voltage_level][2]);

  led_pwm.setPin(4, led_headlights_intensity[led_headlights]);
  led_pwm.setPin(7, led_brakes == 1 ? 0x0000 : 0x1000);
}

void loop() {

  loop_start = millis();

  //==============================
  //==============================
  // DATA READ SECTION
  //==============================
  //==============================

  if (loop_start - ps2_read_last_time > PS2_READ_INTERVAL) {
    ps2_ctrl.read();
    ps2_buttons = ps2_ctrl.getButtonWord();

    if (ps2_ctrl.buttonJustPressed(PSB_CROSS)) {
      advance_display_info();
    }

    if (ps2_ctrl.buttonJustPressed(PSB_CIRCLE)) {
      led_headlights = (led_headlights + 1) % 5;
    }

    if (ps2_ctrl.buttonJustPressed(PSB_PAD_DOWN)) {
      motor_max_spd_idx = motor_max_spd_idx + 1;
      if (motor_max_spd_idx > 2) {
        motor_max_spd_idx = 2;
      }
    }
    if (ps2_ctrl.buttonJustPressed(PSB_PAD_UP)) {
      motor_max_spd_idx = motor_max_spd_idx - 1;
      if (motor_max_spd_idx < 0) {
        motor_max_spd_idx = 0;
      }
    }

    ps2_read_last_time = millis();
  }



  if (loop_start - hw_button_controll_last_time > HW_BUTTON_CONTROL_INTERVAL) {
    if (ads.computeVolts(ads.readADC_SingleEnded(3)) < 1) {
      hw_button_state = hw_button_state + 1;
      if (hw_button_state > 5) {
        hw_button_state = 0;
        advance_display_info();
      }
    } else {
      hw_button_state = 0;
    }
    hw_button_controll_last_time = millis();
  }

  if (loop_start - bmp180_read_last_time > BMP180_READ_INTERVAL) {
    bmp.getPressure(&bmp180_current_pressure);
    bmp.getTemperature(&bmp180_current_temperature);
    bmp180_altitude_change = bmp.pressureToAltitude(bmp180_initial_pressure, bmp180_current_pressure);
    bmp180_read_last_time = millis();
  }

  if (loop_start - ina219_read_last_time > INA219_READ_INTERVAL) {
    motor_battery_voltage_V = ina219.getBusVoltage_V();
    motor_battery_current_mA = ina219.getCurrent_mA();
    ina219_read_last_time = millis();
  }

  if (loop_start - ads1015_read_last_time > ADS1015_READ_INTERVAL) {
    ctrl_battery_voltage_V = ads.computeVolts(ads.readADC_SingleEnded(0));
    ads1015_read_last_time = millis();
  }

  if (loop_start - dist_measure_update_last_time > DIST_MEASURE_UPDATE_INTERVAL) {
    if (dist_measure_is_measuring == 1) {
      dist_measure_is_measuring = 0;
      dist.start_measuring_cm();
    } else {
      dist_measure_distance_cm = dist.get_result_cm();
      // Serial.println("Distance " + String(dist_measure_distance_cm) + "cm");
      dist_measure_is_measuring = 1;

      dist_measure_past_distances[dist_measure_past_dist_idx] = dist_measure_distance_cm;
      dist_measure_past_dist_idx = (dist_measure_past_dist_idx + 1) % DIST_MEASURE_AVGERAGING_LEN;
      dist_measure_averaged_distance = 0;
      Serial.println();
      for (int i = 0; i < DIST_MEASURE_AVGERAGING_LEN; i++) {
        dist_measure_averaged_distance = dist_measure_averaged_distance + dist_measure_past_distances[i];
        Serial.print(" " + String(dist_measure_past_distances[i]));
      }
      Serial.print(" ==> " + String(dist_measure_averaged_distance));
      dist_measure_averaged_distance = dist_measure_averaged_distance / DIST_MEASURE_AVGERAGING_LEN;
      Serial.println(" ==> " + String(dist_measure_averaged_distance));

    }
    dist_measure_update_last_time = millis();
  }


  //==============================
  //==============================
  //  Logic Section
  //==============================
  //==============================

  if (dist_measure_averaged_distance < 30) {
    motor_dist_limit_max_spd_idx = 2;
  } else if (dist_measure_averaged_distance < 50) {
    motor_dist_limit_max_spd_idx = 1;    
  } else {
    motor_dist_limit_max_spd_idx = 0;
  }

  
  if (loop_start - motor_control_last_time > MOTOR_CONTROL_INTERVAL) {
    motor_final_max_spd_idx = max(motor_dist_limit_max_spd_idx, motor_max_spd_idx);
    // Serial.println("DISTANCE LIMIT: " + String(motor_dist_limit_max_spd_idx) + "  MANUAL DRIVE LIMIT: " + String(motor_max_spd_idx) + " => " + String(max(motor_dist_limit_max_spd_idx, motor_max_spd_idx)) + "(" + String(motor_final_max_spd_idx) + ")" + " ->" + String(motor_max_speeds[motor_final_max_spd_idx]));

    if (ps2_buttons & PS2_BTN_L1) {
      update_motor_speed(&left_motor_direction, &left_motor_speed, 1, motor_max_speeds[motor_final_max_spd_idx], MOTOR_SPEED_INCREMENT);
    } else if (ps2_buttons & PS2_BTN_L2) {
      update_motor_speed(&left_motor_direction, &left_motor_speed, -1, motor_max_speeds[motor_final_max_spd_idx], MOTOR_SPEED_INCREMENT);
    } else {
      update_motor_speed(&left_motor_direction, &left_motor_speed, 0, motor_max_speeds[motor_final_max_spd_idx], MOTOR_SPEED_INCREMENT);
    }

    if (ps2_buttons & PS2_BTN_R1) {
      update_motor_speed(&right_motor_direction, &right_motor_speed, 1, motor_max_speeds[motor_final_max_spd_idx], MOTOR_SPEED_INCREMENT);
    } else if (ps2_buttons & PS2_BTN_R2) {
      update_motor_speed(&right_motor_direction, &right_motor_speed, -1, motor_max_speeds[motor_final_max_spd_idx], MOTOR_SPEED_INCREMENT);
    } else {
      update_motor_speed(&right_motor_direction, &right_motor_speed, 0, motor_max_speeds[motor_final_max_spd_idx], MOTOR_SPEED_INCREMENT);
    }
    motor_control_last_time = millis();

    // Update braking lights
    if (ps2_buttons & (PS2_BTN_L1 | PS2_BTN_L2 | PS2_BTN_R1 | PS2_BTN_R2)) {
      led_brakes = 0;
    } else {
      led_brakes = 1;
    }
  }

  if (loop_start - battery_level_update_last_time > BATTERY_LEVEL_UPDATE_INTERVAL) {
    if (ctrl_battery_voltage_V > 4) {
      controll_battery_voltage_level = 0;
    } else if (ctrl_battery_voltage_V > 3.9) {
      controll_battery_voltage_level = 1;
    } else if (ctrl_battery_voltage_V > 3.8) {
      controll_battery_voltage_level = 2;
    } else if (ctrl_battery_voltage_V > 3.7) {
      controll_battery_voltage_level = 3;
    } else if (ctrl_battery_voltage_V > 3.5) {
      controll_battery_voltage_level = 4;
    } else {
      controll_battery_voltage_level = 5;  
    }

    if (motor_battery_voltage_V > 8) {
      motor_battery_voltage_level = 0;
    } else if (motor_battery_voltage_V > 7.8) {
      motor_battery_voltage_level = 1;
    } else if (motor_battery_voltage_V > 7.6) {
      motor_battery_voltage_level = 2;
    } else if (motor_battery_voltage_V > 7.4) {
      motor_battery_voltage_level = 3;
    } else if (motor_battery_voltage_V > 7) {
      motor_battery_voltage_level = 4;
    } else {
      motor_battery_voltage_level = 5;
    }
  }

  //==============================
  //==============================
  //  Output Section
  //==============================
  //==============================

  if (loop_start - display_controll_last_time > DISPLAY_CONTROL_INTERVAL) {
    display_information();
    display_controll_last_time = millis();
  }
  
  updated_leds(controll_battery_voltage_level, motor_battery_voltage_level, led_headlights, led_brakes);

  set_motor_speed(left_motor_direction, left_motor_speed, 1, 0, 2);
  set_motor_speed(right_motor_direction, right_motor_speed, 4, 3, 5);
}



