#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WiFiClient.h>
#include <uri/UriBraces.h>
#include <uri/UriRegex.h>
#include "index.h"
// #include <HCSR04.h>
#include "CytronMotorDriver.h"
#include <ArduinoOTA.h>
#include <TM1637.h>
#include <RCWL_1X05.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>

#define DEBUG

/* WiFi configuration */
#ifndef STASSID
#define STASSID "SSID of the WLAN router" // PUT YOUR "WIFI NAME" HERE0
#define STAPSK "passphrase"               // PUT YOUR WIFI PASSWORD HERE
#define OPT_HOSTNAME "table"              // Optional hostname
#endif
const char *SSID = STASSID;
const char *PASSWORD = STAPSK;
const char *HOSTNAME = OPT_HOSTNAME;

/* Pin configuration */

// esp32 wroom 32
// #define MOTOR_DRIVER_PWM 13 // pwm pin for the motor driver board
// #define MOTOR_DRIVER_DIR 12 // direction pin for the motor driver board
// #define I2C_SDA 34 // i2c - sda Pin
// #define I2C_SCL 32 // i2c - scl Pin
// #define BUTTON_PIN 33
// #define RELAY_PIN 25
// #define LCD_DIO 26
// #define LCD_CLK 27

// esp32-c3
#define MOTOR_DRIVER_PWM 2 // pwm pin for the motor driver board
#define MOTOR_DRIVER_DIR 3 // direction pin for the motor driver board
#define I2C_SDA 9          // i2c - sda Pin
#define I2C_SCL 10         // i2c - scl Pin
#define BUTTON_PIN 0
#define RELAY_PIN 5
#define LCD_DIO 6
#define LCD_CLK 7

/* Motor configuration */
#define MAX_MOTOR_SPEED 255 // speed of the motor from 0-255
#define MIN_MOTOR_SPEED 128 // minimum speed of the motor

/* Maximum and minimum height of the table in mm */
const unsigned int MAX_HEIGHT = 1180;            /* 1200 mm is the offical maximum height from the IKEA manual */
const unsigned int MIN_HEIGHT = 700;             /* 700 mm is the offical minimum height from the IKEA manual */
const unsigned int HEIGHT_DIFFERENCE_TOF = 40;   /* height difference for the VL53L0X sensor, this is the offset for the main sensor */
const unsigned int HEIGHT_DIFFERENCE_RWCL = 157; /* height difference for the RCWL-1X05 sensor, this is the offset for the backup sensor */
const unsigned int MAX_SENSOR_DIFFERENCE = 250;  /* maximum difference between the two sensors in mm, if the difference is bigger, then error is thrown and the system will stop */

/* Height tolerance (in mm) which is needed because the ultrasonic sensor is not really accurate */
const unsigned HEIGHT_TOLERANCE = 2;
const unsigned int MOTOR_SLOW_DOWN_CUSTOM_HEIGHT = 15;

/* Slow down motor at the beginning to avoid a sudden start. */
const unsigned int MOTOR_SLOW_DOWN_INTERVAL = 130; // time beetwen move orders in ms to count as a sudden start, set this to lowest possible value
const unsigned int MOTOR_SLOW_DOWN_TIME = 800;     // time to reach max speed in ms

/* Motor protection values for detecting if the motor is stuck */
const int MOTOR_PROTECTION_DISTANCE = 14;   // minimal distance change in mm
const int MOTOR_PROTECTION_INTERVAL = 3200; // interval in ms

const int NOTIFICATION_TIME = 3600; // time in ms for the notification to be displayed

const int RELAY_INTERVAL = 30000; // interval in ms for the relay to be turned off

int rwclMeasurement = 0; // variable to hold the last measurement of the RCWL-1X05 sensor
int tofMeasurement = 0;  // variable to hold the last measurement of the VL53L0X sensor

WebServer server(80); // use port 80

/* Configure the motor driver */
CytronMD motor(PWM_DIR, MOTOR_DRIVER_PWM, MOTOR_DRIVER_DIR);

/* Configure ultrasonic sensor */
// HCSR04 hc(I2C_SCL, I2C_SDA); // initialisation HCSR04 (ultrasonic sensor) (trig pin , echo pin)
RCWL_1X05 rcwlSensor;

/* Configure laser sensor */
Adafruit_VL53L0X tofSensor = Adafruit_VL53L0X();

/* Configure the 4-digit 7-segment display */
TM1637 tm(LCD_CLK, LCD_DIO);

/* States of the system */
typedef enum
{
  UP,           // table is supposed to go up
  DOWN,         // table is supposed to go down
  HOLD,         // table is supposed to do nothing -> hold still
  CUSTOM_HEIGHT // table goes up/down and holds as soon as it reached the custom height
} state_t;

/* Global state of the system. In HOLD by default -> motor will not move in this state */
state_t g_system_state = HOLD; //
bool g_button_pressed = false;

/* Global variable which shall hold the wanted custom height when requested */
int g_custom_height;
bool g_last_direction;

String notify_text = "meow";
unsigned long notify_time = 0;

unsigned long last_known_height_change = 0;

/* Function prototypes */
void display_index();
void send_homepage_redirection();
void handle_motor_requests();
void handle_height_requests();
void handle_read_height_requests();
int get_current_height();
void move_table(bool up);
void stop_table();
void handle_output();
void handleButtons();
void setup_pins();
void setup_wifi();
void print_connection_info();
void register_server_routes();
void motor_protection();
void handle_lcd();
boolean powerMgmt();
void setupOTA();
void setup_sensors();

/*
 * Displays the index/main page in the browser of the client
 */
void display_index()
{
  String s = MAIN_page;             // read HTML contents from the MAIN_page variable which was stored in the flash (program) memory instead of SRAM, see index.h
  server.send(200, "text/html", s); // send web page
}

/*
 * The server sends a redirection response to the client so it will go back to the homepage after requesting a state change,
 * e.g. when motor up was clicked it shall go back to the homepage
 */
void send_homepage_redirection()
{
  server.sendHeader("Location", "/"); // Client shall redirect to "/"
  server.send(303);
}

/*
 * Handles calls to the URI /motor/<string: action>/ and does state transitions:
 * if for example /motor/up is called, then the system will transition from the previous state
 * to the state UP.
 */
void handle_motor_requests()
{
  String action = server.pathArg(0); // retrieve the given argument/action

  if (action == "up")
  {
    g_system_state = UP;
  }
  else if (action == "stop")
  {
    g_system_state = HOLD;
  }
  else if (action == "down")
  {
    g_system_state = DOWN;
  }
  else
  {
    Serial.println("Error: Action is unknown"); // system will stay in its previous state
  }

  // send response
  send_homepage_redirection();
}

/*
 * Handles calls to the URI /height/<string: height_in_mm>/
 * If a height is given, then the system shall transition into the CUSTOM_HEIGHT state.
 */
void handle_height_requests()
{
  int height = atoi((server.pathArg(0)).c_str()) * 10; // convert string parameter to integer

  // only change the state if the given height is in the height boundaries
  if (height >= MIN_HEIGHT and height <= MAX_HEIGHT)
  {
    g_custom_height = height;       // set the custom height
    g_system_state = CUSTOM_HEIGHT; // transition to the custom height state
  }

  // send response
  send_homepage_redirection();
}

/*
 * Handles calls to the URI /height/
 * Responds with the current height of the ultrasonic sensor
 */
void handle_read_height_requests()
{
  // respond with heights from both sensors in json
  int tofHeight = tofMeasurement;
  int rcwlHeight = rwclMeasurement;
  String response = "{\"tofHeight\":" + String(tofHeight) + ", \"rcwlHeight\":" + String(rcwlHeight) + "}";
  server.send(200, "application/json", response); // send json response with the heights
}

/**
 * Setup the output pins
 */
void setup_pins()
{
  // Pin setup for motor controller
  pinMode(MOTOR_DRIVER_PWM, OUTPUT);
  pinMode(MOTOR_DRIVER_DIR, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  tm.begin();
  tm.setBrightnessPercent(20);
}

void setup_sensors()
{
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100); // wait for Sensors to boot up
  if (not rcwlSensor.begin())
  {
    Serial.println("RWCL_Sensor not found. Check connections and I2C-pullups.");
    tm.clearScreen();
    tm.display("E01");
    notify_text = "E01-chk sensor1";
    while (millis() < 10000)
    {
      ArduinoOTA.handle();
      notify_time = millis();
      handle_lcd();
    }
    ESP.restart();
  }
  else
  {
    rcwlSensor.setFilter(true);
    rcwlSensor.setTimeout(100);
    rcwlSensor.setMode(RCWL_1X05::continuous);
    rcwlSensor.setTemperature(24);
    Serial.println("RWCL_Sensor ready.");
  }
  if (!tofSensor.begin())
  {
    Serial.println("VL53L0X not found. Check connections and I2C-pullups.");
    tm.clearScreen();
    tm.display("E04");
    notify_text = "E04-chk sensor2";
    while (millis() < 10000)
    {
      ArduinoOTA.handle();
      notify_time = millis();
      handle_lcd();
    }
    ESP.restart();
  }
  else
  {
    tofSensor.startRangeContinuous(100);
    Serial.println("VL53L0X ready.");
  }
}

/**
 * Takes care of the wifi configuration
 */
void setup_wifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(OPT_HOSTNAME); // set HOSTNAME
  WiFi.begin(SSID, PASSWORD);

  // Wait for wifi connection
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(250);
    Serial.print(".");
  }

  // Start the mDNS responder for skarsta.local
  if (MDNS.begin("skarsta"))
  {
    Serial.println("MDNS responder started");
  }
}

/**
 * Print information about the wifi connection:
 * SSID, IP, HOSTNAME
 */
void print_connection_info()
{
  // Print connection info
  Serial.print("Connected to ");
  Serial.println(SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("HOSTNAME: ");
  Serial.println(WiFi.getHostname());
  notify_text = WiFi.localIP().toString();
}

/**
 * Register the routes of the server
 */
void register_server_routes()
{
  server.on(F("/"), display_index);                             // route: /
  server.on(UriBraces("/motor/{}"), handle_motor_requests);     // route: /motor/<string: action>/
  server.on(UriBraces("/height/{}"), handle_height_requests);   // route: /height/<string: height_in_mm>/
  server.on(UriBraces("/height"), handle_read_height_requests); // route: /height/ - is being called from client javascript
}

void do_height_measurements()
{
  const int measurementInterval = 25;
  static int lastMeasurementMillis = millis() - measurementInterval;
  static bool lastMeasurementTof = false;

  if (millis() - lastMeasurementMillis >= measurementInterval)
  {
    lastMeasurementMillis = millis();

    if (lastMeasurementTof)
    {
      if (rcwlSensor.update())
      {
        rwclMeasurement = rcwlSensor.read() + HEIGHT_DIFFERENCE_RWCL; // read the distance from the RCWL-1X05 sensor and add the height difference
      }

      lastMeasurementTof = false;
    }
    else
    {
      if (tofSensor.isRangeComplete())
      {
        tofMeasurement = tofSensor.readRange() + HEIGHT_DIFFERENCE_TOF; // read the distance from the VL53L0X sensor and add the height difference
      }

      lastMeasurementTof = true;
    }
  }
}

/*
 * Checks if the table limit is reached. The table limit is reached when the table is at the maximum or minimum height.
 */
bool table_limit_reached(bool up)
{
  int height = tofMeasurement;
  return (up ? height >= MAX_HEIGHT || height <= MIN_HEIGHT - 30 : height >= MAX_HEIGHT + 30 || height <= MIN_HEIGHT);
}

/*
 * Moves the table up or down based on the given direction.
 * The speed of the motor is adjusted at the beginning to avoid a sudden start.
 * The motor will slow down when it is close to the custom height to avoid overshooting.
 * The motor will stop when the table limit is reached to prevent damage or sensor being blocked.
 */
void move_table(bool up)
{
  static int curr_motor_speed = MIN_MOTOR_SPEED;
  static unsigned long last_known_startup_slowdown = 0;

  if (abs(tofMeasurement - rwclMeasurement) > MAX_SENSOR_DIFFERENCE)
  {
    g_system_state = HOLD;
    stop_table();
    tm.clearScreen();
    tm.display("E05");
    notify_text = "E05-sensor diff";
    notify_time = millis();
    Serial.println("Error: Sensor difference too high! TOF:" + String(tofMeasurement) + " RWCL:" + String(rwclMeasurement));
    return;
  }

  if (!table_limit_reached(up))
  {
    g_last_direction = up;
    unsigned long startupSlowdown = millis() - last_known_startup_slowdown;
    unsigned long lastHeightChangeDiff = millis() - last_known_height_change;
    if (startupSlowdown < MOTOR_SLOW_DOWN_TIME or lastHeightChangeDiff > MOTOR_SLOW_DOWN_INTERVAL)
    {
      if (lastHeightChangeDiff > MOTOR_SLOW_DOWN_INTERVAL)
        last_known_startup_slowdown = millis();
      if (startupSlowdown >= MOTOR_SLOW_DOWN_TIME)
        startupSlowdown = 1;
      // curr_motor_speed = MAX_MOTOR_SPEED * ((float)startupSlowdown / ((( MAX_MOTOR_SPEED + MIN_MOTOR_SPEED) / 20.0) * ( MOTOR_SLOW_DOWN_TIME / 10.0 ))) + MIN_MOTOR_SPEED;
      curr_motor_speed = MAX_MOTOR_SPEED * ((float)startupSlowdown / ((255 / MAX_MOTOR_SPEED) * MOTOR_SLOW_DOWN_TIME));
    }

    if (g_system_state == CUSTOM_HEIGHT)
    {
      // decrease speed when the table is close to the custom height to avoid overshooting
      int remaining_dist = abs(g_custom_height - tofMeasurement);
      if (remaining_dist < MOTOR_SLOW_DOWN_CUSTOM_HEIGHT)
      {
        int temp_speed_new = MAX_MOTOR_SPEED * ((float)(remaining_dist - HEIGHT_TOLERANCE) / (((MAX_MOTOR_SPEED + MIN_MOTOR_SPEED) / 20.0) * (MOTOR_SLOW_DOWN_CUSTOM_HEIGHT / 10.0))) + MIN_MOTOR_SPEED; // decrease speed linearly
        if (temp_speed_new < curr_motor_speed)
          curr_motor_speed = temp_speed_new; // only decrease speed
      }
      else if ((remaining_dist - MOTOR_SLOW_DOWN_CUSTOM_HEIGHT) < HEIGHT_TOLERANCE * 2)
        ;
    }
    if (curr_motor_speed > (MAX_MOTOR_SPEED - 7))
      curr_motor_speed = MAX_MOTOR_SPEED; // set speed to max speed if it is close to max speed
    // Serial.print("Moving table ");
    // Serial.print(up ? "up" : "down");
    // Serial.print(" with speed ");
    // Serial.println(curr_motor_speed);
    motor.setSpeed(up ? curr_motor_speed : -curr_motor_speed);
    last_known_height_change = millis();
  }
  else
  {
    g_system_state = HOLD;
    stop_table();
    Serial.print("Table limit reached at ");
    Serial.println(tofMeasurement);
    notify_text = "E02-table lin";
    notify_time = millis();
  }
}
/*
 * Stop the table at the current height
 */
void stop_table()
{
  motor.setSpeed(0);
}

/*
 * Controls the motor based on the system state g_system_state. This is pretty much the core FSM implementation for the state transistions.
 */
void handle_output()
{
  static unsigned long last_not_hold = 0;
  static unsigned long last_hold = 0;
  static int last_state = 0;
  static bool state_changed = false;

  if (g_system_state == HOLD)
  {
    last_hold = millis();
  }
  if (((millis() - last_hold) < 620) and ((millis() - last_not_hold) < 620) and (last_state == HOLD))
  {
    return;
  }
  last_state = g_system_state;
  if (g_system_state != HOLD)
  {
    last_not_hold = millis();
  }

  switch (g_system_state)
  {
  case UP:
    if (!g_last_direction)
      last_known_height_change = 0;
    move_table(true); // motor go up
    break;
  case DOWN:
    if (g_last_direction)
      last_known_height_change = 0;
    move_table(false); // motor go down
    break;
  case HOLD:
    stop_table(); // stop the motor
    break;
  case CUSTOM_HEIGHT:
    // adjust the table height until the height tolerance is ok, e.g.: abs(150-130) = 20, abs(150-170) = 20
    if (abs(g_custom_height - tofMeasurement) >= HEIGHT_TOLERANCE)
    {
      // check if the table is too high or too low and adjust
      if (g_last_direction != (g_custom_height > tofMeasurement))
        last_known_height_change = 0;
      move_table(g_custom_height > tofMeasurement);
    }
    else
    {
      // adjustment is finished, transistion to the hold state to stop the motor
      g_system_state = HOLD;
    }

    break;
  default:
    // stop the motor by transitioning to the hold state
    g_system_state = HOLD;
  }
}

/*
 * Reads the analog button input and maps the values to the corresponding button.
 */
int readAnalogButtonRaw()
{
  int button = analogRead(BUTTON_PIN);
  // if (button < 4090){
  //   Serial.print("Analog: ");
  //   Serial.println(button);
  // }
  if (button >= 4075)
    return 0;
  if (button < 500)
    return 1;
  if (button < 1150)
    return 2;
  if (button < 1800)
    return 3;
  if (button < 2400)
    return 4;
  if (button < 3300)
    return 5;
  if (button < 4075)
    return 6;
  return 0;
}

/*
 * Reads the analog button input and handles the bouncing of the button.
 */
int readAnalogButton()
{
  int button = readAnalogButtonRaw();
  if (button == 0)
    return 0;
  delay(20);
  if (button == readAnalogButtonRaw())
    return button;
  Serial.println("Button bouncing");
  delay(75);
  return readAnalogButton();
}

/*
 * Handles the input from the buttons. The buttons are connected to an analog pin and the values are read from the pin.
 * The values are then mapped to the corresponding button.
 */
void handleButtons()
{
  int button = readAnalogButton();
  // return;
  switch (button)
  {
  case 1:
    g_system_state = UP;
    g_button_pressed = true;
    // Serial.println("Button UP pressed");
    break;
  case 2:
    g_system_state = DOWN;
    g_button_pressed = true;
    // Serial.println("Button Down pressed");
    break;
  case 3:
    g_system_state = CUSTOM_HEIGHT;
    g_button_pressed = true;
    g_custom_height = 730;
    // Serial.println("Button 700 pressed");
    break;
  case 4:
    g_system_state = CUSTOM_HEIGHT;
    g_button_pressed = true;
    g_custom_height = 1000;
    // Serial.println("Button 900 pressed");
    break;
  case 5:
    g_system_state = CUSTOM_HEIGHT;
    g_button_pressed = true;
    g_custom_height = 1100;
    // Serial.println("Button 1200 pressed");
    break;
  case 6:
    g_system_state = HOLD;
    // Serial.println("Button stop pressed");
    notify_text = "stop";
    notify_time = millis();
    break;
  default:
    if (g_button_pressed)
    {
      g_button_pressed = false;
      Serial.println("Button released");
      if (g_system_state == UP || g_system_state == DOWN)
      {
        stop_table();
        g_system_state = HOLD;
      }
    }
  }
}

void setupOTA()
{
  ArduinoOTA.setHostname(OPT_HOSTNAME);
  ArduinoOTA
      .onStart([]()
               {
    stop_table();
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
    tm.clearScreen();
    tm.display("OTA", false, false); })
      .onEnd([]()
             {
    Serial.println("\nEnd");
    tm.colonOff();
    tm.display("succ"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  {
    int progress_percent = (progress / (total / 100));
    Serial.printf("Progress: %u%%\r", progress_percent);
    tm.colonOn();
    tm.display(String("OT"+String(progress_percent))); })
      .onError([](ota_error_t error)
               {
    Serial.printf("Error[%u]: ", error);
    tm.colonOff();
    tm.display("OtEr");
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    } });

  ArduinoOTA.begin();
  Serial.println("OTA ready");
}

/*
 * Protects the motor from getting stuck by checking if the motor is stuck
 * by comparing the current height with the last height. If the height is the same
 * and the system state is the same after a certain time, then the motor is stuck.
 * The motor will be stopped and the system state will be set to HOLD.
 * This is to prevent the motor from getting damaged.
 */
void motor_protection()
{
  static unsigned long last_motor_protection = 0;
  static int last_motor_protection_height = 0;
  static int last_motor_protection_system_state = 0;
  static int last_motor_protection_custom_height = 0;

  if (millis() - last_motor_protection >= MOTOR_PROTECTION_INTERVAL)
  {
    int height = tofMeasurement;
    if (g_system_state != HOLD)
    {
      if (abs(height - last_motor_protection_height) < MOTOR_PROTECTION_DISTANCE and last_motor_protection_system_state == g_system_state and last_motor_protection_custom_height == g_custom_height)
      {
        Serial.println("Motor protection triggered!!!");
        g_system_state = HOLD;
        stop_table();
        notify_text = "E03-chk notor";
        notify_time = millis();
      }
    }
    last_motor_protection = millis();
    last_motor_protection_height = height;
    last_motor_protection_system_state = g_system_state;
    last_motor_protection_custom_height = g_custom_height;
  }
}

void handle_lcd()
{
  static int last_lcd_state = 0;
  const uint8_t symbol_up[] = {0x00, 0x00, 0x00, DisplayDigit().setC()};
  const uint8_t symbol_down[] = {0x00, 0x00, 0x00, DisplayDigit().setB()};
  const uint8_t symbol_middle[] = {0x00, 0x00, 0x00, DisplayDigit().setB().setC()};
  tm.onMode();

  if (millis() - notify_time < NOTIFICATION_TIME)
  {
    if (last_lcd_state != 0)
    {
      last_lcd_state = 0;
      tm.clearScreen();
      tm.changeBrightnessPercent(100);
    }
    if (notify_text.length() > 4)
    {
      tm.display(notify_text)->scrollLeft(230);
    }
    else
    {
      tm.display(notify_text);
    }
  }
  else
  {
    if (last_lcd_state == 0)
    {
      last_lcd_state = 1;
      tm.clearScreen();
      tm.setBrightnessPercent(3);
    }
    String filler = "  ";
    if (g_system_state != HOLD)
    {
      filler = " ";
      if (millis() % 1000 < 500)
      {
        if (last_lcd_state != 2)
        {
          tm.displayRawBytes(symbol_middle, sizeof(symbol_middle));
          last_lcd_state = 2;
        }
      }
      else
      {
        if (g_last_direction and last_lcd_state != 3)
        {
          tm.displayRawBytes(symbol_up, sizeof(symbol_up));
          last_lcd_state = 3;
        }
        else if (!g_last_direction and last_lcd_state != 4)
        {
          tm.displayRawBytes(symbol_down, sizeof(symbol_down));
          last_lcd_state = 4;
        }
      }
    }

    String height;
    // if (g_system_state == CUSTOM_HEIGHT and millis() % 2000 < 200){
    if (g_system_state == CUSTOM_HEIGHT and g_button_pressed)
    {
      height = String(g_custom_height / 10);
    }
    else
    {
      height = String(tofMeasurement / 10);
    }
    if (height.length() < 3)
    {
      height = height + filler;
    }
    tm.display(height);
  }
}

boolean powerMgmt()
{
  static unsigned long last_relay = 0;
  static bool power_mgmt_state = true;
  if (g_system_state != HOLD)
  {
    digitalWrite(RELAY_PIN, HIGH);
    last_relay = millis();
    delay(10);
    power_mgmt_state = true;
    return true;
  }
  else
  {
    if (millis() - last_relay > RELAY_INTERVAL)
    {
      if (power_mgmt_state){
        digitalWrite(RELAY_PIN, LOW);
        tm.offMode();
        power_mgmt_state = false;
      }
      delay(30);
      return false;
    }
    return true;
  }
}

/*
 * Login to the network, setup the server and register URI call handlers.
 */
void setup(void)
{
  Serial.begin(115200);
  setup_pins();
  setup_wifi();

#ifdef DEBUG
  print_connection_info();
#endif

  setupOTA();
  setup_sensors();

  register_server_routes();

  // Start the server
  server.begin();
}

/*
 * Main loop. Gets the current height, retrieves inputs and do state
 * transitions and finally control the motor based on the state.
 */
void loop(void)
{
  ArduinoOTA.handle();
  server.handleClient(); // gets input from a client
  handleButtons();       // handles the input from the buttons
  if (powerMgmt())
  { // handles the relay
    do_height_measurements();
    handle_output();    // controls the height of the table based on the input
    motor_protection(); // protects the motor from getting stuck
    handle_lcd();
  }
}