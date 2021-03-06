/*

This is a liquid cooler management system for PC's and manages the Pump and
Fan speeds. It features a OLED display, 3 buttons for tweaking persistant configs,
a calibration routine for pump speed, and dynamic scaling of fan/pump speed based
on multiple temperature probes.

Why: water cooler pump and fans typically follow the CPU temperatures, This system
follows both the CPU and GPU and adjusts fan / pump speeds according to whichever
component is the hottest in relation to its defined max thresholds.

WARNING: make sure you configure your minimun pump speeds to reasonable values,
failure to do this could cook your hardware. This includes making sure the
pump flow rate is sufficient. You have been warned.

Features:
  * UI showing current temperatures and levels compared to thresholds
  * setup UI for tweaking values
  * beeping alarms when thresholds exceeded
  * fans to 100% when thresholds exceeded
  * dynamic fan PWM control based off CPU and GPU temperature thresholds
  * customizable pump and fan min/max power
  * pump calibration routine ( See Serial output for performance report and copy optimal values )

Calibration:
  * Make sure to open the serial monitor in Arduino before starting!
  * Make sure min and max calibration has been done using Bios Fan speed monitor
    for the pump and fans.
  * During calibration the fans run at the lowest threshold defined.
  * Maintain a 60% load on the CPU while running the calibration
  * Repeat the calibration at least 3 times right after each other.


Notes:
  * Obtain pump / fan min operation values by setting the min's to the lowest it
    can be, and then launching something like your bios fan speed monitor, then
    open setup on the arduino controller, and slowly increment fan1_min and pump1_min
    until you see a rpm change, this is the minimun for your pump/fan. Repeat for
    max values. TODO, add tacometer to arduino to do this as part of calibration


TODO:
  * totaly automatic calibration routine, currently user has to find optimal
    values in serial output.
  * ambient vs water temperature metrics, system should compare to ambient as base

*/


// One Wire for Dallas temperature sensors are all plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// precision modes for the temperature sensors, 1/100th degree vs 1/2 degree accuracy
// 12bit takes 700ms per cycle, 9bit takes 90ms on a UNO
#define TEMPERATURE_PRECISION_12 12
#define TEMPERATURE_PRECISION_9 9

// motherboard PWM spec pins minimun at 20% duty cycle
#define PWM_MINIMUN_CYCLE 255*0.05
#define PWM_MAX_CYCLE 255

// calibration routing definitions
#define CALIBRATION_TIME_MIN 15
#define CALIBRATION_TIME_MAX 120

// Oled on a UNO
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
#define MAX_MEM 2048

// Oled on a Pololu 32u4
//#define OLED_MOSI   9
//#define OLED_CLK   6
//#define OLED_DC    8
//#define OLED_CS    7
//#define OLED_RESET 5

// the fan(s) PWM pin, this connects to PWM on the Radiator Fans.
#define FAN_PIN 3

// the pump(s) PWM pin, this connects to PWM on the Pumps.
#define PUMP_PIN 5

// button panel for the UI uses 3 buttons
#define FUNC_BTN_PIN 8
#define R_BTN_PIN 4
#define L_BTN_PIN 6

// speaker for squaking alarms
#define SPKR_PIN 7


#include <Arduino.h>
// #include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "pgfx.h"
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"



// some state tracking vars for the UI
int btn_func_state = 0;
int last_btn_func_state = 0;
int btn_r_state = 0;
int btn_l_state = 0;
boolean setup_mode = false; // is setup mode active?
boolean button_press = false; // has a button been pressed? ( debounce )
int setup_menu_page = 1; // initial setup menu page
int setup_menu_item = 1; // the initial setup item on the page

// The display is a u8glib compatible one. changing this requires changing
// the pgfx type for display also.
U8GLIB_SSD1306_128X64 display(OLED_CLK, OLED_MOSI, OLED_CS, OLED_DC, OLED_RESET);

// refresh rate control of the display
unsigned int OLED_REFRESH = 125; // ms between redraw
unsigned int LAST_OLED_REFRESH; // last time screen was drawn
unsigned long now; // used for millis()

// structure for temperature, its max and its name
struct temp_struct
{
  float temperature;
  float max;
  char name[4];
};

// names of components, pointed to by config per "monitor"
static const char components[10][5] = {"CPU", "GFX", "HOT", "Cold", "Rad", "Aux1", "Aux2", "Rad1", "Rad2", "AMB"};
#define NUM_COMPONENTS 9 // because array length is not really possible

// fan / pump vars
unsigned long fan_adjust_time; // when the fan speed was last adjusted, smoothing

// temp, max and name structs for 4 temperature probes
struct temp_struct t1;
struct temp_struct t2;
struct temp_struct t3;
struct temp_struct t4;
bool probe_error = false;

// eeprom configuration items
struct config_t
{
    uint8_t t1_max;
    uint8_t t1_name;
    uint8_t t2_max;
    uint8_t t2_name;
    uint8_t t3_max;
    uint8_t t3_name;
    uint8_t t4_max;
    uint8_t t4_name;
    uint8_t aux1_max;
    uint8_t aux1_name;
    uint8_t aux2_max;
    uint8_t aux2_name;
    uint8_t fan1_min;
    uint8_t fan1_max;
    uint8_t pump1_min;
    uint8_t pump1_max;
    boolean hires;
    uint8_t calibration_time;
    boolean calibrate;
} configuration;


// fans and pumps
uint16_t prime_time = 2000; // prime pumps and fans for 2 seconds to purge dust and bubbles
boolean primed = false; // fans and pumps at 100% for a second at least
uint8_t fan1_speed; // fan speed used for PWM
uint8_t pump1_speed; // pump speed used for PWM


// pump calibration system
boolean pump_calibration_started; // initial start
boolean pump_calibration_running; // running
uint16_t pump_calibration_start_time; // start time

// stucture for calibration samples
typedef struct {
  uint8_t speed;
  float temperature;
} calibration_sample;

calibration_sample pump_temperature_samples[16]; // array of 16 sample points
float calibration_start_temp; // initial temperature
boolean pump_calibration_waiting = false; // waiting to settle temperatures
unsigned long pump_calibration_change_time; // last time the speed of the pump was tweaked
uint8_t pump_sample_count = 0;


// graphical gauges
pgfx_HBAR t1bar = pgfx_HBAR(0,0,20,64,"T1",display, SPKR_PIN);
pgfx_HBAR t2bar = pgfx_HBAR(26,0,20,64,"T2",display, SPKR_PIN);
pgfx_HBAR t3bar = pgfx_HBAR(52,0,20,64,"T3",display, SPKR_PIN);
pgfx_HBAR t4bar = pgfx_HBAR(78,0,20,64,"T4",display, SPKR_PIN);
pgfx_HBAR fan1bar = pgfx_HBAR(104,0,20,64,"FP1",display, SPKR_PIN);


// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Holders for temperature probe device addresses
DeviceAddress th1, th2, th3, th4;

//declare reset function at address 0, reboots the arduino
void(* resetFunc) (void) = 0;

void setup(void)
{
  // read the config from eeprom
  EEPROM_readAnything(0, configuration);

  // set the various max thresholds from config
  t1.max = configuration.t1_max;
  strncpy( t1.name, components[configuration.t1_name], 5);
  t2.max = configuration.t2_max;
  strncpy( t2.name, components[configuration.t2_name], 5);
  t3.max = configuration.t3_max;
  strncpy( t3.name, components[configuration.t3_name], 5);
  t4.max = configuration.t4_max;
  strncpy( t4.name, components[configuration.t4_name], 5);

  // set the initial fan and pump speeds to max to purge bubbles and dust.
  fan1_speed = PWM_MAX_CYCLE;
  pump1_speed = PWM_MAX_CYCLE;

  // display the initial state of the display
  display.setFont(u8g_font_ncenB14);
  display.setFontPosTop();
  //display.setRot180(); // rotate the screen
  display.firstPage();
  do {
    // display.drawXBMP( 0, 0, logo_width, logo_height, logo_bits);
    // display.setColorIndex(1);
    display.drawStr(0,0,"Psimax");
    display.drawStr(32,24,"Systems");
    // display.setColorIndex(1);
  } while( display.nextPage() );

  // buttons pin mappins, each pulled down via 10k resistor
  pinMode(FUNC_BTN_PIN, INPUT);
  pinMode(L_BTN_PIN, INPUT);
  pinMode(R_BTN_PIN, INPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);

  // test the speaker
  tone(SPKR_PIN, 80, 125);

  // start serial port
  Serial.begin(9600);
  // Serial.println(F("Controller Boot"));
  // Serial.print("pump speed: ");
//  Serial.println(configuration.pump1_min);
//  Serial.println(components[0]);

  // Start up the library
  sensors.begin();

  // locate devices on the bus
//  Serial.print("Locating devices...");
//  Serial.print("Found ");
//  Serial.print(sensors.getDeviceCount(), DEC);
//  Serial.println(" devices.");

  // report parasite power requirements
//  Serial.print("Parasite power is: ");
//  if (sensors.isParasitePowerMode()) Serial.println("ON");
//  else Serial.println("OFF");

  // assign address manually.  the addresses below will beed to be changed
  // to valid device addresses on your bus.  device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(deviceAddress, index)
  //th1 = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };
  //th2   = { 0x28, 0x3F, 0x1C, 0x31, 0x2, 0x0, 0x0, 0x2 };

  // search for devices on the bus and assign based on an index.  ideally,
  // you would do this to initially discover addresses on the bus and then
  // use those addresses and manually assign them (see above) once you know
  // the devices on your bus (and assuming they don't change).
  //
  // method 1: by index
  //if (!sensors.getAddress(th1, 0)) Serial.println("Unable to find address for Device 0");
  //if (!sensors.getAddress(th2, 1)) Serial.println("Unable to find address for Device 1");

  // method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices,
  // or you have already retrieved all of them.  It might be a good idea to
  // check the CRC to make sure you didn't get garbage.  The order is
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  //oneWire.reset_search();
  // assigns the first address found to th1
  // String u = "Error ";
  if (!oneWire.search(th1)) probe_error=true;
  if (!oneWire.search(th2)) probe_error=true;
  if (!oneWire.search(th3)) probe_error=true;
  if (!oneWire.search(th4)) probe_error=true;

  // set the resolution
  int temperature_precision = TEMPERATURE_PRECISION_9;
  if (configuration.hires) {
    temperature_precision = TEMPERATURE_PRECISION_12;
  }

  // set the probe resolutions
  sensors.setResolution(th1, temperature_precision);
  sensors.setResolution(th2, temperature_precision);
  sensors.setResolution(th3, temperature_precision);
  sensors.setResolution(th4, temperature_precision);

  // if calibration bit set, then set the process in motion
  if (configuration.calibrate) {
    // set the minimun to 20% dity
    pump1_speed = PWM_MINIMUN_CYCLE;
    fan1_speed = PWM_MINIMUN_CYCLE;
    pump_calibration_running = true;
  }

}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// function to print the temperature for a device
// float printTemperature(DeviceAddress deviceAddress)
// {
  // return sensors.getTempC(deviceAddress);
  // float tempC = sensors.getTempC(deviceAddress);
  //Serial.print("Temp C: ");
  // Serial.println(tempC);
  // Serial.print(" Temp F: ");
  // Serial.print(DallasTemperature::toFahrenheit(tempC));
  // return tempC;
// }

// function to print a device's resolution
// void printResolution(DeviceAddress deviceAddress)
// {
//   Serial.print("Resolution: ");
//   Serial.print(sensors.getResolution(deviceAddress));
//   Serial.println();
// }

// main function to print information about a device
// float printData(DeviceAddress deviceAddress)
// {
//   Serial.print("Device Address: ");
//   printAddress(deviceAddress);
//   Serial.print(" ");
//   float t = printTemperature(deviceAddress);
//   Serial.println();
//   return t;
// }

void draw(temp_struct ts1, temp_struct ts2, temp_struct ts3 ,temp_struct ts4) {
  t1bar.update(ts1.temperature, ts1.max, ts1.name);
  t2bar.update(ts2.temperature, ts2.max, ts2.name);
  t3bar.update(ts3.temperature, ts3.max, ts3.name);
  t4bar.update(ts4.temperature, ts4.max, ts4.name);
  fan1bar.update(((float)fan1_speed/(float)configuration.fan1_max)*100, 100, ((float)pump1_speed/(float)configuration.pump1_max)*100, 100, "F/P");
}


// headless monitor the temperatures and tweak fan/pump/aux
void monitor(temp_struct ts) {
  if (ts.temperature >= ts.max) {
    display.setFont(u8g_font_ncenB14);
    display.setFontPosTop();
    display.firstPage();
    do {
     display.drawStr(16, 0, "WARNING");
     display.drawStr(48, 24, ts.name);
    } while( display.nextPage() );
    tone(SPKR_PIN, 80, 125);
    delay(500);
  }
}


// calibration based on the cpu and gpu probe temperatures
// possible constants, desired temperature, ambient temperature, 1/16th of 255 ( rate of increase )
void calibratePump(temp_struct ts1) {
  // upon initialiation, capture initial temperatures, start time and set start bit

  if (pump_calibration_running) {
    if (!pump_calibration_started) {
      Serial.println("Starting Calibration");
      pump1_speed = configuration.pump1_min;
      Serial.println("Settling...");
      delay(10000);
      calibration_start_temp = ts1.temperature;
      pump_calibration_change_time = millis();
      pump_calibration_started = true;
      pump_calibration_waiting = true;
    // if waiting for temperatures to settle, then do that, checking wait time
    } else if (pump_calibration_waiting) {
      Serial.print("Checking if enough time has passed: millis: ");
      Serial.println(millis()-pump_calibration_change_time);
      if ((millis()-pump_calibration_change_time) > ((unsigned long)configuration.calibration_time*1000) ) {
        Serial.print("Saving results for speed: ");
        Serial.println(pump1_speed);
        pump_calibration_waiting = false;
        pump_calibration_change_time = millis();
        pump_temperature_samples[pump_sample_count] = (calibration_sample){pump1_speed, ts1.temperature};
        pump_sample_count++;
      }

    } else {

      Serial.println("Increment speed?");

      // check if max speed has been achieved, finalize calibration
      if (pump_sample_count >= 16) {
        Serial.println("Calibration sample complete");
        pump_calibration_running = false;
        for (int i=0; i<16; i++) {
          Serial.print("Sample, Speed, Temperature, Delta since Start, Return");
          Serial.print(i);
          Serial.print(", ");
          Serial.print(pump_temperature_samples[i].speed);
          Serial.print(", ");
          Serial.print(pump_temperature_samples[i].temperature);
          Serial.print(", ");
          Serial.print(calibration_start_temp - pump_temperature_samples[i].temperature);
          Serial.print(", ");
          Serial.println((calibration_start_temp - pump_temperature_samples[i].temperature)/pump_temperature_samples[i].speed);
        }
        configuration.calibrate = false;
        EEPROM_writeAnything(0, configuration);

        delay(1000);
        resetFunc();
      }


      // increment is 1/16th of the delta between max and min configurated values
      uint8_t increment = ((configuration.pump1_max - configuration.pump1_min) / 16);

      // increase speed by 1/16th of that is left after deducting the 20% minimun
      pump1_speed=pump1_speed+increment;

      // Testing
      //fan1_speed=fan1_speed+increment;

      // pump_calibration_change_time = millis();
      pump_calibration_waiting = true;
      Serial.print("Speed increased to ");
      Serial.println(pump1_speed);
    }
  } else {
    // dont do anything
  }

}

// monitor the temperatures and tweak fan/pump/aux channels
void adjust_pwm_signals(temp_struct ts1, temp_struct ts2, temp_struct ts3, temp_struct ts4) {

  // if system is still priming, just return
  if (!primed || pump_calibration_running) {
    return;
  }

  // put all the hot temps together and aggregates
  uint8_t temperature; // = ts1.temperature + ts2.temperature;

  //  + ts3.max + ts4.max;
  uint8_t max; // = ts1.max + ts2.max;

  // choose the hottest component
  if ((ts1.max-ts1.temperature)<(ts2.max-ts2.temperature)) {
    temperature = ts1.temperature;
    max = ts1.max;
  } else {
    temperature = ts2.temperature;
    max = ts2.max;
  }

  // slow down slowly
  if ((millis() - fan_adjust_time) > 1000) {
    fan1_speed--;
    fan_adjust_time = millis();
  }

  if (temperature >= max) {
    fan1_speed = configuration.fan1_max;
  } else if (temperature >= (0.85*max)) {
    if (fan1_speed < (configuration.fan1_max*0.60)) {
      fan1_speed++;
      fan1_speed++;
    }
  } else if (temperature >= (0.60*max)) {
    if (fan1_speed < (configuration.fan1_max*0.40)) {
      fan1_speed++;
      fan1_speed++;
    }
  } else if (temperature >= (0.40*max)) {
    if (fan1_speed < (configuration.fan1_max*0.30)) {
      fan1_speed++;
      fan1_speed++;
    }
  } else if (temperature >= (0.20*max)) {
    if (fan1_speed < (configuration.fan1_max*0.25)) {
      fan1_speed++;
      fan1_speed++;
    }
  } else if (temperature <= (0.20*max)) {
    fan1_speed = (configuration.fan1_min);
  }

}


// inc / dec int based on input keys
void tweak_value(uint8_t &val, int btn_r_state, int btn_l_state) {
  if (btn_r_state == HIGH && !button_press) {
    val++;
    button_press=true;
  } else if (btn_l_state == HIGH && !button_press) {
    val--;
    button_press=true;
  }
}

// invert booleans on input keys
void tweak_value(boolean &val, int btn_r_state, int btn_l_state) {
  if (btn_l_state == HIGH && !button_press) {
    val = false;
    button_press=true;
  } else if (btn_r_state == HIGH && !button_press) {
    val = true;
    button_press=true;
  }
}

// converts booleans to string for easy display handling
inline const char * const BoolToString(bool b)
{
  return b ? "True" : "False";
}


// locks a variable to between inclusive min and max
// uint8's tend to rollover, for some reason, we have
// a one-way rollover here..
void clamp(uint8_t &val, int minv, int maxv) {
  if (val<=minv ) {
    val = minv;
  } else if (val>=maxv) {
    val = maxv;
  }
}


// setup function displays the GUI for settings
void drawSetup(void) {

  // set the 12px font
  display.setFont(u8g_font_profont12);
  display.setFontPosTop();

  // read all button states
  btn_r_state = digitalRead(R_BTN_PIN);
  btn_l_state = digitalRead(L_BTN_PIN);
  btn_func_state = digitalRead(FUNC_BTN_PIN);

  // a little re-usable buffer for display conversions to str
  char str_buffer[6];

  // page 1 of setup menu
  if (setup_menu_page==1) {
    display.drawStr(0, 0, "Setup 1/6");
    display.drawStr(0, 14, "T1:");
    display.drawStr(0, 28, "T2:");
    display.drawStr(0, 42, "T3:");

    // buffer to put arbitrairy chars into
    dtostrf(configuration.t1_max, 2, 2, str_buffer);
    display.drawStr(40,14, str_buffer);
    dtostrf(configuration.t2_max, 2, 2, str_buffer);
    display.drawStr(40,28, str_buffer);
    dtostrf(configuration.t3_max, 2, 2, str_buffer);
    display.drawStr(40,42, str_buffer);

  // page 2 of setup menu
  } else if (setup_menu_page==2) {
    display.drawStr(0, 0, "Setup 2/7:");
    display.drawStr(0, 14, "T4:");
    display.drawStr(0, 28, "A1:");
    display.drawStr(0, 42, "A2:");

    // buffer to put arbitrairy chars into
    dtostrf(configuration.t4_max, 2, 2, str_buffer);
    display.drawStr(40,14, str_buffer);
    dtostrf(configuration.aux1_max, 2, 2, str_buffer);
    display.drawStr(40,28, str_buffer);
    dtostrf(configuration.aux2_max, 2, 2, str_buffer);
    display.drawStr(40,42, str_buffer);

  // page 3 of setup menu
  } else if (setup_menu_page==3) {
    display.drawStr(0, 0, "Setup 3/7:");
    display.drawStr(0, 14, "12bit:");
    display.drawStr(0, 28, "aux1:");
    display.drawStr(0, 42, "aux2:");

    // buffer to put arbitrairy chars into
    display.drawStr(40,14, BoolToString(configuration.hires));

  } else if (setup_menu_page==4) {
    display.drawStr(0, 0, "Setup 4/7:");
    display.drawStr(0, 14, "T1:");
    display.drawStr(0, 28, "T2:");
    display.drawStr(0, 42, "T3:");

    // t1
    clamp(configuration.t1_name, 0, NUM_COMPONENTS);
    strncpy(str_buffer, components[configuration.t1_name], 5);
    display.drawStr(40,14, str_buffer);

    clamp(configuration.t2_name, 0, NUM_COMPONENTS);
    strncpy(str_buffer, components[configuration.t2_name], 5);
    display.drawStr(40,28, str_buffer);

    clamp(configuration.t3_name, 0, NUM_COMPONENTS);
    strncpy(str_buffer, components[configuration.t3_name], 5);
    display.drawStr(40,42, str_buffer);

  } else if (setup_menu_page==5) {
    display.drawStr(0, 0, "Setup 5/7:");
    display.drawStr(0, 14, "T4:");
    display.drawStr(0, 28, "AUX1:");
    display.drawStr(0, 42, "AUX2:");

    clamp(configuration.t4_name, 0, NUM_COMPONENTS);
    strncpy(str_buffer, components[configuration.t4_name], 5);
    display.drawStr(40,14, str_buffer);
    strncpy(str_buffer, components[configuration.aux1_name], 5);
    display.drawStr(40,28, str_buffer);
    strncpy(str_buffer, components[configuration.aux2_name], 5);
    display.drawStr(40,42, str_buffer);
  } else if (setup_menu_page==6) {
    display.drawStr(0, 0, "Setup 6/7:");
    display.drawStr(0, 14, "FAN1_MIN:");
    display.drawStr(0, 28, "FAN1_MAX:");
    display.drawStr(0, 42, "PUMP1_MIN:");

    clamp(configuration.fan1_min, PWM_MINIMUN_CYCLE, PWM_MAX_CYCLE);
    dtostrf(configuration.fan1_min, 2, 2, str_buffer);
    display.drawStr(60,14, str_buffer);

    clamp(configuration.fan1_max, configuration.fan1_min, PWM_MAX_CYCLE);
    dtostrf(configuration.fan1_max, 2, 2, str_buffer);
    display.drawStr(60,28, str_buffer);

    clamp(configuration.pump1_min, PWM_MINIMUN_CYCLE, PWM_MAX_CYCLE);
    dtostrf(configuration.pump1_min, 2, 2, str_buffer);
    display.drawStr(60,42, str_buffer);

  } else if (setup_menu_page==7) {
    display.drawStr(0, 0, "Setup 7/7:");
    display.drawStr(0, 14, "PUMP1_MAX:");
    display.drawStr(0, 28, "CAL_TIME:");
    display.drawStr(0, 42, "CALIBRATE:");

    // pump minimun
    clamp(configuration.pump1_max, configuration.pump1_min, PWM_MAX_CYCLE);
    dtostrf(configuration.pump1_max, 2, 2, str_buffer);
    display.drawStr(60,14, str_buffer);

    // the time to spend on each level of calibration
    clamp(configuration.calibration_time, CALIBRATION_TIME_MIN, CALIBRATION_TIME_MAX);
    dtostrf(configuration.calibration_time, 2, 2, str_buffer);
    display.drawStr(60,28, str_buffer);

    // calibration bool
    display.drawStr(60,42,  BoolToString(configuration.calibrate));

  }

  // line under the current selected item
  display.drawHLine(0,(setup_menu_item*14)+11,128);

  // check for more func button presses
  if (btn_func_state == HIGH && !button_press) {
    button_press=true;
    setup_menu_item++;

    // every 3 items we change pages
    if (setup_menu_item>=4) {
      setup_menu_page++;
      setup_menu_item = 1;
    }

    // if page is 4 or higher, set to 0, which is exit
    if (setup_menu_page>=8) {
      setup_menu_page = 0; // notifies main loop exiting config
      display.firstPage();
      do {
        display.drawStr(0,0,"Writing EEPROM...");
      } while( display.nextPage() );
      EEPROM_writeAnything(0, configuration);
    }
  }

  // T1 Temp on page 1
  if (setup_menu_page==1 && setup_menu_item == 1) {tweak_value(configuration.t1_max, btn_r_state, btn_l_state);}

  // T2 Temp on page 1
  if (setup_menu_page==1 && setup_menu_item == 2) {tweak_value(configuration.t2_max, btn_r_state, btn_l_state);}

  // T3 Temp on page 1
  if (setup_menu_page==1 && setup_menu_item == 3) {tweak_value(configuration.t3_max, btn_r_state, btn_l_state);}

  // T4  Temp on page 2
  if (setup_menu_page==2 && setup_menu_item == 1) {tweak_value(configuration.t4_max, btn_r_state, btn_l_state);}

  // AUX Temp on page 2
  if (setup_menu_page==2 && setup_menu_item == 2) {tweak_value(configuration.aux1_max, btn_r_state, btn_l_state);}

  // AUX Temp on page 2
  if (setup_menu_page==2 && setup_menu_item == 3) {tweak_value(configuration.aux2_max, btn_r_state, btn_l_state);}

  // HiRes / 12bit mode boolean
  if (setup_menu_page==3 && setup_menu_item == 1) {tweak_value(configuration.hires, btn_r_state, btn_l_state);}

  // page 4
  if (setup_menu_page==4 && setup_menu_item == 1) {tweak_value(configuration.t1_name, btn_r_state, btn_l_state);}
  if (setup_menu_page==4 && setup_menu_item == 2) {tweak_value(configuration.t2_name, btn_r_state, btn_l_state);}
  if (setup_menu_page==4 && setup_menu_item == 3) {tweak_value(configuration.t3_name, btn_r_state, btn_l_state);}

  // page 5
  if (setup_menu_page==5 && setup_menu_item == 1) {tweak_value(configuration.t4_name, btn_r_state, btn_l_state);}
  if (setup_menu_page==5 && setup_menu_item == 2) {tweak_value(configuration.aux1_name, btn_r_state, btn_l_state);}
  if (setup_menu_page==5 && setup_menu_item == 3) {tweak_value(configuration.aux2_name, btn_r_state, btn_l_state);}

  // page 6
  if (setup_menu_page==6 && setup_menu_item == 1) {tweak_value(configuration.fan1_min, btn_r_state, btn_l_state);}
  if (setup_menu_page==6 && setup_menu_item == 2) {tweak_value(configuration.fan1_max, btn_r_state, btn_l_state);}
  if (setup_menu_page==6 && setup_menu_item == 3) {tweak_value(configuration.pump1_min, btn_r_state, btn_l_state);}


  if (setup_menu_page==7 && setup_menu_item == 1) {tweak_value(configuration.pump1_max, btn_r_state, btn_l_state);}
  if (setup_menu_page==7 && setup_menu_item == 2) {tweak_value(configuration.calibration_time, btn_r_state, btn_l_state);}
  if (setup_menu_page==7 && setup_menu_item == 3) {tweak_value(configuration.calibrate, btn_r_state, btn_l_state);}

  if (digitalRead(R_BTN_PIN)==LOW && digitalRead(L_BTN_PIN)==LOW && digitalRead(FUNC_BTN_PIN) == LOW && button_press) {
    button_press=false;
  }
}



void loop(void) {


  // use clamp to ensure minimun and maximun operating levels are maintained
  clamp(fan1_speed, configuration.fan1_min, configuration.fan1_max);
  clamp(pump1_speed, configuration.pump1_min, configuration.pump1_max);

  // write current speeds to pwm pins
  analogWrite(FAN_PIN, fan1_speed);
  analogWrite(PUMP_PIN, pump1_speed);

  // check the func button state, doing a simple debounce
  btn_func_state = digitalRead(FUNC_BTN_PIN);

  // if setup has been quit, the page will be set to 0
  if ( setup_menu_page==0 ) {
    // exiting setup mode, reset the MCU
    delay(1000);
    resetFunc();

  // else if the button state is high, and its not in setup mode and its a new press.
  } else if (btn_func_state == HIGH && !setup_mode && btn_func_state != last_btn_func_state) {
    // enter setup
    button_press = true;
    setup_mode = true;
    display.firstPage();
    drawSetup();
    last_btn_func_state = btn_func_state;
  } else {
    last_btn_func_state = btn_func_state;
  }

  if (!setup_mode) {
    // Serial.print(F("Reading Sensors: "));
    // int m = millis();
    sensors.requestTemperatures();
    // Serial.println(millis()-m);
    t1.temperature = sensors.getTempC(th1);
    t2.temperature = sensors.getTempC(th2);
    t3.temperature = sensors.getTempC(th3);
    t4.temperature = sensors.getTempC(th4);

    // set fans
    adjust_pwm_signals(t1, t2, t3, t4);

    // check the probes compared to the max levels
    monitor(t1);
    monitor(t2);
    monitor(t3);
    monitor(t4);

    // call calibrate method if in calibration mode with the GFX probe temps
    if (pump_calibration_running && primed) {
      calibratePump(t1);
    }
  }


  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  now = millis();
  if ((now - LAST_OLED_REFRESH) > OLED_REFRESH) {
   display.firstPage();
    do {
      if (setup_mode) {
        drawSetup();
      } else {
        draw(t1, t2, t3, t4);
      }
    } while( display.nextPage() );
    LAST_OLED_REFRESH = millis(); // re-record now
  }

  // reset button press if released
  if (digitalRead(R_BTN_PIN)==LOW && digitalRead(L_BTN_PIN)==LOW && digitalRead(FUNC_BTN_PIN) == LOW && button_press) {
    button_press=false;
  }

  // if uptime greater than n, set the primed bit to slow pumps and fans down
  if (!primed) {
    if (millis() > prime_time) {
      primed = true;
      if (!pump_calibration_running) {
        fan1_speed = configuration.fan1_min;
        pump1_speed = configuration.pump1_min;
      }
    }
  }

  Serial.print("PUMP1: ");
  Serial.print(pump1_speed);
  Serial.print(" FAN1: ");
  Serial.println(fan1_speed);

}
