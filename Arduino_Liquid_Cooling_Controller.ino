#include <Arduino.h>

#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "pgfx.h"
#include <avr/pgmspace.h>
//#include "MemoryFree.h"
#include <EEPROM.h>
#include "EEPROMAnything.h"


// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION_12 12
#define TEMPERATURE_PRECISION_9 9

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

// fan pin
#define FAN_PIN 3
unsigned long fan_adjust_time; // var for deceleration / acceleration
#define PUMP_PIN 5

// button panel
#define FUNC_BTN_PIN 8
#define R_BTN_PIN 4
#define L_BTN_PIN 6

// speaker
#define SPKR_PIN 7

// some state tracking vars for the UI
int btn_func_state = 0;
int last_btn_func_state = 0;
int btn_r_state = 0;
int btn_l_state = 0;
boolean setup_mode = false;
boolean button_press = false;
int setup_menu_page = 1;
int setup_menu_item = 1; // the initial setup item on the page

// debugger
//auto debug = [](const char *message){ Serial::printf << message;};

// The display
U8GLIB_SSD1306_128X64 display(OLED_CLK, OLED_MOSI, OLED_CS, OLED_DC, OLED_RESET);

// refresh rate control of the display
unsigned int OLED_REFRESH = 125;
unsigned int LAST_OLED_REFRESH;
unsigned long now;

// structure for temp
struct temp_struct
{
  float temperature;
  float max;
  char name[4];
};


// names of components, pointed to by config per "monitor"
static const char components[10][5] = {"CPU", "GFX", "HOT", "Cold", "Rad", "Aux1", "Aux2", "Rad1", "Rad2", "AMB"};
#define NUM_COMPONENTS 9

// array of temperature probes
//struct temp_struct probes[4];

// storage for 4 temperature probes
struct temp_struct t1;
struct temp_struct t2;
struct temp_struct t3;
struct temp_struct t4;
bool probe_error = false;

// thresholds config structure
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
long prime_time = 2000; // prime pumps and fans for 2 seconds
uint8_t fan1_speed;
uint8_t pump1_speed;
boolean primed = false; // fans and pumps at 100% for a second at least

// pump calibration
boolean pump_calibration_started; // initial start
boolean pump_calibration_running; // running
long pump_calibration_start_time; // start time
typedef struct {
  uint8_t speed;
  float temperature;
} calibration_sample;
calibration_sample pump_temperature_samples[16]; // 16 sample points
float calibration_start_temp; // initial temperature
boolean pump_calibration_waiting = false; // waiting to settle temperatures
long pump_calibration_change_time; // last time the speed of the pump was tweaked
uint8_t pump_sample_count = 0;

// gauges
pgfx_HBAR t1bar = pgfx_HBAR(0,0,20,64,"T1",display, SPKR_PIN);
pgfx_HBAR t2bar = pgfx_HBAR(26,0,20,64,"T2",display, SPKR_PIN);
pgfx_HBAR t3bar = pgfx_HBAR(52,0,20,64,"T3",display, SPKR_PIN);
pgfx_HBAR t4bar = pgfx_HBAR(78,0,20,64,"T4",display, SPKR_PIN);
pgfx_HBAR fan1bar = pgfx_HBAR(104,0,20,64,"F1",display, SPKR_PIN);


// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress th1, th2, th3, th4;

void(* resetFunc) (void) = 0;//declare reset function at address 0

void setup(void)
{
  // read the config from eeprom
  EEPROM_readAnything(0, configuration);

  // set the max thresholds from config
  t1.max = configuration.t1_max;
  strncpy( t1.name, components[configuration.t1_name], 5);
  t2.max = configuration.t2_max;
  strncpy( t2.name, components[configuration.t2_name], 5);
  t3.max = configuration.t3_max;
  strncpy( t3.name, components[configuration.t3_name], 5);
  t4.max = configuration.t4_max;
  strncpy( t4.name, components[configuration.t4_name], 5);

  // set the initial fan speeds to max
  fan1_speed = configuration.fan1_max;
  pump1_speed = configuration.pump1_max;

  // display the bootlogo
  display.setFont(u8g_font_ncenB14);
  display.setFontPosTop();
  //display.setRot180();
  display.firstPage();
  do {
//    display.drawXBMP( 0, 0, logo_width, logo_height, logo_bits);
//    display.setColorIndex(1);
  display.drawStr(0,0,"Psimax");
  display.drawStr(32,24,"Systems");
  display.setColorIndex(1);
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
  sensors.setResolution(th1, temperature_precision);
  sensors.setResolution(th2, temperature_precision);
  sensors.setResolution(th3, temperature_precision);
  sensors.setResolution(th4, temperature_precision);


  if (configuration.calibrate) {
    // set the minimun to 20% dity
    pump1_speed = 255*0.20;
    fan1_speed = 255*0.20;
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
float printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print("Temp C: ");
  Serial.println(tempC);
  // Serial.print(" Temp F: ");
  // Serial.print(DallasTemperature::toFahrenheit(tempC));
  return tempC;
}

// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress)
{
  Serial.print("Resolution: ");
  Serial.print(sensors.getResolution(deviceAddress));
  Serial.println();
}

// main function to print information about a device
float printData(DeviceAddress deviceAddress)
{
  Serial.print("Device Address: ");
  printAddress(deviceAddress);
  Serial.print(" ");
  float t = printTemperature(deviceAddress);
  Serial.println();
  return t;
}

void draw(temp_struct ts1, temp_struct ts2, temp_struct ts3 ,temp_struct ts4) {
  t1bar.update(ts1.temperature, ts1.max, ts1.name);
  t2bar.update(ts2.temperature, ts2.max, ts2.name);
  t3bar.update(ts3.temperature, ts3.max, ts3.name);
  t4bar.update(ts4.temperature, ts4.max, ts4.name);
  fan1bar.update(((float)fan1_speed/(float)configuration.fan1_max)*100.0, 100, ((float)pump1_speed/(float)configuration.pump1_max)*100.0, 100, "F/P");

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
//  else if (ts.temperature >= (0.85*ts.max)) {
//    // raise the fan speed a little at 85% capacity
//    fan1_speed = (configuration.fan1_max*0.85);
//  } else if (ts.temperature >= (0.60*ts.max)) {
//    fan1_speed = (configuration.fan1_max*0.60);
//  } else if (ts.temperature >= (0.40*ts.max)) {
//    fan1_speed = (configuration.fan1_max*0.40);
//  } else if (ts.temperature >= (0.20*ts.max)) {
//    fan1_speed = (configuration.fan1_max*0.20);
//  } else if (ts.temperature <= (0.20*ts.max)) {
//    fan1_speed = (configuration.fan1_min);
//  }
}


// calibration based on the cpu and gpu probe temperatures
// possible constants, desired temperature, ambient temperature, 1/16th of 255 ( rate of increase ) 
void calibratePump(temp_struct ts1) {
  // upon initialiation, capture initial temperatures, start time and set start bit

  if (pump_calibration_running) {
    if (!pump_calibration_started) {
      Serial.println("Starting Calibration");
      calibration_start_temp = ts1.temperature;
      pump_calibration_change_time = millis();
      pump_calibration_started = true;
      pump_calibration_waiting = true;
    // if waiting for temperatures to settle, then do that, checking wait time
    } else if (pump_calibration_waiting) {
      Serial.println("Checking enough time has passed");
      if ((millis()-pump_calibration_change_time) > configuration.calibration_time*1000 ) {
        Serial.println("Saving results");
        pump_calibration_waiting=false;
        pump_temperature_samples[pump_sample_count] = (calibration_sample){pump1_speed, ts1.temperature};
        pump_sample_count++;
      }

    } else {

      Serial.println("Increment speed?");
      
      // check if max speed has been achieved, finalize calibration
      if (pump_sample_count >= 16) {
        Serial.println("Max speed reached");
        pump_calibration_running = false;
        for (int i=0; i<16; i++) {
          Serial.print("Sample: ");
          Serial.print(i);
          Serial.print(" Speed: ");
          Serial.print(pump_temperature_samples[i].speed);
          Serial.print(" Temperature: ");
          Serial.print(pump_temperature_samples[i].temperature);
          Serial.print(" Delta from Start: ");
          Serial.println(calibration_start_temp - pump_temperature_samples[i].temperature);
        }
        configuration.calibrate = false;
        EEPROM_writeAnything(0, configuration);
        delay(1000);
        resetFunc();
      }

      // increase speed by 1/16th of that is left after deducting the 20% minimun
      pump1_speed=pump1_speed+((255-(255*0.20))/16);

      // Testing
      fan1_speed=fan1_speed+((255-(255*0.20))/16);
      
      pump_calibration_change_time = millis();
      pump_calibration_waiting = true;
      Serial.println("Speed increased");
    }
  } else {
    // dont do anything
  }


}



// headless monitor the temperatures and tweak fan/pump/aux
void monitor_fans(temp_struct ts1, temp_struct ts2, temp_struct ts3, temp_struct ts4) {

  // if system is still priming, just return
  if (!primed || pump_calibration_running) {
    return;
  }

  // put all the hot temps together and aggregates
  int temperature; // = ts1.temperature + ts2.temperature;
  //  + ts3.max + ts4.max;
  int max; // = ts1.max + ts2.max;

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
//    Serial.println("Slowing Fan");
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
    if (fan1_speed < (configuration.fan1_max*0.20)) {
      fan1_speed++;
      fan1_speed++;
    }
  } else if (temperature >= (0.20*max)) {
    if (fan1_speed < (configuration.fan1_max*0.05)) {
      fan1_speed++;
      fan1_speed++;
    }
  } else if (temperature <= (0.20*max)) {
    fan1_speed = (configuration.fan1_min);
  }


  // clamp to min speed
  if (fan1_speed < configuration.fan1_min) {
    fan1_speed = configuration.fan1_min;
  }

  if (fan1_speed > configuration.fan1_max) {
    fan1_speed = configuration.fan1_max;
  }

//  Serial.print("FanSpeed:");
//  Serial.println(fan1_speed);

}


// inc / dec int based on input keys
void tweak_value(uint8_t &val, int btn_r_state, int btn_l_state) {
//  Serial.print(F("changing value: "));
//  Serial.println(val);
  if (btn_r_state == HIGH && !button_press) {
//    Serial.println("inc");
    val++;
    button_press=true;
  } else if (btn_l_state == HIGH && !button_press) {
//    Serial.println("dev");
    val--;
    button_press=true;
  }
//  Serial.println("out");
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

// converts booleans to string for easy handling
inline const char * const BoolToString(bool b)
{
  return b ? "True" : "False";
}


// locks a variable to between inclusive min and max
// uint8's tend to rollover, for some reason, we have
// a one-way rollover here..
void clamp(uint8_t &val, int minv, int maxv) {
  if (val<=minv ) {
//    Serial.println(F("Clamping 0"));
    val = minv;
  } else if (val>=maxv) {
//    Serial.println(F("Clamping Max"));
    val = maxv;
  }
}




// setup function displays the GUI for settings
void drawSetup(void) {

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
    display.drawStr(0, 0, "Setup 2/6:");
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
    display.drawStr(0, 0, "Setup 3/6:");
    display.drawStr(0, 14, "12bit:");
    display.drawStr(0, 28, "aux1:");
    display.drawStr(0, 42, "aux2:");

    // buffer to put arbitrairy chars into
    display.drawStr(40,14, BoolToString(configuration.hires));

  } else if (setup_menu_page==4) {
    display.drawStr(0, 0, "Setup 4/6:");
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
    display.drawStr(0, 0, "Setup 5/6:");
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
    display.drawStr(0, 0, "Setup 6/6:");
    display.drawStr(0, 14, "FAN1_MIN:");
    display.drawStr(0, 28, "FAN1_MAX:");
    display.drawStr(0, 42, "PUMP1_MIN:");

    clamp(configuration.fan1_min, 0, 255);
    dtostrf(configuration.fan1_min, 2, 2, str_buffer);
    display.drawStr(60,14, str_buffer);

    clamp(configuration.fan1_max, configuration.fan1_min, 255);
    dtostrf(configuration.fan1_max, 2, 2, str_buffer);
    display.drawStr(60,28, str_buffer);

    clamp(configuration.pump1_min, 0, 255);
    dtostrf(configuration.pump1_min, 2, 2, str_buffer);
    display.drawStr(60,42, str_buffer);

  } else if (setup_menu_page==7) {
    display.drawStr(0, 0, "Setup 7/7:");
    display.drawStr(0, 14, "PUMP1_MAX:");
     display.drawStr(0, 28, "CAL_TIME:");
    display.drawStr(0, 42, "CALIBRATE:");

    clamp(configuration.pump1_max, configuration.pump1_min, 255);
    dtostrf(configuration.pump1_max, 2, 2, str_buffer);
    display.drawStr(60,14, str_buffer);

     clamp(configuration.calibration_time, 3, 120);
     dtostrf(configuration.calibration_time, 2, 2, str_buffer);
     display.drawStr(60,28, str_buffer);

    // calibration bool
    display.drawStr(60,42,  BoolToString(configuration.calibrate));

    // clamp(configuration.fan1_max, configuration.fan1_min, 255);
    // dtostrf(configuration.fan1_max, 2, 2, str_buffer);
    // display.drawStr(60,28, str_buffer);
    //
    // clamp(configuration.pump1_min, 0, 255);
    // dtostrf(configuration.pump1_min, 2, 2, str_buffer);
    // display.drawStr(60,42, str_buffer);

  }


  // line under the current selected item
  display.drawHLine(0,(setup_menu_item*14)+11,128);

  // check for more func button presses
  if (btn_func_state == HIGH && !button_press) {
//    Serial.println(F("Func Button"));
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

  // CPU Temp on page 1
  if (setup_menu_page==1 && setup_menu_item == 1) {tweak_value(configuration.t1_max, btn_r_state, btn_l_state);}

  // GPU Temp on page 1
  if (setup_menu_page==1 && setup_menu_item == 2) {tweak_value(configuration.t2_max, btn_r_state, btn_l_state);}

  // Cold Temp on page 1
  if (setup_menu_page==1 && setup_menu_item == 3) {tweak_value(configuration.t3_max, btn_r_state, btn_l_state);}

  // Hot Water Temp on page 1
  if (setup_menu_page==2 && setup_menu_item == 1) {tweak_value(configuration.t4_max, btn_r_state, btn_l_state);}

  // Rad Temp on page 1
  if (setup_menu_page==2 && setup_menu_item == 2) {tweak_value(configuration.aux1_max, btn_r_state, btn_l_state);}

  // Fan Relative to RAD on page 1
  if (setup_menu_page==2 && setup_menu_item == 3) {tweak_value(configuration.aux2_max, btn_r_state, btn_l_state);}

  // Fan Relative to RAD on page 1
  if (setup_menu_page==3 && setup_menu_item == 1) {tweak_value(configuration.hires, btn_r_state, btn_l_state);}
//  if (setup_menu_page==3 && setup_menu_item == 2) {tweak_value(configuration.hires, btn_r_state, btn_l_state);}
//  if (setup_menu_page==3 && setup_menu_item == 3) {tweak_value(configuration.hires, btn_r_state, btn_l_state);}


  // names of components
  if (setup_menu_page==4 && setup_menu_item == 1) {tweak_value(configuration.t1_name, btn_r_state, btn_l_state);}
  if (setup_menu_page==4 && setup_menu_item == 2) {tweak_value(configuration.t2_name, btn_r_state, btn_l_state);}
  if (setup_menu_page==4 && setup_menu_item == 3) {tweak_value(configuration.t3_name, btn_r_state, btn_l_state);}
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
//    Serial.println(F("Setup: Button Release"));
    button_press=false;
  }
}


void loop(void) {

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
    int m = millis();
    sensors.requestTemperatures();
    // Serial.println(millis()-m);
    t1.temperature = printData(th1);
    t2.temperature = printData(th2);
    t3.temperature = printData(th3);
    t4.temperature = printData(th4);

    // set fans
    monitor_fans(t1, t2, t3, t4);

    // check the probes compared to the max levels
    monitor(t1);
    monitor(t2);
    monitor(t3);
    monitor(t4);

    // call calibrate method if in calibration mode with the GFX probe temps
    if (pump_calibration_running && primed) {
      calibratePump(t2);
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

}
