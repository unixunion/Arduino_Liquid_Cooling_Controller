#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "pgfx.h"
#include <avr/pgmspace.h>
#include "MemoryFree.h"
#include <EEPROM.h>
#include "EEPROMAnything.h"


// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 3
#define TEMPERATURE_PRECISION 12


// Oled on a UNO
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
#define MAX_MEM 2048

// button panel
#define FUNC_BTN_PIN 5
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

// Oled on a Pololu 32u4
//#define OLED_MOSI   9
//#define OLED_CLK   6
//#define OLED_DC    8
//#define OLED_CS    7
//#define OLED_RESET 5

// The display
U8GLIB_SSD1306_128X64 display(OLED_CLK, OLED_MOSI, OLED_CS, OLED_DC, OLED_RESET);

// refresh rate control of the display
unsigned int OLED_REFRESH = 125;
unsigned int LAST_OLED_REFRESH;
unsigned long now;

// storage for 4 temperature probes
float t1;
float t2;
float t3;
float t4;

// thresholds config structure
struct config_t
{
    int cpu_max;
    int gpu_max;
    int cold_max;
    int hot_max;
    int rad_max;
    int fan_max;
} configuration;


// gauges
pgfx_HBAR cpu = pgfx_HBAR(0,0,20,64,"CPU ",display, SPKR_PIN);
pgfx_HBAR gpu = pgfx_HBAR(26,0,20,64,"GFX ",display, SPKR_PIN);
pgfx_HBAR cbar = pgfx_HBAR(52,0,20,64,"COLD",display, SPKR_PIN);
pgfx_HBAR hbar = pgfx_HBAR(78,0,20,64," HOT",display, SPKR_PIN);
pgfx_HBAR rbar = pgfx_HBAR(104,0,20,64," RAD",display, SPKR_PIN);


// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress th1, th2, th3, th4;


void setup(void)
{

  // read the config from eeprom
  EEPROM_readAnything(0, configuration);

  // display the bootlogo
  display.setFont(u8g_font_ncenB14);
  display.setFontPosTop();
  display.setRot180();
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
  pinMode(SPKR_PIN, OUTPUT);
  tone(SPKR_PIN, 80, 125);
  
  // start serial port
  Serial.begin(9600);
  Serial.println(F("Psimax Liquid Cooling Controller"));

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
  if (!oneWire.search(th1)) Serial.println(F("Unable to find address for th1"));
  if (!oneWire.search(th2)) Serial.println(F("Unable to find address for th2"));
  if (!oneWire.search(th3)) Serial.println(F("Unable to find address for th3"));
  if (!oneWire.search(th4)) Serial.println(F("Unable to find address for th4"));

  // set the resolution
  sensors.setResolution(th1, TEMPERATURE_PRECISION);
  sensors.setResolution(th2, TEMPERATURE_PRECISION);
  sensors.setResolution(th3, TEMPERATURE_PRECISION);
  sensors.setResolution(th4, TEMPERATURE_PRECISION);


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
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.print(DallasTemperature::toFahrenheit(tempC));
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

void draw(float t1, float t2, float t3 ,float t4) {
  // graphic commands to redraw the complete screen should be placed here

//    display.drawStr(0,0, "H: " + t1);
//    display.print("H: ");
//    display.println(t1);
//    display.print("C: ");
//    display.println(t2);
//    display.print("dT:");
//    display.println(t1-t2);

    cpu.update(t1, configuration.cpu_max);
    gpu.update(t2, configuration.gpu_max);
    cbar.update(t4, configuration.cold_max);
    hbar.update(t3, configuration.hot_max);
    rbar.update(t3-t4, configuration.rad_max);
//    mbar.update(MAX_MEM-freeMemory(),MAX_MEM);
   

//    display.display();

}


void tweak_value(int &val, int btn_r_state, int btn_l_state) { 
  if (btn_r_state == HIGH && !button_press) {
    val++;
    button_press=true;
  } else if (btn_l_state == HIGH && !button_press) {
    val--;
    button_press=true;
  }
}



// setup function displays the GUI for settings
void drawSetup(void) {

  display.setFont(u8g_font_profont12);
  display.setFontPosTop();

  // read any button states
  btn_r_state = digitalRead(R_BTN_PIN);
  btn_l_state = digitalRead(L_BTN_PIN);
  btn_func_state = digitalRead(FUNC_BTN_PIN);

  // a little re-usable buffer for float to str
  char str_buffer[5];

  // page 1 of setup menu
  if (setup_menu_page==1) {
    display.drawStr(0, 0, "Setup 1/2");
    display.drawStr(0, 14, F("cpu:"));
    display.drawStr(0, 28, F("gpu:"));
    display.drawStr(0, 42, F("cold:"));
  
    // buffer to put arbitrairy chars into
    dtostrf(configuration.cpu_max, 2, 2, str_buffer);
    display.drawStr(40,14, str_buffer);
    dtostrf(configuration.gpu_max, 2, 2, str_buffer);
    display.drawStr(40,28, str_buffer);
    dtostrf(configuration.cold_max, 2, 2, str_buffer);
    display.drawStr(40,42, str_buffer);
  } else if (setup_menu_page==2) {
    display.drawStr(0, 0, "Setup 2/2:");
    display.drawStr(0, 14, F("hot:"));
    display.drawStr(0, 28, F("RAD:"));
    display.drawStr(0, 42, F("fan:"));
  
    // buffer to put arbitrairy chars into
    dtostrf(configuration.hot_max, 2, 2, str_buffer);
    display.drawStr(40,14, str_buffer);
    dtostrf(configuration.rad_max, 2, 2, str_buffer);
    display.drawStr(40,28, str_buffer);
    dtostrf(configuration.fan_max, 2, 2, str_buffer);
    display.drawStr(40,42, str_buffer);
  }

  // line under the current selected item
  display.drawHLine(0,(setup_menu_item*14)+11,128);

  // check for more func button presses
  if (btn_func_state == HIGH && !button_press) {
    Serial.println(F("Func Button"));
    button_press=true;
    setup_menu_item++;

    // every 3 items we change pages
    if (setup_menu_item>=4) {
      setup_menu_page++;
      setup_menu_item = 1;
    }

    // if page is 3 or higher, set to 0, which is exit
    if (setup_menu_page>=3) {
      setup_menu_page = 0; // notifies main loop exiting config
      display.firstPage();
      do {  
        display.drawStr(0,0,F("Writing EEPROM..."));
      } while( display.nextPage() ); 
      EEPROM_writeAnything(0, configuration);
    }
  }

  // CPU Temp on page 1
  if (setup_menu_page==1 && setup_menu_item == 1) {tweak_value(configuration.cpu_max, btn_r_state, btn_l_state);}

  // GPU Temp on page 1
  if (setup_menu_page==1 && setup_menu_item == 2) {tweak_value(configuration.gpu_max, btn_r_state, btn_l_state);}

  // Cold Temp on page 1
  if (setup_menu_page==1 && setup_menu_item == 3) {tweak_value(configuration.cold_max, btn_r_state, btn_l_state);}

  // Hot Water Temp on page 1
  if (setup_menu_page==2 && setup_menu_item == 1) {tweak_value(configuration.hot_max, btn_r_state, btn_l_state);}

  // Rad Temp on page 1
  if (setup_menu_page==2 && setup_menu_item == 2) {tweak_value(configuration.rad_max, btn_r_state, btn_l_state);}

  // Fan Relative to RAD on page 1
  if (setup_menu_page==2 && setup_menu_item == 3) {tweak_value(configuration.fan_max, btn_r_state, btn_l_state);}

  if (digitalRead(R_BTN_PIN)==LOW && digitalRead(L_BTN_PIN)==LOW && digitalRead(FUNC_BTN_PIN) == LOW && button_press) {
    button_press=false;
  }
}


void loop(void) {


  // check the func button state, doing a simple debounce
  btn_func_state = digitalRead(FUNC_BTN_PIN);
  
  //setup_menu_page=0 && btn_func_state == HIGH && setup_mode && btn_func_state != last_btn_func_stat
  if ( setup_menu_page==0 ) {
    // exiting setup mode
    setup_mode = false;
    setup_menu_page = 1;
    display.firstPage();
    last_btn_func_state = btn_func_state;
  } else if (btn_func_state == HIGH && !setup_mode && btn_func_state != last_btn_func_state) {
    // entering setup
    button_press = true;
    setup_mode = true;
    display.firstPage();
    drawSetup();
    last_btn_func_state = btn_func_state;
  } else {
    last_btn_func_state = btn_func_state;
  }

  if (!setup_mode) {
    Serial.println(F("Reading Sensors"));
    sensors.requestTemperatures();
    t1 = printData(th1);
    t2 = printData(th2);
    t3 = printData(th3);
    t4 = printData(th4);
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
  
}
