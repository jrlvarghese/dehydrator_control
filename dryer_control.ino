// Include the library
#include <TM1637Display.h> // for LED display
#include <Adafruit_AHTX0.h> // for AHT temperature and humidity sensor
#include <OneWire.h> // once wire communication for DS18B20 sensor
#include <DallasTemperature.h>  // DS18B20 communication

// Initialise AHT sensor
Adafruit_AHTX0 aht;

// Define the connections pins
#define CLK 6
#define DIO 5

#define MENU_PIN 2
#define ROT_A 4
#define ROT_B 3

#define DS18B20_PIN 7

// setup onewire and DS18B20 
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

// variables to manage rotary encoder
volatile bool pinState = false;
bool prevPinState = false;
volatile unsigned long lastInterruptTime = 0;
unsigned long debounce_delay = 15000;
volatile bool pinALastState = true;
bool pinACurrState = true;
// bool prevPinState = false;
unsigned int encoder_count = 0;
unsigned int prev_encoder_count = 0;

// variables for managing menu
bool menuState = false;
int menuItem = 0;
int prevMenuItem = -1;
bool selected = false;

// variable for internal timings and delay
unsigned long curr_time = 0;
unsigned long serial_time = 0;
unsigned long display_time = 0;
unsigned long menu_time = 0;
unsigned long sub_menu_time = 0;
int disp_count = 0;
unsigned long sensor_time = 0;

int value = 0;
int prevValue = -1;

// variables for humidity and temperature
int setTemperature = 0;
int setHumidity = 0;
int temperature = 0;
int humidity = 0;
float ds_temperature = 0;

// Create a display object of type TM1637Display
TM1637Display display = TM1637Display(CLK, DIO);

// Create an array that turns all segments ON
const uint8_t allON[] = {0xff, 0xff, 0xff, 0xff};

// Create an array that turns all segments OFF
const uint8_t allOFF[] = {0x00, 0x00, 0x00, 0x00};

// Create an array that sets individual segments per digit to display the word "dOnE"
const uint8_t done[] = {
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,           // d
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
  SEG_C | SEG_E | SEG_G,                           // n
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G            // E
};

// // Create degree celsius symbol
// const uint8_t celsius[] = {
//   SEG_A | SEG_B | SEG_F | SEG_G,  // Degree symbol
//   SEG_A | SEG_D | SEG_E | SEG_F   // C
// };

const uint8_t celsius[] = {
  0x00,
  SEG_D | SEG_E | SEG_F | SEG_A
};

const uint8_t humid[] = {
  SEG_B | SEG_C | SEG_E | SEG_F | SEG_G
};

const uint8_t temp[] = {
  SEG_D | SEG_E | SEG_F | SEG_G
};

const uint8_t dash[] = {
  SEG_G, SEG_G, SEG_G, SEG_G
};

const uint8_t arr[2][1] = {{temp},{humid}};

// variables for temp and humidity from aht_21
sensors_event_t aht_hum, aht_temp;

void setup() {
  Serial.begin(9600);
  // Set up the pins
  pinMode(MENU_PIN, INPUT_PULLUP);
  pinMode(ROT_A, INPUT);
  pinMode(ROT_B, INPUT);
  pinMode(CLK, OUTPUT);
  pinMode(DIO, OUTPUT);

  // Attach interrupt for the menu pin
  attachInterrupt(digitalPinToInterrupt(MENU_PIN),isr, RISING);

  // Set the brightness to 5 (0=dimmest 7=brightest)
	display.setBrightness(4);

	// Set all segments ON
	display.setSegments(dash);
  delay(500);
  //clear display
	display.clear();
  // set pin default pin states
  pinState = false;
  prevPinState = pinState;

  // initialize DS18B20 temperature sensor
  sensors.begin();

  // initialize humidity sensor AHT_20
  if (! aht.begin()) {
    // Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }

  // set default values for temperature and humidity
  setTemperature = 58;
  setHumidity = 60;
  // selected = false;
  // menuState = false;
}

void loop() {
  // keep track on current time to manage delays
  curr_time = millis();
  if(curr_time - serial_time > 1000){
    Serial.print("Menu: ");
    Serial.print(menuState);
    Serial.print(", PinState: ");
    Serial.print(pinState);
    Serial.print(", Previou pin state: ");
    Serial.println(prevPinState);
    serial_time = curr_time;
  }

  if(curr_time - sensor_time > 1000){
    // read humidity and temperature from aht_21
    aht.getEvent(&aht_hum, &aht_temp);
    // read temperature from DS18B20
    sensors.requestTemperatures();
    ds_temperature = sensors.getTempCByIndex(0);

    temperature = aht_temp.temperature;
    humidity = aht_hum.relative_humidity;
    sensor_time = curr_time;
  }

  // update display every 2 sec
  if(curr_time - display_time > 2000){
    disp_count>200?disp_count = 0:disp_count++;
    if(disp_count%2==0){
      display.clear();
      // display.showNumberDec(setTemperature, false, 2, 0);
      display.showNumberDec(int(ds_temperature), false, 2, 0);
      display.setSegments(celsius, 2, 2);
    }else{
      display.clear();
      // display.showNumberDec(setHumidity, false, 2, 0);
      display.showNumberDec(humidity, false, 2, 0);
      display.setSegments(humid, 1, 3);
    }
    
    display_time = curr_time;
  }
  
  
  // check if buttonState changed so enter into menu
  if(pinState != prevPinState){
    menuState = true;
    prevMenuItem = -1;
    prevPinState = pinState;
  }
  // if menu selected enter menu loop
  while(menuState){
    // update menu item 
    menuItem = updateViaEncoder(menuItem, 0, 1);
    //if menuItem selection changed update display and reset menu time
    if(menuItem != prevMenuItem){
      updateMenu(menuItem, setTemperature, setHumidity);   
      menu_time = curr_time;  // Update menu time to prevent exiting from menu
    }
    prevMenuItem = menuItem;  // Update the prevMenuItem to keep track on change

    if(pinState != prevPinState){
      selected = true;
      prevPinState = pinState;
    }
    
    while(selected){
      curr_time = millis();
      // Update set temperature or humidity value based on menu selection
      if(menuItem == 0)setTemperature = updateViaEncoder(setTemperature, 40, 65);
      if(menuItem == 1)setHumidity = updateViaEncoder(setHumidity, 20, 90);
      
      switch(menuItem){
        case 0:
          setTemperature = updateViaEncoder(setTemperature, 40, 65);
          if(prevValue != setTemperature){
            updateMenu(menuItem, setTemperature, setHumidity);
            sub_menu_time = curr_time; // to prevent from auto exiting if rotatory encoder is turning
            prevValue = setTemperature;
          }
          break;
        case 1:
          setHumidity = updateViaEncoder(setHumidity, 20, 90);
          if(prevValue != setHumidity){
            updateMenu(menuItem, setTemperature, setHumidity);
            sub_menu_time = curr_time; // to prevent from auto exiting if rotatory encoder is turning
            prevValue = setHumidity;
          }
          break;  
        default:
          break;      
          
      }
      // blink display to indicate it's selected
      if(curr_time - display_time > 300){
        disp_count++;
        // following two lines enable blinking effect
        if(disp_count%2 == 0)updateMenu(menuItem, setTemperature, setHumidity);
        else display.setSegments(allOFF);
        display_time = curr_time;
      }
      // if time exceeds a limit or when the button is pressed exit selection
      if((curr_time - sub_menu_time > 20000) || (pinState != prevPinState)){
        selected = false;
        menuState = false;
        prevMenuItem = -1;
        menuItem = 0;
        disp_count = 0;
        prevPinState = pinState;
        break;  // exit the loop
      }
    }// endo of while loop inside submenu
    
    
    curr_time = millis();
    if(curr_time - menu_time > 30000){
      menuState = false;
      menu_time = curr_time;
      prevMenuItem = -1;
      break;
    }
    
  }// end of while loop for main menu

}

/* Interrupt service routine to detect menu pin pressed or not*/
void isr(){
  // get the current millis()
  // unsigned long currentInterruptTime = millis();
  // debounce using the interval parameter
  if((long)(micros()-lastInterruptTime) >= debounce_delay){
    pinState = !pinState;
    // remember the interrupt time to maintain the debounce state
    lastInterruptTime = micros();
  }
}

/* Function to get encoder readings */
unsigned int getEncoder(unsigned int cnt){
  // Read present state of the rotary encoder pin A
  pinACurrState = digitalRead(ROT_A);
  // Check rotation status and increment or decrement based on direction
  if(pinACurrState != pinALastState && pinACurrState == true){
    if(digitalRead(ROT_B) != pinACurrState){
      cnt--;
    }else{
      cnt++;
    }
    //Serial.println(count);
  }
  // remember the last state
  pinALastState = pinACurrState;
  //    delay(1);
  return cnt;
}

int updateViaEncoder(int value, int min, int max){
  // Read present state of the rotary encoder pin A
  pinACurrState = digitalRead(ROT_A);
  // Check rotation status and increment or decrement based on direction
  if(pinACurrState != pinALastState && pinACurrState == true){
    if(digitalRead(ROT_B) != pinACurrState){
      value--;
    }else{
      value++;
    }
    //Serial.println(count);
  }
  // remember the last state
  pinALastState = pinACurrState;
  //    delay(1);
  // check if value exceeding the limits
  (value>max)?value=max:value;
  (value<min)?value=min:value;

  return value;
}

/* FUNCTION TO UPDATE MENU BASED ON SELECTION */
void updateMenu(int menuItem, int t, int h){
  // display.showNumberDec(menuItem);  // Update display if there is channge
  if(menuItem == 0){
    display.clear();
    display.setSegments(temp,1,0);
    display.showNumberDec(t, false, 2, 2);
  }
  if(menuItem == 1){
    display.clear();
    display.setSegments(humid,1,0);
    display.showNumberDec(h, false, 2, 2);
  }
}