// Include the library
#include <TM1637Display.h>

// Define the connections pins
#define CLK 6
#define DIO 5

#define MENU_PIN 2
#define ROT_A 4
#define ROT_B 3

// variables to manage rotary encoder
volatile bool pinState = false;
bool prevPinState = true;
volatile unsigned long lastInterruptTime = 0;
unsigned long debounce_delay = 15000;
volatile bool pinALastState = true;
bool pinACurrState = true;
// bool prevPinState = false;
unsigned int encoder_count = 0;
unsigned int prev_encoder_count = 0;

// variables for managing menu
bool menuState = false;

// variable for internal timings and delay
unsigned long curr_time = 0;
unsigned long serial_time = 0;
unsigned long display_time = 0;
unsigned long menu_time = 0;
int disp_count = 0;

int value = 0;
int prevValue = -1;

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
  SEG_G | SEG_D | SEG_E
};

const uint8_t humid[] = {
  SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,
  0x00
};

const uint8_t dash[] = {
  SEG_G, SEG_G, SEG_G, SEG_G
};

void setup() {
  Serial.begin(9600);
  // Set up the pins
  pinMode(MENU_PIN, INPUT_PULLUP);
  pinMode(ROT_A, INPUT);
  pinMode(ROT_B, INPUT);
  pinMode(CLK, OUTPUT);
  pinMode(DIO, OUTPUT);

  // Attach interrupt for the menu pin
  attachInterrupt(digitalPinToInterrupt(MENU_PIN),isr, FALLING);

  // Set the brightness to 5 (0=dimmest 7=brightest)
	display.setBrightness(4);

	// Set all segments ON
	display.setSegments(dash);
  delay(500);

	display.clear();

  prevPinState = pinState;

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

  // update display every 1 sec
  if(curr_time - display_time > 1000){
    display.showNumberDec(value);
    display_time = curr_time;
  }
  
  // check if buttonState changed so enter into menu
  if(pinState != prevPinState){
    menuState = true;
    prevPinState = pinState;
  }
  // if menu selected enter menu loop
  while(menuState){
    // Get encoder readings
    encoder_count = getEncoder(encoder_count);
    if(encoder_count > prev_encoder_count){
      value++;
      // update menu time to prevent exiting from menu
      menu_time = curr_time;
    }
    if(encoder_count < prev_encoder_count){
      value--;
      // update menu time to prevent exiting from menu
      menu_time = curr_time;
    }
    prev_encoder_count = encoder_count;

    // update display with a blink effect inside menu
    if(millis() - display_time > 300){
      disp_count++; // keep a track on count to create blink effect
      if(disp_count%2==0){
        display.showNumberDec(value);
      }else{
        display.setSegments(allOFF);
      }
      if(disp_count > 20)disp_count = 0;
      display_time = curr_time;
    }
    // exit menu if after 2 seconds
    if(curr_time - menu_time > 30000){
      menuState = false;
      menu_time = curr_time;
      break;
    }
    
  }

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