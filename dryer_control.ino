// Include the library
#include <TM1637Display.h>

// Define the connections pins
#define CLK 2
#define DIO 3

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

void setup() {
  // Set the brightness to 5 (0=dimmest 7=brightest)
	display.setBrightness(4);

	// Set all segments ON
	display.setSegments(allON);

	delay(1000);
	display.clear();

}

void loop() {
	
	// Prints 15°C
	int temperature = 15;
  for(int i=0; i<100; i++){
    temperature = i;
    display.showNumberDec(temperature, false, 2, 0);
	  display.setSegments(celsius, 2, 2);
    delay(500);
  }
	

	delay(2000);
	// Prints dOnE
	// display.setSegments(done);
}