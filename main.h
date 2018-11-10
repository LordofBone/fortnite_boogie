// for most code thanks to:
// https://www.dfrobot.com/wiki/index.php/DFPlayer_Mini_SKU:DFR0299
// https://www.dfrobot.com/wiki/index.php/Gravity:_Flexible_8x8_RGB_LED_Matrix_SKU:_DFR0461
// https://www.arduino.cc/en/Tutorial/Button

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN 6

// setup leds
Adafruit_NeoPixel strip = Adafruit_NeoPixel(64, PIN, NEO_GRB + NEO_KHZ800);

// setup serial
SoftwareSerial mySoftwareSerial(10, 11); // RX, TX

// setup mp3 device
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;     // the number of the pushbutton pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

// set boogie mode to off
bool boogieOn = false;

// set timer to over 60000 (1 min) to ensure the first mp3 kicks off first time
static unsigned long timer = 60001;

void setup()
{

  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  // End of trinket special code

  // start led setup
  strip.begin();
  // found this brightness level to be enough to light up surroundings and not stress battery pack too much
  strip.setBrightness(120);
  strip.show(); // Initialize all pixels to 'off'
  
  // begin serial (for debugging etc.)
  mySoftwareSerial.begin(9600);
  Serial.begin(115200);
  
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(mySoftwareSerial)) {  // Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  Serial.println(F("DFPlayer Mini online."));
  
  // decent volume that doesnt push the speaker/batteries too far  
  myDFPlayer.volume(20);  // Set volume value. From 0 to 30
 
}

void loop()
{
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn boogie on after 5 seconds and reset timer to ensure it plays mp3
    timer = 60001;
    delay(5000);
    boogieOn = !boogieOn;

  }
  // during loop if boogie mode is on and play mp3
  if (boogieOn == true) {
	// this ensures that while looping the overall process mp3 will loop after a minute has passed, not before
    if (millis() - timer > 60000) {
      timer = millis();
      myDFPlayer.play(1);  // Play the first mp3
    }
    
	// initiate led cycle
    rainbowCycle(1);

    if (myDFPlayer.available()) {
      printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.

    
    }
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
      // turn boogie off and stop music      
      boogieOn = !boogieOn;
      myDFPlayer.pause();  // pause the mp3
      colorWipe(0, 1); // clear the led panel
      delay(5000);
  
    }
  }
}

// detail for printing information to serial for debugging
void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

  // Fill the dots one after the other with a color
  void colorWipe(uint32_t c, uint8_t wait) {
    for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
    }
  }
  
// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
uint16_t i, j;

for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
  for(i=0; i< strip.numPixels(); i++) {
	strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
  }
  strip.show();
  delay(wait);
 }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
WheelPos = 255 - WheelPos;
 if(WheelPos < 85) {
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
 }
 if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
 }
 WheelPos -= 170;
 return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}