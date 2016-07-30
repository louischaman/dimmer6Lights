/************************************************************************************************************************************
 * ShiftPWM blocking RGB fades example, (c) Elco Jacobs, updated August 2012.
 *
 * ShiftPWM blocking RGB fades example. This example uses simple delay loops to create fades.
 * If you want to change the fading mode based on inputs (sensors, buttons, serial), use the non-blocking example as a starting point.
 * Please go to www.elcojacobs.com/shiftpwm for documentation, fuction reference and schematics.
 * If you want to use ShiftPWM with LED strips or high power LED's, visit the shop for boards.
 ************************************************************************************************************************************/
 
// ShiftPWM uses timer1 by default. To use a different timer, before '#include <ShiftPWM.h>', add
// #define SHIFTPWM_USE_TIMER2  // for Arduino Uno and earlier (Atmega328)
// #define SHIFTPWM_USE_TIMER3  // for Arduino Micro/Leonardo (Atmega32u4)

// Clock and data pins are pins from the hardware SPI, you cannot choose them yourself if you use the hardware SPI.
// Data pin is MOSI (Uno and earlier: 11, Leonardo: ICSP 4, Mega: 51, Teensy 2.0: 2, Teensy 2.0++: 22) 
// Clock pin is SCK (Uno and earlier: 13, Leonardo: ICSP 3, Mega: 52, Teensy 2.0: 1, Teensy 2.0++: 21)

// You can choose the latch pin yourself.
const int ShiftPWM_latchPin=8;

// ** uncomment this part to NOT use the SPI port and change the pin numbers. This is 2.5x slower **
// #define SHIFTPWM_NOSPI
// const int ShiftPWM_dataPin = 11;
// const int ShiftPWM_clockPin = 13;


// If your LED's turn on if the pin is low, set this to true, otherwise set it to false.
const bool ShiftPWM_invertOutputs = false; 

// You can enable the option below to shift the PWM phase of each shift register by 8 compared to the previous.
// This will slightly increase the interrupt load, but will prevent all PWM signals from becoming high at the same time.
// This will be a bit easier on your power supply, because the current peaks are distributed.
const bool ShiftPWM_balanceLoad = false;

#include <ShiftPWM.h>   // include ShiftPWM.h after setting the pins!

// Here you set the number of brightness levels, the update frequency and the number of shift registers.
// These values affect the load of ShiftPWM.
// Choose them wisely and use the PrintInterruptLoad() function to verify your load.
// There is a calculator on my website to estimate the load.

const constexpr unsigned char maxBrightness = 255;
const constexpr unsigned char pwmFrequency = 200;
const constexpr uint8_t numRegisters = 1;
const constexpr uint8_t numLights = 8;
const constexpr uint8_t rxBufferSize = 2+ numLights * 2;  // Size of the Rx buffer.
/**
 * @brief   convert an ascii character to hex.
 * @param   ascii character '0'-'9' or 'a'-'f'
 * @return  0-16 or 0 if invalid character.
 */
uint8_t hexToInt(uint8_t character) {
    if ((character >= '0') && (character <= '9')) return (character - '0');
    else if ((character >= 'a') && (character <= 'f')) {
        return ((character - 'a') + 10);
    } else return 0;
}

/**
 *  @brief  Simple serial protocol handler. 
 *  @note   The protocol consists of a start character is followed by a series of hexadecimal values, sent as 2 ascii characters.
 */
const constexpr uint8_t startChar = 'l';
void serialRx(uint8_t * lev) {
    uint8_t buf[rxBufferSize];
    memset(buf, 0, rxBufferSize);
    // Wait for start char
    while(Serial.read() != startChar) {}
    // fill serial buffer.
    for (uint8_t i = 0; i < rxBufferSize; i++) {
        while(!Serial.available()) {}
        buf[i] = Serial.read();
        // parse the characters:
        if ( i & 0x01 ) {
            uint8_t temp1 = buf[i-1];
            uint8_t temp2 = buf[i];
            // parse:
            uint8_t level = hexToInt(temp1);
            level += hexToInt(temp2) << 4;
            Serial.print(level, HEX);
            Serial.print(' ');
            lev[0] = level;
        }
    }
}

void setup(){
  Serial.begin(115200);

  // Sets the number of 8-bit registers that are used.
  ShiftPWM.SetAmountOfRegisters(numRegisters);

  // SetPinGrouping allows flexibility in LED setup. 
  // If your LED's are connected like this: RRRRGGGGBBBBRRRRGGGGBBBB, use SetPinGrouping(4).
  ShiftPWM.SetPinGrouping(1); //This is the default, but I added here to demonstrate how to use the funtion
  
  ShiftPWM.Start(pwmFrequency,maxBrightness);
}



void loop()
{    
  uint8_t levels[8];
  serialRx(levels);
  // Turn all LED's off.
  ShiftPWM.SetAll(levels[0]);


  // Fade in and out 2 outputs at a time
//  for(int output=0;output<numRegisters*8-1;output++){
//    ShiftPWM.SetAll(0);
//    for(int brightness=0;brightness<maxBrightness;brightness++){
//      ShiftPWM.SetOne(output+1,brightness);
//      ShiftPWM.SetOne(output,maxBrightness-brightness);
//      delay(1);
//    }
//  }
}
