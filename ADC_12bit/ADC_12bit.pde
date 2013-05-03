// includes
#include <SPI.h>

// Pin defines
#define SELPIN 10 // Chip selection Pin 

// read values
int16_t readvalue; 

///////////////////////////////////////////////////////////
// Setup function. Launched initially by the Max32
///////////////////////////////////////////////////////////
void setup()
{ 
    //set pin mode and value
    pinMode(SELPIN, OUTPUT); 
    digitalWrite(SELPIN,HIGH); 

    Serial.begin(115200); 
 
    // Start the SPI library:
    SPI.begin();
    SPI.setDataMode(SPI_MODE3);
    SPI.setClockDivider(SPI_CLOCK_DIV4); //DIV2 gets us wrong values. Must be too fast....
} 

///////////////////////////////////////////////////////////
// Max32 loop function
///////////////////////////////////////////////////////////
void loop() 
{ 
    // Read and display values
    readvalue = read_adc(1);
    Serial.println(readvalue,DEC);
}

///////////////////////////////////////////////////////////
// ADC SPI channel reading function
//
// In   : chanNb    - Channel number
//
// Out  : advalue   - read value
///////////////////////////////////////////////////////////
int read_adc(int channel)
{
    int16_t adcvalue = 0;
    byte commandbits = B11000000; //command bits - start, mode, chn (3 bits), dont care (3 bits)

    //allow channel selection
    commandbits|=((channel-1)<<3);

    //Select adc
    digitalWrite(SELPIN,LOW);

    int16_t adcvalue0 = SPI.transfer(commandbits); // spits 1 intersting bit
    int16_t adcvalue1 = SPI.transfer(0x00); // spits 8 interesting bits
    int16_t adcvalue2 = SPI.transfer(0x00); // spits 3 interesting bits
    
    adcvalue0 &= 0b00000001; // only the first bit interests us
    adcvalue1 &= 0b11111111; // all bits interest us
    adcvalue2 &= 0b00000111; // 3 last bits interest us
    
    adcvalue = (adcvalue0 << 11) | (adcvalue1 << 3) | adcvalue2;

    //turn off adc
    digitalWrite(SELPIN, HIGH);
    
    return adcvalue;
}
