#include "mbed.h"
#include "ME007-ULS-V1.h"

    UltraSonic::UltraSonic(PinName trigger, PinName receive): _trigger(trigger), _receive(receive), _usonic(NC,receive) {
    
    _trigger=1; // Set trigger high, low power, standby mode
   
    _usonic.baud(9600);
    _usonic.attach(this, &UltraSonic::RxInterrupt);
    nDataBytesRead = 0;
    sampleBufIndex = 0;
    distanceAvailable = false;    
    state = READ_HEADER;
    temperature = 999; // Initialise to indicate error condition
}

/**
*   Get error corrected distance and temperature
*/
uint16_t UltraSonic::getDistance(int16_t *temp) {
    
    int i,j;
    for (i = 0; i < sampleBufSize; i++) {
        for ( j = i+1; j<sampleBufSize; j++) {
            if ((SampleBuf[j].value == SampleBuf[i].value) && (SampleBuf[j].value != 0)) {
                SampleBuf[i].votes += 1;
                SampleBuf[j].value = 0;  // remove matched value
            }
        }
    }
     /**  Returns the sample with highest 'votes' in the samples buffer
     */
     int highest = 0;
     uint8_t index = 0;
     for (i = 0; i < sampleBufSize; i++) {
         if (SampleBuf[i].value != 0)
            if (SampleBuf[i].votes > highest) {
                highest = SampleBuf[i].votes;
                index = i;
            }
     }
    distanceAvailable = false;   // reset

    // Check range and temperature are valid
    // if not return error indicator
    if ((temperature < -998) || (temperature > 998))
      temperature = (int16_t) 999; // Error
    *temp = temperature; 
    int16_t range = SampleBuf[index].value/10;
    if ((range < 20 ) || (range > 400))
      return (uint16_t ) 999; // Error
    else
      return (uint16_t) range; 
}

/**
*  Trigger a new sample reading by wiggling trigger pin
*/

void UltraSonic::triggerSample(void) {
    _trigger=0;
    wait_us(120); // low for min of 100us
    _trigger=1; // back to standby mode  
} 

  void UltraSonic::pinsOn(void) {
         _trigger = DigitalOut (PA_1, 1);  // sensor in stanby mode
        // _usonic = Serial (NC, PA_3); // renable USB serial
       //  _usonic.baud(9600);
       //_receive = DigitalIn (PA_3);
       // _usonic.attach(this, &UltraSonic::RxInterrupt);
    };

/** Interupt Routine to read in data from serial port
*   Identifies a new serial data frame from sensor by frame header
*   Adds sampled distance readings to a samples buffer
*   Sets temperature from last sample
*/
void UltraSonic::RxInterrupt() {

   uint8_t bytein;
   
   if (!(_usonic.readable())) return; //check input is valid
   /** Reads byte until a start of frame header byte found  */
   bytein = _usonic.getc();
   if ((state == READ_HEADER) && (bytein == 0xFF)) {  // NB - frame header byte is alway 0xFF
        //process the byte as a header byte    
        state = READ_DATA;
        nDataBytesRead = 0; // Skip frame header byte
    }
    else { 
        /** Start of frame header byte found so read 5 data bytes  */  
        if (state == READ_DATA) {
            // process incoming serial byte
            framebuf[nDataBytesRead] = bytein;
            ++nDataBytesRead;   
            if (nDataBytesRead == framesize) {
                // Complete data frame is available in frame buffer
                state = READ_HEADER; // back to searching for frame start (header) byte
                SampleBuf[sampleBufIndex].value = framebuf[0]<<8 | framebuf[1]; // high and low distances bytes to samples buffer
                // Note we aren't checking the serial data frame checksum as data frame is fairly reliable
                SampleBuf[sampleBufIndex].votes = 0;  // Initialise sample buffer
                sampleBufIndex++;
                if (sampleBufIndex < sampleBufSize) {
                    getusonic.attach(this,&UltraSonic::triggerSample, 0.06);  // Trigger next ultrasonic range sample - NB must be > than 60ms!
                }
                else {
                    sampleBufIndex = 0; // reset to beginning
                    distanceAvailable = true; // Samples buffer is full so distance can be estimated
                    // Get the temperature from last sample - as temperature reading is relatively consistent
                    temperature = framebuf[2]<<8 | framebuf[3]; // Temperature in 1/10 degree centigrade
                    if (framebuf[2] && 0x80) {  // hightest bit set for negative temperature
                        temperature = -((framebuf[2] & 0x7F)<<8 | framebuf[3]); // Clear negative bit and set temperature negative
                    }
                    else {
                        temperature = framebuf[2]<<8 | framebuf[3]; // Temperature positive
                    } 
                }
            } 

        }
    }

    return; 

 }
