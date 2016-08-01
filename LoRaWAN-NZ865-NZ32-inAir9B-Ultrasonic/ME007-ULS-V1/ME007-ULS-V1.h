#ifndef ULTRASONIC_ME007ULSV1_H
#define ULTRASONIC_ME007ULSV1_H

#include "mbed.h"

/**
*   Driver for ME007-ULS-V1 Waterproof Ultrasonic Detector Module
*/
class UltraSonic {
        
protected:

    DigitalOut _trigger; // pin to trigger a sensor reading
    DigitalIn _receive; // pin on which to receive serial data from sensor
    Serial _usonic;
    #define framesize 5  // This is always 5 for ME007-ULS-V1 in serial mode
    #define sampleBufSize 10 // Determines number of samples read each time distance measured

    uint8_t framebuf[framesize];  // buffers a single frame
    uint8_t nDataBytesRead;
    uint8_t sampleBufIndex;   
       
    Timeout getusonic;
    
    struct samplebuf {
      uint16_t value; // distance value
      uint8_t votes;  // votes per distance value
    }; samplebuf SampleBuf[sampleBufSize];  // buffers a set of distance reading samples
    
    int16_t temperature;
    
     enum states {READ_HEADER, READ_DATA} state; /** Data frame read states   */
     
public:
    /**
    * UltraSonic constructor
    * @param trigger pin - triggers an ultrasonic reading
    * @param receive pin - receives serial port characters
    */
    UltraSonic(PinName trigger, PinName receive);
     
      
    volatile bool distanceAvailable;  // Is true when a final distance reading is available
    
    /**
    *   Gets the distance and temperature
    *   @return distance in centimeters
    *   @param temp - is centigrade temperature
    */
    uint16_t getDistance(int16_t *temp);
    
    /**
    * Triggers a sample reading
    */
    void triggerSample(void);

    /**
    * Set pins to off
    */
    void pinsOff(void);

        /**
    * Set pins to on
    */
    void pinsOn(void);
    
 
 private:
    
    /**
    *   Callback function that is invoked when a distance and temperature frame character is received
    */    
    void RxInterrupt(void);
    

}; // class end

#endif //ULTRASONIC_ME007ULSV1