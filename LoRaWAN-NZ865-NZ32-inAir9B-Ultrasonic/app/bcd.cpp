#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bcd.h"

using namespace std;

/*!
*  Transforms a signed integer up to 99 to a 2 digits packed bcd
*/
static uint8_t uint2bcd2(uint16_t input)
{
    uint8_t high = 0;
    while (input >= 10)  {               // Count tens
        high++;
        input -= 10;
    }
    return  (uint8_t) ((high << 4) | input);        // Add ones and return answer
}

/*!
*  Transforms a signed integer up to 999 to a 3 digits packed bcd
*  with sign (0xD is negative, 0xC is positive)
*/
static uint16_t int2bcd3(int16_t input)
{
    uint8_t high = 0;
    uint8_t sign = ((input >= 0) ? 0xC : 0xD);
    input = abs(input);

    while (input >= 100)  {         // Count hundreds
        high++;
        input -= 100;
    }
    high <<= 4;
    while (input >= 10) {                // Count tens
        high++;
        input -= 10;
    }
    return  (((high << 4) | input) << 4) | sign;        // Add ones and return answer
}

/*!
*  Transforms an unsigned integer up to 999 to a 3 digits packed bcd
*/
static uint16_t uint2bcd3(uint16_t input)
{
    uint16_t high = 0;
    while (input >= 100)  {         // Count hundreds
        high++;
        input -= 100;
    }
    high <<= 4;
    while (input >= 10) {                // Count tens
        high++;
        input -= 10;
    }
    return  (high << 4) | input;        // Add ones and return answer
}

/*!
*  Reverse copies an integer to a position in a byte array
*/

template <class TYPE>
static void revcopy (uint8_t *outbuf, uint8_t outbufpos, TYPE val ) {
    uint8_t *array = (uint8_t*)(&val);
    for (uint8_t i = 0; i < sizeof(val); i++) {
        outbuf[i + outbufpos] = array[sizeof(val) - 1 - i];
    }
  }

  void formatPayload(uint8_t *outbuf, uint16_t range, int16_t temperature, uint16_t volts ) {
    revcopy(outbuf, 0, uint2bcd3(range));
    revcopy(outbuf, 2, int2bcd3(temperature));
    revcopy(outbuf, 4, uint2bcd3(volts));
}

