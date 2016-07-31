/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: End device comissioning parameters

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __LORA_COMISSIONING_H__
#define __LORA_COMISSIONING_H__

/*!
 * When set to 1 the application uses the Over-the-Air activation procedure
 * When set to 0 the application uses the Personalization activation procedure
 */
#define OVER_THE_AIR_ACTIVATION                     1

/*!
 * Indicates if the end-device is to be connected to a private or public network
 */
#define LORAWAN_PUBLIC_NETWORK                      true

#if( OVER_THE_AIR_ACTIVATION != 0 )

/*!
 * IEEE Organizationally Unique Identifier ( OUI ) (big endian)
 */
#define IEEE_OUI         0x01, 0x02, 0x03
//#define IEEE_OUI           0xBE,0x7A,0x02

/*!
 * Mote device IEEE EUI (big endian)
 */
// #define LORAWAN_DEVICE_EUI                          { IEEE_OUI, 0x44, 0x55, 0x66, 0x77, 0x88 }
#define LORAWAN_DEVICE_EUI                          { IEEE_OUI, 0x04, 0x05, 0x06, 0x07, 0x08 }
//#define LORAWAN_DEVICE_EUI { IEEE_OUI,0x00,0x00,0x00,0x00,0x21 } 


/*!
 * Application IEEE EUI (big endian)
 */
//#define LORAWAN_APPLICATION_EUI                     { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
#define LORAWAN_APPLICATION_EUI                     { 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01 }


/*!
 * AES encryption/decryption cipher application key
 */
#define LORAWAN_APPLICATION_KEY                     { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16 }

#else

/*!
 * Current network ID
 */
#define LORAWAN_NETWORK_ID                          ( uint32_t )0

/*!
 * Device address on the network (big endian)
 */
//#define LORAWAN_DEVICE_ADDRESS                      ( uint32_t )0x12345678
#define LORAWAN_DEVICE_ADDRESS                      ( uint32_t )0x01C3D543

/*!
 * AES encryption/decryption cipher network session key
 */
//#define LORAWAN_NWKSKEY  { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }
#define LORAWAN_NWKSKEY  {0xA5,0x86,0x87,0x0A,0xAE,0x0C,0xA9,0x07,0xC9,0xCD,0x0F,0x32,0x7F,0xB4,0x4C,0xBE}


/*!
 * AES encryption/decryption cipher application session key
 */
// #define LORAWAN_APPSKEY   { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }
#define LORAWAN_APPSKEY   { 0xC9,0x44,0x13,0xCD,0x62,0xE2,0xA2,0xA7,0x0D,0x82,0x55,0x66,0x85,0xD0,0x01,0xF7 }

#endif

#endif // __LORA_COMISSIONING_H__
