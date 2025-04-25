#ifndef __LORA_COMMISSIONING_H__
#define __LORA_COMMISSIONING_H__

/*!
 * When set to 1 the application uses the Over-the-Air activation procedure
 * When set to 0 the application uses the Personalization activation procedure
 */
#define OVER_THE_AIR_ACTIVATION                     1

/*!
 * Indicates if the end-device is to be connected to a private or public network
 */
#define LORAWAN_PUBLIC_NETWORK                      true

/*!
 * Mote device IEEE EUI (big endian)
 * Device EUI: C750F39BF0E894C7
 */
#define LORAWAN_DEVICE_EUI                          { 0xC7, 0x94, 0xE8, 0xF0, 0x9B, 0xF3, 0x50, 0xC7 }

/*!
 * Application IEEE EUI (big endian)
 * Join EUI (aka App EUI): 50923268D4332B44
 */
#define LORAWAN_APPLICATION_EUI                     { 0x50, 0x92, 0x32, 0x68, 0xD4, 0x33, 0x2B, 0x44 }

/*!
 * AES encryption/decryption cipher application key
 * App Key: 07D4FC37A010E35060ECDFF8B381445E
 */
#define LORAWAN_APPLICATION_KEY                     { 0x07, 0xD4, 0xFC, 0x37, 0xA0, 0x10, 0xE3, 0x50, \
                                                      0x60, 0xEC, 0xDF, 0xF8, 0xB3, 0x81, 0x44, 0x5E }

/*!
 * Current network ID
 */
#define LORAWAN_NETWORK_ID                          ( uint32_t )0

/*!
 * Device address on the network (big endian)
 * (Used only for ABP — not relevant for OTAA)
 */
#define LORAWAN_DEVICE_ADDRESS                      ( uint32_t )0x007e6ae1

/*!
 * AES encryption/decryption cipher network session key (ABP only)
 */
#define LORAWAN_NWKSKEY                             { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                                                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

/*!
 * AES encryption/decryption cipher application session key (ABP only)
 */
#define LORAWAN_APPSKEY                             { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                                                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

#endif // __LORA_COMMISSIONING_H__
