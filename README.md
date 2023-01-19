# Ublox ZEDF9P
Driver for Ublox ZEDF9P GNSS Sensor (firmware version HPG 1.32)

## Introduction

This driver implements a serial interface for the Ubloz ZEDF9P GNSS sensor. Supported features include
1. Configuring the GNSS sensor using CFG_VALSET messages 
2. Polling configurations using CFG_VALGET messages
3. Polling and subscribing to various message types transmitted by the Ublox ZEDF9P GNSS sensor. 
    1. MONVER messages (for polling versioning data)
    2. NavPVT messages (for receiving GNSS data)

## Dependancies
1. [Libserial](https://github.com/crayzeewulf/libserial)

## Initializing The Interface With The Ublox ZEDF9P GNSS Sensor

The ublox_ZEDF9P::ublox_ZEDF9P object contains functions for interacting with the Ublox ZEDF9P GNSS sensor. Once created, we need to initialize the serial connection before we can start interacting with the device. We can optionally set the debug level as well to choose the level of loggin verbosity. 

```
ublox_ZEDF9P::ublox_ZEDF9P gps_;
gps_.set_debug_level(1);
gps_.initializeSerial(device_, baudrate_);
```

## Configuring The Ublox ZEDF9P GNSS Sensor

The Ublox ZEDF9P GNSS can be configured using CFG-VALSET messages (see the [interface description](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf)). This is done by sending a message of type ublox_msgs::CfgMSG to the GNSS sensor. Up to 64 configurations can be added per configuration message.

## Polling For Configuration

## Polling A Ublox Message

## Subscribing To A Ublox Message

## Defining Your Own Message Types

Users can define their own (valid) message types in the include/ublox_msgs/msgs folder and subscribe 

## Debug Levels
1. Basic debugging messages
2. Bytes sent are printed
3. Decoded message bytes are printed
4. All received bytes are printed

## External Links
1. Interface description for ublox firmware HPG 1.32 [here](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf)
2. Integration manual for integrating the Ublox ZEDF9P GNSS sensor [here](https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf)
3. Device configuration files [here](https://www.ardusimple.com/configuration-files/)
4. U-Center download [here](https://www.u-blox.com/en/product/u-center)