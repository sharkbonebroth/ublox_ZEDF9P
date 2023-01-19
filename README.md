# Ublox ZEDF9P
Driver for Ublox ZEDF9P GNSS Sensor (firmware version HPG 1.32)

## Introduction

This driver implements a serial interface for the Ubloz ZEDF9P GNSS sensor. Supported features include
1. Configuring the GNSS sensor using CFG_VALSET messages 
2. Polling configurations using CFG_VALGET messages
3. Polling and subscribing to various message types transmitted by the Ublox ZEDF9P GNSS sensor. 

## Dependancies
1. [Libserial](https://github.com/crayzeewulf/libserial)

## Initializing The Serial Interface

The ublox_ZEDF9P::ublox_ZEDF9P object contains functions for interacting with the Ublox ZEDF9P GNSS sensor. Once created, we need to initialize the serial connection before we can start interacting with the device. We can optionally set the debug level as well to choose the level of loggin verbosity. 

```
ublox_ZEDF9P::ublox_ZEDF9P gps_;
gps_.set_debug_level(1);
gps_.initializeSerial(device_, baudrate_);
```

## Configuring The GNSS Sensor

The Ublox ZEDF9P GNSS can be configured using CFG-VALSET messages (see the [interface description](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf)). This is done by sending a message of type ublox_msgs::Valset to the GNSS sensor using the send_config_message function. Up to 64 configurations can be added per configuration message. 

To add a configuration, use the add_config function with the relevant key corresponding to the configuration you would like to set, and the data type associated with that key. Information regarding keys are found in the interface description.

For an example, check out the [demo_config](examples/demo_config.cpp) example.

## Polling For Configuration

Polling for a configuration uses CFG-VALGET messages (see the [interface description](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf)). This is done by sending a message of type ublox_msgs::Valget to the GNSS sensor using the send_config_message. Up to 64 configuration requests can be added per configuration message. 

To add a configuration request, use the add_config_request function with the relevant key corresponding to the configuration you would like to set. Information regarding keys are found in the interface description.

For an example, check out the [demo_poll_config](examples/demo_poll_config.cpp) example.

## Polling A Ublox Message

A Ublox message can be polled using the poll function. This is useful for getting one-time messages such as MONVER messages. An example of this can be seen in the poll_MONVER convenience function [here](src/ublox_ZEDF9P.cpp)

For an example, check out the [demo_poll_MONVER](examples/demo_poll_MONVER.cpp) example.

## Subscribing To A Ublox Message

Subscribe to a ublox message by calling the subscribe function with a valid callback function as an argument.

For an example, check out the [demo_subscribe_navpvt](examples/demo_subscribe_navpvt.cpp) example.

## Defining Your Own Message Types

Users can define their own (valid) message types in the include/ublox_msgs/msgs folder and subscribe or poll for them accordingly. To do so, create a message struct/class like those defined in the [msgs](include/ublox_msgs/msgs) folder. 

If the message is intended to be read from the GNSS receiver, it should have a valid initialize_from_stream function. This function should initialize the relevant data fields of the message from the payload section of a UBX message received from the  GNSS receiver.

If the message is intended to be sent to the GNSS receiver, it should have a valid write_to_data_stream and serialized_length functions. The write_to_data_stream function should write the message's data fields to an output buffer in the right order for the payload section of the UBX message corresponding to that message. The serialized_length function should return the length of the payload section of the UBX message corresponding to that message.

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