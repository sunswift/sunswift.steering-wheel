/* ***********************************************
    File: scandal_config.h
    Scandal Configuration for Tilt Sensor
	 
	File name: scandal_config.h 
	Author: Etienne Le Sueur
	Date: 28/08/2011
    -------------------------------------------------------------------------- */  

#ifndef __SCANDAL_CONFIG__
#define __SCANDAL_CONFIG__

#include <scandal/devices.h>

/* Define the type of device being used */
#ifdef THIS_DEVICE_TYPE
#error "Device type multiply defined (redefined to MINING2)"
#endif
#define THIS_DEVICE_TYPE       STEERINGWHEEL

/* Number of channels */
#define NUM_IN_CHANNELS		STEERINGWHEEL_NUM_IN_CHANNELS
#define NUM_OUT_CHANNELS 	STEERINGWHEEL_NUM_OUT_CHANNELS

/* Size of send/receive buffers */
#define CAN_TX_BUFFER_BITS	4
#define CAN_TX_BUFFER_MASK	0x0F
#define CAN_TX_BUFFER_SIZE 	(1<<CAN_TX_BUFFER_BITS)

#define CAN_RX_BUFFER_BITS	4
#define CAN_RX_BUFFER_MASK	0x0F
#define CAN_RX_BUFFER_SIZE	(1<<CAN_RX_BUFFER_BITS)

#endif
