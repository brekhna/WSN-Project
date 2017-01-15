

// Contiki-specific includes:
#include "contiki.h"
#include "dev/leds.h"			// Use LEDs.
#include "sys/clock.h"			// Use CLOCK_SECOND.
#include "cc2420.h"				// For cc2420_set_channel().
#include "sys/node-id.h"		// Manipulate the node_id.
#include "net/rime/rime.h"		// Establish connections.
#include "random.h"
#include "linkaddr.h"
// Standard C includes:
#include <stdio.h>			// For printf.
#include<stdbool.h>
#include "stdint.h"


enum msg_type
{
  wakeup_beacon= 0, 
  dummy_packet = 1,
  sensor_value= 2,
  ack = 3,
  sleep_beacon = 4
};


enum node_state
{
  sleep = 0,
  active = 1,
  idle = 2,

};


struct message
{
	int msg_id;
	enum  msg_type message_type  ;
	int path[10];
	int source_node_id;
	int destination_node_id;
	int next_node_id;
	float temp_sensor_reading;
	float light_sensor_reading;
	float humidity_sensor_reading;

};


struct sensor_values
{
	float humidity;
	float temperature;
	_Bool light;

};

enum node_state mote_state= sleep ;
_Bool wakeup_beacon_rebroadcasted = 0;
_Bool sleep_beacon_rebroadcasted = 0;
_Bool dummy_packet_broadcasted = 0;
_Bool ack_received = 0;
float RSSI_table[10];
float sensor_value_table[10][4];

