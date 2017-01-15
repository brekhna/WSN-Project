// Wireless sensor network Laboratory WS 16/17
// Project: Smart farming , Group: 2 
// Contributors: Sandeep, Iffat, Ilkin

/* Description of the file */
// This file contains the main process for the mote. 
// The main process handles the tasks to be done by Gateway after periodic (30s) wakeup. The steps involved are :

/*
1. Gateway broadcasts "wakeup_beacon" message in the network and changes state from "sleep" to "active" .
2. Each mote after receiving the "wakeup_beacon" retransmits the message ONCE again and changes state to "active" from "sleep" .
3. After each motes are in "active state", they send the "dummy_packet" message multiple times (5 times at random interval of 10-50ms). 
   This is done for network discovery.i.e to find out the RSSI values of all the neighbouring nodes. 
4. Each mote maintains a table with the node Id and RSSI values of corresponding nodes.
5. Based on RSSI values, Dijkstra algorithm is used to calculate the shortest path from the node to the gateway. 
6. Each mote activates the sensors, gets sensor readings, creates a packet and send it to gateway via the calculated path.
7. If ACK isnot received withing certain time (10ms), gain the message is retransmitted for a maximum of m times.(m = 5)
8. Even if after max. retransmissions ACK isnot received, another path is calculated and same process is repeated.
9. If ACK is received by each mote, they change their state to "Idle" in which they don't send their sensor readings but are ready to 
   work as relay for other motes.
10. Gateway senses the channel for 5 seconds to see if it is idle or not. If it finds channel idle, it broadcasts "sleep_beacon"
    in the network and changes its state to "sleep" again. Each mote after receiving the "sleep_beacon", rebroadcasts the message ONCE 
    goes to sleep and changes state to "sleep".static
11. If gateway doesnot sense the channel during that time, it waits another 5s and senses the channel. It is repeated 3 times.
12. After 30s, the same cycle is repeated from step 1. 

*/


// Contiki-specific includes:
#include "contiki.h"
#include "dev/leds.h"			// Use LEDs.
#include "sys/clock.h"			// Use CLOCK_SECOND.
#include "cc2420.h"				// For cc2420_set_channel().
#include "sys/node-id.h"		// Manipulate the node_id.
#include "net/rime/rime.h"		// Establish connections.
#include "random.h"
#include "main.h"
#include "linkaddr.h"
#include "dev/i2cmaster.h"  // Include IC driver
#include "lib/sensors.h"    //required for sensor_sensor struct that is used to define phidget struct

#include "platform/z1/dev/z1-phidgets.h"
// Standard C includes:
#include <stdio.h>			// For printf.
#include<stdbool.h>
#include "stdint.h"

#define UNICAST_RIME_CHANNEL 146
#define BROADCAST_RIME_CHANNEL 129
#define my_node_id 1
#define RANDOM_RAND_MAX 10
//---------------- FUNCTION PROTOTYPES ----------------

// Broadcast connection setup:
static struct broadcast_conn broadcastConn;


static
// Unicast connection setup:
static struct unicast_conn unicastConn;


// Global variable declaration
/*
enum node_state mote_state= sleep ;
_Bool wakeup_beacon_rebroadcasted = 0;
_Bool sleep_beacon_rebroadcasted = 0;
_Bool dummy_packet_broadcasted = 0;
_Bool ack_received = 0;
float RSSI_table[10];

*/

static linkaddr_t generateLinkAddress(uint8_t nodeId);

// Defines thphidgetse behavior of a connection upon receiving data.
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	int bytes = 0;
	struct message received_message;
	bytes = packetbuf_copyto(&received_message);


	leds_on(LEDS_GREEN);
	 printf("Broadcast message received from %d.%d: '%s' [RSSI %d]\n",
			 from->u8[0], from->u8[1],
			(char *)packetbuf_dataptr(),
			packetbuf_attr(PACKETBUF_ATTR_RSSI));
	 printf("mote state is %d and received message is %d \n", &mote_state, &received_message.message_type );
	if ((mote_state == sleep) && (received_message.message_type == wakeup_beacon )) // if mote is in sleep state and wakeup_beacon is received from gateway, it wakes up
	{
		mote_state = active ;
		printf("mote wakes up \n");
		if(wakeup_beacon_rebroadcasted == 0)
		{
			printf(" wakeup_beacon rebroadcasted \n");
			wakeup_beacon_rebroadcasted = 1;
			packetbuf_copyfrom(&received_message, sizeof(received_message));
			broadcast_send(&broadcastConn);

		}
	}

	else if ((mote_state == idle) && (received_message.message_type == sleep_beacon )) // if mote is in idle state and sleep_beacon is received from gateway, it sleeps down
	{
		mote_state = sleep ;
		if(sleep_beacon_rebroadcasted== 0)
		{
			printf("sleep beacon rebroadcasted and mote goes to sleep \n");
			sleep_beacon_rebroadcasted = 1;
			packetbuf_copyfrom(&received_message, sizeof(received_message));
			broadcast_send(&broadcastConn);
		}
	}

	else if ((mote_state == active) && (received_message.message_type == dummy_packet ))
	{
		printf("The mote is active and dummy packet is received from other motes. Now fill up the RSSI table\n");
		RSSI_table[received_message.source_node_id] = packetbuf_attr(PACKETBUF_ATTR_RSSI);


	}



	leds_off(LEDS_GREEN);
	return;
}


static const struct broadcast_callbacks broadcast_callbacks = {broadcast_recv};


static void
unicast_recv(struct unicast_conn *c, const linkaddr_t *from)
{

	int bytes = 0, i ;
	struct message received_message;
	bytes = packetbuf_copyto(&received_message);

	printf("unicast_recv function called\n");
	leds_on(LEDS_GREEN);
	printf("Unicast message received from %d.%d: '%s' [RSSI %d]\n",
	  			 from->u8[0], from->u8[1],
	  			(char *)packetbuf_dataptr(),
	  			packetbuf_attr(PACKETBUF_ATTR_RSSI));


	if(((mote_state == active) || (mote_state == idle)) && (received_message.message_type == sensor_value ) && (received_message.next_node_id == my_node_id))
	{
       // check the next hop and send to it
		for ( i=0; ; i++)
		{
			if(received_message.path[i]== my_node_id )
			continue;
		}
		packetbuf_copyfrom(&received_message, sizeof(received_message));
		linkaddr_t next_node = generateLinkAddress(received_message.path[i+1]);
		unicast_send(&unicastConn, &next_node);

	}

	else if(((mote_state == active)
			|| (mote_state == idle)) && (received_message.message_type == ack ))
	{

		if(received_message.destination_node_id == my_node_id ) // Ack received for the sent data
		{
			printf("ACK received for the sent sensor value, now go to idle state.");
			ack_received = 1;
			mote_state = idle;

		}

		else
		{
			for (i=0; ;i++)
			{
				if(received_message.path[i]== my_node_id )
					continue;
			}
			packetbuf_copyfrom(&received_message, sizeof(received_message));
			linkaddr_t next_node = generateLinkAddress(received_message.path[i-1]);
			unicast_send(&unicastConn, &next_node);
		}
	}

	leds_off(LEDS_GREEN);
	return;
}

static const struct unicast_callbacks unicast_call = {unicast_recv};

// Prints the contents of the Rx buffer.
static void print_buffer_contents(void);
// Checks if the loaded RIME address is all zeroes.
static void check_for_invalid_addr(void);
// Prints the current settings.
static void print_settings(void);




//--------------------- PROCESS CONTROL BLOCK ---------------------generateLinkAddress
PROCESS(main_process, "Main process");
AUTOSTART_PROCESSES(&main_process);
//------------------------ PROCESS' THREADS ------------------------
PROCESS_THREAD(main_process, ev, data) {
	PROCESS_EXITHANDLER( broadcast_close(&broadcastConn); )
	PROCESS_BEGIN();
	// Configure your team's channel (11 - 26).
	cc2420_set_channel(13);
	// Set and load the node ID to generate a RIME address:
	node_id_burn(my_node_id);
	node_id_restore();
	print_settings();
	check_for_invalid_addr();

	unicast_open(&unicastConn , UNICAST_RIME_CHANNEL,&unicast_call);
	broadcast_open(&broadcastConn, BROADCAST_RIME_CHANNEL, &broadcast_callbacks);


	// set timers
	static struct etimer dummy_packet_timer;
	static struct timer ack_timer;


	// set variables
	struct message dummy_message, sensor_reading_message;
	int i;

	// If all is OK, we can start the other two processes:
	while(1)
	{
		// Contiki processes cannot start loops that never end.
		printf("mote started \n");

		//while (mote_state == sleep)
		//PROCESS_WAIT_EVENT();

		PROCESS_WAIT_UNTIL(mote_state == active);

		printf(" goes inside IF statement \n");
		if((wakeup_beacon_rebroadcasted == 1) && (mote_state == active) && (dummy_packet_broadcasted == 0) )
		{
			// Broadcast the dummy packet for 5 times (random interval)
			etimer_set(&dummy_packet_timer, CLOCK_SECOND/200 + 0.1*random_rand()/RANDOM_RAND_MAX);
			for(i=1; i<=4; i++)
			{
				PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&dummy_packet_timer));

				printf("Broadcast dummy packet\r\n");
				dummy_message.message_type = dummy_packet ;
				dummy_message.source_node_id = my_node_id;

				packetbuf_copyfrom(&dummy_message,sizeof(dummy_message));
				broadcast_send(&broadcastConn);

				leds_toggle(LEDS_BLUE);
				etimer_reset(&dummy_packet_timer);
			}
			dummy_packet_broadcasted = 1;
		}

		// Now find the shorted path, get the sensor values and create a message
		sensor_reading_message.source_node_id = my_node_id;
		sensor_reading_message.destination_node_id = 0;
		sensor_reading_message.message_type = sensor_value ;
		//sensor_reading_message.path = calculate_path(RSSI_table, 1);

		sensor_reading_message.temp_sensor_reading = 0;
		sensor_reading_message.light_sensor_reading = 0;
		sensor_reading_message.humidity_sensor_reading = 0;


		// Send the message to the next node in the path.
		leds_on(LEDS_RED);
		packetbuf_copyfrom(&sensor_reading_message, sizeof(sensor_reading_message));
		linkaddr_t receiver_node = generateLinkAddress(sensor_reading_message.path[1]);
		unicast_send(&unicastConn, &receiver_node);
		leds_off(LEDS_RED);
		timer_set(&ack_timer, CLOCK_SECOND/10 + 0.1*random_rand()/RANDOM_RAND_MAX);

		for(i=0 ; i<5 ; i++)
		{
			if (timer_expired(&ack_timer))
					 {
						 if(ack_received == 1)
						 {
							 continue;
						 }

						 else
						 {
							 timer_reset(&ack_timer);
							 packetbuf_copyfrom(&sensor_reading_message, sizeof(sensor_reading_message));
							 unicast_send(&unicastConn, &receiver_node);
						 }
					 }

		}

     // what to do if ACK is not received even after 5 attempts.


	}
	PROCESS_END();
}

/**
 * Print the received message
 */
static void print_buffer_contents(void){
	char rxBuffer[PACKETBUF_SIZE];
	int rxBytes = 0;
	/*
	 * get the buffered message and message size
	 */
	rxBytes = packetbuf_copyto(&rxBuffer);
	if (rxBytes>0) {
		printf("%s\r\n\n",rxBuffer);
	}
}
// Checks if the loaded RIME address is all zeroes.
static void check_for_invalid_addr(void) {
	// Boolean flag to check invalid address.
	static int iAmError = 0;
	// All-zeroes address.
	static linkaddr_t errAddr;
	errAddr.u8[0] = 0;
	errAddr.u8[1] = 0;
	// Check if this mote got an invalid address.
	iAmError = linkaddr_cmp(&errAddr, &linkaddr_node_addr);
	// Turn ON all LEDs if we loaded an invalid address.
	if(iAmError){
		printf("\nLoaded an invalid RIME address (%u.%u)! "
				"Reset the device.\n\n",
				linkaddr_node_addr.u8[0],
				linkaddr_node_addr.u8[1]);
		// Freezes the app here. Reset needed.
		while (1){
			leds_on(LEDS_ALL);
		}
	}
}
// Prints the current settings.
static void print_settings(void) {
	printf("\n-----------------------------\n");
	printf("Node ID = \t%u\n", node_id);
	printf("RIME addr = \t%u.%u\n",
			linkaddr_node_addr.u8[0],
			linkaddr_node_addr.u8[1]);
	printf("Using radio channel %d\n", cc2420_get_channel());
	printf("-----------------------------\n");
}


// Generate link address from node ID
static linkaddr_t generateLinkAddress(uint8_t nodeId){
	linkaddr_t addr;

	addr.u8[0] = nodeId;
	addr.u8[1] = 0;

	return addr;
}



/////////////////////////////////////////////////////////////////////////

get_sensors_value()
{
 SENSORS_ACTIVATE(phidgets);
 get_temp();
 getLightValue(1.5881,40.157,PHIDGET3V_2 );
 getHumidityValue(PHIDGET5V_2);
}

 void get_temp()
{
  	int16_t
	raw,    // Raw data as received from the sensor.
	tempint,  // Integer part of the temperature reading.
	sign;   // Used to evaluate negative temperature readings.

	uint16_t
	tempfrac, // Decimal part of the temperature reading.
	absraw;   // Absolute value of raw data.
	char
	minus = ' ';  // Used to print negative temperatures.
	struct message sensor_reading;

  raw = tmp102_read_temp_raw();
		// Now we have to process the raw data and print it:
		sign = 1;
		absraw = raw;
		// In case we got negative readings:
		if (raw < 0) {
		  // Perform two's complement
		  absraw = (raw ^ 0xFFFF) + 1;
		  sign = -1;
		}
		// Extract the integer part:
		tempint= (absraw >> 8) * sign;
		sensor_reading.temp_sensor_reading = tempint;

}


void getLightValue(float m, float b, uint8_t phidget_input){
	//Read voltage from the phidget interface
	float voltage;
	float SensorValue;
	int lux;
	struct message sensor_reading;
	//SENSORS_ACTIVATE(phidgets);
	voltage=phidgets.value(phidget_input);
	//Convert the voltage in lux with the provided formula
	SensorValue = voltage/4.096;
	lux = (int)(m * SensorValue + b);
	//Return the value of the light in boolan whether it is night or noon
	if (lux < 400)
		sensor_reading.light_sensor_reading==0;
	else
		sensor_reading.light_sensor_reading==1;
}

 void getHumidityValue(uint8_t phidget_input){
	//Read voltage from the phidget interface
	float voltage;
	float SensorValue;
	int humidity;
	struct message sensor_reading;

	//SENSORS_ACTIVATE(phidgets);
	voltage=phidgets.value(phidget_input);
	//Convert the voltage in lux with the provided formula
	SensorValue = voltage/4.096;
	humidity = (int)(SensorValue*0.1906 - 40.2) ;

	if (humidity < 50)
		sensor_reading.humidity_sensor_reading==0;
	else
		sensor_reading.humidity_sensor_reading==1;
}

