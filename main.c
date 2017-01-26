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
   This is done for networkpacketbuf_attr(PACKETBUF_ATTR_RSSI) discovery.i.e to find out the RSSI values of all the neighbouring nodes.
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
#include "net/rime/unicast.h"
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

#define UNICAST_RIME_CHANNEL 150
#define BROADCAST_RIME_CHANNEL 130
#define my_node_id 6
#define RANDOM_RAND_MAX 10
//---------------- FUNCTION P1.ROTOTYPES ----------------

// Broadcast connection setup:
static struct broadcast_conn broadcastConn;



// Unicast connection setup:
static struct unicast_conn unicastConn;


// Global variable declaration
struct message received_message,dummy_message, sensor_reading_message;

 enum node_state mote_state= sleep ;
_Bool wakeup_beacon_rebroadcasted = 0;
 _Bool sleep_beacon_rebroadcasted = 0;
 _Bool dummy_packet_broadcasted = 0;
 _Bool ack_received = 0;
 _Bool path_found = 0;
 float RSSI_table[2][6]= {{0,1,2,3,4,5},
						  {0,0,0,0,0,0}};
float sorted_RSSI_table[2][6] = {{0,1,2,3,4,5},
		  	  	  	  	  	  	  {0,0,0,0,0,0}};
float sensor_value_table[6][4];
int path_array[6]= {0,0,0,0,0,0};
int path_index;


static linkaddr_t generateLinkAddress(uint8_t nodeId);

//--------------------- PROCESS CONTROL BLOCK ---------------------generateLinkAddress
PROCESS(main_process, "Main process");
AUTOSTART_PROCESSES(&main_process);


// Defines thphidgetse behavreceived_messageior of a connection upon receiving data.
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	int bytes = 0;
	static int i ;

	bytes = packetbuf_copyto(&received_message);
	void* data ;


	leds_on(LEDS_GREEN);
	 printf("Broadcast message received from %d.%d: '%s' [RSSI %d]\r\n",
			 from->u8[0], from->u8[1],
			(char *)packetbuf_dataptr(),
			packetbuf_attr(PACKETBUF_ATTR_RSSI));
	 printf("the received message type is %d \r\n",received_message.message_type);


	if ((received_message.message_type == dummy_packet ))
	{
		printf("The mote is active and dummy packet is received from other motes. Now fill up the RSSI table\r\n");
		//RSSI_table[1][received_message.source_node_id] = packetbuf_attr(PACKETBUF_ATTR_RSSI);
		//RSSI_table[1][my_node_id] = -91;

		if((packetbuf_attr(PACKETBUF_ATTR_RSSI) > -70) && (path_found == 0))
		{
			received_message.path[received_message.path_array_index] = my_node_id;
			path_found = 1;
			path_index = received_message.path_array_index;
			received_message.path_array_index +=1;
			for(i=0; i<6 ; i++)
			{
				path_array[i] = received_message.path[i] ;
				printf("path array is %d and i=%d and path array index is %d\r\n", path_array[i],i,path_index ) ;
			}
			printf(" Resending dummy packet to other motes for network and path discovery \r\n");
			packetbuf_copyfrom(&received_message,sizeof(dummy_message));
			broadcast_send(&broadcastConn);
			process_post(&main_process, 'event',data); // post an event to main_process

		}

	}

	leds_off(LEDS_GREEN);
	return;
}


static const struct broadcast_callbacks broadcast_callbacks = {broadcast_recv};


static void
unicast_recv(struct unicast_conn *c, const linkaddr_t *from)
{

	int bytes = 0, i ;
	//struct message received_message;
	bytes = packetbuf_copyto(&received_message);

	void* data ;

	printf("unicast_recv function called\r\n");
	leds_on(LEDS_GREEN);
	printf("Unicast message received from %d.%d: '%s' [RSSI %d]\r\n",
	  			 from->u8[0], from->u8[1],
	  			(char *)packetbuf_dataptr(),
	  			packetbuf_attr(PACKETBUF_ATTR_RSSI));
	printf("the received message type  is %d \r\n",received_message.message_type);

	if((received_message.message_type == sensor_value ) && (received_message.next_node_id == my_node_id))
	{

		received_message.next_node_id = received_message.path[received_message.path_array_index -1 ];
		received_message.path_array_index -= 1;
		//received_message.path[received_message.path_array_index] = received_message.next_node_id;



		linkaddr_t next_node = generateLinkAddress(received_message.next_node_id);
		packetbuf_copyfrom(&received_message, sizeof(received_message));
		printf("message sent to  mote %d in the path.\r\n", received_message.next_node_id);
		unicast_send(&unicastConn, &next_node);

	}

	else if((received_message.message_type == ack ))
	{

		if(received_message.destination_node_id == my_node_id ) // Ack received for the sent data
		{
			printf("ACK received for the sent sensor value\r\n");
			ack_received = 1;
			//mote_state = idle;
			process_post(&main_process, 'ack_event',data); // post an event to main_process

		}

		else
		{
			/*
			for (i=0; ;i++)
			{
				if(received_message.path[i]== my_node_id )
					continue;
			}*/

			received_message.path_array_index += 1;



			packetbuf_copyfrom(&received_message, sizeof(received_message));
			linkaddr_t next_node = generateLinkAddress(received_message.path[received_message.path_array_index]);
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
// Prints the current setthhe re2 ceived messageid iis 1 tings.
static void print_settings(void);

struct sensor_readings get_sensors_value(void);
static float get_temp(void);
_Bool  getLightValue(float m, float b, uint8_t phidget_input);
float getHumidityValue(uint8_t phidget_input);
int get_next_node_id(struct message sensor_message);
void sort_RSSI_table(void);
void delay (float seconds);



//------------------------ PROCESS' THREADS ------------------------
PROCESS_THREAD(main_process, ev, data) {
	PROCESS_EXITHANDLER( broadcast_close(&broadcastConn); )
	PROCESS_BEGIN();
	// Configure your team's channel (11 -26).
	cc2420_set_channel(13);
	cc2420_set_txpower(31);
	// Set and load the node ID to generate a RIME address:
	node_id_burn(my_node_id);
	node_id_restore();
	print_settings();
	check_for_invalid_addr();

	unicast_open(&unicastConn , UNICAST_RIME_CHANNEL, &unicast_call);
	broadcast_open(&broadcastConn, BROADCAST_RIME_CHANNEL, &broadcast_callbacks);


	// set timers
	static struct etimer dummy_packet_timer, ack_timer, wait_timer;



	// set variables
	//struct message dummy_message, sensor_reading_message;
	struct sensor_readings sensor_data;
	static int i;

	// If all is OK, we can start the other two processes:
	while(1)
	{
		// Contiki processes cannot start loops that never end.
		printf("mote started \r\n");

		PROCESS_WAIT_EVENT_UNTIL(ev);




		// Now find the shortest path, get the sensor values and create a message
		sensor_reading_message.source_node_id = my_node_id;
		sensor_reading_message.destination_node_id = 0;
		sensor_reading_message.message_type = sensor_value ;

		// get sensor values from sensors
		sensor_data = get_sensors_value();
		sensor_reading_message.temp_sensor_reading = sensor_data.temperature;
		sensor_reading_message.light_sensor_reading = sensor_data.light;
		sensor_reading_message.humidity_sensor_reading = sensor_data.humidity;


		//sort_RSSI_table();  // sort the RSSI table for calculating the path

		sensor_reading_message.next_node_id = path_array[path_index -1 ];
		sensor_reading_message.path_array_index = path_index -1;
		//sensor_reading_message.path[0] = my_node_id;
		//sensor_reading_message.path[sensor_reading_message.path_array_index] = sensor_reading_message.next_node_id;

		// Send the message to the next node in the path.
		printf("sending sensor values to mote %d \r\n",sensor_reading_message.next_node_id );
		leds_on(LEDS_RED);
		packetbuf_copyfrom(&sensor_reading_message, sizeof(sensor_reading_message));
		linkaddr_t receiver_node = generateLinkAddress(sensor_reading_message.next_node_id);
		unicast_send(&unicastConn, &receiver_node);
		leds_off(LEDS_RED);
		etimer_set(&ack_timer, 2*CLOCK_SECOND+ 0.1*random_rand()/RANDOM_RAND_MAX);

		for(i=0;i<5;i++)
		{
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&ack_timer));
			 if(ack_received == 1)
				{
					printf("ack received\r\n");
					break;

				}

			 else
				{

				 	 packetbuf_copyfrom(&sensor_reading_message, sizeof(sensor_reading_message));
					 printf("resending packet \r\n") ;
					 unicast_send(&unicastConn, &receiver_node);
				}

			etimer_reset(&ack_timer);

		}

     // what to do if ACK is not received even after 5 attempts.

		delay(10.0);


		path_found = 0;

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
				"Reset the device.\r\n",
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
	printf("\n-----------------------------\r\n");
	printf("Node ID = \t%u\n", node_id);
	printf("RIME addr = \t%u.%u\n",
			linkaddr_node_addr.u8[0],
			linkaddr_node_addr.u8[1]);
	printf("Using radio channel %d\n", cc2420_get_channel());
	printf("--------------get_next_node_id---------------\r\n");
}


// Generate link address from node ID
static linkaddr_t generateLinkAddress(uint8_t nodeId){
	linkaddr_t addr;

	addr.u8[0] = nodeId;
	addr.u8[1] = 0;

	return addr;
}



/////////////////////////////////////////////////////////////////////////
// this function returns all the sensor readings

struct sensor_readings get_sensors_value()
{
 struct sensor_readings sensor_data;

 SENSORS_ACTIVATE(phidgets);  // activate the sensors

 sensor_data.temperature = get_temp();
 sensor_data.light= getLightValue(1.5881,40.157,PHIDGET3V_2 );
 sensor_data.humidity = getHumidityValue(PHIDGET5V_2);

 return sensor_data;
}


// function to get temperature reading from temperature sensor
 float get_temp()
{
  	int16_t
	raw,    // Raw data as received from the sensor.
	tempint,  // Integer part of the temperature reading.
	sign;   // Used to evaluate negative temperature readings.

	uint16_t
	tempfrac, // Decimal part of the temperature reading.
	absraw;   // Absolute value of raw datget_sensors_valuea.
	char
	minus = ' ';  // Used to print negative temperatures.

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
		//sensor_reading.temp_sensor_reading = tempint;


	return tempint;

}


 // function to get light value from light sensor
 _Bool  getLightValue(float m, float b, uint8_t phidget_input){
	//Read voltage from the phidget interface
	float voltage;
	float SensorValue;
	int lux;
	_Bool light = 0;
	//SENSORS_ACTIVATE(phidgets);
	voltage=phidgets.value(phidget_input);
	//Convert the voltage in lux with the provided formula
	SensorValue = voltage/4.096;
	lux = (int)(m * SensorValue + b);
	//Return the value of the light in boolan whether it is night or noon
	if (lux < 400)
		light = 0;
	else
		light = 1;

	return light;
}


// function to get humidity reading from humidity sensor

 float getHumidityValue(uint8_t phidget_input){
	//Read voltage from the phidget interface
	float voltage;
	float SensorValue;
	float humidity;

	//SENSORS_ACTIVATE(phidgets);
	voltage=phidgets.value(phidget_input);
	//Convert the voltage in lux with the provided formula
	SensorValue = voltage/4.096;
	humidity = (SensorValue*0.1906 - 40.2) ;

	/*
	if (humidity < 50)
		sensor_reading.humidity_sensor_reading==0;
	else
		sensor_reading.humidity_sensor_reading==1;
    */

	return humidity;

}




 int get_next_node_id(struct message sensor_message)
 {
	static int i,next_node;
	static int j=1;
	 //float RSSI_value = -90;


	 next_node = sorted_RSSI_table[0][0];

	 for(i=0 ; i<= sensor_message.path_array_index; i++)
	 	 {
	 		 if((next_node == sensor_message.path[i])  || (next_node == my_node_id))
	 		 {
	 			next_node = sorted_RSSI_table[0][j++];

	 			printf("the next node id will be %d , i=%d , \r\n" , next_node,i);
	 		 }
	 	 }

	 return next_node;

 }



void sort_RSSI_table()
 {
	 static int n=10, c=10 ,d,  swap_index;
	 static float swap_value;

	 for (c = 0 ; c < ( n - 1 ); c++)
	   {
	     for (d = 0 ; d < n - c - 1; d++)
	     {
	       if (sorted_RSSI_table[1][d] > sorted_RSSI_table[1][d+1]) /* For decreasing order use < */
	       {
	         swap_value      = RSSI_table[1][d];
	         sorted_RSSI_table[1][d]   = sorted_RSSI_table[1][d+1];
	         sorted_RSSI_table[1][d+1] = swap_value;

	         swap_index      = RSSI_table[0][d];
	         sorted_RSSI_table[0][d]   = sorted_RSSI_table[0][d+1];
	         sorted_RSSI_table[0][d+1] = swap_index;


	       }
	     }
	   }
return;
 }


void delay(float seconds)
{
	static struct timer delay_timer;
	static int i;

	timer_set(&delay_timer, seconds*CLOCK_SECOND);
	printf("delay of %f seconds \r\n", seconds );

	while(1)
	{
		if(timer_expired(&delay_timer))
			break;
	}
	//PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&delay_timer));

	//etimer_reset(&delay_timer);
	 return;

}
