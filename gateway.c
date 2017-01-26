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
    goes to sleep and changes state to "sleep".
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
// Standard C includes:
#include <stdio.h>			// For printf.
#include<stdbool.h>
#include "stdint.h"

#define UNICAST_RIME_CHANNEL 150
#define BROADCAST_RIME_CHANNEL 130
#define my_node_id 1
#define RANDOM_RAND_MAX 10
//---------------- FUNCTION PROTOTYPES ----------------

// Broadcast connection setup:
static struct broadcast_conn broadcastConn;


// Unicast connection setup:
static struct unicast_conn unicastConn;

static linkaddr_t generateLinkAddress(uint8_t nodeId);

// Global variable declaration
//enum node_state mote_state= sleep ;
_Bool wakeup_beacon_rebroadcasted = 0;
//_Bool sleep_beacon_rebroadcasted = 0;
_Bool dummy_packet_broadcasted = 0;
_Bool ack_received = 0;
float RSSI_table[6];
float sensor_value_table[6][4];
struct message dummy_message;
struct message received_message , ack_message;

// Defines the behavior of a connection upon receiving data.
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	leds_on(LEDS_GREEN);
	 printf("Broadcast message received from %d.%d: '%s' [RSSI %d\r\n",
			 from->u8[0], from->u8[1],
			(char *)packetbuf_dataptr(),
			packetbuf_attr(PACKETBUF_ATTR_RSSI));

	leds_off(LEDS_GREEN);
	return;
}
_Bool sensor_table_full;
static const struct broadcast_callbacks broadcast_callbacks = {broadcast_recv};



static void
unicast_recv(struct unicast_conn *c, const linkaddr_t *from)
{

	int bytes = 0, i ;
	bytes = packetbuf_copyto(&received_message);

	printf("` function called\n");
	leds_on(LEDS_GREEN);
	printf("Unicast message received from %d.%d: '%s' [RSSI %d]\r\n",
	  			 from->u8[0], from->u8[1],
	  			(char *)packetbuf_dataptr(),
	  			packetbuf_attr(PACKETBUF_ATTR_RSSI));


	if((received_message.message_type == sensor_value ))
	{
       // check the sender mote and send ACK to it

		// writing the sensor values in a 2d table
		sensor_value_table[received_message.source_node_id][0] = 1;
		sensor_value_table[received_message.source_node_id][1] = received_message.temp_sensor_reading;
		sensor_value_table[received_message.source_node_id][2] = received_message.light_sensor_reading;
		sensor_value_table[received_message.source_node_id][3] = received_message.humidity_sensor_reading;



		for( i=0; i<6; i++)
		{
			ack_message.path[i] = received_message.path[i] ;
		}
		printf("unicast_mes received from node_%d\r\n", ack_message.source_node_id);
		ack_message.source_node_id = my_node_id;
		ack_message.destination_node_id = ack_message.source_node_id;
		ack_message.message_type = ack ;

		/*
		for (i=0; ;i++)
				{
					if(ack_message.path[i]== my_node_id )
					continue;
				}
		*/
		packetbuf_copyfrom(&ack_message, sizeof(ack_message));
		linkaddr_t next_node = generateLinkAddress(ack_message.path[1]);
		unicast_send(&unicastConn, &next_node);

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
// check whether sensor table is full or not
static _Bool check_sensor_table(void) ;



//--------------------- PROCESS CONTROL BLOCK ---------------------
PROCESS(gateway_process, "Gateway process");
AUTOSTART_PROCESSES(&gateway_process);
//------------------------ PROCESS' THREADS ------------------------
PROCESS_THREAD(gateway_process, ev, data) {
	PROCESS_EXITHANDLER( broadcast_close(&broadcastConn); )
	PROCESS_BEGIN();
	// Configure your team's channel (11 - 26).
	cc2420_set_channel(13);
	cc2420_set_txpower(31);  // power must be in the range 0..31
	// Set and load the node ID to generate a RIME address:
	node_id_burn(my_node_id);
	node_id_restore();
	print_settings();
	check_for_invalid_addr();

	unicast_open(&unicastConn , UNICAST_RIME_CHANNEL,&unicast_call);
	broadcast_open(&broadcastConn, BROADCAST_RIME_CHANNEL, &broadcast_callbacks);


	// set timers
	static struct etimer wakeup_timer , dummy_packet_timer, sensor_table_check_timer;
	static struct timer ack_timer, wakeup_rept_timer;


	// set variables
	static int i;
	static _Bool sensor_table_full;
	void* data;

	// If all is OK, we can start the other two processes:
	while(1)
	{
		etimer_set(&wakeup_timer, CLOCK_SECOND*20);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&wakeup_timer));

		//mote_state = active;
		printf(" Gateway wakes up .\r\n\n");
		// broadcast the wakeup beacon
		//wakeup_message.source_node_id = my_node_id;
		//wakeup_message.msg_type = wakeup_beacon ;
		//packetbuf_copyfrom(&wakeup_message,sizeof(wakeup_message));
		/*
		timer_set(&wakeup_rept_timer,CLOCK_SECOND/200 + 0.1*random_rand()/RANDOM_RAND_MAX);
		for(i=0; i<=1; i++)
		{
			if(timer_expired(&wakeup_rept_timer))
			{
				broadcast_send(&broadcastConn);
			}
			timer_restart(&wakeup_rept_timer);
			printf("wakeup msg is send %d times\r\n", i);
		}

		*/

		// broadcast dummy packet 2 times at random intervals
		if((dummy_packet_broadcasted == 0) )
				{
					// Broadcast the dummy packet for 5 times (random interval)
					etimer_set(&dummy_packet_timer, CLOCK_SECOND/200 + 0.1*random_rand()/RANDOM_RAND_MAX);
					for(i=0; i<=1; i++)
					{
						PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&dummy_packet_timer));

						printf("Broadcast dummy packet i= %d\r\n", i);
						dummy_message.message_type = dummy_packet ;
						dummy_message.source_node_id = my_node_id;
						dummy_message.path[0]= my_node_id;
						dummy_message.path_array_index = 1;
						printf("Sent message is:%d and Source is:%d\r\n",dummy_message.message_type,dummy_message.source_node_id);

						packetbuf_copyfrom(&dummy_message,sizeof(dummy_message));
						broadcast_send(&broadcastConn);
						//linkaddr_t next_node = generateLinkAddress(1);
						//unicast_send(&unicastConn, &next_node);


						leds_toggle(LEDS_BLUE);
						etimer_reset(&dummy_packet_timer);
					}
						//dummy_packet_broadcasted = 1;
						etimer_stop(&dummy_packet_timer);

				}


        // check whether sensor table is full or not. Check 10 times on interval of 500 ms. continue if full in between. continue even if not full till 5s.
		etimer_set(&sensor_table_check_timer, CLOCK_SECOND);
			for(i=0; i<=20; i++)
			{
			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sensor_table_check_timer));
			sensor_table_full = check_sensor_table();

			if (sensor_table_full)
				{
				//process_post(&gateway_process, 'event', data);
				continue;
				}


			else
			etimer_reset(&sensor_table_check_timer);

			}

			//PROCESS_WAIT_EVENT_UNTIL(ev) ;

			/*broadcast sleep beacon and go to sleep
			printf("Broadcast sleep beacon \r\n");
			sleep_message.msg_type = sleep_beacon;
			sleep_message.source_node_id = my_node_id;
			packetbuf_copyfrom(&sleep_message,sizeof(sleep_message));
			broadcast_send(&broadcastConn);
			mote_state = sleep;
			dummy_packet_broadcasted = 0;


			*/
			etimer_reset(&wakeup_timer);
	}
	PROCESS_END();
}

/**
 * Print the received messagelinkaddr_node_addr
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
				"Reset the device.\r\n\n",
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
	printf("Node ID = %u\r\n", node_id);
	printf("RIME addr = %u.%u\r\n",
			linkaddr_node_addr.u8[0],
			linkaddr_node_addr.u8[1]);
	printf("Using radio channel %d\r\n", cc2420_get_channel());
	printf("-----------------------------\r\n");
}


static _Bool check_sensor_table(void)
{
	int i, count=0 ;
	_Bool result= 0;


	for(i=0; i<=9 ; i++)
	{
		if(sensor_value_table[i][0]== 1 )
			count++ ;
	}

	if(count == 9 )
	result = 1;

	return result;


}


// Generate link address from node ID
static linkaddr_t generateLinkAddress(uint8_t nodeId){
	linkaddr_t addr;

	addr.u8[0] = nodeId;
	addr.u8[1] = 0;

	return addr;
}

