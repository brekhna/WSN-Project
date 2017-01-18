/*
   Wireless Sensor Networks Laboratory

   Technische Universität München
   Lehrstuhl für Kommunikationsnetze
   http://www.lkn.ei.tum.de

   copyright (c) 2014 Chair of Communication Networks, TUM

   contributors:
 * Thomas Szyrkowiec
 * Mikhail Vilgelm
 * Octavio Rodríguez Cervantes

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, version 2.0 of the License.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   LESSON 3: Temperature Sensor.
 */

// Contiki-specific includes:
#include "contiki.h"
#include "sys/node-id.h"  // Manipulate the node_id.
#include "net/rime/rime.h"  // Establish connections.
#include "dev/leds.h"   // Use LEDs.
#include "sys/clock.h"    // Use CLOCK_SECOND.
#include "dev/i2cmaster.h"  // Include IC driver
#include "lib/sensors.h"                 //required for sensor_sensor struct that is used to define phidget struct
#include "platform/z1/dev/z1-phidgets.h" //definition of the phidget struct
#include "dev/tmp102.h"     // Include sensor driver
#include "dev/temperature-sensor.h"

// Standard C includes:
#include <stdio.h>      // For printf.

// Reading frequency in seconds.
#define LIGHT_READ_INTERVAL CLOCK_SECOND*5
#define NO_LIGHT -1

#define PHIDGET5V_1 0
#define PHIDGET5V_2 1
#define PHIDGET3V_1 2
#define PHIDGET3V_2 3

int16_t
	raw,    // Raw data as received from the sensor.
	tempint,  // Integer part of the temperature reading.
	sign,  // Used to evaluate negative temperature readings.
	hum;

uint16_t
	tempfrac, // Decimal part of the temperature reading.
	absraw;   // Absolute value of raw data.
	char
	minus = ' ';// Used to print negative temperatures.


static int getLightSensorValue(float m, float b, uint8_t phidget_input);
static int getHumidityValue(uint8_t phidget_input);


/*** CONNECTION DEFINITION***/

//Callback function for received packet processing.
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {

	leds_on(LEDS_GREEN);

	uint8_t len = strlen( (char *)packetbuf_dataptr() );
	uint16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);

	printf("Got RX packet (broadcast) from: %d.%d, len: %d, RSSI: %d\r\n",from->u8[0], from->u8[1],len,rssi);

	leds_off(LEDS_GREEN);
}

//Connection information
static struct broadcast_conn broadcastConn;

//Assign callback functions to the connection
static const struct broadcast_callbacks broadcast_callbacks = {broadcast_recv};

/*** CONNECTION DEFINITION END ***/


//--------------------- PROCESS CONTROL BLOCK ---------------------
PROCESS (temp_process, "Test Temperature process");
AUTOSTART_PROCESSES (&temp_process);

//------------------------ PROCESS' THREAD ------------------------
PROCESS_THREAD (temp_process, ev, data) {
	PROCESS_EXITHANDLER(broadcast_close(&broadcastConn););
	PROCESS_BEGIN ();

	// Start and setup the phidget interface with default values.
	tmp102_init();
	SENSORS_ACTIVATE(phidgets);

	//set your group's channel
	cc2420_set_channel(13);

	//open the connection
	broadcast_open(&broadcastConn,129,&broadcast_callbacks);

	static struct etimer temp_reading_timer;

	int light=NO_LIGHT;
	int e;

	while (1) {
		etimer_set(&temp_reading_timer, LIGHT_READ_INTERVAL);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&temp_reading_timer));

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
				// Extract the fractional part in 1/10000 of degree:
				//tempfrac = ((absraw >> 4) % 16) * 625;
				// Obtain the sign:
				if((tempint == 0) & (sign == -1)){
				  	minus = '-';

				} else {
				  	minus = ' ';
				}

		//call the getLightSensorValue function and retrieve the light sensor's value
		light = getLightSensorValue(1.5881,40.157,PHIDGET5V_1 );
		hum=getHumidityValue(PHIDGET3V_2);

		char message[40];
		sprintf(message,"Light: %d",light);
		printf("%s\r\n", message);
		printf("Temperature: %c%d\r\n",minus,tempint);
		printf("Humidity: %d\r\n",hum);
		//packetbuf_copyfrom( message, strlen(message)+1 );
		//broadcast_send(&broadcastConn);
	}

	SENSORS_DEACTIVATE(phidgets);
	PROCESS_END ();
}


/*function for outputting the lux value read from sensor
*@param m: calibration value m inscribed on the back of the sensor
*@param b: calibration value b inscribed on the back of the sensor
*@param phidget_input: phidget input value. Use PHIDGET5V_1 or PHIDGET3V_2 depending on where the sensor is connected to.
*@return int : lux value with a max of 1000.
*/
static int getLightSensorValue(float m, float b, uint8_t phidget_input){
	//Read voltage from the phidget interface
	float voltage;
	float SensorValue;
	int lux;

	SENSORS_ACTIVATE(phidgets);
	voltage=phidgets.value(phidget_input);
	//Convert the voltage in lux with the provided formula
	SensorValue = voltage/4.096;
	lux = (int)(m * SensorValue + b);
	//Return the value of the light with maximum value equal to 1000
	if (lux > 1000)
		lux = 1000;

	return lux;
}
static int getHumidityValue(uint8_t phidget_input){
	//Read voltage from the phidget interface
	float voltage;
	float SensorValue;
	int humidity;
	//SENSORS_ACTIVATE(phidgets);
	voltage=phidgets.value(phidget_input);
	//Convert the voltage in lux with the provided formula
	SensorValue = voltage/4.096;
	humidity = (int)(SensorValue*0.1906 - 40.2) ;
	return humidity;
}
