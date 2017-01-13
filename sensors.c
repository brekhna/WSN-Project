// Wireless sensor network Laboratory WS 16/17
// Project: Smart farming , Group: 2 
// Contributors: Sandeep, Iffat, Ilkin

/* Description of the file */

/*
This function returns the values of the sensors in an array. 

*/

get_sensors_value()
{
 SENSORS_ACTIVATE(phidgets);
 get_temp();
 getLightValue(1.5881,40.157,PHIDGET3V_2 ); 
 getHumidityValue(PHIDGET5V_2);
}

static void get_temp()
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
		message.temp_sensor_reading_int = tempint;
		// Extract the fractional part in 1/10000 of degree:
		//tempfrac = ((absraw >> 4) % 16) * 625;
		// Obtain the sign:
		if((tempint == 0) & (sign == -1)){
		  	minus = '-';
		  	message.temp_sensor_reading_sign = minus;
			
		} else {
		  	minus = ' ';
		  	message.temp_sensor_reading_sign = minus;
		}  
}
static void getLightValue(float m, float b, uint8_t phidget_input){
	//Read voltage from the phidget interface
	float voltage;
	float SensorValue;
	int lux;
	//SENSORS_ACTIVATE(phidgets);
	voltage=phidgets.value(phidget_input);
	//Convert the voltage in lux with the provided formula
	SensorValue = voltage/4.096;
	lux = (int)(m * SensorValue + b);
	//Return the value of the light in boolan whether it is night or noon
	if (lux < 400)
		message.light_sensor_reading==FALSE;
	else 
		message.light_sensor_reading==TRUE;
}
static void getHumidityValue(uint8_t phidget_input){
	//Read voltage from the phidget interface
	float voltage;
	float SensorValue;
	int humidity;
	//SENSORS_ACTIVATE(phidgets);
	voltage=phidgets.value(phidget_input);
	//Convert the voltage in lux with the provided formula
	SensorValue = voltage/4.096;
	humidity = (int)(SensorValue*0.1906 - 40.2) ;
	
	if (humidity < 50)
		message.humidity_sensor_reading==FALSE;
	else 
		message.humidity_sensor_reading==TRUE;
}
