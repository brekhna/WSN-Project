// Wireless sensor network Laboratory WS 16/17
// Project: Smart farming , Group: 2 
// Contributors: Sandeep, Iffat, Ilkin

/* Description of the file */

/*
This function returns the values of the sensors in an array. 

*/

get_sensors_value()
{
 get_temp();
 light = getLightSensorValue(1.5881,40.157,PHIDGET3V_2 ); 
}
static void get_temp()
{
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
		tempint  = (absraw >> 8) * sign;
		// Extract the fractional part in 1/10000 of degree:
		//tempfrac = ((absraw >> 4) % 16) * 625;
		// Obtain the sign:
		if((tempint == 0) & (sign == -1)){
		  minus = '-';
		} else {
		  minus = ' ';
		}  
}
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
