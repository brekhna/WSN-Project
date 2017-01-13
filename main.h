










enum msg_type
{
  wakeup_beacon= 0, 
  dummy_packet = 1,
  sensor_value= 2,
  ack = 3,
  sleep_beacon = 4
};


enum mote_state 
{
  sleep = 0,
  active = 1,
  idle = 2,

};


struct message
{
	int msg_id;
	enum message_type  msg_type;
	int path[10];
	int source_node_id;
	int destination_node_id;
	float temp_sensor_reading_int;
	char temp_sensor_reading_sign;
	_Bool light_sensor_reading;
	_Bool humidity_sensor_reading;

};
