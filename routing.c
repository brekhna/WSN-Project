// Wireless sensor network Laboratory WS 16/17
// Project: Smart farming , Group: 2 
// Contributors: Sandeep, Iffat, Ilkin

/* Description of the file */
// This file contains all the functions required for routing using Dijkstra algorithm. 



/* 
This function calculates the shortest path from each mote to the Gateway and return the path array. 
The arguments of the function are the RSSI table and a number 1 or 2 which denotes whether it is first or second path search. The RSSI 
table is a float array of 10 elements (since we use 10 motes). The gateway is the index id 0 and all other motes have id from 1-9.  
So, 0th index contains RSSI value of gateway node , 1st index contains RSSI value of node id 1 and so on. The second argument, 1 or 2 
is used for error recovery. At first path search attempt, 1 is passed . But if ACK is not received by a node, it sends again path search request 
with the value 2 (which means second attempt). In second attempt, path is calculated by discarding the node with highest RSSI value 
(assumption is that the nearest node has some error. We can search other ways too.) . The function returns an array of 10 indexes where 0th 
index is the id of node and 1st index is the id of another node in the path and so on. 

*/

calculate_path()
{
  
}
