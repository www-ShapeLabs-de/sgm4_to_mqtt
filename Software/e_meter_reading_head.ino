/*------------------------------------------------------------------------------
 Copyright:	  Henning Schaper  mailto: henningschaper@gmx.de
 Author:		 Henning Schaper
 Remarks:		none
 known Problems: none
 Version:		v1.4	01.04.2022
 Description:  	 Hassio 
 				 Processor: ESP8266 (ESP12F)
 				 Features:  - homeassistant autoconfig
 				 			- OTA
 				 			- JSON over MQTT 
 				 			- 
 				 			- 
 				 			- static wifi, no wifimanager (more stable)

------------------------------------------------------------------------------*/

//----------------------------------------------------------------------------//
//-------------------------------- Includes  ---------------------------------//
//----------------------------------------------------------------------------//
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <SoftwareSerial.h>

//----------------------------------------------------------------------------//
//-------------------------------- Config  -----------------------------------//
//----------------------------------------------------------------------------//
#define SOFTWARE_VERSION		"v1.4"
#define WLAN_SSID 				"myssid"
#define WLAN_PASS 				"mywifipasswd"
#define MQTT_SERVER 			"192.168.1.2"
#define MQTT_SERVERPORT 		1883 					// use 8883 for SSL
#define MQTT_USERNAME 			"mqttuser"
#define MQTT_KEY 				"mosquitto"
#define LOCATION				"my_location"
#define NAME 					"E_Meter"
//#define OTA_PASS				"admin"
#define STATE_UPDATE_INTERVAL	5000 					// MQTT state update interval in [ms] (<5000ms could be critical for parsing in worst case)
#define AVAILABILITY_ONLINE 	"online"
#define AVAILABILITY_OFFLINE 	"offline"
//#define DEBUG 										// Enable Debug Level1
//#define DEBUG2 										// Enable Debug Level2

//----------------------------------------------------------------------------//
//--------------------------------- Macros -----------------------------------//
//----------------------------------------------------------------------------//
#define SOFT_RX_PIN		4
#define SOFT_TX_PIN		5
#define SML_MESSAGE_SIZE 600

//----------------------------------------------------------------------------//
//----------------------------- Constants & LUTs -----------------------------//
//----------------------------------------------------------------------------//
const uint8_t startSequence[] = 						{ 0x1B, 0x1B, 0x1B, 0x1B, 0x01, 0x01, 0x01, 0x01 }; //start sequence of SML protocol
const uint8_t stopSequence[]  = 						{ 0x1B, 0x1B, 0x1B, 0x1B, 0x1A }; //end sequence of SML protocol

const uint8_t Zaehlerstand_Bezug_sequence[] = 			{0x77, 0x07, 0x01, 0x00, 0x01, 0x08, 0x00, 0xFF}; 
//const uint8_t Zaehlerstand_Einspeisung_sequence[] = 	{0x77, 0x07, 0x01, 0x00, 0x02, 0x08, 0x00, 0xFF}; 
const uint8_t Wirkleistung_sequence[] = 				{0x77, 0x07, 0x01, 0x00, 0x10, 0x07, 0x00, 0xFF}; 
const uint8_t Spannung_L1_sequence[] = 					{0x77, 0x07, 0x01, 0x00, 0x20, 0x07, 0x00, 0xFF}; 
const uint8_t Spannung_L2_sequence[] = 					{0x77, 0x07, 0x01, 0x00, 0x34, 0x07, 0x00, 0xFF}; 
const uint8_t Spannung_L3_sequence[] = 					{0x77, 0x07, 0x01, 0x00, 0x48, 0x07, 0x00, 0xFF}; 
const uint8_t Strom_L1_sequence[] = 					{0x77, 0x07, 0x01, 0x00, 0x1F, 0x07, 0x00, 0xFF}; 
const uint8_t Strom_L2_sequence[] = 					{0x77, 0x07, 0x01, 0x00, 0x33, 0x07, 0x00, 0xFF}; 
const uint8_t Strom_L3_sequence[] = 					{0x77, 0x07, 0x01, 0x00, 0x47, 0x07, 0x00, 0xFF}; 
//const uint8_t Phasenwinkel_U-L2_U-L1_sequence[] = 		{0x77, 0x07, 0x01, 0x00, 0x51, 0x07, 0x01, 0xFF}; 
//const uint8_t Phasenwinkel_U-L3_U-L1_sequence[] = 		{0x77, 0x07, 0x01, 0x00, 0x51, 0x07, 0x02, 0xFF}; 
const uint8_t Phi_L1_sequence[] = 						{0x77, 0x07, 0x01, 0x00, 0x51, 0x07, 0x04, 0xFF}; 
const uint8_t Phi_L2_sequence[] = 						{0x77, 0x07, 0x01, 0x00, 0x51, 0x07, 0x0F, 0xFF}; 
const uint8_t Phi_L3_sequence[] = 						{0x77, 0x07, 0x01, 0x00, 0x51, 0x07, 0x1A, 0xFF}; 
const uint8_t Frequenz_sequence[] = 					{0x77, 0x07, 0x01, 0x00, 0x0E, 0x07, 0x00, 0xFF}; 

const float cos_table [] = 	{	1.000000, 0.999847, 0.999389, 0.998626, 0.997557, 0.996184, 0.994507, 0.992526, 0.990241, 0.987655,
 								0.984766, 0.981577, 0.978088, 0.974300, 0.970215, 0.965833, 0.961156, 0.956186, 0.950923, 0.945370, 
 								0.939529, 0.933400, 0.926987, 0.920290, 0.913312, 0.906055, 0.898521, 0.890713, 0.882633, 0.874283, 
 								0.865666, 0.856785, 0.847642, 0.838240, 0.828582, 0.818671, 0.808510, 0.798102, 0.787451, 0.776558, 
 								0.765429, 0.754066, 0.742472, 0.730652, 0.718608, 0.706345, 0.693866, 0.681175, 0.668277, 0.655174, 
 								0.641871, 0.628371, 0.614680, 0.600802, 0.586739, 0.572498, 0.558081, 0.543495, 0.528742, 0.513827, 
 								0.498756, 0.483532, 0.468161, 0.452647, 0.436994, 0.421208, 0.405293, 0.389255, 0.373097, 0.356826, 
 								0.340446, 0.323961, 0.307378, 0.290701, 0.273935, 0.257085, 0.240157, 0.223155, 0.206086, 0.188953, 
 								0.171763, 0.154520, 0.137230, 0.119898, 0.102529, 0.085129, 0.067703, 0.050257, 0.032795, 0.015323, 0.000000};

//----------------------------------------------------------------------------//
//------------------------------- Variables  ---------------------------------//
//----------------------------------------------------------------------------//
uint8_t smlMessage[SML_MESSAGE_SIZE];
uint16_t parse_index = 0;
uint16_t smlIndex = 0;
uint8_t inByte = 0;
uint8_t startIndex = 0;
uint8_t stopIndex = 0;
uint8_t sml_read_state = 0;

double Zaehlerstand = 0;
float Wirkleistung = 0;
float SpannungL1 = 0;
float SpannungL2 = 0;
float SpannungL3 = 0;
float StromL1 = 0;
float StromL2= 0;
float StromL3= 0;
float Phi_L1 = 0;
float Phi_L2 = 0;
float Phi_L3 = 0;
float Netzfrequenz = 0;

char identifier[24];
char MQTT_TOPIC_AVAILABILITY[128];
char MQTT_TOPIC_STATE[128];
char MQTT_TOPIC_COMMAND[128];

uint32_t PreviousStateUpdateTime = 0;
uint32_t CurrentTime = millis();
bool parse_error_flag = false;

//----------------------------------------------------------------------------//
//--------------------------------- Objects ----------------------------------//
//----------------------------------------------------------------------------//
SoftwareSerial SoftSerial(SOFT_RX_PIN, SOFT_TX_PIN);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

//----------------------------------------------------------------------------//
//------------------------------ Main Program --------------------------------//
//----------------------------------------------------------------------------//
void setup()
{
	Serial.begin(115200);
	Serial.println("Start INIT");
	SoftSerial.begin(9600);
	
	snprintf(identifier, sizeof(identifier), "%s.%s",LOCATION,NAME); 
	snprintf(MQTT_TOPIC_AVAILABILITY, 127, "%s/%s/status", LOCATION, NAME);
	snprintf(MQTT_TOPIC_STATE, 127, "%s/%s/state", LOCATION, NAME);
	snprintf(MQTT_TOPIC_COMMAND, 127, "%s/%s/command", LOCATION, NAME);

	connect_WiFi();
	setupOTA();
	
	mqttClient.setServer(MQTT_SERVER, MQTT_SERVERPORT);
	mqttClient.setKeepAlive(10);
	mqttClient.setBufferSize(2048);
	mqttClient.setCallback(mqttCallback);

	mqttReconnect();
	Serial.println("INIT COMPLETE!");
}

void loop()
{
	mqttClient.loop();
	ArduinoOTA.handle();

	if(!sml_read_state)
		get_sml_sync();
	else if (sml_read_state == 1)
		get_sml_message();
	else if(sml_read_state == 2)
	{
		Zaehlerstand = get_total_consumption_from_SML(Zaehlerstand_Bezug_sequence);
		Wirkleistung = get_values_from_SML(Wirkleistung_sequence);
		SpannungL1 = get_values_from_SML(Spannung_L1_sequence);
		SpannungL2 = get_values_from_SML(Spannung_L2_sequence);
		SpannungL3 = get_values_from_SML(Spannung_L3_sequence);
		StromL1 = get_values_from_SML(Strom_L1_sequence);
		StromL2 = get_values_from_SML(Strom_L2_sequence);
		StromL3 = get_values_from_SML(Strom_L3_sequence);
		Phi_L1 = get_values_from_SML(Phi_L1_sequence);
		Phi_L2 = get_values_from_SML(Phi_L2_sequence);
		Phi_L3 = get_values_from_SML(Phi_L3_sequence);
		Netzfrequenz = get_values_from_SML(Frequenz_sequence);

		if(!parse_error_flag)							//Parsing successfull completed?
			sml_read_state = 3;							//
		else											//else: Error@Parsing! Discard any data, to prevent publishing zero-values ore other wrong values
			sml_read_state = 0;							//go to next state of statemachine -> STATE 3: publishing data
	}
	
	if(STATE_UPDATE_INTERVAL <= (CurrentTime - PreviousStateUpdateTime))
	{
		if (WiFi.status() != WL_CONNECTED) 
			connect_WiFi(); 

		if (!mqttClient.connected())
			mqttReconnect();
		
		if(sml_read_state == 3)
		{
			#ifdef DEBUG
				Serial.println("Publish");
			#endif
			publishState();
			sml_read_state = 0;
		}
	}
	CurrentTime = millis();							// Get current time
}

//----------------------------------------------------------------------------//
//----------------------- Standard Functions for ESP -------------------------//
//----------------------------------------------------------------------------//
void Reset_ESP()
{
	WiFi.disconnect(true);
	delay(1000);
	ESP.restart();	
}

void connect_WiFi()
{
	Serial.print("Connecting to ");
	Serial.println(WLAN_SSID);
	
	WiFi.mode(WIFI_STA);
	WiFi.begin(WLAN_SSID, WLAN_PASS);

	while (WiFi.status() != WL_CONNECTED)
	{
		delay(100);
		Serial.print(".");
	}

	Serial.println("");
	Serial.println("WiFi connected");
	Serial.println("IP address: ");
	Serial.println(WiFi.localIP());
}

void setupOTA()
{
	ArduinoOTA.onStart([]() { Serial.println("Start"); });
	ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });

	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
	{
		Serial.printf("Progress: %u%%\r\n", (progress / (total / 100)));
	});

	ArduinoOTA.onError([](ota_error_t error)
	{
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");
	});

	ArduinoOTA.setHostname(identifier);
	#ifdef OTA_PASS
		ArduinoOTA.setPassword(OTA_PASS);
	#endif
	ArduinoOTA.begin();
}


void mqttReconnect()
{
	for(int attempt = 0; attempt < 3; ++attempt) 
	{
		Serial.print("Connecting MQTT....");
		if(mqttClient.connect(identifier, MQTT_USERNAME, MQTT_KEY, AVAILABILITY_ONLINE, 1, true, AVAILABILITY_OFFLINE)) 
		{	
			Serial.println("Connected!");
			mqttClient.publish(MQTT_TOPIC_AVAILABILITY, AVAILABILITY_ONLINE, true);
			
			Serial.println("Sending Autoconfig....");

			publish_autoconfig_entity("WiFi","dBm","mdi:wifi",3);
			publish_autoconfig_entity("Zaehlerstand","Wh","mdi:flash",1);
			publish_autoconfig_entity("Wirkleistung","W","mdi:alpha-p-circle",3);
			publish_autoconfig_entity("SummePhasenleistung","W","mdi:alpha-p-circle",3);
			publish_autoconfig_entity("Spannung_L1","V","mdi:alpha-u-circle-outline",3);
			publish_autoconfig_entity("Spannung_L2","V","mdi:alpha-u-circle-outline",3);
			publish_autoconfig_entity("Spannung_L3","V","mdi:alpha-u-circle-outline",3);
			publish_autoconfig_entity("Strom_L1","A","mdi:alpha-i-circle",3);
			publish_autoconfig_entity("Strom_L2","A","mdi:alpha-i-circle",3);
			publish_autoconfig_entity("Strom_L3","A","mdi:alpha-i-circle",3);
			publish_autoconfig_entity("cosPhi_L1"," ","mdi:angle-acute",3);
			publish_autoconfig_entity("cosPhi_L2"," ","mdi:angle-acute",3);
			publish_autoconfig_entity("cosPhi_L3"," ","mdi:angle-acute",3);
			publish_autoconfig_entity("Leistung_L1","W","mdi:alpha-p-circle-outline",3);
			publish_autoconfig_entity("Leistung_L2","W","mdi:alpha-p-circle-outline",3);
			publish_autoconfig_entity("Leistung_L3","W","mdi:alpha-p-circle-outline",3);
			publish_autoconfig_entity("Netzfrequenz","Hz","mdi:sine-wave",3);
			publish_autoconfig_entity("Reset"," ","mdi:replay" ,2);

			Serial.println("Subscribing Topics....");
			mqttClient.subscribe(MQTT_TOPIC_COMMAND);
			Serial.println("Success!");
			break;
		}
		else
		{
			delay(5000);
			Serial.print("FAILED, Retry...");
		}
	}
}

//----------------------------------------------------------------------------//
//----------------------------- MQTT Functions -------------------------------//
//----------------------------------------------------------------------------//
void mqttCallback(char* topic, byte* payload, unsigned int length)
{
	if (strcmp(topic, MQTT_TOPIC_COMMAND) == 0)
	{
		DynamicJsonDocument commandJson(256);
		char payloadText[length + 1];
		snprintf(payloadText, length + 1, "%s", payload);
		DeserializationError err = deserializeJson(commandJson, payloadText);

		if (!err)
		{
			String reset = commandJson["Reset"].as<String>();
			if (reset == "on")
				Reset_ESP();									//ECHO
		}
	}
}

void publishState()
{
	DynamicJsonDocument stateJson(512);
	char payload[512];
	
	float P_L1 = my_cos_phi(Phi_L1)*SpannungL1*StromL1;
	float P_L2 = my_cos_phi(Phi_L1)*SpannungL2*StromL2;
	float P_L3 = my_cos_phi(Phi_L1)*SpannungL3*StromL3;

	stateJson["Reset"] = "off";
	stateJson["WiFi"] = WiFi.RSSI();
	stateJson["Zaehlerstand"] = String(Zaehlerstand,1);
	stateJson["Wirkleistung"] = String(Wirkleistung,0);
	stateJson["Spannung_L1"] = String(SpannungL1,1);
	stateJson["Spannung_L2"] = String(SpannungL2,1);
	stateJson["Spannung_L3"] = String(SpannungL3,1);
	stateJson["Strom_L1"] = String(StromL1,2);
	stateJson["Strom_L2"] = String(StromL2,2);
	stateJson["Strom_L3"] = String(StromL3,2);
	stateJson["cosPhi_L1"] = String(my_cos_phi(Phi_L1),2);
	stateJson["cosPhi_L2"] = String(my_cos_phi(Phi_L2),2);
	stateJson["cosPhi_L3"] = String(my_cos_phi(Phi_L3),2);
	stateJson["Leistung_L1"] = String(P_L1,2);
	stateJson["Leistung_L2"] = String(P_L2,2);
	stateJson["Leistung_L3"] = String(P_L3,2);
	stateJson["SummePhasenleistung"] = String(P_L1+P_L2+P_L3);
	stateJson["Netzfrequenz"] = String(Netzfrequenz,1);

	serializeJson(stateJson, payload);
	mqttClient.publish(MQTT_TOPIC_STATE, payload, true);

	PreviousStateUpdateTime = millis();
}

//TYPE: 1=Energy-Meter, 2=Switch, 3=Sensor
void publish_autoconfig_entity(const char value_name[], const char unit[], const char icon[], uint8_t type)
{
	char mqttPayload[1024];
	DynamicJsonDocument autoconfPayload(1024);
	DynamicJsonDocument device(256);
	char MQTT_TOPIC_AUTOCONF_TOPIC[128];

	device["identifiers"] = String("[") + identifier + String("]");
	device["manufacturer"] = "ShapeLabs.de";
	device["model"] = NAME;
	device["name"] = identifier;
	device["sw_version"] = SOFTWARE_VERSION;
	autoconfPayload["device"] = device.as<JsonObject>();
	autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
	autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
	autoconfPayload["name"] = identifier + String(" ") + value_name;
	autoconfPayload["unique_id"] = identifier + String("_") + value_name;
	autoconfPayload["icon"] = icon;
	autoconfPayload["value_template"] = String("{{value_json.") + value_name + String("}}");
	
	if(type==1)
	{
		autoconfPayload["device_class"] = "energy";
		autoconfPayload["state_class"] = "total_increasing";
		type = 3;
	}
	else if(type==2)
	{
		autoconfPayload["command_topic"] = MQTT_TOPIC_COMMAND;
		autoconfPayload["payload_on"] = String("{\"") + value_name + String("\": \"on\"}");
		autoconfPayload["payload_off"] = String("{\"") + value_name + String("\": \"off\"}");
		autoconfPayload["state_on"] = "on";
		autoconfPayload["state_off"] = "off";
		snprintf(MQTT_TOPIC_AUTOCONF_TOPIC, 127, "homeassistant/switch/%s/%s_%s/config", LOCATION, NAME, value_name);
	}
	if(type == 3)
	{
		autoconfPayload["unit_of_measurement"] = unit;
		snprintf(MQTT_TOPIC_AUTOCONF_TOPIC, 127, "homeassistant/sensor/%s/%s_%s/config", LOCATION, NAME, value_name);
	}

	serializeJson(autoconfPayload, mqttPayload);
	mqttClient.publish(MQTT_TOPIC_AUTOCONF_TOPIC, mqttPayload, true);
	device.clear();
	autoconfPayload.clear();
	delay(100);
}

//----------------------------------------------------------------------------//
//----------------------------- SML Functions --------------------------------//
//----------------------------------------------------------------------------//
void get_sml_sync()																//STATE 0: Function to finde the start sequence of an incomming sml-packet
{
	clear_all_sml();															//clear RX-Buffer, clear message buffer from any old data fragments, reset all flags and counter
	delay(20);																	//wait until there's any fresh data in the RX-buffer again

	while(SoftSerial.available())												//start to read the RX-buffer while there is any data
	{
		inByte = SoftSerial.read(); 											//reading data
		
		if(inByte == startSequence[startIndex]) 								//in case incomming byte matches the start sequence at position 0,1,2...
		{
			smlMessage[startIndex++] = inByte; 									//set smlMessage element at position 0,1,2 to inByte value
			if(startIndex == sizeof(startSequence)) 							//all start sequence values have been identified
			{
				smlIndex = startIndex;
				#ifdef DEBUG 
					Serial.println("SML start found");							//some debug output...
				#endif
				sml_read_state = 1;												//go to next state of statemachine -> STATE 1: recording
				//startIndex = 0;
				break;															//leaving the loop
			}
		}
		else
			startIndex = 0;														//Jump back, next try to hit the start sequence
	}
}

void get_sml_message()															//STATE 1: Function to record the complete sml-packet till it's end by checking continously for end-secuence
{
	while(SoftSerial.available())												//start to read the RX-buffer while there is any data
	{
		inByte = SoftSerial.read();												//reading data
		smlMessage[smlIndex++] = inByte;										//writing data to message buffer

		#ifdef DEBUG2															//debug output on hw-serial of incomming data
			if(!(inByte & 0xF0))												//print leading zero if nessecary
				Serial.print("0");
			Serial.print(inByte, HEX);											//print value as hex
			Serial.print(" ");							
			if(!(smlIndex%30))													//newline after 30 values
				Serial.print("\r\n");
		#endif

		if(inByte == stopSequence[stopIndex])									//in case incomming byte matches the start sequence at position 0,1,2...
		{
			stopIndex++;														// hit, try next digit in next pass of loop
			if(stopIndex == sizeof(stopSequence))								// complete stop sequence found
			{
				#ifdef DEBUG 							
					Serial.print("\r\n");
					Serial.println("SML-message valid recording complete");
				#endif

				sml_read_state = 2;												//go to next state of statemachine -> STATE 2: parsing of all the data
				break;															// leaving the loop
			}
		}
		else
			stopIndex = 0;

		if(smlIndex > sizeof(smlMessage))										//no stop sequence found in the complete message buffer, prevending buffer overflow
		{
			#ifdef DEBUG
				Serial.print("\r\n");
				Serial.println("ERROR no valid SML-message found");
			#endif

			sml_read_state = 0;													// canceling, next try, -> STATE 0: clearing buffers & flags, searching start sequence in incomming data
			break;																// leaving the loop
		}
	}
}


double get_total_consumption_from_SML(const uint8_t* sequence) 
{
	uint32_t temp = 0;
	uint8_t sequence_index = 0; 												//start at position 0 of recorded sml-message
	float scaler = 0.0;
	double output = 0;

	for(uint16_t x=parse_index; x<sizeof(smlMessage); x++)						//for as long there are not reached the end of message buffer
	{ 
		if(smlMessage[x] == sequence[sequence_index]) 							//compare with sequence
		{ 
			sequence_index++;
			if(sequence_index == 8)												//in complete sequence is found
			{
				scaler = pow(10.0,(int8_t)smlMessage[x+16]);

				temp += smlMessage[x+18];
				temp <<= 8; 
				temp +=smlMessage[x+19];
				temp <<= 8; 
				temp +=smlMessage[x+20];
				temp <<= 8; 
				temp +=smlMessage[x+21];
				parse_index = x+21;

				output= (double)temp*scaler;

				#ifdef DEBUG
					Serial.print("sequence_found...4Byte parse complete: ");
					Serial.println(String(output));
				#endif

				return(output);
			}
		}
		else 
			sequence_index = 0;													//Jump back...
	}
	
	#ifdef DEBUG
		Serial.println("Parse ERROR 4bit - No Sequence found");
	#endif
	parse_error_flag = true;
	return 0;
}

float get_values_from_SML(const uint8_t* sequence) 
{
	uint16_t temp = 0;
	uint8_t sequence_index = 0; 												//start at position 0 of exctracted SML message
	float scaler = 0.0;
	float output = 0.0;

	for(uint16_t x=parse_index; x<sizeof(smlMessage); x++)						//for as long there are element in the exctracted SML message
	{ 
		if(smlMessage[x] == sequence[sequence_index]) 							//compare with sequence
		{ 
			
			sequence_index++;
			if(sequence_index == 8)												//in complete sequence is found
			{
				Serial.print("sequence_found...");

				scaler = pow(10.0,(int8_t)smlMessage[x+6]);						
				
				if((smlMessage[x+7] & 0x0F) == 2)								////Checking datatyp of Value: 0x_2 = 1Byte
				{
					temp = smlMessage[x+8];
					parse_index = x+7;											//Store parse index for next value. //!!!!Note the right order of parse-function calls, depending to the order of values in the message
					
					if((smlMessage[x+7] & 0xF0) == 0x60)						//Checking datatyp of Value 0x6_ = u_int, 	0x5_ = int					
						output = (uint8_t)temp*scaler;
					else if((smlMessage[x+7] & 0xF0) == 0x50)
						output = (int8_t)temp*scaler;
					
					#ifdef DEBUG
						Serial.print("1Byte parse complete: ");
						Serial.println(String(output));
					#endif
					
					return(output);
				}
				else if((smlMessage[x+7] & 0x0F) == 3)							//Checking datatyp of Value: 0x_3 = 2Byte
				{
					temp = smlMessage[x+8];
					temp <<= 8;
					temp += smlMessage[x+9];
					parse_index = x+8;											//Store parse index for next value. //!!!!Note the right order of parse-function calls, depending to the order of values in the message
					
					if((smlMessage[x+7] & 0xF0) == 0x60)						//Checking datatyp of Value 0x6_ = u_int, 	0x5_ = int	
						output = (uint16_t)temp*scaler;
					else if((smlMessage[x+7] & 0xF0) == 0x50)
						output = (int16_t)temp*scaler;
					
					#ifdef DEBUG
						Serial.print("2Byte parse complete: ");
						Serial.println(String(output));
					#endif

					return(output);
				}
				else
				{
					Serial.println("Parse ERROR 1 - Wrong data type");
					parse_error_flag = true;
					return 0;
				}
			}
		}
		else 
			sequence_index = 0;													//Jump back...
	}
	
	#ifdef DEBUG2
		Serial.println("Parse ERROR 2 - No sequence found");
	#endif
	parse_error_flag = true;
	return 0;
}

void clear_all_sml(void)
{
	while(SoftSerial.available())												//empty the RX-buffer
		SoftSerial.read();
	for(uint16_t i = 0; i<sizeof(smlMessage);i++)								//empty message-buffer
		smlMessage[i]=0;
	parse_index = 0;															//reset all indexes and flags
	smlIndex = 0;
	inByte = 0;
	startIndex = 0;
	stopIndex = 0;
	sml_read_state = 0;
	parse_error_flag = false;
}


//----------------------------------------------------------------------------//
//---------------------------- Other Functions -------------------------------//
//----------------------------------------------------------------------------//
float my_cos_phi(float phi)
{
	if(phi <= 90) 			return(cos_table[(uint16_t)phi]);
	else if (phi <= 180)	return(cos_table[180-(uint16_t)phi]);
	else if (phi <= 270)	return(cos_table[(uint16_t)phi-180]);
	else					return(cos_table[360-(uint16_t)phi]);
}


