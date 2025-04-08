//==============================================================
// Pico_4ToF_Wifi based on
// https://github.com/openlcb/OpenLCB_Single_Thread/tree/master/examples/Pico_8ServoWifiGC
//
// Modified John Callingham 2025, added;-
// - removed all servo code.
// - added four Time of Flight sensors.
// - each sensor has four distance thresholds.
// - events produced when a threshold is passed.
// - responds with current state to a JMRI sensor query.
// MOdified DPH 2024
// Copyright 2019 Alex Shepherd and David Harris
//==============================================================
#define DEBUG Serial
#define OLCB_NO_BLUE_GOLD
#define NOCAN

const char* ssid     = "RPi-JMRI";     // <-- fill in with your network name
const char* password = "rpI-jmri";         // <-- fill in with youy network password
const char* openLCB_can  = "openlcb-can";  // <-- change this if necessary
#include "PicoWifiGC.h"

#include <Arduino.h>
#include "Global.h"

// Board definitions
#define MANU "J Callingham"  // The manufacturer of node
#define MODEL "Pico_4ToF_Wifi" // The model of the board
#define HWVERSION "0.1"   // Hardware version
#define SWVERSION "0.1"   // Software version

// To Reset the Node Number, Uncomment and edit the next line
#define NODE_ADDRESS  5,1,1,1,0x91,0x03

// Set to 1 to Force Reset EEPROM to Factory Defaults 
// Need to do this at least once.  
#define RESET_TO_FACTORY_DEFAULTS 0

#define NUM_EVENT NUM_SENSOR * NUM_THRESHOLD * 2 // Each threshold has two events, one for each threshold being passed.

#include "mdebugging.h"           // debugging
#include "processor.h"
#include "processCAN.h"
#include "OpenLCBHeader.h"

// For the ToF sensors
#include "ToFSensor.h"
ToFSensor sensor;

// For the thresholds
#include "Threshold.h"
Threshold threshold;
  
// CDI (Configuration Description Information) in xml, must match MemStruct
// See: http://openlcb.com/wp-content/uploads/2016/02/S-9.7.4.1-ConfigurationDescriptionInformation-2016-02-06.pdf
extern "C" {
    #define N(x) xN(x)     // allow the insertion of the value (x) ..
    #define xN(x) #x       // .. into the CDI string. 
const char configDefInfo[] PROGMEM =
// ===== Enter User definitions below =====
  CDIheader R"(
    <group>
        <name>Node Congiguration</name>
        <group replication=')" N(NUM_SENSOR) R"('>
            <name>ToF Sensors</name>
            <repname>1</repname>
            <string size='24'><name>Description</name></string>
            <group replication=')" N(NUM_THRESHOLD) R"('>
                <name>Thresholds</name>
                <repname>1</repname>
                <int size='1'>
                  <name>Near threshold (mm)</name>
                  <min>0</min>
                  <max>200</max>
                  <hints><slider tickSpacing='40' immediate='yes' showValue='true'></slider></hints>
                </int>
                <int size='1'>
                  <name>Far threshold (mm)</name>
                  <min>0</min>
                  <max>200</max>
                  <hints><slider tickSpacing='40' immediate='yes' showValue='true'></slider></hints>
                </int>
                <eventid>
                  <name>Near EventID</name>
                  <description>Sent when an object moves nearer to the sensor than the near threshold</description>
                </eventid>
                <eventid>
                  <name>Far EventID</name>
                  <description>Sent when an object moves further away from the sensor than the far threshold</description>
                </eventid>
            </group>
        </group>
    </group>
    )" CDIfooter;
// ===== Enter User definitions above =====
} // end extern

// ===== MemStruct =====
//   Memory structure of EEPROM, must match CDI above
    typedef struct { 
          EVENT_SPACE_HEADER eventSpaceHeader; // MUST BE AT THE TOP OF STRUCT - DO NOT REMOVE!!!
          
          char nodeName[20];  // optional node-name, used by ACDI
          char nodeDesc[24];  // optional node-description, used by ACDI
      // ===== Enter User definitions below =====
          struct {
            char sensordesc[24];        // description of this Sensor
            struct {
              uint8_t thresholdNear;       // threshold for ON event
              uint8_t thresholdFar;       // Threshold for OFF event
              EventID eidNear;       // ON eventID
              EventID eidFar;       // OFF eventID
            } threshold[NUM_THRESHOLD];
          } sensor[NUM_SENSOR];
      // ===== Enter User definitions above =====
      // items below will be included in the EEPROM, but are not part of the CDI
    } MemStruct;      // type definition
    
// This is called to initialize the EEPROM during Factory Reset.
void userInitAll() {
  NODECONFIG.put(EEADDR(nodeName), ESTRING("this node's name"));
  NODECONFIG.put(EEADDR(nodeDesc), ESTRING("this node's description"));
      
  for (uint8_t i = 0; i < NUM_SENSOR; i++) {
    NODECONFIG.put(EEADDR(sensor[i].sensordesc), ESTRING(""));
  
    // Set sensible default thresholds.
    NODECONFIG.put(EEADDR(sensor[i].threshold[0].thresholdNear), (uint8_t) 20);
    NODECONFIG.put(EEADDR(sensor[i].threshold[0].thresholdFar), (uint8_t) 30);
    NODECONFIG.put(EEADDR(sensor[i].threshold[1].thresholdNear), (uint8_t) 50);
    NODECONFIG.put(EEADDR(sensor[i].threshold[1].thresholdFar), (uint8_t) 60);
    NODECONFIG.put(EEADDR(sensor[i].threshold[2].thresholdNear), (uint8_t) 90);
    NODECONFIG.put(EEADDR(sensor[i].threshold[2].thresholdFar), (uint8_t) 100);
    NODECONFIG.put(EEADDR(sensor[i].threshold[3].thresholdNear), (uint8_t) 150);
    NODECONFIG.put(EEADDR(sensor[i].threshold[3].thresholdFar), (uint8_t) 160);
  }
}
    
extern "C" {
    // ===== eventid Table =====
    // useful macros to help fill the table

    // Each threshold has two events.
    #define REG_THRESHOLD(s,p) PEID(sensor[s].threshold[p].eidNear), PEID(sensor[s].threshold[p].eidFar)

    // Each sensor has four thresholds.
    #define REG_SENSOR(s) REG_THRESHOLD(s,0), REG_THRESHOLD(s,1), REG_THRESHOLD(s,2), REG_THRESHOLD(s,3)

    //  Array of the offsets to every eventID in MemStruct/EEPROM/mem, and P/C flags
    const EIDTab eidtab[NUM_EVENT] PROGMEM = {
        // There are four sensors.
        REG_SENSOR(0), REG_SENSOR(1), REG_SENSOR(2), REG_SENSOR(3)
    };
    
    // SNIP Short node description for use by the Simple Node Information Protocol
    // See: http://openlcb.com/wp-content/uploads/2016/02/S-9.7.4.3-SimpleNodeInformation-2016-02-06.pdf
    extern const char SNII_const_data[] PROGMEM = "\001" MANU "\000" MODEL "\000" HWVERSION "\000" OlcbCommonVersion ; // last zero in double-quote
} // end extern "C"
    
// PIP Protocol Identification Protocol uses a bit-field to indicate which protocols this node supports
// See 3.3.6 and 3.3.7 in http://openlcb.com/wp-content/uploads/2016/02/S-9.7.3-MessageNetwork-2016-02-06.pdf
uint8_t protocolIdentValue[6] = {   //0xD7,0x58,0x00,0,0,0};
        pSimple | pDatagram | pMemConfig | pPCEvents | !pIdent    | pTeach     | !pStream   | !pReservation, // 1st byte
        pACDI   | pSNIP     | pCDI       | !pRemote  | !pDisplay  | !pTraction | !pFunction | !pDCC        , // 2nd byte
        0, 0, 0, 0                                                                                           // remaining 4 bytes
    };

// determine the state of each eventid
enum evStates { VALID=4, INVALID=5, UNKNOWN=7 };
uint8_t userState(uint16_t index) {

  // Determine the current state for the threshold for this event index.
  State currentState = threshold.getStateForEventIndex(index);

  switch (currentState) {
    case State::Unknown:
      break;
    case State::Near:
      break;
    case State::Far:
      break;
    default:
      break;
  }

  if (index % 2 == 0) {
    // index is even so this is a Near event.
    if (currentState == State::Near) return VALID; else return INVALID;
  } else {
    // index is odd so this is a Far event.State_
    if (currentState == State::Far) return VALID; else return INVALID;
  }

  return UNKNOWN; // TO DO: === will never be called but would be useful for sensors which do not exist. ===
}

// ===== Process Consumer-eventIDs =====
void pceCallback(uint16_t index) {
  // Invoked when an event is consumed; drive pins as needed
  // from index of all events.
  dP("\neventid callback: index="); dP((uint16_t)index);
}
    
void setThresholds() {
  threshold_t currentThreshold;

  for (uint8_t i = 0; i < NUM_SENSOR; i++) {
    // Check if this sensor is connected.
    int connected = sensor.read(i);

    for (uint8_t j = 0; j < NUM_THRESHOLD; j++) {
      // Set currentState for all thresholds for this sensor.
      if (connected == -1) {
        currentThreshold.currentState = State::Unknown;
      } else {
        currentThreshold.currentState = State::Far;
      }

      currentThreshold.sensor = i;
      currentThreshold.threshold = j;
      currentThreshold.valueNear = NODECONFIG.read(EEADDR(sensor[i].threshold[j].thresholdNear));
      currentThreshold.valueFar = NODECONFIG.read(EEADDR(sensor[i].threshold[j].thresholdFar));
      currentThreshold.eventIndexNear = (i * NUM_THRESHOLD * 2) + (j * 2);
      currentThreshold.eventIndexFar = (i * NUM_THRESHOLD * 2) + (j * 2) + 1;

      threshold.set(currentThreshold);
    }
  }

  threshold.print();
}

void produceFromInputs() {
  // Called from loop(), this looks at changes in input pins
  // and decides which events to send with pce.produce(i).

  // Implement a non blocking delay and read the range from all sensors when the delay expires.
  static long nextSensorRead = 0;
  if (millis() < nextSensorRead) return;
  // The timeout has expired.
  nextSensorRead = millis() + 10;

  // Read each of the sensors in turn.
  for (uint8_t currentSensor = 0; currentSensor < NUM_SENSOR; currentSensor++) {
    int range = sensor.read(currentSensor);

    if (range == -1) {
      // This sensor is not connected.
      continue;
    }

    // Has the range passed a threshold for this sensor?
    // Check all thresholds for this sensor.
    for (uint8_t currentThreshold = 0; currentThreshold < NUM_THRESHOLD; currentThreshold++) {
      int eventToSend = threshold.check(currentSensor, currentThreshold, range);
      if (eventToSend != -1) OpenLcb.produce(eventToSend);
    }
  }
}

void userSoftReset() {}
void userHardReset() {}
    
#include "OpenLCBMid.h"   // Essential - do not move or delete

// Callback from a Configuration write
// Use this to detect changes in the node's configuration
// This may be useful to take immediate action on a change.
void userConfigWritten(uint32_t address, uint16_t length, uint16_t func)
{
  dPS("\nuserConfigWritten: Addr: ", (uint32_t)address); 
  dPS("  Len: ", (uint16_t)length); 
  dPS("  Func: ", (uint8_t)func);

  // Reset the thresholds.
  setThresholds();
}
    
// ==== Setup does initial configuration ======================
void setup()
{   
  #ifdef DEBUG
    uint32_t stimer = millis();
    Serial.begin(115200);
    while (!Serial && (millis() - stimer < 5000));   // wait for 5 secs for USB/serial connection to be established
    dP("\n Pico-8ServoWifiGC");
    delay(1000);
  #endif

  NodeID nodeid(NODE_ADDRESS);       // this node's nodeid
  Olcb_init(nodeid, RESET_TO_FACTORY_DEFAULTS);

  // Initialise the ToF sensor(s).
  sensor.begin();
  
  // Initialise the thresholds.
  setThresholds();

  dP("\n initialization finished");

  dP("\n NUM_EVENT="); dP(NUM_EVENT);
}

// ==== Loop ==========================
void loop() {
  //MDNS.update();  // IS THIS NEEDED?
  bool activity = Olcb_process();
  static long nextdot = 0;
  if(millis()>nextdot) {
    nextdot = millis()+2000;
    //dP("\n.");
  }
  
  produceFromInputs();

}
