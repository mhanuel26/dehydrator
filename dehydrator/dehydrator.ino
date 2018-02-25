#include <avr/io.h>
#include <avr/interrupt.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <SHT1X.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <EEPROM.h>

#define EEADDR   0   // Start of EEPROM

#define DETECT 2     // zero cross detect Arduino I/O
#define GATE 9       // TRIAC gate Arduino I/O
#define PULSE 10      // trigger pulse width (counts)
#define RELAY 10     // RELAY ON/OFF  Arduino I/O
#define ONE_WIRE 4   // DS28b20 ONE WIRE  Arduino I/O

#define TC_OUT_MIN  40        // Minimum Value is lowest delay from Zero crossing - "97% AC Phase" Control.
#define TC_OUT_MAX  220       // Maximum Value is longest delay from Zero crossing - "3% AC Phase" Control.
#define TEMP_MIN    35
#define TEMP_MAX    75
#define ds_json_tag   "temperature" 
#define sht15_temp_tag  "temperature"
#define sht15_rh_tag    "humidity"
#define STATUS_TAG    "state"

#define VER   9

#define RH_READ_DELAY   1000
#define TEMP_READ_DELAY 800 //can only read digital temp sensor every ~750ms
#define PID_SAMPLE_TIME 900

struct configObj{
  int ver;
  int setpoint;
  boolean state;
};

// PID variables
double aggKp=5, aggKi=2.5, aggKd=1.5;
double consKp=2, consKi=0.5, consKd=0.5;

//variables for storing values
double tempC = 0;
//float tempF = 0;
float humidity = 0;
bool notify = false;
unsigned long lastTempUpdate = millis();
unsigned long lastRHUpdate = millis();
unsigned long notify_time = 10000;      // 10 seconds -  this might be made configurable
unsigned long notify_keeper = millis();
unsigned long sample = 2000;            // take a sample every 2 seconds

volatile int tc_value = TC_OUT_MIN;
volatile boolean heater_en = false;    // Initially OFF   
boolean unit_on_off = false;           // Initially OFF   
boolean toggle = false;
boolean valid  = false;
boolean one_time = false;
boolean send_ack = false;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

double temperature, temperatureF;
double Setpoint;
double Output, lastOutput;

unsigned long led_keeper = millis();

configObj cnfData;
int eeAddress = EEADDR;                //EEPROM address to start reading from

PID dehydratorPID(&temperature, &Output, &Setpoint, aggKp, aggKi, aggKd, REVERSE);

StaticJsonBuffer<300> jsonBuffer;
JsonObject& root = jsonBuffer.createObject();
JsonObject& reported = jsonBuffer.createObject();
JsonArray& sht15_array = root.createNestedArray("sht15");
JsonObject& sht15_json = jsonBuffer.createObject();

OneWire  ds(ONE_WIRE);  //  (a 4.7K resistor is necessary)
DallasTemperature temperatureSensors(&ds);

//Create an instance of the SHT1X sensor
SHT1x sht15(A4, A5);    //Data, SCK

void setup(){
  delay(500);
  EEPROM.get( eeAddress, cnfData );
  if(cnfData.ver != VER){
    cnfData.ver = VER;
    cnfData.state = false;
    cnfData.setpoint = 0;
  }
  
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  
  // set up pins
  pinMode(DETECT, INPUT);     //zero cross detect
//  digitalWrite(DETECT, HIGH); //enable pull-up resistor
  pinMode(GATE, OUTPUT);      //TRIAC gate control
  pinMode(11, OUTPUT);      
  pinMode(RELAY, OUTPUT);     

  
  if(cnfData.state == true){
    Setpoint = 1.0*cnfData.setpoint;
    digitalWrite(RELAY,LOW);   // Initially ON 
    unit_on_off = true;
    reported[STATUS_TAG] = "on";
    root["setpoint"] = double_with_n_digits(Setpoint, 1);
  }else{
    digitalWrite(RELAY,HIGH);   // Initially OFF   
    unit_on_off = false;
    reported[STATUS_TAG] = "off";
    root["setpoint"] = 0;
  }
  

  pinMode(3, OUTPUT);      // DEBUG LED
  digitalWrite(3, HIGH);  
  
  cli();           // disable all interrupts
  // set up Timer1 
  //(see ATMEGA 328 data sheet pg 134 for more details)
  TCCR1A = 0;    //timer control registers set for
  TCCR1B = 0;    //normal operation, timer disabled

  // set up zero crossing interrupt
  attachInterrupt(0,zeroCrossingInterrupt, RISING);    
    //IRQ0 is pin 2. Call zeroCrossingInterrupt 
    //on rising signal
  sei();             // enable all interrupts
  digitalWrite(GATE,LOW);      // set TRIAC gate to low
  temperatureSensors.begin();
  temperatureSensors.requestTemperatures();
  temperatureSensors.setWaitForConversion(false);
  while (!handle_dallas(0)) {}       // wait until temp sensor updated
  // init the sht15 json
  sht15_json["error"] = "noerr";
  sht15_json[sht15_temp_tag] = 0;
  sht15_json[sht15_rh_tag] = 0;
  sht15_array.add(sht15_json);
  
  dehydratorPID.SetSampleTime(PID_SAMPLE_TIME);
  // +1 something we will shutdown heater but leave air fan on
  dehydratorPID.SetOutputLimits(TC_OUT_MIN, TC_OUT_MAX+1);   
  dehydratorPID.SetMode(AUTOMATIC);
}  

//Interrupt Service Routines

void zeroCrossingInterrupt(){  // zero cross detect
  
  cli();
  if(heater_en){
    OCR1A = tc_value;
    // turn on CTC mode:
    TCCR1B |= (1 << WGM12);
    // Set CS10 for 256 prescaler
    TCCR1B |= (1 << CS12);
    TIMSK1 = 0x03;    //enable comparator A and overflow interrupts
    TCNT1 = 0;   //reset timer - count from zero
  }
  sei();
  
  toggle = toggle ^ 1;
  digitalWrite(11, toggle);
}

ISR(TIMER1_COMPA_vect){     //comparator match
  digitalWrite(GATE,HIGH);  //set TRIAC gate to high
  TCNT1 = 65536-PULSE;      //trigger pulse width
}

ISR(TIMER1_OVF_vect){     //timer1 overflow
  digitalWrite(GATE,LOW); //turn off TRIAC gate
  TCCR1B = 0x00;          //disable timer stopd unintended triggers
}

void loop(){
  // update DS18B20 Sensor
  handle_dallas(0);
  // update sht15 temperature and humidity sensor
  handle_sht15();
  // process command from esp8266 WiFi Bridge
  if (stringComplete) {
    StaticJsonBuffer<300> jsonBuffer;
    JsonObject& received = jsonBuffer.parseObject(inputString);

    boolean saveCfg = false;
    // Setpoint (Temperature in Integer value)
    int sp = received["setpoint"];
    if((sp >= TEMP_MIN) && (sp <= TEMP_MAX)){
      Setpoint = sp*1.0;
      cnfData.setpoint = (int)Setpoint;
      saveCfg = true;
      valid = true;
      root["setpoint"] = double_with_n_digits(Setpoint, 1);
    }else if(sp == 0){
      // do nothing workaround
    }else{
      // turn off dehydrator if other value
      digitalWrite(RELAY,HIGH);     // set RELAY to OFF state
      heater_en = false;
      valid = false;
    }
    // ON/OFF State
    
    const char* state = received["state"];
    if(!strcmp(state, "on")){
//      send_ack = true;
      if(valid){
        unit_on_off = true;
        digitalWrite(RELAY,LOW);      // set RELAY to ON state
        heater_en = true;
        // remember we are ON state
        cnfData.state = true;         // next reboot will be ON
        saveCfg = true;
        reported[STATUS_TAG] = "on";
      }else{
        reported[STATUS_TAG] = "off";
      }
    }else if(!strcmp(state, "off")){
//      send_ack = true;
      unit_on_off = false;
      digitalWrite(RELAY,HIGH);     // set RELAY to OFF state
      heater_en = false;
      cnfData.state = false;        // next reboot will be OFF at start
      saveCfg = true;
      reported[STATUS_TAG] = "off";
    }else{
      // do nothing
    }
    if(saveCfg == true){
      eeAddress = EEADDR; 
      EEPROM.put(eeAddress, cnfData);
    }
    // Notifications
    const char* notify_cfg = received["notify"];
    if(!strcmp(notify_cfg, "on")){
      send_ack = true;
      notify = true;
      notify_keeper = millis();
    }else if(!strcmp(notify_cfg, "off")){
      notify = false;
    }else{
      // do nothing
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }

  if(unit_on_off){
    double gap = abs(Setpoint-temperature);
    if(gap <= 2.0){
      dehydratorPID.SetTunings(consKp, consKi, consKd);
    }else{
      dehydratorPID.SetTunings(aggKp, aggKi, aggKd);
    }
    dehydratorPID.Compute();
    // ensure the Output of PID library is on allowed range
    if((Output >= TC_OUT_MIN) && (Output <= TC_OUT_MAX)){
      root["output"] = double_with_n_digits(Output, 1);
      root["setpoint"] = double_with_n_digits(Setpoint, 1);
      // Turn On heater when in AC phase control range
      heater_en = true;
      tc_value = Output;
      one_time = true;
    }else if(Output > TC_OUT_MAX){
        heater_en = false;
        if(one_time){
          cli();
          TCCR1A = 0;    //timer control registers set for
          TCCR1B = 0;    //normal operation, timer disabled
          digitalWrite(GATE,LOW);      // set TRIAC gate to low
          one_time = false;
          digitalWrite(RELAY,HIGH);     // set RELAY to OFF state
          delay(200);
          digitalWrite(RELAY,LOW);     // set RELAY to OFF state
          sei();
        }
        root["output"] = double_with_n_digits(Output, 1);
        root["setpoint"] = double_with_n_digits(Setpoint, 1);
    }else{
      root["output"] = "error";
    }
  }
  
  if(notify){
    if((millis() - notify_keeper > notify_time) || send_ack){
      root["reported"] = reported;
      // send the json data
      root.printTo(Serial);
      // neccesary to process the message frame
      Serial.println();
      notify_keeper = millis();
      send_ack = false;
    }
  }

  if(millis() - led_keeper > 500){
    digitalWrite(3, !digitalRead(3));
    led_keeper = millis();
  }
  
}

bool handle_dallas(int index){
  if(millis() - lastTempUpdate  > TEMP_READ_DELAY){
    temperature = temperatureSensors.getTempCByIndex(index); //get temp reading
    temperatureF = temperature * 1.8 + 32.0;
    reported[ds_json_tag] = double_with_n_digits(temperature, 1);
    lastTempUpdate = millis();
    temperatureSensors.setWaitForConversion(false);
    temperatureSensors.requestTemperatures(); //request reading for next time
    return true;
  }
  return false;
}

void handle_sht15(void){
  if(millis() - lastRHUpdate  > RH_READ_DELAY){
    // now get the sht15 sensor data
    readSensor();
    lastRHUpdate = millis();
    if(tempC < -40.0){
      sht15_json["error"] = "comms";
    }else{
      sht15_json["error"] = "noerr";
      sht15_json[sht15_temp_tag] = double_with_n_digits(tempC, 1);
      sht15_json[sht15_rh_tag] = double_with_n_digits(humidity, 1);
    }
    sht15_array.set(0, sht15_json);
  }
}

void readSensor()
{
  // Read values from the sensor
  tempC = sht15.readTemperatureC();
//  tempF = sht15.readTemperatureF();
  humidity = sht15.readHumidity();  
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

