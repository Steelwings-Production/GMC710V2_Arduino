//###############################################################################
//Version 0.5
//Licence
//Developer Frank Brunner
//###############################################################################

#include <Adafruit_MCP23X17.h>

Adafruit_MCP23X17 mcp_r; //MCP Rotary Encoder U1
Adafruit_MCP23X17 mcp_p; //MCP Pusch Buttonsb U2

bool debugOn = true;
String debugString;
String serialRead;

unsigned long setTimer;
unsigned long setTimerEncoderInc;
unsigned long setTimerEncoderDec;
unsigned long setHeartBeatTimer;
unsigned long setTimerDetachVS;
unsigned long setTimerDetachHDG;
unsigned long setTimerDetachALT;
unsigned long setTimerDetachCRS01;
unsigned long setTimerDetachCRS02;
unsigned long setReceivingHeartBeatTimer;

unsigned long setTimerSecondloop;


//Module deklaration
const String moduleType = "GMC710";
bool connectionEstablished = false;
String receiveVal;

//Status variables for Rotary Encoders
bool HDG_selector_interrupt_set = false;
bool CRS01_selector_interrupt_set = false;
bool CRS02_selector_interrupt_set = false;
bool ALT_selector_interrupt_set = false;
bool VS_selector_interrupt_set = false;

//Declaration for HDG Selector Variables
const uint8_t HDG_Selector_PinA_mcp_p = 13;
const uint8_t HDG_Selector_PinB_mcp_p = 14;
const uint8_t HDG_Selector_Push_mcp_p = 15;
const uint8_t HDG_Selector_Rotary_Joystick_Button_UP = 1;
const uint8_t HDG_Selector_Rotary_Joystick_Button_DOWN = 2;
const uint8_t HDG_Selector_Push_Joystick_Button = 3;
bool HDG_Selector_PushButton_State = false;

//Declaration for CRS1 Variables
const uint8_t CRS01_Selector_PinA_mcp_p = 8;
const uint8_t CRS01_Selector_PinB_mcp_p = 9;
const uint8_t CRS01_Selector_Push_mcp_p = 10;
const uint8_t CRS01_Selector_Rotary_Joystick_Button_UP = 4;
const uint8_t CRS01_Selector_Rotary_Joystick_Button_DOWN = 5;
const uint8_t CRS01_Selector_Push_Joystick_Button = 6;
bool CRS01_Selector_PushButton_State = false;

//Declaration for CRS2 Variables
const uint8_t CRS02_Selector_PinA_mcp_r = 9;
const uint8_t CRS02_Selector_PinB_mcp_r = 8;
const uint8_t CRS02_Selector_Push_mcp_r = 5;
const uint8_t CRS02_Selector_Rotary_Joystick_Button_UP = 7;
const uint8_t CRS02_Selector_Rotary_Joystick_Button_DOWN = 8;
const uint8_t CRS02_Selector_Push_Joystick_Button = 9;
bool CRS02_Selector_PushButton_State = false;

//Declaration for Altimeter Selector Variables
const uint8_t ALT_Selector_PinA_mcp_r = 13;
const uint8_t ALT_Selector_PinB_mcp_r = 14;
const uint8_t ALT_Selector_Rotary_Joystick_Button_UP = 10;
const uint8_t ALT_Selector_Rotary_Joystick_Button_DOWN = 11;
const uint8_t ALT_Selector_Push_Joystick_Button = 12;

//Declaration for VerticalSpeed Selector Variables
const uint8_t VS_Selector_PinA_mcp_r = 4;
const uint8_t VS_Selector_PinB_mcp_r = 3;
const uint8_t VS_Selector_Rotary_Joystick_Button_UP = 13;
const uint8_t VS_Selector_Rotary_Joystick_Button_DOWN = 14;
const uint8_t VS_Selector_Push_Joystick_Button = 15;

//Button Pin declaration
//ButtonState means the functio is enganged (Autopilot on)
const uint8_t HDG_PushButton_Pin = 0; //U2
uint8_t HDG_PushButton_Led = 1; //U2
bool HDG_PushButton_Pressed = false; //U2
bool HDG_PushButton_State = false; //U2
const uint8_t APR_PushButton_Pin = 2; //U2
const uint8_t APR_PushButton_Led = 3; //U2
bool APR_PushButton_Pressed = false; //U2
bool APR_PushButton_State = false; //U2
const uint8_t NAV_PushButton_Pin = 7; //U2
const uint8_t NAV_PushButton_Led = 6; //U2
bool NAV_PushButton_Pressed = false; //U2
bool NAV_PushButton_State = false; //U2
const uint8_t BC_PushButton_Pin = 12; //U2
const uint8_t BC_PushButton_Led = 11; //U2
bool BC_PushButton_Pressed = false; //U2
bool BC_PushButton_State = false; //U2
const uint8_t FD_PushButton_Pin = 5; //U2
const uint8_t FD_PushButton_Joystik = 27;//U2
bool FD_PushButton_Pressed = false;//U2
bool FD_PushButton_State = false;//U2

const uint8_t ALT_PushButton_Led = 0; //U1
bool ALT_PushButton_Pressed = false;//U1
bool ALT_PushButton_State = false;//U1
const uint8_t VS_PushButton_Pin = 2; //U1
const uint8_t VS_PushButton_Led = 1;//U1
bool VS_PushButton_Pressed = false; //U1
bool VS_PushButton_State = false;  //U1
const uint8_t VNAV_PushButton_Pin = 12;  //U1
const uint8_t VNAV_PushButton_Led = 1;  //U1
bool VNAV_PushButton_Pressed = false;  //U1
bool VNAV_PushButton_State = false;  //U1
const uint8_t FLC_PushButton_Pin = 6;  //U1
const uint8_t FLC_PushButton_Led = 7;  //U1
bool FLC_PushButton_Pressed = false;  //U1
bool FLC_PushButton_State = false;  //U1
const uint8_t SPD_PushButton_Pin = 10;//U1
const uint8_t SPD_PushButton_Joystik = 28;//U1
bool SPD_PushButton_Pressed = false;//U1
bool SPD_PushButton_State = false;//U1

//Arduino
const uint8_t ALT_PushButton_Pin = 23; //D
const uint8_t AP_PushButton_Pin = 10; //D
const uint8_t AP_PushButton_Led = 9; //D
bool AP_PushButton_Pressed = false; //D
bool AP_PushButton_State = false; //D
const uint8_t BANK_PushButton_Pin = 12; //D
const uint8_t BANK_PushButton_Led = 11; //D
bool BANK_PushButton_Pressed = false; //D
bool BANK_PushButton_State = false; //D
const uint8_t YD_PushButton_Pin = 8;  //Arduino
const uint8_t YD_PushButton_Led = 7;  //Arduino
bool YD_PushButton_Pressed = false;   //Arduino
bool YD_PushButton_State = false;   //Arduino
const uint8_t XFR_PushButton_Pin = 18;  //Arduino
const uint8_t XFR_PushButton_Led_l = 13;  //Arduino
const uint8_t XFR_PushButton_Led_r = 19;  //Arduino
bool XFR_PushButton_Pressed = false;   //Arduino
bool XFR_PushButton_State = false;   //Arduino

const uint8_t noLed = 100; // Set the falue 100 the function will trigger no led
bool ButtonPressed = false;


//mcp Select PIN on the Arduino
const uint8_t CS_PIN_mcp_r = 2;
const uint8_t CS_PIN_mcp_p = 3;
const uint8_t InteruptPin_mcp_r = 9;
const uint8_t InteruptPin_mcp_p = 10;

//Pin Mode of Arduino
int pinArduinoInput[] = {8, 10, 12, 18, 23};
int pinArduinoLed[] = {7, 9, 11, 13, 19};
//Pin Mode MCP_r
int pinMCPRInput[] = {2, 3, 4, 5, 6, 8, 9, 10, 12, 13, 14};
int pinMCPRLed[] = {0, 1, 7, 11};
//Pin Mode MCP_p
int pinMCPPInput[] = {0, 2, 5, 7, 8, 9, 10, 12, 13, 14, 15};
int pinMCPPLed[] = {1, 3, 6, 11};

bool detatchedVS = false;
bool detatchedHDG = false;
bool detatchedALT = false;
bool detatchedCRS01 = false;
bool detatchedCRS02 = false;

String receivingValuesON[] = {"HDG1", "APR1", "NAV1", "BC1", "YD1", "BK1", "ALT1","VS1","FLC1","AP1" };
String receivingValuesOFF[] = {"HDG0", "APR0", "NAV0", "BC0", "YD0", "BK0", "ALT0","VS0","FLC0","AP0" };
uint8_t ledPins[] = {HDG_PushButton_Led, APR_PushButton_Led, NAV_PushButton_Led, BC_PushButton_Led, YD_PushButton_Led, BANK_PushButton_Led, ALT_PushButton_Led,VS_PushButton_Led,FLC_PushButton_Led,AP_PushButton_Led };
String mcps[] = {"mcp_p", "mcp_p", "mcp_p", "mcp_p", "arduino", "arduino", "mcp_r", "mcp_r","mcp_r","arduino" };

void setup() {
  //for debugging start Serial
  Serial.begin(115200);
  Serial.setTimeout(10);
  delay(2500);

  // uncomment appropriate mcps.begin
  mcp_r.begin_SPI(CS_PIN_mcp_r);
  mcp_p.begin_SPI(CS_PIN_mcp_p);
  // End of Starting MCP's


  //Arduino Pin Mode
  for (int i = 0; i < (sizeof(pinArduinoInput) / sizeof(pinArduinoInput[0])) ; ++i) {
    pinMode(pinArduinoInput[i], INPUT_PULLUP);
    debug(debugOn, "Arduino INPUPT PIN Enabled:" + String(pinArduinoInput[i]));
  }
  for (int i = 0; i < (sizeof(pinArduinoLed) / sizeof(pinArduinoLed[0])) ; ++i) {
    pinMode(pinArduinoLed[i], OUTPUT);
    digitalWrite(pinArduinoLed[i], LOW);
    debug(debugOn, "Arduino enable LED on:" + String(pinArduinoLed[i]));
  }
  digitalWrite(13, HIGH);

  //Interrupt Setup MCP
  mcp_r.setupInterrupts(true, true, LOW);
  //SET PIN on MCP_Rotary_Encoder
  for (byte i = 0; i < (sizeof(pinMCPRInput) / sizeof(pinMCPRInput[0])) ; ++i) {
    mcp_r.pinMode(pinMCPRInput[i], INPUT_PULLUP);
    mcp_r.setupInterruptPin(pinMCPRInput[i], LOW);
    debug(debugOn, "Enable Interupt mcp_r:" + String(pinMCPRInput[i]));
  }
  for (byte i = 0; i < (sizeof(pinMCPRLed) / sizeof(pinMCPRLed[0])) ; ++i) {
    mcp_r.pinMode(pinMCPRLed[i], OUTPUT);
    mcp_r.digitalWrite(pinMCPRLed[i], LOW);
    debug(debugOn, "Enable Output mcp_r:" + String(pinMCPRLed[i]));
  }

  mcp_p.setupInterrupts(true, true, LOW);
  //SET PIN on MCP_Rotary_Encoder
  for (byte i = 0; i < (sizeof(pinMCPPInput) / sizeof(pinMCPPInput[0])) ; ++i) {
    mcp_p.pinMode(pinMCPPInput[i], INPUT_PULLUP);
    mcp_p.setupInterruptPin(pinMCPPInput[i], LOW);
    debug(debugOn, "Enable Interupt mcp_p:" + String(pinMCPPInput[i]));
  }
  for (byte i = 0; i < (sizeof(pinMCPPLed) / sizeof(pinMCPPLed[0])) ; ++i) {
    mcp_p.pinMode(pinMCPPLed[i], OUTPUT);
    mcp_p.digitalWrite(pinMCPPLed[i], LOW);
    debug(debugOn, "Enable Output mcp_p:" + String(pinMCPPLed[i]));
  }


  //Timers initialitation
  setTimer = millis();
  setTimerEncoderInc = millis();
  setTimerEncoderDec = millis();
  setHeartBeatTimer = millis();
  setTimerSecondloop = millis();


  //connectionEstablished = true;
  //END PIN Definition
  //mcp_p.detachInterrupt();
}
void loop() {
  //Section Establish Connection

  if (!connectionEstablished) {
    connectionEstablished = connectPC(connectionEstablished, moduleType);
  } else {
    //Section One it is reserved only for Encoders
    VS_selector_interrupt_set = interruptHandlerVS(VS_Selector_PinA_mcp_r, VS_Selector_PinB_mcp_r, VS_selector_interrupt_set, mcp_r);
    HDG_selector_interrupt_set = interruptHandlerHDG(HDG_Selector_PinA_mcp_p, HDG_Selector_PinB_mcp_p, HDG_selector_interrupt_set, mcp_p);
    CRS01_selector_interrupt_set = interruptHandlerCRS01(CRS01_Selector_PinA_mcp_p, CRS01_Selector_PinB_mcp_p, CRS01_selector_interrupt_set, mcp_p);
    CRS02_selector_interrupt_set = interruptHandlerCRS02(CRS02_Selector_PinA_mcp_r, CRS02_Selector_PinB_mcp_r, CRS02_selector_interrupt_set, mcp_r);
    ALT_selector_interrupt_set = interruptHandlerALT(ALT_Selector_PinA_mcp_r, ALT_Selector_PinB_mcp_r, ALT_selector_interrupt_set, mcp_r);

    encodersHandler(mcp_p );
    encodersHandler01(mcp_r);
    //----Section One End----
    //every Second sending a heartbeat to PC


    //Section TWO everything that is not that time criticality as the Encoder
    //The loop is triggert with to timer
    if (millis() - setTimer > 200 && millis() - setTimerSecondloop > 100) {

      sendHeartBeat();

      detatchedHDG = encoderDetacher(detatchedHDG, setTimerDetachHDG, HDG_Selector_PinA_mcp_p, HDG_Selector_PinB_mcp_p, 1000 , mcp_p);
      detatchedVS = encoderDetacher(detatchedVS, setTimerDetachVS, VS_Selector_PinA_mcp_r, VS_Selector_PinB_mcp_r, 1000 , mcp_r);
      detatchedALT = encoderDetacher(detatchedALT , setTimerDetachALT, ALT_Selector_PinA_mcp_r, ALT_Selector_PinB_mcp_r, 1000 , mcp_r);
      detatchedCRS01 = encoderDetacher(detatchedCRS01, setTimerDetachCRS01, CRS01_Selector_PinA_mcp_p, CRS01_Selector_PinB_mcp_p, 1000 , mcp_p);
      detatchedCRS02 = encoderDetacher(detatchedCRS02, setTimerDetachCRS02, CRS02_Selector_PinA_mcp_r, CRS02_Selector_PinB_mcp_r, 1000 , mcp_r);

      //Section of Pusch Buttons from the Rotary Encoder HDG,CRS01,CRS02
      HDG_Selector_PushButton_State = rotaryPushButton(HDG_Selector_Push_mcp_p, HDG_Selector_PushButton_State, "HEADING_BUG_SET", mcp_p);

      CRS01_Selector_PushButton_State = rotaryPushButton(CRS01_Selector_Push_mcp_p, CRS01_Selector_PushButton_State, "VOR1_SET", mcp_p);

      CRS02_Selector_PushButton_State = rotaryPushButton(CRS02_Selector_Push_mcp_r, CRS02_Selector_PushButton_State, "VOR2_SET", mcp_r);
      //----Section End----

      //Section of all Push Buttons
      HDG_PushButton_Pressed = pushButton(HDG_PushButton_Pin, HDG_PushButton_Pressed, HDG_PushButton_State, "AP_HDG_HOLD", mcp_p);

      APR_PushButton_Pressed = pushButton(APR_PushButton_Pin, APR_PushButton_Pressed, APR_PushButton_State, "AP_APR_HOLD", mcp_p);

      NAV_PushButton_Pressed = pushButton(NAV_PushButton_Pin, NAV_PushButton_Pressed, NAV_PushButton_State, "AP_NAV1_HOLD", mcp_p);

      BC_PushButton_Pressed = pushButton(BC_PushButton_Pin, BC_PushButton_Pressed, BC_PushButton_State, "AP_BC_HOLD", mcp_p);

      VS_PushButton_Pressed = pushButton(VS_PushButton_Pin, VS_PushButton_Pressed, VS_PushButton_State, "AP_VS_HOLD", mcp_r);

      //VNAV_PushButton_Pressed=pushButtonArduino(VNAV_PushButton_Pin, VNAV_PushButton_Pressed, VNAV_PushButton_State, "AP_ALT_HOLDE");

      FLC_PushButton_Pressed = pushButton(FLC_PushButton_Pin, FLC_PushButton_Pressed, FLC_PushButton_State, "AP_FLIGHT_LEVEL_CHANGE", mcp_r);

      FD_PushButton_Pressed = pushButton(FD_PushButton_Pin, FD_PushButton_Pressed, FD_PushButton_State, "TOGGLE_FLIGHT_DIRECTOR", mcp_p);

      SPD_PushButton_Pressed = pushButton(SPD_PushButton_Pin, SPD_PushButton_Pressed, SPD_PushButton_State, "AP_MANAGED_SPEED_IN_MACH_TOGGLE", mcp_r);

      //----Section End----

      //Section of all Push Buttons the are Connected direct on the Arduino
      ALT_PushButton_Pressed = pushButtonArduino(ALT_PushButton_Pin, ALT_PushButton_Pressed, ALT_PushButton_State, "AP_ALT_HOLD");

      AP_PushButton_Pressed = pushButtonArduino(AP_PushButton_Pin, AP_PushButton_Pressed, AP_PushButton_State, "AP_MASTER");

      YD_PushButton_Pressed = pushButtonArduino(YD_PushButton_Pin, YD_PushButton_Pressed, YD_PushButton_State, "YAW_DAMPER_TOGGLE");

      BANK_PushButton_Pressed = pushButtonArduino(BANK_PushButton_Pin, BANK_PushButton_Pressed, BANK_PushButton_State, "AP_BANK_HOLD");

      XFR_PushButton_Pressed = xfrPushButtonArduino(XFR_PushButton_Pin, XFR_PushButton_Pressed, XFR_PushButton_State, "AP_XFR", XFR_PushButton_Led_l, XFR_PushButton_Led_r);

      receiveVal = Serial.readString();
      receivingValues(receiveVal);
      //if no HB is alive from the Serial Interface the variable will be set on false-> trigger init phase
      connectionEstablished = heartBeat(receiveVal);

      setTimerSecondloop = millis();
      Serial.flush();
    }
  }
}
//----Section Two End----
bool connectPC(bool _connectionEstablished, String _moduleType) {
  Serial.println(_moduleType);
  delay(200);
  _connectionEstablished = handShake(_connectionEstablished);
  setReceivingHeartBeatTimer = millis();
  return _connectionEstablished;
}
bool handShake(bool _connectionEstablished ) {
  String _receiveVal = Serial.readString();
  if (_receiveVal.indexOf("handshake") != -1) {
    _connectionEstablished = true;
  }
  return _connectionEstablished;
}
bool heartBeat(String _receiveVal) {
  if (_receiveVal.indexOf("HEARTBEAT") != -1) {
    setReceivingHeartBeatTimer = millis();
    return true;
  } else {
    if ((millis() - setReceivingHeartBeatTimer) > 10000) {
      return false;
    } else {
      return true;
    }
  }
}
void sendHeartBeat() {
  if ( millis() - setHeartBeatTimer > 1000) {
    Serial.println("HEARTBEAT");
    setHeartBeatTimer = millis();
  }
}
bool checkIfDisconnected(bool _heardBeat) {
  uint8_t waitTimeUntilStartReconnecting = 5000;
  //wenn der HB vorhanden ist
  if (_heardBeat) {
    //Der Counter wird zurückgesetzt
    setHeartBeatTimer = millis();
    return true;
  } else {
    //es wird geprüft wie lange schon kein lebenzeichen mehr gesendetn wurde
    if (millis() - setHeartBeatTimer >  waitTimeUntilStartReconnecting) {
      return false;
    }
  }
}
void receivingValues(String receiveVal) {
  //Read the incomming Datat
  for (int i = 0; i < (sizeof(receivingValuesON) / sizeof(receivingValuesON[0])) ; ++i) {

    if (receiveVal.indexOf(receivingValuesON[i]) != -1) {
      if (mcps[i] == "mcp_p") {
        ledOn(ledPins[i], mcp_p);
      }
      if (mcps[i] == "mcp_r") {
        ledOn(ledPins[i], mcp_r);
      }
      if (mcps[i] == "arduino") {
        ledOnArduino(ledPins[i]);
      }
    }
    if (receiveVal.indexOf(receivingValuesOFF[i]) != -1) {
      if (mcps[i] == "mcp_p") {
        ledOff(ledPins[i], mcp_p);
      }
      if (mcps[i] == "mcp_r") {
        ledOff(ledPins[i], mcp_r);
      }
      if (mcps[i] == "arduino") {
        ledOffArduino(ledPins[i]);
      }
    }
  }
}
bool rotaryPushButton(uint8_t _pin, bool _buttonPressed, String _action, Adafruit_MCP23X17 mcp) {
  if (!mcp.digitalRead(_pin) == 1 and !_buttonPressed) {
    Serial.println(_action);
    _buttonPressed = true;
  }
  if (!mcp.digitalRead(_pin) == 0 and _buttonPressed) {
    _buttonPressed = false;
  }
  return _buttonPressed;
}
bool pushButtonArduino(uint8_t _pin, bool _buttonPressed, bool _buttonState,  String _action) {
  if (!digitalRead(_pin) == 1 and !_buttonPressed and !_buttonState) {
    Serial.println(_action);
    _buttonPressed = true;
  }
  if (!digitalRead(_pin) == 1 and !_buttonPressed and _buttonState) {
    Serial.println(_action);
    _buttonPressed = true;
  }
  if (!digitalRead(_pin) == 0 and _buttonPressed) {
    _buttonPressed = false;
  }
  return _buttonPressed;
}
bool xfrPushButtonArduino(uint8_t _pin, bool _buttonPressed, bool _buttonState,  String _action, uint8_t ledPinL, uint8_t ledPinR) {
  if (!digitalRead(_pin) == 1 and !_buttonPressed and !_buttonState) {
    Serial.println(_action);
    ledOnArduino(ledPinR);
    ledOffArduino(ledPinL);
    _buttonPressed = true;
    XFR_PushButton_State = true;
  }
  if (!digitalRead(_pin) == 1 and !_buttonPressed and _buttonState) {
    Serial.println(_action);
    ledOnArduino(ledPinL);
    ledOffArduino(ledPinR);
    _buttonPressed = true;
    XFR_PushButton_State = false;
  }
  if (!digitalRead(_pin) == 0 and _buttonPressed) {
    _buttonPressed = false;
  }
  return _buttonPressed;
}
bool pushButton(uint8_t _pin, bool _buttonPressed, bool _buttonState,  String _action, Adafruit_MCP23X17 mcp) {
  if (!mcp.digitalRead(_pin) == 1 and !_buttonPressed and !_buttonState) {
    Serial.println(_action);
    _buttonPressed = true;
  }
  if (!mcp.digitalRead(_pin) == 1 and !_buttonPressed and _buttonState) {
    Serial.println(_action);
    _buttonPressed = true;
  }
  if (!mcp.digitalRead(_pin) == 0 and _buttonPressed) {
    _buttonPressed = false;
  }
  return _buttonPressed;
}
bool interruptHandler(uint8_t selectorPinA, uint8_t selectorPinB, bool selector_interrupt, Adafruit_MCP23X17 mcp) {
  if (!selector_interrupt)
  {
    if (mcp.digitalRead(selectorPinA) == 1 && mcp.digitalRead(selectorPinB) == 1 ) {
      selector_interrupt = true;

    }
  }
  return selector_interrupt;
}
bool interruptHandlerVS(uint8_t selectorPinA, uint8_t selectorPinB, bool selector_interrupt, Adafruit_MCP23X17 mcp) {
  if (!selector_interrupt)
  {
    if (mcp.digitalRead(selectorPinA) == 1 && mcp.digitalRead(selectorPinB) == 1 ) {
      selector_interrupt = true;
      setTimerDetachVS = 0;
      if (detatchedVS) {
        mcp_r.setupInterruptPin(VS_Selector_PinA_mcp_r);
        mcp_r.setupInterruptPin(VS_Selector_PinB_mcp_r);
        detatchedVS = false;
      }

    }
  }
  return selector_interrupt;
}
bool interruptHandlerHDG(uint8_t _PinA, uint8_t _PinB, bool _selector_interrupt, Adafruit_MCP23X17 _mcp) {
  if (!_selector_interrupt)
  {
    if (_mcp.digitalRead(_PinA) == 1 && _mcp.digitalRead(_PinB) == 1 ) {
      _selector_interrupt = true;
      setTimerDetachHDG = 0;
      if (detatchedHDG) {
        _mcp.setupInterruptPin(_PinA);
        _mcp.setupInterruptPin(_PinB);
        detatchedHDG = false;
      }
    }
  }
  return _selector_interrupt;
}
bool interruptHandlerALT(uint8_t _PinA, uint8_t _PinB, bool _selector_interrupt, Adafruit_MCP23X17 _mcp) {
  if (!_selector_interrupt)
  {
    if (_mcp.digitalRead(_PinA) == 1 && _mcp.digitalRead(_PinB) == 1 ) {
      _selector_interrupt = true;
      setTimerDetachALT = 0;
      if (detatchedALT) {
        _mcp.setupInterruptPin(_PinA);
        _mcp.setupInterruptPin(_PinB);
        detatchedALT = false;
      }
    }
  }
  return _selector_interrupt;
}
bool interruptHandlerCRS01(uint8_t _PinA, uint8_t _PinB, bool _selector_interrupt, Adafruit_MCP23X17 _mcp) {
  if (!_selector_interrupt)
  {
    if (_mcp.digitalRead(_PinA) == 1 && _mcp.digitalRead(_PinB) == 1 ) {
      _selector_interrupt = true;
      setTimerDetachCRS01 = 0;
      if (detatchedCRS01) {
        _mcp.setupInterruptPin(_PinA);
        _mcp.setupInterruptPin(_PinB);
        detatchedCRS01 = false;
      }
    }
  }
  return _selector_interrupt;
}
bool interruptHandlerCRS02(uint8_t _PinA, uint8_t _PinB, bool _selector_interrupt, Adafruit_MCP23X17 _mcp) {
  if (!_selector_interrupt)
  {
    if (_mcp.digitalRead(_PinA) == 1 && _mcp.digitalRead(_PinB) == 1 ) {
      _selector_interrupt = true;
      setTimerDetachCRS02 = 0;
      if (detatchedCRS02) {
        _mcp.setupInterruptPin(_PinA);
        _mcp.setupInterruptPin(_PinB);
        detatchedCRS02 = false;
      }
    }
  }
  return _selector_interrupt;
}
bool encoderDetacher(bool _detached, unsigned long _timer, uint8_t _pinA, uint8_t _pinB, uint8_t timeUntilDetach, Adafruit_MCP23X17 _mcp) {

  if (_timer != 0 && !_detached) {
    if ((millis() - _timer) > timeUntilDetach) {
      _mcp.disableInterruptPin(_pinA);
      _mcp.disableInterruptPin(_pinB);
      _detached = true;
      debug(debugOn, "Detached");
    }
  }
  return _detached;
}
void encodersHandler(Adafruit_MCP23X17 mcp) {
  int encoderDelay = 250;
  //the interrups will be reset
  uint8_t lastPin = mcp.getLastInterruptPin();
  if (lastPin == 255) {
    return ;
  }
  if (lastPin == HDG_Selector_PinA_mcp_p or lastPin == HDG_Selector_PinB_mcp_p) {
    if (HDG_selector_interrupt_set) {
      HDG_selector_interrupt_set = false;
      if (mcp.digitalRead(HDG_Selector_PinA_mcp_p) == 0) {
        if (millis() - setTimerEncoderDec > encoderDelay) {
          Serial.println("HEADING_BUG_DEC");
          setTimerEncoderInc = millis();
        }
      } else {
        if (millis() - setTimerEncoderInc > encoderDelay) {
          Serial.println("HEADING_BUG_INC");
          setTimerEncoderDec = millis();
        }
      }
      setTimerDetachHDG = millis();
      setTimer = millis();
    }


    return;
  }
  if (lastPin == CRS01_Selector_PinA_mcp_p or lastPin == CRS01_Selector_PinB_mcp_p) {
    if (CRS01_selector_interrupt_set) {
      CRS01_selector_interrupt_set = false;
      if (mcp.digitalRead(CRS01_Selector_PinA_mcp_p) == 0) {
        if (millis() - setTimerEncoderDec > 100) {
          Serial.println("VOR1_OBI_INC");
          setTimerEncoderInc = millis();
        }
      } else {
        if (millis() - setTimerEncoderInc > 100) {
          Serial.println("VOR1_OBI_DEC");
          setTimerEncoderDec = millis();
        }
      }
      setTimerDetachCRS01 = millis();
      setTimer = millis();
    }
    return;
  }
}
void encodersHandler01(Adafruit_MCP23X17 mcp) {
  int encoderDelay = 250;
  //the interrups will be reset
  uint8_t lastPin = mcp.getLastInterruptPin();

  if (lastPin == 255) {
    return;
  }
  if (lastPin == CRS02_Selector_PinA_mcp_r or lastPin == CRS02_Selector_PinB_mcp_r) {
    if (CRS02_selector_interrupt_set) {
      CRS02_selector_interrupt_set = false;
      if (mcp.digitalRead(CRS02_Selector_PinA_mcp_r) == 0) {
        if (millis() - setTimerEncoderDec > encoderDelay) {
          Serial.println("VOR2_OBI_INC");
          setTimerEncoderInc = millis();
        }
      } else {
        if (millis() - setTimerEncoderInc > encoderDelay) {
          Serial.println("VOR2_OBI_DEC");
          setTimerEncoderDec = millis();
        }
      }
      setTimerDetachCRS02 = millis();
      setTimer = millis();
    }

    return;
  }
  if (lastPin == ALT_Selector_PinA_mcp_r or lastPin == ALT_Selector_PinB_mcp_r) {
    if (ALT_selector_interrupt_set) {
      ALT_selector_interrupt_set = false;
      if (mcp.digitalRead(ALT_Selector_PinA_mcp_r) == 0) {
        if (millis() - setTimerEncoderDec > encoderDelay) {
          Serial.println("AP_ALT_VAR_INC");
          setTimerEncoderInc = millis();
        }
      } else {
        if (millis() - setTimerEncoderInc > encoderDelay) {
          Serial.println("AP_ALT_VAR_DEC");
          setTimerEncoderDec = millis();
        }
      }
      setTimerDetachALT = millis();
      setTimer = millis();
    }

    return;
  }
  if (lastPin == VS_Selector_PinA_mcp_r or lastPin == VS_Selector_PinB_mcp_r) {
    if (VS_selector_interrupt_set) {
      VS_selector_interrupt_set = false;
      if (mcp.digitalRead(VS_Selector_PinA_mcp_r) == 0) {
        if (millis() - setTimerEncoderDec > encoderDelay) {
          Serial.println("AP_VS_VAR_DEC");
          setTimerEncoderInc = millis();
        }
      } else {
        if (millis() - setTimerEncoderInc > encoderDelay) {
          Serial.println("AP_VS_VAR_INC");
          setTimerEncoderDec = millis();
        }
      }
      setTimerDetachVS = millis();
      setTimer = millis();
    }
    return;
  }


}
void ledOn(uint8_t ledPin, Adafruit_MCP23X17 mcp) {
  if (ledPin != 100) {
    mcp.digitalWrite(ledPin, HIGH);
  }
}
void ledOnArduino(uint8_t ledPin) {
  if (ledPin != 100) {
    digitalWrite(ledPin, HIGH);
  }
}
void ledOff(uint8_t _ledPin, Adafruit_MCP23X17 _mcp) {
  if (_ledPin != 100) {
    _mcp.digitalWrite(_ledPin, LOW);
  }
}
void ledOffArduino(uint8_t _ledPin) {
  if (_ledPin != 100) {
    digitalWrite(_ledPin, LOW);
  }
}
void debug(bool _debugOn, String _consoleMessage) {
  if (_debugOn) {
    Serial.println(_consoleMessage);
  }
}
