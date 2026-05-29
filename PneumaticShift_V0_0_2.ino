// =============================================================================
//  PneumaticShift_V0_0_2.ino
//  Version: V0_0_2
//  Pneumatic gearbox shift controller — Arduino Mega 2560
//  5x 5/2 solenoid valves (A1,A2,A3,B1,B2), 1x clutch valve (C1)
//  12x reed switches, 3x push buttons
//
//  Changelog:
//  V0_0_1 - Initial release. First upshift from neutral always selects 1st gear.
//  V0_0_2 - Added reverse gear (A3 cylinder, Rev gate left of 1-2).
//            Added C1 clutch cylinder (energise = disengage clutch).
//            Clutch disengages before any cylinder moves, re-engages after confirmed.
//            Horizontal gate swap when already at neutral engage line.
//            Sequential upshift/downshift updated for reverse.
// =============================================================================
//
//  VALVE PLUMBING
//  A1 (gate):    norm-EXTENDED  - energising RETRACTS
//  A2 (gate):    norm-RETRACTED - energising EXTENDS
//  A3 (reverse): norm-RETRACTED - energising EXTENDS
//  B1 (engage):  norm-EXTENDED  - energising RETRACTS
//  B2 (engage):  norm-RETRACTED - energising EXTENDS
//  C1 (clutch):  norm-RETRACTED - energising DISENGAGES clutch
//
//  GATE TRUTH TABLE
//  Rev  A1=0(ext) A2=1(ext) A3=1(ext)
//  1-2  A1=0(ext) A2=1(ext) A3=0(ret)
//  3-4  A1=0(ext) A2=0(ret) A3=0(ret)  <- power-off default
//  5-6  A1=1(ret) A2=0(ret) A3=0(ret)
//
//  ENGAGE TRUTH TABLE
//  forward  B1=0(ext) B2=1(ext)
//  neutral  B1=0(ext) B2=0(ret)  <- power-off default
//  back     B1=1(ret) B2=0(ret)
//
//  GEAR VALVE STATES {A1,A2,A3,B1,B2}
//  N  0 0 0  0 0    R  0 1 1  0 1
//  1  0 1 0  0 1    2  0 1 0  1 0
//  3  0 0 0  0 1    4  0 0 0  1 0
//  5  1 0 0  0 1    6  1 0 0  1 0
//
//  SEQUENCING RULES
//  1. C1 energised (clutch disengaged) BEFORE any gear cylinder moves.
//  2. Same-gate: B cylinders swap simultaneously.
//  3. Cross-gate: disengage B -> change A -> engage B.
//  4. Horizontal swap: if from and to are both at neutral engage, slide gate only.
//  5. Reverse: A3 only moves from neutral. Route must be N<->R.
//  6. C1 de-energised only after all cylinder reeds confirmed.
//
//  PIN ASSIGNMENTS
//  Valves (HIGH=energise): A1=2  A2=3  A3=4  B1=5  B2=6  C1=7
//  Reeds  (LOW=triggered): A1ext=22 A1ret=23 A2ext=24 A2ret=25
//                          A3ext=26 A3ret=27 B1ext=28 B1ret=29
//                          B2ext=30 B2ret=31 C1ext=32 C1ret=33
//  Buttons (LOW=pressed):  Up=34  Down=35  Neutral=36
//  LEDs: Status=13  Fault=12
//
// =============================================================================

const uint8_t PIN_VALVE_A1=2, PIN_VALVE_A2=3, PIN_VALVE_A3=4;
const uint8_t PIN_VALVE_B1=5, PIN_VALVE_B2=6, PIN_VALVE_C1=7;

const uint8_t PIN_REED_A1_EXT=22, PIN_REED_A1_RET=23;
const uint8_t PIN_REED_A2_EXT=24, PIN_REED_A2_RET=25;
const uint8_t PIN_REED_A3_EXT=26, PIN_REED_A3_RET=27;
const uint8_t PIN_REED_B1_EXT=28, PIN_REED_B1_RET=29;
const uint8_t PIN_REED_B2_EXT=30, PIN_REED_B2_RET=31;
const uint8_t PIN_REED_C1_EXT=32, PIN_REED_C1_RET=33;

const uint8_t PIN_BTN_UP=34, PIN_BTN_DOWN=35, PIN_BTN_NEU=36;
const uint8_t PIN_LED_STATUS=13, PIN_LED_FAULT=12;

const uint16_t PHASE_TIMEOUT_MS=1500, CLUTCH_TIMEOUT_MS=1000;
const uint16_t DEBOUNCE_MS=50, SETTLE_MS=50;

// Gear indices
const int8_t GEAR_N=0,GEAR_R=1,GEAR_1=2,GEAR_2=3,GEAR_3=4,GEAR_4=5,GEAR_5=6,GEAR_6=7;
const int8_t GEAR_COUNT=8;
const char* GEAR_NAMES[8]={"N","R","1","2","3","4","5","6"};

// Valve states {A1,A2,A3,B1,B2}
const uint8_t GEAR_VALVES[8][5]={
  {0,0,0,0,0}, // N
  {0,1,1,0,1}, // R
  {0,1,0,0,1}, // 1
  {0,1,0,1,0}, // 2
  {0,0,0,0,1}, // 3
  {0,0,0,1,0}, // 4
  {1,0,0,0,1}, // 5
  {1,0,0,1,0}, // 6
};

// Intermediate neutral-engage states {A1,A2,A3,B1,B2}
const uint8_t INTER_NR [5]={0,1,1,0,0};
const uint8_t INTER_N12[5]={0,1,0,0,0};
const uint8_t INTER_N34[5]={0,0,0,0,0};
const uint8_t INTER_N56[5]={1,0,0,0,0};

const uint8_t* GEAR_INTER[8]={
  INTER_N34,INTER_NR,INTER_N12,INTER_N12,
  INTER_N34,INTER_N34,INTER_N56,INTER_N56
};

// Sequential shifts (-1=not possible)
const int8_t SHIFT_UP  [8]={ 2,-1, 3, 4, 5, 6, 7,-1};
const int8_t SHIFT_DOWN[8]={-1, 0, 0, 2, 3, 4, 5, 6};

// Reed pin lookup
const uint8_t REED_EXT_PINS[5]={PIN_REED_A1_EXT,PIN_REED_A2_EXT,PIN_REED_A3_EXT,PIN_REED_B1_EXT,PIN_REED_B2_EXT};
const uint8_t REED_RET_PINS[5]={PIN_REED_A1_RET,PIN_REED_A2_RET,PIN_REED_A3_RET,PIN_REED_B1_RET,PIN_REED_B2_RET};
const char*   CYL_NAMES[5]={"A1","A2","A3","B1","B2"};

enum SystemState{STATE_IDLE,STATE_SHIFTING,STATE_FAULT};
SystemState sysState=STATE_IDLE;
int8_t currentGear=GEAR_N;
bool   faultActive=false;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

bool cylExtended(uint8_t idx,uint8_t v){
  // A1(0),B1(3) norm-ext: extended when de-energised
  return (idx==0||idx==3)?(v==0):(v==1);
}

bool isNeutralEngage(const uint8_t v[5]){return v[3]==0&&v[4]==0;}
bool sameGate(int8_t a,int8_t b){return GEAR_INTER[a]==GEAR_INTER[b];}

// ---------------------------------------------------------------------------
// Reed confirmation
// ---------------------------------------------------------------------------

void setFault(); // forward declaration

bool waitForCylinder(uint8_t idx,bool expectExt){
  uint8_t pin=expectExt?REED_EXT_PINS[idx]:REED_RET_PINS[idx];
  unsigned long t=millis();
  while(digitalRead(pin)!=LOW){
    if(millis()-t>PHASE_TIMEOUT_MS){
      Serial.print(F("FAULT: timeout ")); Serial.print(CYL_NAMES[idx]);
      Serial.println(expectExt?F(" ext"):F(" ret"));
      setFault(); return false;
    }
    delay(5);
  }
  delay(SETTLE_MS); return true;
}

bool waitAllCylinders(const uint8_t v[5]){
  for(uint8_t i=0;i<5;i++) if(!waitForCylinder(i,cylExtended(i,v[i]))) return false;
  return true;
}

bool waitClutch(bool expectDisengaged){
  uint8_t pin=expectDisengaged?PIN_REED_C1_EXT:PIN_REED_C1_RET;
  unsigned long t=millis();
  while(digitalRead(pin)!=LOW){
    if(millis()-t>CLUTCH_TIMEOUT_MS){
      Serial.println(expectDisengaged?F("FAULT: clutch no disengage"):F("FAULT: clutch no engage"));
      setFault(); return false;
    }
    delay(5);
  }
  delay(SETTLE_MS); return true;
}

// ---------------------------------------------------------------------------
// Valve control
// ---------------------------------------------------------------------------

void applyValves(const uint8_t v[5]){
  digitalWrite(PIN_VALVE_A1,v[0]?HIGH:LOW);
  digitalWrite(PIN_VALVE_A2,v[1]?HIGH:LOW);
  digitalWrite(PIN_VALVE_A3,v[2]?HIGH:LOW);
  digitalWrite(PIN_VALVE_B1,v[3]?HIGH:LOW);
  digitalWrite(PIN_VALVE_B2,v[4]?HIGH:LOW);
}

// ---------------------------------------------------------------------------
// Clutch control
// ---------------------------------------------------------------------------

bool clutchOpen(){
  Serial.println(F("  Clutch: disengaging"));
  digitalWrite(PIN_VALVE_C1,HIGH);
  if(!waitClutch(true)) return false;
  Serial.println(F("  Clutch: disengaged"));
  return true;
}

bool clutchClose(){
  Serial.println(F("  Clutch: re-engaging"));
  digitalWrite(PIN_VALVE_C1,LOW);
  if(!waitClutch(false)) return false;
  Serial.println(F("  Clutch: engaged"));
  return true;
}

// ---------------------------------------------------------------------------
// Fault handling
// ---------------------------------------------------------------------------

void setFault(){
  digitalWrite(PIN_VALVE_C1,LOW);
  const uint8_t safe[5]={0,0,0,0,0};
  applyValves(safe);
  faultActive=true; sysState=STATE_FAULT; currentGear=GEAR_N;
  digitalWrite(PIN_LED_FAULT,HIGH); digitalWrite(PIN_LED_STATUS,LOW);
  Serial.println(F("FAULT: all valves off (3-4 gate, neutral, clutch engaged)."));
  Serial.println(F("Hold UP+DOWN to clear."));
}

void clearFault(){
  const uint8_t safe[5]={0,0,0,0,0};
  applyValves(safe); digitalWrite(PIN_VALVE_C1,LOW);
  faultActive=false; sysState=STATE_IDLE; currentGear=GEAR_N;
  digitalWrite(PIN_LED_FAULT,LOW); digitalWrite(PIN_LED_STATUS,HIGH);
  Serial.println(F("Fault cleared — reset to neutral."));
}

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------

void logValves(const uint8_t v[5]){
  Serial.print(F("  Valves:    A1=")); Serial.print(v[0]);
  Serial.print(F(" A2="));            Serial.print(v[1]);
  Serial.print(F(" A3="));            Serial.print(v[2]);
  Serial.print(F(" B1="));            Serial.print(v[3]);
  Serial.print(F(" B2="));            Serial.println(v[4]);
  Serial.print(F("  Cylinders: A1=")); Serial.print(cylExtended(0,v[0])?F("EXT"):F("RET"));
  Serial.print(F(" A2="));            Serial.print(cylExtended(1,v[1])?F("EXT"):F("RET"));
  Serial.print(F(" A3="));            Serial.print(cylExtended(2,v[2])?F("EXT"):F("RET"));
  Serial.print(F(" B1="));            Serial.print(cylExtended(3,v[3])?F("EXT"):F("RET"));
  Serial.print(F(" B2="));            Serial.println(cylExtended(4,v[4])?F("EXT"):F("RET"));
}

// ---------------------------------------------------------------------------
// Shift sequencer
// ---------------------------------------------------------------------------

bool executeShift(int8_t from,int8_t to){
  Serial.print(F("\nShifting: ")); Serial.print(GEAR_NAMES[from]);
  Serial.print(F(" -> "));        Serial.println(GEAR_NAMES[to]);

  const uint8_t* fromV    =GEAR_VALVES[from];
  const uint8_t* toV      =GEAR_VALVES[to];
  const uint8_t* fromInter=GEAR_INTER[from];
  const uint8_t* toInter  =GEAR_INTER[to];
  bool samegate   =sameGate(from,to);
  bool fromNeutral=isNeutralEngage(fromV);
  bool toNeutral  =isNeutralEngage(toV);
  bool horizSwap  =(!samegate)&&fromNeutral&&toNeutral;

  // Horizontal gate swap — both sides already at neutral engage
  if(horizSwap){
    Serial.println(F("  Horizontal gate swap"));
    logValves(toV);
    if(!clutchOpen())           return false;
    applyValves(toV);
    if(!waitAllCylinders(toV)) return false;
    if(!clutchClose())          return false;
    Serial.print(F("Complete: ")); Serial.println(GEAR_NAMES[to]);
    return true;
  }

  // Phase 1: disengage B if not already at neutral engage
  if(!samegate&&!fromNeutral){
    Serial.println(F("  Phase 1: B disengage"));
    logValves(fromInter);
    if(!clutchOpen())                return false;
    applyValves(fromInter);
    if(!waitAllCylinders(fromInter)) return false;
    if(!clutchClose())               return false;
  }

  // Phase 2: gate change
  if(!samegate){
    Serial.println(F("  Phase 2: gate change"));
    logValves(toInter);
    if(!clutchOpen())               return false;
    applyValves(toInter);
    if(!waitAllCylinders(toInter)) return false;
    if(!clutchClose())              return false;
  }

  // Phase 3: engage B to target (skip if target is neutral engage)
  if(!toNeutral||samegate){
    Serial.print(F("  Phase 3: engage -> ")); Serial.println(GEAR_NAMES[to]);
    logValves(toV);
    if(!clutchOpen())           return false;
    applyValves(toV);
    if(!waitAllCylinders(toV)) return false;
    if(!clutchClose())          return false;
  }

  Serial.print(F("Complete: ")); Serial.println(GEAR_NAMES[to]);
  return true;
}

// ---------------------------------------------------------------------------
// Button handling
// ---------------------------------------------------------------------------

struct Button{uint8_t pin;bool lastState,pressed;uint32_t pressTime;};
Button btnUp  ={PIN_BTN_UP,  false,false,0};
Button btnDown={PIN_BTN_DOWN,false,false,0};
Button btnNeu ={PIN_BTN_NEU, false,false,0};

void updateButton(Button& btn){
  bool r=(digitalRead(btn.pin)==LOW);
  if(r&&!btn.lastState) btn.pressTime=millis();
  btn.pressed=(r&&(millis()-btn.pressTime>DEBOUNCE_MS));
  btn.lastState=r;
}

// ---------------------------------------------------------------------------
// Shift trigger
// ---------------------------------------------------------------------------

void triggerShift(int8_t target){
  if(target<0||target>=GEAR_COUNT||target==currentGear) return;
  if(sysState!=STATE_IDLE) return;
  sysState=STATE_SHIFTING;
  digitalWrite(PIN_LED_STATUS,LOW);
  if(executeShift(currentGear,target)){
    currentGear=target;
    sysState=STATE_IDLE;
    digitalWrite(PIN_LED_STATUS,HIGH);
  }
}

// ---------------------------------------------------------------------------
// Serial commands: 0/n=neutral r=reverse 1-6=gear u=up d=down s=status c=clear
// ---------------------------------------------------------------------------

void printStatus(){
  Serial.println(F("\n--- Status ---"));
  Serial.print(F("Gear: ")); Serial.println(GEAR_NAMES[currentGear]);
  Serial.print(F("Clutch: "));
  Serial.println(digitalRead(PIN_VALVE_C1)==HIGH?F("disengaged"):F("engaged"));
  logValves(GEAR_VALVES[currentGear]);
  Serial.println(F("Commands: 0/n=neutral r=reverse 1-6=gear u=up d=down s=status c=clear"));
}

void handleSerial(){
  if(!Serial.available()) return;
  char cmd=Serial.read();
  if(cmd=='c'||cmd=='C'){if(faultActive) clearFault(); return;}
  if(cmd=='s'||cmd=='S'){printStatus(); return;}
  if(faultActive){Serial.println(F("Fault active. Send c to clear.")); return;}
  if(sysState==STATE_SHIFTING){Serial.println(F("Shifting.")); return;}
  int8_t req=-1;
  if     (cmd=='0'||cmd=='n'||cmd=='N') req=GEAR_N;
  else if(cmd=='r'||cmd=='R')           req=GEAR_R;
  else if(cmd=='1')                     req=GEAR_1;
  else if(cmd=='2')                     req=GEAR_2;
  else if(cmd=='3')                     req=GEAR_3;
  else if(cmd=='4')                     req=GEAR_4;
  else if(cmd=='5')                     req=GEAR_5;
  else if(cmd=='6')                     req=GEAR_6;
  else if(cmd=='u'||cmd=='U')           req=SHIFT_UP[currentGear];
  else if(cmd=='d'||cmd=='D')           req=SHIFT_DOWN[currentGear];
  if(req<0) return;
  if(req==currentGear){Serial.print(F("Already in ")); Serial.println(GEAR_NAMES[currentGear]); return;}
  triggerShift(req);
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------

void setup(){
  Serial.begin(115200);
  while(!Serial&&millis()<2000);

  pinMode(PIN_VALVE_A1,OUTPUT); digitalWrite(PIN_VALVE_A1,LOW);
  pinMode(PIN_VALVE_A2,OUTPUT); digitalWrite(PIN_VALVE_A2,LOW);
  pinMode(PIN_VALVE_A3,OUTPUT); digitalWrite(PIN_VALVE_A3,LOW);
  pinMode(PIN_VALVE_B1,OUTPUT); digitalWrite(PIN_VALVE_B1,LOW);
  pinMode(PIN_VALVE_B2,OUTPUT); digitalWrite(PIN_VALVE_B2,LOW);
  pinMode(PIN_VALVE_C1,OUTPUT); digitalWrite(PIN_VALVE_C1,LOW);

  const uint8_t rp[]={22,23,24,25,26,27,28,29,30,31,32,33};
  for(uint8_t i=0;i<12;i++) pinMode(rp[i],INPUT_PULLUP);

  pinMode(PIN_BTN_UP,INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN,INPUT_PULLUP);
  pinMode(PIN_BTN_NEU,INPUT_PULLUP);
  pinMode(PIN_LED_STATUS,OUTPUT); digitalWrite(PIN_LED_STATUS,HIGH);
  pinMode(PIN_LED_FAULT, OUTPUT); digitalWrite(PIN_LED_FAULT, LOW);

  currentGear=GEAR_N; sysState=STATE_IDLE;
  Serial.println(F("\n====================================="));
  Serial.println(F("  PneumaticShift V0_0_2 - ready"));
  Serial.println(F("====================================="));
  printStatus();
}

void loop(){
  handleSerial();
  if(faultActive){
    updateButton(btnUp); updateButton(btnDown);
    if(btnUp.pressed&&btnDown.pressed) clearFault();
    digitalWrite(PIN_LED_FAULT,(millis()/500)%2);
    return;
  }
  if(sysState!=STATE_IDLE) return;
  updateButton(btnUp); updateButton(btnDown); updateButton(btnNeu);
  if(btnNeu.pressed&&currentGear!=GEAR_N){triggerShift(GEAR_N); return;}
  if(btnUp.pressed&&!btnDown.pressed){
    int8_t n=SHIFT_UP[currentGear];
    if(n>=0) triggerShift(n); else Serial.println(F("Top gear."));
    return;
  }
  if(btnDown.pressed&&!btnUp.pressed){
    int8_t n=SHIFT_DOWN[currentGear];
    if(n>=0) triggerShift(n); else Serial.println(F("Bottom gear."));
    return;
  }
  digitalWrite(PIN_LED_STATUS,(millis()/1000)%2);
}
