// =============================================================================
//  PneumaticShift_V0.0.1.ino
//  Version: V0.0.1
//  Pneumatic gearbox shift controller
//  Hardware: Arduino Mega 2560
//  4x 5/2 solenoid valves, 4x reed switches, 3x push buttons
//
//  Changelog:
//  V0.0.1 — Initial release. First upshift from neutral always selects 1st gear.
// =============================================================================
//
//  VALVE PLUMBING SUMMARY
//  ----------------------
//  A1 (gate):   normally EXTENDED  — energising RETRACTS  it
//  A2 (gate):   normally RETRACTED — energising EXTENDS   it
//  B1 (engage): normally EXTENDED  — energising RETRACTS  it
//  B2 (engage): normally RETRACTED — energising EXTENDS   it
//
//  GEAR TRUTH TABLE (valve: 0=de-energised, 1=energised)
//  -------------------------------------------------------
//  Gear  A1  A2  B1  B2   Gate   Engage
//  N      0   0   0   0   3-4    neutral
//  1      0   1   0   1   1-2    forward
//  2      0   1   1   0   1-2    back
//  3      0   0   0   1   3-4    forward
//  4      0   0   1   0   3-4    back
//  5      1   0   0   1   5-6    forward
//  6      1   0   1   0   5-6    back
//
//  PIN ASSIGNMENTS
//  ---------------
//  Valve outputs (HIGH = energise solenoid)
//  Valve A1 → Pin 2
//  Valve A2 → Pin 3
//  Valve B1 → Pin 4
//  Valve B2 → Pin 5
//
//  Reed switch inputs (INPUT_PULLUP — LOW = cylinder at position)
//  Reed A1 extended  → Pin 22
//  Reed A1 retracted → Pin 23
//  Reed A2 extended  → Pin 24
//  Reed A2 retracted → Pin 25
//  Reed B1 extended  → Pin 26
//  Reed B1 retracted → Pin 27
//  Reed B2 extended  → Pin 28
//  Reed B2 retracted → Pin 29
//
//  Button inputs (INPUT_PULLUP — LOW = pressed)
//  Up-shift   → Pin 30
//  Down-shift → Pin 31
//  Neutral    → Pin 32
//
//  Status LED → Pin 13 (built-in)
//  Fault LED  → Pin 12
//
// =============================================================================

// -----------------------------------------------------------------------------
//  Pin definitions
// -----------------------------------------------------------------------------

// Valve outputs
const uint8_t PIN_VALVE_A1 = 2;
const uint8_t PIN_VALVE_A2 = 3;
const uint8_t PIN_VALVE_B1 = 4;
const uint8_t PIN_VALVE_B2 = 5;

// Reed switches — cylinder positions
const uint8_t PIN_REED_A1_EXT = 22;
const uint8_t PIN_REED_A1_RET = 23;
const uint8_t PIN_REED_A2_EXT = 24;
const uint8_t PIN_REED_A2_RET = 25;
const uint8_t PIN_REED_B1_EXT = 26;
const uint8_t PIN_REED_B1_RET = 27;
const uint8_t PIN_REED_B2_EXT = 28;
const uint8_t PIN_REED_B2_RET = 29;

// Buttons
const uint8_t PIN_BTN_UP   = 30;
const uint8_t PIN_BTN_DOWN = 31;
const uint8_t PIN_BTN_NEU  = 32;

// LEDs
const uint8_t PIN_LED_STATUS = 13;
const uint8_t PIN_LED_FAULT  = 12;

// -----------------------------------------------------------------------------
//  Timing constants (milliseconds)
// -----------------------------------------------------------------------------

const uint16_t PHASE_TIMEOUT_MS   = 1500;  // max time to wait for a reed to confirm
const uint16_t DEBOUNCE_MS        =   50;  // button debounce
const uint16_t BUTTON_HOLD_MS     =  300;  // min hold time before shift fires
const uint16_t SETTLE_MS          =   50;  // brief settle after reed confirmed

// -----------------------------------------------------------------------------
//  Gear definitions
// -----------------------------------------------------------------------------

// Gear indices
const int8_t GEAR_N = 0;
const int8_t GEAR_1 = 1;
const int8_t GEAR_2 = 2;
const int8_t GEAR_3 = 3;
const int8_t GEAR_4 = 4;
const int8_t GEAR_5 = 5;
const int8_t GEAR_6 = 6;
const int8_t GEAR_COUNT = 7;

const char* GEAR_NAMES[GEAR_COUNT] = {"N","1","2","3","4","5","6"};

// Valve states for each gear: {A1, A2, B1, B2}
// 0 = de-energised, 1 = energised
const uint8_t GEAR_VALVES[GEAR_COUNT][4] = {
  //  A1  A2  B1  B2
  {   0,  0,  0,  0  },  // N
  {   0,  1,  0,  1  },  // 1
  {   0,  1,  1,  0  },  // 2
  {   0,  0,  0,  1  },  // 3
  {   0,  0,  1,  0  },  // 4
  {   1,  0,  0,  1  },  // 5
  {   1,  0,  1,  0  },  // 6
};

// Up-shift map: GEAR_UP[g] = next gear up, -1 = already at top
const int8_t GEAR_UP[GEAR_COUNT]   = { 1, 3, 1, 5, 3, -1, 5 };
//  N→1, 1→3 (skip — see note), but we use sequential shifting below

// Sequential up/down shift targets
const int8_t SHIFT_UP[GEAR_COUNT]   = { 1,  2,  3,  4,  5,  6, -1 };
//  N→1, 1→2, 2→3, 3→4, 4→5, 5→6, 6→top (no shift)

const int8_t SHIFT_DOWN[GEAR_COUNT] = {-1,  0,  1,  2,  3,  4,  5 };
//  N down → nothing, 1 down → N, 2 down → 1, etc.

// -----------------------------------------------------------------------------
//  Intermediate states used during gate changes
//  Gate-neutral valve states: engage (B) is neutral, gate (A) set for that gate
// -----------------------------------------------------------------------------

// Intermediate state indices
const uint8_t INTER_N12 = 0;  // 1-2 gate, neutral engage
const uint8_t INTER_N34 = 1;  // 3-4 gate, neutral engage  (same as gear N)
const uint8_t INTER_N56 = 2;  // 5-6 gate, neutral engage

const uint8_t INTER_VALVES[3][4] = {
  //  A1  A2  B1  B2
  {   0,  1,  0,  0  },  // N12
  {   0,  0,  0,  0  },  // N34 (= gear N)
  {   1,  0,  0,  0  },  // N56
};

// Which intermediate neutral goes with each gear
const int8_t GEAR_INTER[GEAR_COUNT] = {
  INTER_N34,  // N
  INTER_N12,  // 1
  INTER_N12,  // 2
  INTER_N34,  // 3
  INTER_N34,  // 4
  INTER_N56,  // 5
  INTER_N56,  // 6
};

const char* INTER_NAMES[3] = {"N12","N34","N56"};

// -----------------------------------------------------------------------------
//  State machine
// -----------------------------------------------------------------------------

enum SystemState {
  STATE_IDLE,       // sitting in a gear, waiting for input
  STATE_SHIFTING,   // shift sequence in progress
  STATE_FAULT,      // reed switch timeout or conflict detected
};

SystemState sysState  = STATE_IDLE;
int8_t      currentGear = GEAR_N;
int8_t      targetGear  = GEAR_N;
bool        faultActive = false;

// -----------------------------------------------------------------------------
//  Cylinder state helpers
// -----------------------------------------------------------------------------
//  A1 and B1 are normally extended.
//  Energising them RETRACTS.
//  A2 and B2 are normally retracted.
//  Energising them EXTENDS.

bool cylinderExtended(uint8_t valveIndex, uint8_t valveState) {
  if (valveIndex == 0 || valveIndex == 2) {
    // A1 or B1: norm-extended, energised = retracted
    return (valveState == 0);
  } else {
    // A2 or B2: norm-retracted, energised = extended
    return (valveState == 1);
  }
}

// -----------------------------------------------------------------------------
//  Reed switch confirmation
//  Returns true when the cylinder for valveIndex reaches its expected position.
//  Times out after PHASE_TIMEOUT_MS and sets fault.
// -----------------------------------------------------------------------------

bool waitForCylinder(uint8_t valveIndex, bool expectExtended) {
  uint8_t pinConfirm;
  switch (valveIndex) {
    case 0: pinConfirm = expectExtended ? PIN_REED_A1_EXT : PIN_REED_A1_RET; break;
    case 1: pinConfirm = expectExtended ? PIN_REED_A2_EXT : PIN_REED_A2_RET; break;
    case 2: pinConfirm = expectExtended ? PIN_REED_B1_EXT : PIN_REED_B1_RET; break;
    case 3: pinConfirm = expectExtended ? PIN_REED_B2_EXT : PIN_REED_B2_RET; break;
    default: return false;
  }

  const char* names[4] = {"A1","A2","B1","B2"};
  unsigned long start = millis();

  while (digitalRead(pinConfirm) != LOW) {
    if (millis() - start > PHASE_TIMEOUT_MS) {
      Serial.print(F("FAULT: timeout waiting for "));
      Serial.print(names[valveIndex]);
      Serial.println(expectExtended ? F(" extended") : F(" retracted"));
      setFault();
      return false;
    }
    delay(5);
  }

  delay(SETTLE_MS);
  return true;
}

// Wait for ALL four cylinders to confirm their positions for a given valve state array
bool waitAllCylinders(const uint8_t valves[4]) {
  for (uint8_t i = 0; i < 4; i++) {
    bool ext = cylinderExtended(i, valves[i]);
    if (!waitForCylinder(i, ext)) return false;
  }
  return true;
}

// -----------------------------------------------------------------------------
//  Valve control
// -----------------------------------------------------------------------------

void applyValves(const uint8_t valves[4]) {
  digitalWrite(PIN_VALVE_A1, valves[0] ? HIGH : LOW);
  digitalWrite(PIN_VALVE_A2, valves[1] ? HIGH : LOW);
  digitalWrite(PIN_VALVE_B1, valves[2] ? HIGH : LOW);
  digitalWrite(PIN_VALVE_B2, valves[3] ? HIGH : LOW);
}

void applyGear(int8_t gear) {
  applyValves(GEAR_VALVES[gear]);
}

void applyIntermediate(uint8_t inter) {
  applyValves(INTER_VALVES[inter]);
}

// -----------------------------------------------------------------------------
//  Fault handling
// -----------------------------------------------------------------------------

void setFault() {
  // De-energise all valves — system returns to power-off safe state (N in 3-4 gate)
  uint8_t safe[4] = {0, 0, 0, 0};
  applyValves(safe);
  faultActive = true;
  sysState = STATE_FAULT;
  currentGear = GEAR_N;
  digitalWrite(PIN_LED_FAULT, HIGH);
  digitalWrite(PIN_LED_STATUS, LOW);
  Serial.println(F("FAULT: all valves de-energised. System in safe state (3-4 gate neutral)."));
  Serial.println(F("Press UP + DOWN together to clear fault."));
}

void clearFault() {
  faultActive = false;
  sysState = STATE_IDLE;
  currentGear = GEAR_N;
  applyGear(GEAR_N);
  digitalWrite(PIN_LED_FAULT, LOW);
  digitalWrite(PIN_LED_STATUS, HIGH);
  Serial.println(F("Fault cleared. System reset to neutral."));
}

// -----------------------------------------------------------------------------
//  Serial logging helpers
// -----------------------------------------------------------------------------

void logValveState(const uint8_t valves[4]) {
  Serial.print(F("  Valves: A1="));
  Serial.print(valves[0]);
  Serial.print(F(" A2="));
  Serial.print(valves[1]);
  Serial.print(F(" B1="));
  Serial.print(valves[2]);
  Serial.print(F(" B2="));
  Serial.println(valves[3]);
  Serial.print(F("  Cylinders: A1="));
  Serial.print(cylinderExtended(0,valves[0]) ? F("EXT") : F("RET"));
  Serial.print(F(" A2="));
  Serial.print(cylinderExtended(1,valves[1]) ? F("EXT") : F("RET"));
  Serial.print(F(" B1="));
  Serial.print(cylinderExtended(2,valves[2]) ? F("EXT") : F("RET"));
  Serial.print(F(" B2="));
  Serial.println(cylinderExtended(3,valves[3]) ? F("EXT") : F("RET"));
}

void logPhase(const char* label, const uint8_t valves[4]) {
  Serial.print(F("  Phase: "));
  Serial.println(label);
  logValveState(valves);
}

// -----------------------------------------------------------------------------
//  Shift sequencer
//  Handles the full H-pattern sequencing logic:
//    1. If engage position needs to change to neutral first — do it
//    2. If gate needs to change — change it
//    3. Move to target engage position
// -----------------------------------------------------------------------------

bool executeShift(int8_t from, int8_t to) {
  Serial.print(F("\nShifting: "));
  Serial.print(GEAR_NAMES[from]);
  Serial.print(F(" -> "));
  Serial.println(GEAR_NAMES[to]);

  const uint8_t* fromValves = GEAR_VALVES[from];
  const uint8_t* toValves   = GEAR_VALVES[to];

  uint8_t fromInter = GEAR_INTER[from];
  uint8_t toInter   = GEAR_INTER[to];

  bool sameGate = (fromInter == toInter);
  bool fromNeutralEngage = (fromValves[2] == INTER_VALVES[fromInter][2] &&
                            fromValves[3] == INTER_VALVES[fromInter][3]);

  // ── PHASE 1: disengage (return B valves to neutral for this gate) ──────────
  // Skip if already at neutral engage, or if same gate (B valves swap together)
  if (!sameGate && !fromNeutralEngage) {
    logPhase("Disengage — return to gate neutral", INTER_VALVES[fromInter]);
    applyIntermediate(fromInter);
    if (!waitAllCylinders(INTER_VALVES[fromInter])) return false;
  }

  // ── PHASE 2: gate change (A valves) ────────────────────────────────────────
  if (!sameGate) {
    logPhase("Gate change", INTER_VALVES[toInter]);
    applyIntermediate(toInter);
    if (!waitAllCylinders(INTER_VALVES[toInter])) return false;
  }

  // ── PHASE 3: engage (B valves to target position) ──────────────────────────
  // For same-gate shifts, both B valves swap simultaneously through neutral
  logPhase("Engage target gear", toValves);
  applyValves(toValves);
  if (!waitAllCylinders(toValves)) return false;

  Serial.print(F("Shift complete: gear "));
  Serial.println(GEAR_NAMES[to]);
  return true;
}

// -----------------------------------------------------------------------------
//  Button handling
// -----------------------------------------------------------------------------

struct Button {
  uint8_t  pin;
  bool     lastState;
  bool     pressed;
  uint32_t pressTime;
};

Button btnUp   = {PIN_BTN_UP,   false, false, 0};
Button btnDown = {PIN_BTN_DOWN, false, false, 0};
Button btnNeu  = {PIN_BTN_NEU,  false, false, 0};

void updateButton(Button& btn) {
  bool reading = (digitalRead(btn.pin) == LOW);
  if (reading && !btn.lastState) {
    btn.pressTime = millis();
  }
  if (reading && (millis() - btn.pressTime > DEBOUNCE_MS)) {
    btn.pressed = true;
  } else {
    btn.pressed = false;
  }
  btn.lastState = reading;
}

// -----------------------------------------------------------------------------
//  Serial command parser
//  Send single characters over Serial to trigger shifts:
//  '0' = N, '1'-'6' = gears, 'u' = up, 'd' = down, 'n' = neutral, 'c' = clear fault
//  's' = print current status
// -----------------------------------------------------------------------------

void handleSerial() {
  if (!Serial.available()) return;
  char cmd = Serial.read();

  if (cmd == 'c' || cmd == 'C') {
    if (faultActive) clearFault();
    return;
  }
  if (cmd == 's' || cmd == 'S') {
    printStatus();
    return;
  }
  if (faultActive) {
    Serial.println(F("System in fault state. Send 'c' to clear."));
    return;
  }
  if (sysState == STATE_SHIFTING) {
    Serial.println(F("Shift in progress — command ignored."));
    return;
  }

  int8_t requested = -1;
  if (cmd == '0' || cmd == 'n' || cmd == 'N') requested = GEAR_N;
  else if (cmd == '1') requested = GEAR_1;
  else if (cmd == '2') requested = GEAR_2;
  else if (cmd == '3') requested = GEAR_3;
  else if (cmd == '4') requested = GEAR_4;
  else if (cmd == '5') requested = GEAR_5;
  else if (cmd == '6') requested = GEAR_6;
  else if (cmd == 'u' || cmd == 'U') requested = SHIFT_UP[currentGear];
  else if (cmd == 'd' || cmd == 'D') requested = SHIFT_DOWN[currentGear];

  if (requested < 0) return;
  if (requested == currentGear) {
    Serial.print(F("Already in gear "));
    Serial.println(GEAR_NAMES[currentGear]);
    return;
  }
  triggerShift(requested);
}

void printStatus() {
  Serial.println(F("\n--- Status ---"));
  Serial.print(F("Gear:  ")); Serial.println(GEAR_NAMES[currentGear]);
  Serial.print(F("State: "));
  switch (sysState) {
    case STATE_IDLE:     Serial.println(F("idle")); break;
    case STATE_SHIFTING: Serial.println(F("shifting")); break;
    case STATE_FAULT:    Serial.println(F("FAULT")); break;
  }
  Serial.println(F("Current valve state:"));
  logValveState(GEAR_VALVES[currentGear]);
  Serial.println(F("Reed switches (LOW=triggered):"));
  Serial.print(F("  A1 ext=")); Serial.print(digitalRead(PIN_REED_A1_EXT) == LOW ? "YES" : "no");
  Serial.print(F(" ret="));     Serial.println(digitalRead(PIN_REED_A1_RET) == LOW ? "YES" : "no");
  Serial.print(F("  A2 ext=")); Serial.print(digitalRead(PIN_REED_A2_EXT) == LOW ? "YES" : "no");
  Serial.print(F(" ret="));     Serial.println(digitalRead(PIN_REED_A2_RET) == LOW ? "YES" : "no");
  Serial.print(F("  B1 ext=")); Serial.print(digitalRead(PIN_REED_B1_EXT) == LOW ? "YES" : "no");
  Serial.print(F(" ret="));     Serial.println(digitalRead(PIN_REED_B1_RET) == LOW ? "YES" : "no");
  Serial.print(F("  B2 ext=")); Serial.print(digitalRead(PIN_REED_B2_EXT) == LOW ? "YES" : "no");
  Serial.print(F(" ret="));     Serial.println(digitalRead(PIN_REED_B2_RET) == LOW ? "YES" : "no");
  Serial.print(F("Commands: 0-6=gear  u=up  d=down  n=neutral  s=status  c=clear fault\n"));
}

// -----------------------------------------------------------------------------
//  Shift trigger
// -----------------------------------------------------------------------------

void triggerShift(int8_t target) {
  if (target < 0 || target >= GEAR_COUNT) return;
  if (target == currentGear) return;
  if (sysState != STATE_IDLE) return;

  sysState = STATE_SHIFTING;
  digitalWrite(PIN_LED_STATUS, LOW);

  bool ok = executeShift(currentGear, target);

  if (ok) {
    currentGear = target;
    sysState = STATE_IDLE;
    digitalWrite(PIN_LED_STATUS, HIGH);
  }
  // fault case handled inside executeShift via setFault()
}

// -----------------------------------------------------------------------------
//  setup()
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);

  // Valve outputs — de-energise all on startup
  pinMode(PIN_VALVE_A1, OUTPUT); digitalWrite(PIN_VALVE_A1, LOW);
  pinMode(PIN_VALVE_A2, OUTPUT); digitalWrite(PIN_VALVE_A2, LOW);
  pinMode(PIN_VALVE_B1, OUTPUT); digitalWrite(PIN_VALVE_B1, LOW);
  pinMode(PIN_VALVE_B2, OUTPUT); digitalWrite(PIN_VALVE_B2, LOW);

  // Reed switches
  pinMode(PIN_REED_A1_EXT, INPUT_PULLUP);
  pinMode(PIN_REED_A1_RET, INPUT_PULLUP);
  pinMode(PIN_REED_A2_EXT, INPUT_PULLUP);
  pinMode(PIN_REED_A2_RET, INPUT_PULLUP);
  pinMode(PIN_REED_B1_EXT, INPUT_PULLUP);
  pinMode(PIN_REED_B1_RET, INPUT_PULLUP);
  pinMode(PIN_REED_B2_EXT, INPUT_PULLUP);
  pinMode(PIN_REED_B2_RET, INPUT_PULLUP);

  // Buttons
  pinMode(PIN_BTN_UP,   INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  pinMode(PIN_BTN_NEU,  INPUT_PULLUP);

  // LEDs
  pinMode(PIN_LED_STATUS, OUTPUT);
  pinMode(PIN_LED_FAULT,  OUTPUT);
  digitalWrite(PIN_LED_STATUS, HIGH);
  digitalWrite(PIN_LED_FAULT,  LOW);

  // System starts with all valves off — power-off safe state = N in 3-4 gate
  currentGear = GEAR_N;
  sysState    = STATE_IDLE;

  Serial.println(F("\n====================================="));
  Serial.println(F("  PneumaticShift V0.0.1 — ready"));
  Serial.println(F("====================================="));
  printStatus();
}

// -----------------------------------------------------------------------------
//  loop()
// -----------------------------------------------------------------------------

void loop() {
  // Handle serial commands
  handleSerial();

  // Handle fault — only allow clear via UP+DOWN held together
  if (faultActive) {
    updateButton(btnUp);
    updateButton(btnDown);
    if (btnUp.pressed && btnDown.pressed) {
      clearFault();
    }
    digitalWrite(PIN_LED_FAULT, (millis() / 500) % 2);  // blink fault LED
    return;
  }

  if (sysState != STATE_IDLE) return;

  // Read buttons
  updateButton(btnUp);
  updateButton(btnDown);
  updateButton(btnNeu);

  // Neutral button
  if (btnNeu.pressed && currentGear != GEAR_N) {
    triggerShift(GEAR_N);
    return;
  }

  // Up-shift
  if (btnUp.pressed && !btnDown.pressed) {
    int8_t next = SHIFT_UP[currentGear];
    if (next >= 0) triggerShift(next);
    else Serial.println(F("Already at top gear."));
    return;
  }

  // Down-shift
  if (btnDown.pressed && !btnUp.pressed) {
    int8_t next = SHIFT_DOWN[currentGear];
    if (next >= 0) triggerShift(next);
    else Serial.println(F("Already at bottom gear."));
    return;
  }

  // Status LED heartbeat when idle
  digitalWrite(PIN_LED_STATUS, (millis() / 1000) % 2);
}
