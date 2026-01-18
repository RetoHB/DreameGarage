  /*************************************************************

   Dreame Garage Door v1.2
   Reto Huber
   2026
   for Xiao ESP32C3 controller
   
 *************************************************************/

#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <esp_task_wdt.h>

#define WIFI_SSID       "mySSID"
#define WIFI_PASS       "myPass"
#define IO_USERNAME     "myUsername"
#define IO_KEY          "myKey"
#define FEED_KEY        "dreame-status";

#define limitSwitch D4
#define LED1        D5
#define LED2        D6

#define PWM_PIN D8     // PWM output to L298N ENA
#define IN1     D9     // Input1 to L298N
#define IN2    D10     // Input2 to L298N

const unsigned long accelMotorTime = 1000;     // constant for time motor accelerates and decelerates
unsigned long fullSpeedMotorTime = 2000;       // global variable for time motor runs at full speed
const unsigned long fullSpeedUpTime = 1850;    // constant for time motor runs up
const unsigned long fullSpeedDownTime = 2100;  // constant for time motor runs down
int maxPWM = 200;                              // global variable for maximum speed
const int maxPWMup = 160;                      // constant for maximum speed up
const int maxPWMdown = 120;                    // constant for maximum speed down
int pwmValue = 0;                              // global variable for current speed
unsigned long stateStartTime = 0;              // timing variable for motor motion phases (IDLE, ACCEL, FULLSPEED, DECEL)

bool openDoor = false;      // door action to perform
bool lastOpenDoor = false;  // last door action
bool creepDone = false;

bool chargingWaitActive = false;
bool emptying = false;
unsigned long chargingWaitStart = 0;
const unsigned long chargingCloseDelay = 300000; // on charging wait 5 minutes before closing door

// ---------------- LED Blink Variables ----------------
bool LEDstate = LOW;
unsigned long lastBlink = 0;
const unsigned long BLINK_INTERVAL = 200;  // ms

// ------------ Adafruit IO poll variables -------------
unsigned long lastPoll = 0;                    // time of last poll to Adafruit IO
const unsigned long POLL_INTERVAL = 500;      // interval in ms to poll Adafruit IO
String dreame_state = "";                      // current state of Dreame
String dreame_status = "";                      // current status of Dreame
String dreame_room = "";                       // current room Dreame is in
String dreame_task = "";                       // current task of Dreama
String last_dreame_state = "";                 // last state of Dreame
String last_dreame_status = "";                 // last status of Dreame
String last_dreame_room = "";                  // last room Dreame was in
String last_dreame_task = "";                  // last task of Dreame
bool updatedData = false;                      // Flag to indicate new data arrived

// -------------------- Web Server ---------------------
WebServer server(80);

// -----------------------------------------------------

void handleRoot();
void handleOpen();
void handleClose();
void handleStatus();

//void readAdafruitValue();
bool readAdafruitFeed();
void pollAdafruitIO();
void evaluateState();
void updateDoor();
void startCalibration();
void startOpening();
void startClosing();
bool limitReached();
void handleMotorSequence();
void updateLED();
void setLEDs();

// ----------- State machine for door state -----------
enum DoorState {
  DOOR_IDLE,
  DOOR_CALIBRATING,
  DOOR_OPENING,
  DOOR_CLOSING
};
DoorState doorState = DOOR_IDLE;

// -------- State machine for motion phases ----------

enum MotionState {
  IDLE,
  ACCEL,
  FULLSPEED,
  DECEL
};
MotionState motionState = IDLE;

// ============================================================
// setup() function
// ============================================================

void setup()
{
  // Initialize hardware watchdog (timeout 60 seconds)
  esp_task_wdt_init(60, true);    // true = reset system on timeout
  esp_task_wdt_add(NULL);         // add current (loop) task to WDT
  
  pinMode(limitSwitch, INPUT_PULLUP);
  
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nDreame Garage Door controller");
  Serial.println("*****************************");
  Serial.println("\nConnecting to WiFi...");

  // ------------- WiFi connection ---------------

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address:\t");
  IPAddress myIP = WiFi.localIP();
  Serial.println(myIP);

  // ---------------- OTA updates ----------------

  ArduinoOTA.setHostname("Dreame Garage Door");   // optional, shows in IDE

  ArduinoOTA.onStart([]() {
    esp_task_wdt_delete(NULL);
    Serial.println("Watchdog disabled for OTA");
    Serial.println("Start OTA update");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd OTA update");
    esp_task_wdt_add(NULL);
    Serial.println("Watchdog re-enabled after OTA");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA ready");

  // ---------- Setup HTTP Routes for Webserver ------------
  
  server.on("/", handleRoot);
  server.on("/open", handleOpen);
  server.on("/close", handleClose);
  server.on("/status", handleStatus);
  server.begin();
  Serial.println("HTTP server started.");

  startCalibration();
}

// ============================================================
// main loop()
// ============================================================

void loop()
{  
  // ---------- Reconnect to WiFi if connection lost ------------
    
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    setLEDs(HIGH);
    WiFi.reconnect();
    delay(5000);
    return;
  }

  // ------------- Timer to poll AdafruitIO value ---------------
  
  unsigned long currentMillis = millis();
  if (currentMillis - lastPoll >= POLL_INTERVAL) {
    if (doorState == DOOR_IDLE) {
      lastPoll = currentMillis;
      pollAdafruitIO();
      evaluateState();
    }
  }
  
  updateDoor();            // Command the door
  updateLED();             // Command the LEDs
  ArduinoOTA.handle();     // Handle OTA
  esp_task_wdt_reset();    // reset watchdog

  server.handleClient();   // Handle incoming HTTP requests
}

// ============================================================
// Function: runMotorSequence(direction, fullSpeedTime, maxPWMValue)
// Call once to start the sequence; it runs non-blocking.
// ============================================================

void runMotorSequence(String direction, unsigned long fullSpeedTime, int maxPWMValue) {
  if (motionState != IDLE) return;     // motor already running

  fullSpeedMotorTime = fullSpeedTime;  // update global duration
  maxPWM = maxPWMValue;                // update global max PWM
  stateStartTime = millis();           // start time of current motion phase
  motionState = ACCEL;

  // Set motor direction
  if (direction == "down") {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (direction == "up") {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}

// ============================================================
// Function: handleMotorSequence()
// Non-blocking motor control handler, call this continuously inside loop()
// ============================================================

void handleMotorSequence() {
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - stateStartTime;   // time current motion phase is already running

  switch (motionState) {
    // in IDLE deactivate motor controller
    case IDLE:
      analogWrite(PWM_PIN, 0);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      return;

    case ACCEL:
    // in ACCEL steadily increase PWM value while motor acceleration time is not up
      if (elapsed <= accelMotorTime) {
        pwmValue = map(elapsed, 0, accelMotorTime, 0, maxPWM);   // map(value, fromLow, fromHigh, toLow, toHigh)
      } else {                                                   // when time is up change to FULLSPEED
        pwmValue = maxPWM;
        motionState = FULLSPEED;
        stateStartTime = currentTime;
      }
      break;

    case FULLSPEED:
    // in FULLSPEED run motor at maximum PWM
      pwmValue = maxPWM;
      if (elapsed >= fullSpeedMotorTime) {   // when time is up change to DECEL
        motionState = DECEL;
        stateStartTime = currentTime;
      }
      break;

    case DECEL:
    // in DECEL steadily lower PWM value while motor deceleration time is not up
      if (elapsed <= accelMotorTime) {
        pwmValue = map(elapsed, 0, accelMotorTime, maxPWM, 0);   // map(value, fromLow, fromHigh, toLow, toHigh)
      } else {                                                   // when time is up change to FULLSPEED
        pwmValue = 0;
        motionState = IDLE;
      }
      break;
  }

  analogWrite(PWM_PIN, pwmValue);   // continously write the current PWM value to the output pin
}

// ============================================================
// Function: updateDoor()
// Move the door
// ============================================================

void updateDoor() {
  handleMotorSequence();                // handle the motor continously

  // -------- Handle delayed close after charging --------
  if (chargingWaitActive && (doorState == DOOR_IDLE)) {
    if (millis() - chargingWaitStart >= chargingCloseDelay) {
      Serial.println("Charging delay expired → closing door");
      chargingWaitActive = false;
      openDoor = false;
    }
  }

  switch (doorState) {
    case DOOR_CALIBRATING:
      if (limitReached()) {             // limit switch has triggered
        pwmValue = 0;                   // stop the motor
        Serial.println("Calibration complete. Limit switch triggered.");
        doorState = DOOR_IDLE;          // door is IDLE now
        motionState = IDLE;             // ensure motor sequence fully stops
        startClosing();                 // close the door after calibration
      }
      break;

      case DOOR_OPENING:
        if (limitReached()) {            // limit switch has triggered
          analogWrite(PWM_PIN, 0);       // stop the motor
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, LOW);
          Serial.println("Door fully open (limit switch triggered)");
          doorState = DOOR_IDLE;         // door is IDLE now
          motionState = IDLE;            // ensure motor sequence fully stops
          creepDone = false;             // reset for next cycle
        } 
        else if (motionState == IDLE && !creepDone) {  // motor stopped because runtime is up, but door is not at limit
          Serial.println("Top not reached, creeping slowly upward...");
          creepDone = true;                  // only once!
          setLEDs(HIGH);
          runMotorSequence("up", 2000, 100); // reposition gently
        }
        break;

    case DOOR_CLOSING:
      if (motionState == IDLE) {
        Serial.println("Door fully closed.");
        doorState = DOOR_IDLE;
      }
      break;

    case DOOR_IDLE:
      if (openDoor != lastOpenDoor) {   // check if openDoor command has changed
      lastOpenDoor = openDoor;          // set the new openDoor command as lastOpenDoor command
        if (openDoor) startOpening();
        else startClosing();
      }
      break;
    }
}

// ============================================================
// Functions to change state of state machine and command the motor
// ============================================================
  
void startCalibration() {
  Serial.println("Calibrating: moving up until limit switch...");
  doorState = DOOR_CALIBRATING;
  creepDone = false;
  runMotorSequence("up", 8000, 100); // slower calibration
}

void startOpening() {
  chargingWaitActive = false;
  Serial.println("Opening door...");
  doorState = DOOR_OPENING;
  openDoor = true;
  runMotorSequence("up", fullSpeedUpTime, maxPWMup);
}

void startClosing() {
  chargingWaitActive = false;
  Serial.println("Closing door...");
  doorState = DOOR_CLOSING;
  openDoor = false;
  creepDone = false;
  runMotorSequence("down", fullSpeedDownTime, maxPWMdown);
}

// ============================================================
// Function: limitReached()
// Check if limitSwitch has triggered
// ============================================================

bool limitReached() {
  return digitalRead(limitSwitch) == LOW;  // invert if sensor active HIGH
}

// ============================================================
// Function: setLEDs()
// Change LED state with one command
// ============================================================

void setLEDs(bool state) {
  digitalWrite(LED1, state);
  digitalWrite(LED2, state);
}

// ============================================================
// Function: updateLED()
// control LEDs depending on door state
// ============================================================

void updateLED() {
  unsigned long now = millis();
  bool moving = (doorState == DOOR_CALIBRATING ||   // if doorState is one of these values, door is moving
                 doorState == DOOR_OPENING ||
                 doorState == DOOR_CLOSING);

  if (moving) {   // if the door is moving LEDs blink
    if (now - lastBlink >= BLINK_INTERVAL) {
      lastBlink = now;
      LEDstate = !LEDstate;
      setLEDs(LEDstate);
    }
  } else {   // Else, solid ON when open, OFF when closed
    if (doorState == DOOR_IDLE && openDoor)
      setLEDs(HIGH);
    else
      setLEDs(LOW);
  }
}

// ============================================================
// HTTP handlers
// ============================================================

// ---- setting up a formatted page to deliver the message ----

void sendPage(String title, String body) {
  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'></head><body><h2>" + title + "</h2><p>" + body + "</p>"
                "<p><a href=\"/open\">Open Door</a></p>"
                "<p><a href=\"/close\">Close Door</a></p>"
                "<p><a href=\"/status\">Check Status</a></p>"
                "</body></html>";
  server.send(200, "text/html", html);
}

// --------- "root page" offering the command links -----------
  
void handleRoot() {
  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'></head><body>";
  sendPage("Dreame Garage Door Controller", "");
}

// ------------------ "open command" page ---------------------

void handleOpen() {
  chargingWaitActive = false;
  openDoor = true;
  sendPage("Dreame Garage Door Controller", "Door <b>opening</b> command received.");
  Serial.println("HTTP: /open called");
}

// ----------------- "close command" page ---------------------

void handleClose() {
  chargingWaitActive = false;
  openDoor = false;
  sendPage("Dreame Garage Door Controller", "Door <b>closing</b> command received.");
  Serial.println("HTTP: /close called");
}

// --------------------- "status" page ------------------------

void handleStatus() {
  String status;

  switch (doorState) {
    case DOOR_IDLE:         status = "Idle"; break;
    case DOOR_CALIBRATING:  status = "Calibrating"; break;
    case DOOR_OPENING:      status = "Opening"; break;
    case DOOR_CLOSING:      status = "Closing"; break;
  }

  String body = "Door is currently: <b>" + status + "</b><br>Last state was: " + dreame_status + "<br>Current room is: " + dreame_room + "<br>Current task is: " + dreame_task + "<br> ESP reset reason: " + esp_reset_reason() + "<br>";
  sendPage("Dreame Garage Door Controller", body);
}

// ============================================================
// Function: readAdafruitValue()
// get current state of Dreame from AdafruitIO
// ============================================================

/*void readAdafruitValue() {
  HTTPClient http;

  String url = "https://io.adafruit.com/api/v2/";
  url += IO_USERNAME;
  url += "/feeds/";
  url += FEED_KEY;
  url += "/data/last";

  http.begin(url);
  http.addHeader("X-AIO-Key", IO_KEY);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();

    // Parse JSON
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (!error) {
      String value = doc["value"].as<String>();

      if (lastValue != value) {
      
        // Any change cancels pending delayed close
        chargingWaitActive = false;
      
        if ((value == "charging") || (value == "idle") || (value == "standby")) {
          Serial.println("Charging detected → will close in 5 minutes");
          chargingWaitActive = true;
          chargingWaitStart = millis();
        }
        else if (value == "sleeping") {
          Serial.println("Dreame at base → close immediately");
          openDoor = false;
        }
        else {
          Serial.println("Cleaning → open door");
          openDoor = true;
        }
      
        lastValue = value;
      }
    } else {
      Serial.print("JSON parse error: ");
      Serial.println(error.c_str());
    }
  } else {
    Serial.printf("HTTP request failed, code: %d\n", httpCode);
  }

  http.end();
}*/

// ============================================================
// Function: readAdafruitFeed()
// get value of feedKey from AdafruitIO
// ============================================================

bool readAdafruitFeed(const String& feedKey, String& outValue) {
  HTTPClient http;
  String url = "https://io.adafruit.com/api/v2/";
  url += IO_USERNAME;
  url += "/feeds/";
  url += feedKey;
  url += "/data/last";

  http.begin(url);
  http.addHeader("X-AIO-Key", IO_KEY);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    http.end();
    return false;
  }

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, http.getString());
  http.end();

  if (err) return false;

  outValue = doc["value"].as<String>();
  return true;
}

// ============================================================
// Function: pollAdafruitIO()()
// get current values of Dreame from AdafruitIO via readAdafruitFeed()
// ============================================================

void pollAdafruitIO() {
  String newState, newStatus, newRoom, newTask;
  bool changed = false;

  if (readAdafruitFeed("dreame-state", newState)) {
    if (newState != last_dreame_state) {
      last_dreame_state = newState;
      dreame_state = newState;
      changed = true;
    }
  }

  if (readAdafruitFeed("dreame-status", newStatus)) {
    if (newStatus != last_dreame_status) {
      last_dreame_status = newStatus;
      dreame_status = newStatus;
      changed = true;
    }
  }

  if (readAdafruitFeed("dreame-room", newRoom)) {
    if (newRoom != last_dreame_room) {
      last_dreame_room = newRoom;
      dreame_room = newRoom;
      changed = true;
    }
  }

  if (readAdafruitFeed("dreame-task", newTask)) {
    if (newTask != last_dreame_task) {
      last_dreame_task = newTask;
      dreame_task = newTask;
      changed = true;
    }
  }

  updatedData = changed;
}

// ============================================================
// Function: evaluateState()
// Evaluate Dreame status and take door action
// ============================================================

void evaluateState() {
  if (!updatedData) return;
  updatedData = false;

  // Any change cancels pending delayed close
  chargingWaitActive = false;

  if (dreame_state == "auto_emptying") {
    Serial.println("Dreame auto-emptying → close immediately");
    openDoor = false;
    emptying = true;
  }

  if (emptying && (dreame_state != "auto_emptying")) {
    Serial.println("Dreame auto-emptying finished → open again");
    openDoor = true;
  }

  if (dreame_room != "Kitchen") {
    Serial.println("Dreame not in Kitchen → close immediately");
    openDoor = false;
  }
  else {      
    if ((dreame_status == "charging") || (dreame_status == "idle") || (dreame_status == "standby")) {
      Serial.println("Charging detected → will close in 5 minutes");
      chargingWaitActive = true;
      chargingWaitStart = millis();
    }
    else if (dreame_status == "sleeping") {
      Serial.println("Dreame at base → close immediately");
      openDoor = false;
    }
    else {
      Serial.println("Cleaning → open door");
      openDoor = true;
    }
  }
}



