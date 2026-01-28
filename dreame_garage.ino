  /*************************************************************

   Dreame Garage Door v1.3
   Reto Huber
   2026
   for Xiao ESP32C3 controller
   
 *************************************************************/

#include <WiFi.h>
#include <HTTPClient.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <esp_task_wdt.h>
#include <time.h>

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
unsigned long fullSpeedMotorTime = 0;       // global variable for time motor runs at full speed
const unsigned long fullSpeedUpTime = 1850;    // constant for time motor runs up
const unsigned long fullSpeedDownTime = 2100;  // constant for time motor runs down
int maxPWM = 0;                              // global variable for maximum speed
const int maxPWMup = 160;                      // constant for maximum speed up
const int maxPWMdown = 120;                    // constant for maximum speed down
int pwmValue = 0;                              // global variable for current speed
unsigned long stateStartTime = 0;              // timing variable for motor motion phases (IDLE, ACCEL, FULLSPEED, DECEL)
bool watchdogNightMode = false;                // track current watchdog mode

bool openDoor = false;      // door action to perform
bool lastOpenDoor = false;  // last door action
bool creepDone = false;

// ---------------- LED Blink Variables ----------------
bool LEDstate = LOW;
unsigned long lastBlink = 0;
const unsigned long BLINK_INTERVAL = 200;  // ms

// ------------ Adafruit IO poll variables -------------
unsigned long lastPoll = 0;                   // time of last poll to Adafruit IO
const unsigned long POLL_INTERVAL = 2000;     // interval in ms to poll Adafruit IO
String dreame_state = "";                     // current state of Dreame
String dreame_status = "";                    // current status of Dreame
String dreame_room = "";                      // current room Dreame is in
String dreame_task = "";                      // current task of Dreama
String last_dreame_state = "";                // last state of Dreame
String last_dreame_status = "";               // last status of Dreame
String last_dreame_room = "";                 // last room Dreame was in
String last_dreame_task = "";                 // last task of Dreame
bool updatedData = false;                     // Flag to indicate new data arrived
int lastAIOerror = 0;
unsigned long lastIOupdate = 0;               // time of last update from Adafruit IO
unsigned long timeBetweenUpdates = 0;         // time between Adafruit IO updates
struct tm lastResetTime;                      // last time controller was booted

// -------------------- Web Server ---------------------
AsyncWebServer server(80);

void setupTime();
void configureWatchdog();
void updateWatchdogByTime();
void pollAdafruitIO();
String buildPage();
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

  setupTime();

  // ---------- Setup handlers for Webserver ------------

  // --------- "root page" offering the command links -----------
    
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html",
                  buildPage("Dreame Garage Door Controller", ""));
  });

  // ------------------ "open command" page ---------------------

  server.on("/open", HTTP_GET, [](AsyncWebServerRequest *request) {
    openDoor = true;
    request->send(200, "text/html",
                  buildPage("Dreame Garage Door Controller", "Door <b>opening</b> command received."));
  });

  // ----------------- "close command" page ---------------------

  server.on("/close", HTTP_GET, [](AsyncWebServerRequest *request) {
    openDoor = false;
    request->send(200, "text/html",
                  buildPage("Dreame Garage Door Controller", "Door <b>closing</b> command received."));
  });

  // --------------------- "status" page ------------------------

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    String status;
    switch (doorState) {
      case DOOR_IDLE:        status = "Idle"; break;
      case DOOR_CALIBRATING: status = "Calibrating"; break;
      case DOOR_OPENING:     status = "Opening"; break;
      case DOOR_CLOSING:     status = "Closing"; break;
    }

    struct tm timeinfo;
    char buf1[32];
    char buf2[32];
    getLocalTime(&timeinfo);
    strftime(buf1, sizeof(buf1), "%H:%M", &timeinfo);
    strftime(buf2, sizeof(buf2), "%H:%M", &lastResetTime);

    String body = "Door is currently: " + status + "<br>Last state was: " + dreame_status + "<br>Current room is: " + dreame_room + "<br>Current task is: " + dreame_task + "<br> ESP reset reason: " + esp_reset_reason() + "<br>Last reset time: " + String(buf2) + "<br>Last AdafruitIO error: " + lastAIOerror + "<br> Time between AIO updates (ms): " + timeBetweenUpdates + "<br>Current time: " + String(buf1) +"<br>";

    request->send(200, "text/html",
                  buildPage("Dreame Garage Door Controller", body));
    });               

  server.begin();
  Serial.println("HTTP server started.");
   

  // ----------------- Start door calibration ---------------------

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
    updateWatchdogByTime(); // update watchdog periodically
  }
  
  updateDoor();            // Command the door
  updateLED();             // Command the LEDs
  ArduinoOTA.handle();     // Handle OTA
  esp_task_wdt_reset();    // reset watchdog
}

// ============================================================
// Function: setupTime()
// Setup NTP time
// ============================================================

 void setupTime() {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    
    // Central European Time with automatic DST
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
    tzset();
  
    Serial.println("Waiting for NTP time sync...");
    struct tm timeinfo;
    while (true) {
      if (getLocalTime(&timeinfo)) {
        if (timeinfo.tm_year > (2025 - 1900)) break;  // real time arrived
      }
      Serial.print(".");
      delay(500);
    }
    delay(5000);
    Serial.println("\nTime synchronized");
    getLocalTime(&lastResetTime);
 }

// ============================================================
// Function: configureWatchdog(uint32_t timeoutSeconds)
// helper to reconfigure watchdog safely
// ============================================================

void configureWatchdog(uint32_t timeoutSeconds) {
  esp_task_wdt_delete(NULL);          // remove current task
  esp_task_wdt_deinit();              // stop watchdog
  esp_task_wdt_init(timeoutSeconds, true);  // restart with new timeout
  esp_task_wdt_add(NULL);             // re-add loop task

  Serial.print("Watchdog timeout set to ");
  Serial.print(timeoutSeconds);
  Serial.println(" seconds");
}

// ============================================================
// Function: updateWatchdogByTime()
// switch watchdog automatically based on time
// ============================================================

void updateWatchdogByTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;

  int hour = timeinfo.tm_hour;

  bool nightNow = (hour >= 20 || hour < 8);

  if (nightNow != watchdogNightMode) {
    watchdogNightMode = nightNow;

    if (nightNow)
      configureWatchdog(3600);
    else
      configureWatchdog(60);
  }
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
  Serial.println("Opening door...");
  doorState = DOOR_OPENING;
  openDoor = true;
  runMotorSequence("up", fullSpeedUpTime, maxPWMup);
}

void startClosing() {
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
// HTML page builder
// ============================================================

// ---- setting up a formatted page to deliver the message ----

String buildPage(String title, String body) {
  return
    "<html><head>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "</head><body>"
    "<h2>" + title + "</h2>"
    "<p>" + body + "</p>"
    "<p><a href=\"/open\">Open Door</a></p>"
    "<p><a href=\"/close\">Close Door</a></p>"
    "<p><a href=\"/status\">Check Status</a></p>"
    "</body></html>";
}

// ============================================================
// Function: pollAdafruitIO()()
// get current values of Dreame from AdafruitIO
// ============================================================

void pollAdafruitIO() {
  bool changed = false;

  HTTPClient http;
  String url = "https://io.adafruit.com/api/v2/";
  url += IO_USERNAME;
  url += "/groups/";
  url += AIO_GROUP_KEY;
  url += "/feeds";

  http.begin(url);
  http.addHeader("X-AIO-Key", IO_KEY);

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    lastAIOerror = httpCode;
    http.end();
    return;
  }

  StaticJsonDocument<2048> doc;
  DeserializationError err = deserializeJson(doc, http.getString());
  http.end();

  if (err) {
    lastAIOerror = -1;   // JSON error
    return;
  }

  for (JsonObject feed : doc.as<JsonArray>()) {
    if (feed["last_value"].isNull()) continue;

    String key   = feed["key"].as<String>();
    String value = feed["last_value"].as<String>();

    if (key == "dreame-state") {
      timeBetweenUpdates = millis() - lastIOupdate;
      lastIOupdate = millis();
      if (value != last_dreame_state) {
        last_dreame_state = value;
        dreame_state = value;
        changed = true;
      }
    }
    else if (key == "dreame-status") {
      if (value != last_dreame_status) {
        last_dreame_status = value;
        dreame_status = value;
        changed = true;
      }
    }
    else if (key == "dreame-room") {
      if (value != last_dreame_room) {
        last_dreame_room = value;
        dreame_room = value;
        changed = true;
      }
    }
    else if (key == "dreame-task") {
      if (value != last_dreame_task) {
        last_dreame_task = value;
        dreame_task = value;
        changed = true;
      }
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
  
  if (dreame_room != "Kitchen") {
    Serial.println("Dreame not in Kitchen → close");
    openDoor = false;
    return;
  }  

  if (dreame_state == "auto_emptying") {
    Serial.println("Dreame auto-emptying → close");
    openDoor = false;
    return;
  }

  if (((dreame_status == "charging") ||
       (dreame_status == "idle") ||
       (dreame_status == "standby") ||
       (dreame_status == "sleeping") ||
       (dreame_status == "unavailable")) &&
       (dreame_task != "cleaning")) {
    Serial.println("Dreame is docked and task completed → close");
    openDoor = false;
  }
  else {
    Serial.println("Dreame is in Kitchen and still working → open");
    openDoor = true;
  }
}
