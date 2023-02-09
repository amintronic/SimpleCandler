#include "NotoSansBold15.h"
#include "NotoSansBold36.h"
#include "Unicode_Test_72.h"
#include "TFT_eSPI.h"
#include "max6675.h"
#include "PID_v1.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

const char* host = "candler01";
const char* ssid = "exNoon";
const char* password = "NooneGhandi@2019";

WebServer server(80);

/*
 * Login page
 */

const char* loginIndex =
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>ESP32 Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
             "<td>Username:</td>"
             "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='admin' && form.pwd.value=='admin')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";

/*
 * Server Index Page
 */

const char* serverIndex =
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')"
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";


TFT_eSPI lcd = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&lcd);

uint16_t rgb_to_rgb16(float r, float g, float b);

#define PIN_POWER_ON      15
// Rotary Encoder Inputs
#define SW 11
#define DT 12
#define CLK 13

#define _HEAT_CTRL_PIN    43
#define _BUZZER_PIN       1

#define _TH_CLK           16
#define _TH_SO            17
#define _TH_CS            21

#define _B_NOT_SELECTED   rgb_to_rgb16(.5, .5, .5)
#define _B_SELECTED       rgb_to_rgb16(.0, .4, .0)
#define _B_TO_MODIFY      rgb_to_rgb16(.9, .9, .0)
#define _F_NOT_SELECTED   rgb_to_rgb16(.75, .75, .75)
#define _F_SELECTED       rgb_to_rgb16(1., 1., 1.)
#define _F_TO_MODIFY      rgb_to_rgb16(.0, .0, .0)

#define _HEAT_FORWARD     false
#define _HEAT_BACKWARD    true

uint8_t rotaryDelta = 100;
uint32_t heatAlarmWait = 0;
bool heatingDirection = false;
bool rotaryDir;
bool rotaryRolledFlag = false;
bool buttonState = false, buttonLongPressed = false;
bool setAlarmFlag = false;
bool aroundTargetTemp = false, reachedTargetTemp = false;

uint8_t tempSens = 100;
uint8_t tempMem[3] = {185, 175, 140};
uint8_t tempMemSelect[3] = {0, 0, 0};
int8_t tempModifySelect = 0;

uint32_t lastButtonPress, lastRotaryUpdate, lastGUIupdate, aroundTargetTempTime;
uint32_t timer_1, timer_heater, timer_pid, timer_alarm;

int dutyCycle = -1;

MAX6675 thermocouple(_TH_CLK, _TH_CS, _TH_SO);

/* Filter */
#define _FIR  4
double InputList[_FIR];
uint8_t InputList_p = 0;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
// double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=40, consKi=0.2, consKd=40;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);


void setup() {
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  Serial.begin(115200);
  Serial.println("Program Started ...");

  lcd.init();
  lcd.fillScreen(TFT_BLACK);
  lcd.setRotation(3);    // 0=Portrait, 1=landscape

  sprite.createSprite(320, 170);
  sprite.setSwapBytes(true);

  Input = thermocouple.readFahrenheit();
  Setpoint = 0;
  myPID.SetMode(MANUAL);

  pinMode(_HEAT_CTRL_PIN, OUTPUT);
  digitalWrite(_HEAT_CTRL_PIN, LOW);
  pinMode(_BUZZER_PIN, OUTPUT);
  digitalWrite(_BUZZER_PIN, LOW); 

  // ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  // ledcAttachPin(_HEAT_CTRL_PIN, PWMChannel); 
  // ledcWrite(PWMChannel, 512);

  // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, _HEAT_CTRL_PIN); // To drive a RC servo, one MCPWM generator is enough
  // mcpwm_config_t pwm_config = {
  //     .frequency = 15,
  //     .cmpr_a = 0,
  //     .duty_mode = MCPWM_DUTY_MODE_0,
  //     .counter_mode = MCPWM_UP_COUNTER,
  // };
  // mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50);
  // mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 10000);

	pinMode(CLK, INPUT_PULLUP);
	pinMode(DT, INPUT_PULLUP);
	pinMode(SW, INPUT_PULLUP);
  attachInterrupt(CLK, readEncoder, CHANGE);
  attachInterrupt(DT, readEncoder, CHANGE);

  WiFi.begin(ssid, password);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

  // Serial.println("Running ...");
  // delay(1000);
  server.handleClient();

  if (millis() - timer_alarm > 2500 && setAlarmFlag)
  {
    digitalWrite(_BUZZER_PIN, LOW);
    setAlarmFlag = false;
    timer_alarm = millis();
  }
  else if (millis() - timer_alarm > 2000 && setAlarmFlag)
    digitalWrite(_BUZZER_PIN, HIGH);
  else if (millis() - timer_alarm > 1500 && setAlarmFlag)
    digitalWrite(_BUZZER_PIN, LOW);
  else if (millis() - timer_alarm > 1000 && setAlarmFlag)
    digitalWrite(_BUZZER_PIN, HIGH);
  else if (millis() - timer_alarm > 500 && setAlarmFlag)
    digitalWrite(_BUZZER_PIN, LOW);

  if (millis() - timer_pid >= 250)
  {
    InputList[InputList_p++] = thermocouple.readFahrenheit();
    if (InputList_p >= _FIR)  InputList_p = 0;
    Input = 0;
    for (uint8_t i = 0; i < _FIR; i++)
      Input += InputList[i];
    Input /= _FIR;

    if (tempMemSelect[0] == 1 || tempMemSelect[1] == 1 || tempMemSelect[2] == 1)
    {
      if (reachedTargetTemp == false)
      {
        if ((heatingDirection == _HEAT_FORWARD  && Input >= (Setpoint - 3) && Input < (Setpoint + 5)) ||
            (heatingDirection == _HEAT_BACKWARD && Input <= (Setpoint + 2))) {
          if (aroundTargetTemp == false)
          {
            aroundTargetTemp = true;
            heatAlarmWait = (heatingDirection == _HEAT_FORWARD) ? 180000 : 0;
            aroundTargetTempTime = millis();
          }
        }
        else
        {
          aroundTargetTemp = false;
        }

        if (aroundTargetTemp == true && (millis() - aroundTargetTempTime) > heatAlarmWait)
        {
          reachedTargetTemp = true;
          setAlarm();
        }
      }
      
      // double gap = abs(Setpoint - Input);
      // if (gap < 10)
      //   myPID.SetTunings(consKp, consKi, consKd);
      // else
      //   myPID.SetTunings(aggKp, aggKi, aggKd);
      myPID.Compute();
      // Serial.print("Setpoint: ");
      // Serial.print(Setpoint);
      // Serial.print("Input: ");
      // Serial.print(Input);
      // Serial.print(" Output: ");
      // Serial.println(Output);
      dutyCycle = Output;
    }
    else {
      reachedTargetTemp = false;      
    }
    timer_pid = millis();
  }

  if (dutyCycle == 0 || (tempMemSelect[0] != 1 && tempMemSelect[1] != 1 && tempMemSelect[2] != 1)) {
    digitalWrite(_HEAT_CTRL_PIN, LOW);
  }
  else
  {
    if (millis() - timer_heater > 500)
    {
      digitalWrite(_HEAT_CTRL_PIN, HIGH);
      timer_heater = millis();
    }
    else if (millis() - timer_heater > dutyCycle)
    {
      digitalWrite(_HEAT_CTRL_PIN, LOW);
    }
  }

  if (rotaryRolledFlag)
  {
    rotaryRolledFlag = false;
    rotaryRolled();
  }
  buttonRead();

  if (millis() - lastGUIupdate > 33)
  {
    sprite.fillSprite(TFT_BLACK);
    // sprite.fillRoundRect(10, 10, 30, 30, 3, rgb_to_rgb16(.1, .1, .1));

    makeTempMemory(3, 13, tempMemSelect[0], tempModifySelect == 0, tempMem[0]);
    makeTempMemory(3, 63, tempMemSelect[1], tempModifySelect == 1, tempMem[1]);
    makeTempMemory(3, 113, tempMemSelect[2], tempModifySelect == 2, tempMem[2]);
    
    sprite.unloadFont();
    sprite.loadFont(NotoSansBold36);
    sprite.setTextDatum(TL_DATUM);
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.drawString(String("F"), 265, 63);
    sprite.unloadFont();
    sprite.loadFont(Unicode_Test_72);
    sprite.setTextDatum(TR_DATUM);
    sprite.drawString(String(int(Input)), 250, 60);
    sprite.drawCircle(260, 60, 4, TFT_WHITE);
    sprite.drawCircle(260, 60, 3, TFT_WHITE);
    sprite.pushSprite(0, 0);
    lastGUIupdate = millis();
  }

  delay(1);
}

void setAlarm()
{
  setAlarmFlag = true;
  timer_alarm = millis();
  digitalWrite(_BUZZER_PIN, HIGH);
}

void makeTempMemory(uint8_t x, uint8_t y, uint8_t select, bool modifiable, uint8_t value)
{
  uint16_t b_color, f_color, r_color;

  if (select == 0) {b_color = _B_NOT_SELECTED; f_color = _F_NOT_SELECTED;}
  else if (select == 1) {b_color = _B_SELECTED; f_color = _F_SELECTED;}
  else if (select == 2) {b_color = _B_TO_MODIFY; f_color = _F_TO_MODIFY;}
  if (modifiable) r_color = TFT_RED;
  else r_color = TFT_BLACK;
  sprite.fillSmoothRoundRect(x, y, 99, 44, 4, r_color);
  // sprite.drawRoundRect(2, 12, 101, 46, 4, r_color);
  sprite.fillSmoothRoundRect(x+2, y+2, 95, 40, 4, b_color);
  sprite.loadFont(NotoSansBold36);  // NotoSansBold15, NotoSansBold36, Unicode_Test_72
  sprite.setTextDatum(TR_DATUM);
  sprite.setTextColor(f_color, b_color);
  sprite.drawString(String(value), x+70, y+7);
  sprite.unloadFont();
  sprite.loadFont(NotoSansBold15);
  sprite.setTextDatum(TL_DATUM);
  sprite.drawString(String("F"), x+82, y+9);
  sprite.drawCircle(x+77, y+10, 2, f_color);
}

void rotaryRolled()
{
  if (tempMemSelect[0] != 2 && tempMemSelect[1] != 2 && tempMemSelect[2] != 2)
  {
    if (rotaryDir == false)
    {
      tempModifySelect++;
      if (tempModifySelect > 2)
        tempModifySelect = 0;
    }
    else
    {
      tempModifySelect--;
      if (tempModifySelect < 0)
        tempModifySelect = 2;
    }
  }
}

void buttonUp()
{
  if (tempModifySelect == 0)
  {
    if (tempMemSelect[0] == 1 || tempMemSelect[0] == 2) {
      tempMemSelect[0] = 0;
      myPID.SetMode(MANUAL);
      reachedTargetTemp = false;
      aroundTargetTemp = false;
    }      
    else if (tempMemSelect[0] == 0) {
      tempMemSelect[0] = 1; tempMemSelect[1] = 0; tempMemSelect[2] = 0;
      Setpoint = tempMem[0];
      heatingDirection = (Input < Setpoint) ? false : true;
      reachedTargetTemp = false;
      myPID.SetMode(AUTOMATIC);
    }
  }
  else if (tempModifySelect == 1)
  {
    if (tempMemSelect[1] == 1 || tempMemSelect[1] == 2) {
      tempMemSelect[1] = 0;
      myPID.SetMode(MANUAL);
      reachedTargetTemp = false;
      aroundTargetTemp = false;
    }
    else if (tempMemSelect[1] == 0) {
      tempMemSelect[0] = 0; tempMemSelect[1] = 1; tempMemSelect[2] = 0;
      Setpoint = tempMem[1];
      heatingDirection = (Input < Setpoint) ? false : true;
      reachedTargetTemp = false;
      myPID.SetMode(AUTOMATIC);
    }
  }
  else
  {
    if (tempMemSelect[2] == 1 || tempMemSelect[2] == 2) {
      tempMemSelect[2] = 0;
      myPID.SetMode(MANUAL);
      reachedTargetTemp = false;
      aroundTargetTemp = false;
    }
    else if (tempMemSelect[2] == 0) {
      tempMemSelect[0] = 0; tempMemSelect[1] = 0; tempMemSelect[2] = 1;
      Setpoint = tempMem[2];
      heatingDirection = (Input < Setpoint) ? false : true;
      reachedTargetTemp = false;
      myPID.SetMode(AUTOMATIC);
    }
  }
  // Serial.println("Button Up!");
}

void buttonLongPress()
{
  tempMemSelect[tempModifySelect] = 2;
  for (int i = 0; i < 3; i++)
    if (i != tempModifySelect)
      tempMemSelect[i] = 0;
  // Serial.println("Button Long Press!");
}

void buttonRead()
{
  if (buttonState == false && digitalRead(SW) == LOW)
  {
    if (millis() - lastButtonPress > 50) {
      buttonState = true;
    }
    lastButtonPress = millis();
  }
  else if (buttonState == true && digitalRead(SW) == LOW && (millis() - lastButtonPress > 1000))
  {
    buttonLongPressed = true;
    buttonLongPress();
    lastButtonPress = millis();
  }
  else if (buttonState == true && digitalRead(SW) == HIGH)
  {
    if (millis() - lastButtonPress > 50 && buttonLongPressed == false) {
      buttonState = false;
      buttonUp();
    }
    if (buttonLongPressed)
    {
      buttonState = false;
      buttonLongPressed = false;
    }
    lastButtonPress = millis();
  }
}

void readEncoder() {
  static uint8_t state = 0;
  bool CLKstate = digitalRead(CLK);
  bool DTstate = digitalRead(DT);
  switch (state) {
      case 0:                         // Idle state, encoder not turning
          if (!CLKstate){             // Turn clockwise and CLK goes low first
              state = 1;
          } else if (!DTstate) {      // Turn anticlockwise and DT goes low first
              state = 4;
          }
          break;
      // Clockwise rotation
      case 1:                     
          if (!DTstate) {             // Continue clockwise and DT will go low after CLK
              state = 2;
          } 
          break;
      case 2:
          if (CLKstate) {             // Turn further and CLK will go high first
              state = 3;
          }
          break;
      case 3:
          if (CLKstate && DTstate) {  // Both CLK and DT now high as the encoder completes one step clockwise
              state = 0;
              // if (millis() - lastRotaryUpdate > 80)
              {
                if (tempMemSelect[0] == 2 || tempMemSelect[1] == 2 || tempMemSelect[2] == 2)
                  tempMem[tempModifySelect] -= 5;
                if (tempMem[tempModifySelect] < 50)
                  tempMem[tempModifySelect] = 50;
                rotaryDir = false;
                rotaryRolledFlag = true;
                // lastRotaryUpdate = millis();
              }
          }
          break;
      // Anticlockwise rotation
      case 4:                         // As for clockwise but with CLK and DT reversed
          if (!CLKstate) {
              state = 5;
          }
          break;
      case 5:
          if (DTstate) {
              state = 6;
          }
          break;
      case 6:
          if (CLKstate && DTstate) {
              state = 0;
              // if (millis() - lastRotaryUpdate > 80)
              {
                if (tempMemSelect[0] == 2 || tempMemSelect[1] == 2 || tempMemSelect[2] == 2)
                  tempMem[tempModifySelect] += 5;
                if (tempMem[tempModifySelect] > 210)
                  tempMem[tempModifySelect] = 210;
                rotaryDir = true;
                rotaryRolledFlag = true;
                // lastRotaryUpdate = millis();
              }
          }
          break; 
  }
}

uint16_t rgb_to_rgb16(float r, float g, float b)
{
  r *= 31.0;
  g *= 63.0;
  b *= 31.0;
  return uint16_t(((uint16_t(r) & 0x001F) << 11) + ((uint16_t(g) & 0x003F) << 5) + (uint16_t(b) & 0x001F));
}

