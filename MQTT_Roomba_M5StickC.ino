#include <M5StickC.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

#include "Roomba.h"

// WiFi 設定
const char ssid[] = "";
const char password[] = "";
WiFiClient wifiClient;

// MQTT 設定
const char* mqttBrokerAddr = "";
const char* mqttUserName = "";
const char* mqttPassword = "";
const int mqttPort = 1883;
const char* mqttClientID = "Roomba";
PubSubClient mqttClient(mqttBrokerAddr, mqttPort, wifiClient);

// Roombaインスタンス
Roomba roomba(Serial1, 100);

// グローバル変数
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;
float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;
float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;
boolean sending = false;
unsigned long lastUpdateTime = 0;
int lastWiFiStatus;
byte oimode = Roomba::OI_OFF;

void setup() {
  M5.begin();
  M5.IMU.Init();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.println("  X       Y       Z");
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.println("  Pitch   Roll    Yaw");
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 0, 26); // EXT_IO
  roomba.start();
  oimode = Roomba::OI_PASSIVE;
  WiFi.begin(ssid, password);
  WiFi.begin();
  lastWiFiStatus = WiFi.status();
  Serial.println();
  mqttClient.setCallback(mqttCallback);

  // 別タスクの起動
  xTaskCreatePinnedToCore(task0, "Task0", 4096, NULL, 1, NULL, 0);

}

// MQTTの再接続に時間がかかるみたいなので、別タスクで動かす
void task0(void* param)
{
  while (true) {
    if (checkWifi()) {
      if (checkMQTT()) {
//        M5.dis.drawpix(0, 0x0000ff);  // WIFI,MQTT接続は青
        mqttClient.loop();
      } else {
//        M5.dis.drawpix(0, 0xffff00);  // MQTT未接続は黄色
        vTaskDelay(1000); // 再接続まで間隔を開ける
      }
    } else {
//      M5.dis.drawpix(0, 0x00ff00);  // Wifi切断は赤（色はなぜかBRG順）
    }
    vTaskDelay(1); // タスク内の無限ループには必ず入れる
  }
}

void loop() {
  unsigned long t = millis();
  M5.update();
  // 500msごとに更新
  if (t - lastUpdateTime >= 500) {
    lastUpdateTime = t;
    readSensor(); // センサー読み取り
    updateLcd();  // LCD更新
    if (sending) {
      sendSensorData(); // センサーデータ送信
    }
  }
  // ボタンBが押されたら送信の有効無効を切り替える
  if (M5.BtnB.wasPressed()) {
    sending = !sending;
  }
  static unsigned long pressms = 0;
  // ボタンAが押されたら
  if (M5.BtnA.wasPressed()) {
    // 即送信する
    mqttClient.publish("roomba/out/ButtonA", "Pressed");
    roomba.start();
    oimode = Roomba::OI_PASSIVE;
    pressms = millis();
  }
  // ボタンAが離されたら
  if (M5.BtnA.wasReleased()) {
    // 即送信する
    mqttClient.publish("roomba/out/ButtonA", "Released");
    if (millis() - pressms >= 2000) {
      roomba.stop(); // 長押しで操作モード終了
      oimode = Roomba::OI_OFF;
    }
  }
  if (Serial.available() > 0) {
    int c = Serial.read();
    switch (c) {
      case '1':
        Serial.println("Start");
        roomba.start();
        break;
      case '2':
        Serial.println("Reset");
        roomba.reset();
        break;
      case '3':
        Serial.println("Stop");
        roomba.stop();
        break;
      case '4':
        Serial.println("Safe");
        roomba.safe();
        break;
      case '5':
        Serial.println("Full");
        roomba.full();
        break;
      case '6': // motor forward
        Serial.println("motor forward");
        roomba.driveDirect(200, 200);
        break;
      case '7': // motor stop
        Serial.println("motor stop");
        roomba.driveDirect(0, 0);
        break;
      case '8': // motor back
        Serial.println("motor back");
        roomba.driveDirect(-200, -200);
        break;
      case '9': // dock
        Serial.println("seek dock");
        roomba.seekDock();
        break;
      default:
        break;
    }
  }
  static unsigned long lastsensorms = 0;
  if (t - lastsensorms >= 100) {
    lastsensorms = t;
    if (oimode != Roomba::OI_OFF) {
      oimode = roomba.getOIMode();
      if (!roomba.isTimeoutError()) {
        Serial.print("oimode: ");
        Serial.print(oimode);
      }
      static byte lastBumps = 0;
      static byte lastDrops = 0;
      byte bumpsdrops = roomba.getBumpsAndWheelDrops();
      if (!roomba.isTimeoutError()) {
        byte bumps = bumpsdrops & Roomba::BUMP_MASK;
        byte drops = (bumpsdrops & Roomba::DROP_MASK) >> 2;
        Serial.print(", bumps: ");
        Serial.print(bumps);
        Serial.print(", drops: ");
        Serial.println(drops);
        if (bumps != lastBumps) {
          lastBumps = bumps;
          mqttClient.publish("roomba/out/bumps", String(bumps).c_str());
        }
        if (drops != lastDrops) {
          lastDrops = drops;
          mqttClient.publish("roomba/out/drops", String(drops).c_str());
        }
      } else {
        Serial.println("");
      }
    }
  }
  // 受信データの読み飛ばし
  if (Serial1.available() > 0) {
    char buf[256];
    int n = 0;
    while (Serial1.available() > 0) {
      int n = Serial1.available();
      if (n >= 256) {
        n = 255;
      }
      Serial1.readBytes(buf, n);
      buf[n] = 0;
      Serial.print(buf);
      delay(10);
    }
    // 最後が改行でない場合だけ改行を付加する
    if(n > 0 && buf[n - 1] != '¥n') {
      Serial.println("");
    }
  }
}

void readSensor() {
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  M5.IMU.getAhrsData(&pitch, &roll, &yaw);
}
void sendSensorData() {
  mqttClient.publish("roomba/out/AccX", String(accX).c_str());
  mqttClient.publish("roomba/out/AccY", String(accY).c_str());
  mqttClient.publish("roomba/out/AccZ", String(accZ).c_str());
  mqttClient.publish("roomba/out/GyroX", String(gyroX).c_str());
  mqttClient.publish("roomba/out/GyroY", String(gyroY).c_str());
  mqttClient.publish("roomba/out/GyroZ", String(gyroZ).c_str());
  mqttClient.publish("roomba/out/Pitch", String(pitch).c_str());
  mqttClient.publish("roomba/out/Roll", String(roll).c_str());
  mqttClient.publish("roomba/out/Yow", String(yaw).c_str());
}

void updateLcd() {
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print(sending ? "Sending" : "       ");
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", gyroX, gyroY, gyroZ);
  M5.Lcd.setCursor(140, 30);
  M5.Lcd.print("o/s");
  M5.Lcd.setCursor(0, 40);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", accX, accY, accZ);
  M5.Lcd.setCursor(140, 40);
  M5.Lcd.print("G");
  M5.Lcd.setCursor(0, 60);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", pitch, roll, yaw);
  M5.Lcd.setCursor(0, 70);
  M5.Lcd.print("               ");
  M5.Lcd.setCursor(0, 70);
  if (WiFi.status() == WL_CONNECTED) {
    M5.Lcd.print(WiFi.localIP());
  } else {
    M5.Lcd.print("WiFi:Disconnect");
  }
  M5.Lcd.setCursor(100, 70);
  M5.Lcd.print(mqttClient.connected() ? "MQTT:Conn" : "MQTT:Disc");
}

boolean checkWifi() {
  static int lastWiFiStatus = WL_IDLE_STATUS;
  int wifiStatus = WiFi.status();
  if (lastWiFiStatus != WL_CONNECTED && wifiStatus == WL_CONNECTED) {
    printSomewhere("WiFi connected.");
    printSomewhere("IP address: ");
    printSomewhere(WiFi.localIP().toString().c_str());
  } else if (lastWiFiStatus != wifiStatus && wifiStatus != WL_CONNECTED) {
    printSomewhere("WiFi disconnected.");
    printSomewhere(wifiStatus);
    if (wifiStatus == WL_CONNECT_FAILED) {
      WiFi.begin(ssid, password);
    }
  }
  lastWiFiStatus = wifiStatus;
  return (wifiStatus == WL_CONNECTED);
}

boolean checkMQTT() {
  unsigned long t = millis();
  boolean conn = mqttClient.connected();
  if (!conn) {
    printSomewhere("MQTT Disconnect. Connecting...");
    conn = mqttClient.connect(mqttClientID, mqttUserName, mqttPassword);
    if (conn) {
      printSomewhere("MQTT Connect OK.");
      mqttClient.subscribe("roomba/in/#");
    } else {
      printSomewhere("MQTT Connect failed, rc=");
      // http://pubsubclient.knolleary.net/api.html#state に state 一覧が書いてある
      printSomewhere(mqttClient.state());
    }
  }
  return conn;
}

String payloadToString(const byte* payload, unsigned int length) {
  char s[length + 1];
  strncpy(s, (char*)payload, length);
  s[length] = '\0';
  return String(s);
}
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // MQTTトピックが来たときに呼ばれる
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String p = payloadToString(payload, length);
  Serial.println(p);
  String t(topic);
  if (!t.startsWith("roomba/in/")) {
    return;
  }
  t.remove(0, strlen("roomba/in/"));
  Serial.println(t);
  if (t.equals("drive")) {
    // payload:/区切り 例: 100/-100
    int i = p.indexOf("/");
    String strL = p.substring(0, i);
    String strR = p.substring(i + 1);
    int l = strL.toInt();
    int r = strR.toInt();
    Serial.println(String("L: ") + String(l) + String(", R: ") + String(r));
    roomba.full();
    roomba.driveDirect(l, r);
  }
  if (t.equals("mainbrush")) {
    if (p.equals("on")) {
      checkOIFull();
      roomba.mainBrushOn();
    } else if (p.equals("off")) {
      checkOIFull();
      roomba.mainBrushOff();
    }
  }
  if (t.equals("sidebrush")) {
    if (p.equals("on")) {
      checkOIFull();
      roomba.sideBrushOn();
    } else if (p.equals("off")) {
      checkOIFull();
      roomba.sideBrushOff();
    }
  }
  if (t.equals("vacuum")) {
    if (p.equals("on")) {
      checkOIFull();
      roomba.vacuumOn();
    } else if (p.equals("off")) {
      checkOIFull();
      roomba.vacuumOff();
    }
  }
  if (t.equals("command")) {
    if (p.equals("clean")) {
      roomba.clean();
    }
    if (p.equals("dock")) {
      roomba.seekDock();
    }
    if (p.equals("start")) {
      roomba.start();
    }
    if (p.equals("stop")) {
      roomba.full(); // CleanやSeekDockを止めるため
      delay(100);
      roomba.stop();
    }
  }
}

void checkOIFull() {
    if (oimode != Roomba::OI_FULL) {
      roomba.full();
      delay(100);
    }
}
void checkOISafe() {
    if (oimode != Roomba::OI_SAFE) {
      roomba.safe();
      delay(100);
    }
}

//-------------------------------
//  Print for Debug
//-------------------------------
void initPrintSomewhere(void)
{
}
void printSomewhere(int num)
{
  char strx[128] = {0};
  sprintf(strx, "%d", num);
  Serial.println(strx);
  //M5.Lcd.printf("%s",strx);
}
void printSomewhere(const char* txt)
{
  Serial.println(txt);
  //M5.Lcd.printf(txt);
}
