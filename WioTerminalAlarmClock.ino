#include <StreamLib.h> // 
#include <ArduinoJson.h> // see https://arduinojson.org/
#include <rpcWiFi.h> // see https://github.com/Seeed-Studio/Seeed_Arduino_rpcWiFi
#include <millisDelay.h> // see https://github.com/ansonhe97/millisDelay
#include "RTC_SAMD51.h" // see https://github.com/Seeed-Studio/Seeed_Arduino_RTC
#include <Seeed_FS.h> // see https://github.com/Seeed-Studio/Seeed_Arduino_FS 
#include "SD/Seeed_SD.h"

// based on the examples at https://wiki.seeedstudio.com/Wio-Terminal-Wi-Fi/

// define variables to store the WiFi SSID and PSK:
char ssid[33];
char password[255];
// these will be loaded from "WiFi.json" on the SD card
// the expected content of the WiFi.json is:
// {"ssid":"myWiFiSSID","psk":"myPassword"}

millisDelay updateDelay; // the update delay object. used for ntp periodic update.
millisDelay serialDelay; // the serial delay object. used for updates to the serial port.
millisDelay displayDelay; // the display delay object. used for updating the display.
// The Normal, Short and Long delays, in milliseconds:
const unsigned long DelayNormal = 60 * 60 * 1000;
const unsigned long DelayShort = 5 * 60 * 1000;
const unsigned long DelayLong = 8 * 60 * 60 * 1000;
unsigned int synchCount = 0;
const unsigned int SynchCountNormal = 2;
const unsigned int SynchCountLong = SynchCountNormal + 3;
const unsigned int SynchCountReset = SynchCountLong + 5;
unsigned int localPort = 2390;      // local port to listen for UDP packets
//char timeServer[] = "time.nist.gov"; // external NTP server e.g. time.nist.gov
char timeServer[] = "0.uk.pool.ntp.org";
//char timeServer[] = "85.199.214.101";
//char timeServer[] = "1.ntp.talktalk.net";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// declare a time object
DateTime lastSynchTime;

// define WiFI client
WiFiClient client;

//The udp library class
WiFiUDP udp;

RTC_SAMD51 rtc;

// for use by the Adafuit RTClib library
char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

// For drawing the time on the LCD
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>

#define TFT_GREY 0x5AEB

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

// end of LCD drawing stuff

void setup() {
  // Setup the LCD screen to display the time:
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(2);
  tft.setCursor(35, 25);
  tft.println(F("WioTerminal"));
  tft.setCursor(35, 45);
  tft.println(F("Alarm"));
  tft.setCursor(35, 65);
  tft.println(F("Clock"));
  tft.setCursor(5, 85);
  tft.print(F("wait for serial:"));

  Serial.begin(115200);
  serialDelay.start(30000);
  // wait for serial port to connect. Needed for native USB
  while (!Serial)
  {
    if (serialDelay.justFinished()) {
      // timed-out waiting for serial connection, quit waiting
      tft.println(F("timed out"));
      break;
    }
  }

  if (serialDelay.isRunning()) {
    tft.println(F("okay"));
  }

  Serial.println("WioTerminalAlarmClock for Seeed Wio Terminal");

  // check if rtc present; should never happen unless on wrong hardware
  if (!rtc.begin()) {
    haltMessage("FATAL: No RTC!");
  }

  // get and print the current rtc time
  Serial.print("RTC time is: ");
  sendTimeViaSerial();

  Serial.print("Initializing SD card...");
  if (!SD.begin(SDCARD_SS_PIN, SDCARD_SPI)) {
    haltMessage("SD Card init Failed!");
  }
  Serial.println("initialization done.");

  // get the WiFi details from the SD card
  if (!loadWiFiDetails()) {
    haltMessage("loadWiFiDetails Failed");
  }

  // only use the SD card to load up data, done;
  SD.end();

  serialDelay.start(60 * 1000);
  updateDelay.start(10);
  tft.fillScreen(TFT_BLACK);
}

void loop() {
  if (updateDelay.justFinished()) {
    // Check WiFi is connected
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi not connected");
      connectToWiFi(ssid, password);
    }

    tft.setTextSize(2);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("NTP SYNCH ...", 10, 10);

    if (!synchRTCtoNTPtime(timeServer)) {
      Serial.println("Failed to get time from network time server.");
      tft.drawString("NTP SYNCH FAILED", 10, 10);
      synchCount = 0;
      updateDelay.start(DelayShort);
    }
    else {
      // repeat timer vlaue depends on how many successfull synchs we've had in a row
      if (++synchCount >= SynchCountReset) {
        synchCount = 0;
      }

      if (synchCount >= SynchCountLong) {
        updateDelay.start(DelayLong);
        tft.drawString("NTP SYNCH GOOD", 10, 10);
      } else if (synchCount >= SynchCountNormal) {
        updateDelay.start(DelayNormal);
        tft.drawString("NTP SYNCH OKAY", 10, 10);
      } else {
        updateDelay.start(DelayShort);
        tft.drawString("NTP SYNCH DONE", 10, 10);
      }
    }
    // not calling ntp time frequently, stop releases resources
    udp.stop();
    displayDelay.start(10);
  }

  if (serialDelay.justFinished()) {
    serialDelay.repeat();
    sendTimeViaSerial();
  }

  if (displayDelay.justFinished()) {
    displayDelay.repeat();
    drawTime();
  }
}

void sendTimeViaSerial() {
  // get and print the adjusted rtc time
  Serial.println(rtc.now().timestamp(DateTime::TIMESTAMP_FULL));
}

void connectToWiFi(const char* ssid, const char* pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);

  //register event handler
  WiFi.onEvent(WiFiEvent);

  Serial.println("Waiting for WIFI connection...");

  //Initiate connection
  WiFi.begin(ssid, pwd);

  millisDelay wifiDelay;
  wifiDelay.start(500);

  while (WiFi.status() != WL_CONNECTED) {
    if (wifiDelay.justFinished()) {
      WiFi.begin(ssid, pwd);
      wifiDelay.restart();
    }
  }

  Serial.println("Connected.");
  printWiFiStatus();
}

bool synchRTCtoNTPtime(const char* address) {
  // module returns a unsigned long time value as seconds since
  // Jan 1, 1970 unix time or 0 if a problem encounted

  //only send data when connected
  if (WiFi.status() == WL_CONNECTED) {
    //initializes the UDP state
    //This initializes the transfer buffer
    udp.begin(WiFi.localIP(), localPort);

    //sendNTPpacket(address); // send an NTP packet to a time server
    // set all bytes in the buffer to 0
    for (int i = 0; i < NTP_PACKET_SIZE; ++i) {
      packetBuffer[i] = 0;
    }
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;

    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    udp.beginPacket(address, 123); //NTP requests are to port 123
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();

    // wait for a reply from the NTP server
    bool gotPacket = false;
    millisDelay serverTimeout;
    serverTimeout.start(3000);
    while (serverTimeout.isRunning()) {
      if (udp.parsePacket()) {
        gotPacket = true;
        break;
      }
    }

    if (gotPacket) {
      // We've received a packet, read the data from it to the buffer
      udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

      // the timestamp starts at byte 40 of the received packet and
      // is four bytes, or two words, long. Extract the two words:
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      //      const unsigned long seventyYears = 2208988800UL;
      // BUT we are going to wait for the next second, so take off one second less:
      const unsigned long seventyYearsLessOneSecond = 2208988799UL;
      // subtract seventy years (less one second):
      unsigned long epoch = secsSince1900 - seventyYearsLessOneSecond;

      //      return epoch; - don't return, we will set the RTC in here now

      // note that this epoch is in whole seconds;
      // need to convert the fractional seconds to milliseconds, see answer to
      // https://arduino.stackexchange.com/questions/49567/synching-local-clock-usign-ntp-to-milliseconds
      // also see
      // https://forum.arduino.cc/t/heres-how-to-get-a-more-accurate-rtc-clock-set-from-an-ntp-time-server/506179
      // which has the plan of waiting until the next whole second, then setting the time on the RTC \o/

      // Now get the fractional part
      uint32_t NTPmillis = (packetBuffer[44] << 24) | (packetBuffer[45] << 16) | (packetBuffer[46] << 8) | packetBuffer[47];

      // Get the fractional part to delay to the next whole second as milliseconds
      int32_t fractionalPart = 1000 - (int32_t)(((float)NTPmillis / UINT32_MAX) * 1000);

      if (fractionalPart > 0) {
        // Burn off the remaining fractional part of the existing second
        delay(fractionalPart);
      }

      // record last synch time:
      lastSynchTime = DateTime(epoch);
      DateTime previousTime = rtc.now();
      rtc.adjust(lastSynchTime);

      Serial.print("rtc time updated; was:");
      Serial.print(previousTime.timestamp(DateTime::TIMESTAMP_FULL));
      Serial.print(", now:");
      Serial.println(lastSynchTime.timestamp(DateTime::TIMESTAMP_FULL));
      Serial.print("milliseconds delay was:");
      Serial.println(fractionalPart);

      return true;
    }
    else {
      // were not able to parse the udp packet successfully
      // clear down the udp connection
      udp.stop();
      return false;
    }
  }
  else {
    // network not connected
    return false;
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.println("");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  Serial.println("");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event)
{
  String eventText;

  //  Serial.printf("[WiFi-event] event: %d\n", event);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TC_DATUM);
  tft.setTextSize(2);
  eventText = "[WiFi-event] event: " + String(event);
  Serial.println(eventText);
  //  tft.fillRect(0, 0, 320, 20, TFT_BLACK);
  tft.fillRect(0, 200, 320, 40, TFT_BLACK);
  tft.drawString(eventText, 160, 200);

  switch (event) {
    case SYSTEM_EVENT_WIFI_READY:
      eventText = "WiFi interface ready";
      break;
    case SYSTEM_EVENT_SCAN_DONE:
      eventText = "Completed scan for access points";
      break;
    case SYSTEM_EVENT_STA_START:
      eventText = "WiFi client started";
      break;
    case SYSTEM_EVENT_STA_STOP:
      eventText = "WiFi clients stopped";
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      eventText = "Connected to access point";
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      eventText = "Disconnected from WiFi access point";
      break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
      eventText = "Authentication mode of access point has changed";
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      eventText = "Obtained IP address";
      break;
    case SYSTEM_EVENT_STA_LOST_IP:
      eventText = "Lost IP address and IP address is reset to 0";
      break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
      eventText = "WiFi Protected Setup (WPS): succeeded in enrollee mode";
      break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
      eventText = "WiFi Protected Setup (WPS): failed in enrollee mode";
      break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
      eventText = "WiFi Protected Setup (WPS): timeout in enrollee mode";
      break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
      eventText = "WiFi Protected Setup (WPS): pin code in enrollee mode";
      break;
    case SYSTEM_EVENT_AP_START:
      eventText = "WiFi access point started";
      break;
    case SYSTEM_EVENT_AP_STOP:
      eventText = "WiFi access point  stopped";
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      eventText = "Client connected";
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      eventText = "Client disconnected";
      break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
      eventText = "Assigned IP address to client";
      break;
    case SYSTEM_EVENT_AP_PROBEREQRECVED:
      eventText = "Received probe request";
      break;
    case SYSTEM_EVENT_GOT_IP6:
      eventText = "IPv6 is preferred";
      break;
    case SYSTEM_EVENT_ETH_START:
      eventText = "Ethernet started";
      break;
    case SYSTEM_EVENT_ETH_STOP:
      eventText = "Ethernet stopped";
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      eventText = "Ethernet connected";
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      eventText = "Ethernet disconnected";
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      eventText = "Obtained IP address";
      break;
    default: break;
  }
  Serial.println(eventText);
  //  tft.fillRect(0, 220, 320, 40, TFT_BLACK);
  tft.drawString(eventText, 160, 220);
}

void SetAlarm(DateTime alarm, RTC_SAMD51::Alarm_Match match) {
  //      DateTime alarm = DateTime(now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second() + 15);
  rtc.setAlarm(0, alarm);
  rtc.enableAlarm(0, match);
  //    rtc.enableAlarm(0, rtc.MATCH_HHMMSS);
  rtc.attachInterrupt(alarmMatch);
}

void alarmMatch(uint32_t flag) {
  Serial.println("Alarm Match!");
}

// Draw the time on the display
void drawTime() {
  static  byte omm = 99, oss = 99;
  static byte xcolon = 0, xsecs = 0;

  tft.setTextSize(1);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);

  DateTime now = rtc.now();
  uint8_t hh = now.hour();
  uint8_t mm = now.minute();
  uint8_t ss = now.second();

  // Update digital time
  int xpos = 0;
  int ypos = 85; // Top left corner ot clock text, about half way down
  int ysecs = ypos + 24;

  if (omm != mm) { // Redraw hours and minutes time every minute
    omm = mm;
    // Draw hours and minutes
    if (hh < 10) {
      xpos += tft.drawChar('0', xpos, ypos, 8);    // Add hours leading zero for 24 hr clock
    }
    xpos += tft.drawNumber(hh, xpos, ypos, 8);             // Draw hours
    xcolon = xpos; // Save colon coord for later to flash on/off later
    xpos += tft.drawChar(':', xpos, ypos - 8, 8);
    if (mm < 10) {
      xpos += tft.drawChar('0', xpos, ypos, 8);    // Add minutes leading zero
    }
    xpos += tft.drawNumber(mm, xpos, ypos, 8);             // Draw minutes
    xsecs = xpos; // Sae seconds 'x' position for later display updates
  }
  if (oss != ss) { // Redraw seconds time every second
    oss = ss;
    xpos = xsecs;

    if (ss % 2) { // Flash the colons on/off
      //      tft.setTextColor(0x39C4, TFT_BLACK);        // Set colour to grey to dim colon
      tft.setTextColor(TFT_GREY, TFT_BLACK);        // Set colour to grey to dim colon
      tft.drawChar(':', xcolon, ypos - 8, 8);     // Hour:minute colon
      xpos += tft.drawChar(':', xsecs, ysecs, 6); // Seconds colon
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);    // Set colour back to yellow
    } else {
      tft.drawChar(':', xcolon, ypos - 8, 8);     // Hour:minute colon
      xpos += tft.drawChar(':', xsecs, ysecs, 6); // Seconds colon
    }

    //Draw seconds
    if (ss < 10) {
      xpos += tft.drawChar('0', xpos, ysecs, 6);    // Add leading zero
    }
    tft.drawNumber(ss, xpos, ysecs, 6);                     // Draw seconds
  }
}

bool loadWiFiDetails() {
  File myFile = SD.open("/WiFi.json", FILE_READ);
  DynamicJsonDocument doc(1024);
  DeserializationError err = deserializeJson(doc, myFile);
  myFile.close();
  // Is there an error after all?
  if (err) {
    Serial.print(F("deserializeJson() failed with code "));
    Serial.println(err.c_str());
  } else {
    // Get a reference to the root object
    JsonObject obj = doc.as<JsonObject>();

    const char* id = obj["ssid"];
    if (id != nullptr) {
      Serial.print("SSID=");
      Serial.println(id);
      strcpy(ssid, id);
    } else {
      Serial.println("SSID not found");
    }

    const char* pwd = obj["psk"];
    // Is there an error after all?
    if (pwd != nullptr) {
      Serial.print("PSK=");
      Serial.println(pwd);
      strcpy(password, pwd);
    } else {
      Serial.println("PSK not found");
    }
  }
  return (ssid[0] != 0 && password[0] != 0);
}

void haltMessage(char* message) {
  // display a message and stop operating
  while (true) {
    Serial.println(message);
    tft.setCursor(5, 105);
    tft.println(message);
    tft.setCursor(5, 125);
    tft.println(F("Halted Operation"));
    delay(10000);
  }
}
