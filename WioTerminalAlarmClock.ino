#include <rpcWiFi.h> // see https://github.com/Seeed-Studio/Seeed_Arduino_rpcWiFi
#include <millisDelay.h> // see https://github.com/ansonhe97/millisDelay
#include "RTC_SAMD51.h" // see https://github.com/Seeed-Studio/Seeed_Arduino_RTC

// based on the examples at https://wiki.seeedstudio.com/Wio-Terminal-Wi-Fi/

// insert your SSID/PSK here:
const char ssid[] = "SSID"; 
const char password[] = "PSK"; 

millisDelay updateDelay; // the update delay object. used for ntp periodic update.
millisDelay serialDelay; // the serial delay object. used for updates to the serial port.
millisDelay displayDelay; // the display delay object. used for updating the display.
// The Normal, Short and Long delays, in milliseconds:
const unsigned long DelayNormal = 60 * 60 * 1000;
const unsigned long DelayShort = 5 * 60 * 1000;
const unsigned long DelayLong = 8 * 60 * 60 * 1000;

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

byte omm = 99, oss = 99;
byte xcolon = 0, xsecs = 0;
unsigned int colour = 0;
// end of LCD drawing stuff 

void setup() {
    Serial.begin(115200);
    serialDelay.start(10000);
    // wait for serial port to connect. Needed for native USB
    while (!Serial)
    {
      if(serialDelay.justFinished()){
        // timed-out waiting for serial connection, quit waiting
        break;
      }
    }

    Serial.println("WioTerminalAlarmClock for Seeed Wio Terminal");
 
    // check if rtc present; should never happen unless on wrong hardware
    if (!rtc.begin()) {
        while (true) {
          Serial.println("Couldn't find RTC; FATAL; Halted Operation");
          delay(10000); // stop operating
        }
    }
 
    // get and print the current rtc time
    Serial.print("RTC time is: ");
    sendTimeViaSerial();
    
    // setup network before rtc check 
    connectToWiFi(ssid, password);
 
    // get the time via NTP (udp) call to time server
    // getNTPtime returns epoch UTC time 
    // if required adjust for timezone and daylight savings time
    // To store the time returned by the NTP server
    unsigned long devicetime = getNTPtime(timeServer);
    if (devicetime == 0) {
        Serial.print("Failed to get time from network time server:");
        Serial.println(timeServer);
        // start millisdelays timers as required
        updateDelay.start(DelayShort);
    } else {
      // adjust time using ntp time
      rtc.adjust(DateTime(devicetime));
      // print boot update details
      Serial.println("RTC (boot) time updated.");
      // get and print the adjusted rtc time
      Serial.print("Adjusted RTC (boot) time is: ");
      sendTimeViaSerial();
      // record last synch time:
      lastSynchTime = DateTime(devicetime);
      // start millisdelays timers
      updateDelay.start(DelayNormal);
    }

    // not calling ntp time frequently, stop releases resources
    udp.stop();

    serialDelay.start(60*1000);

    // Setup the LCD screen to display the time:
    tft.init();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    displayDelay.start(250);
}
 
void loop() {
    if (updateDelay.justFinished()) {
        // Check WiFi is connected
        if(WiFi.status() != WL_CONNECTED){
          Serial.println("WiFi not connected");
          connectToWiFi(ssid, password);          
        }
        
        // To store the time returned by the NTP server
        unsigned long devicetime = getNTPtime(timeServer);
        if (devicetime == 0) {
            char* ntpFailMessage = "Failed to get time from network time server.";
            Serial.println(ntpFailMessage);
            tft.drawString(ntpFailMessage,10,10);
            updateDelay.start(DelayShort);
        }
        else {
            DateTime previousTime = rtc.now();
            rtc.adjust(DateTime(devicetime));
            Serial.print("rtc time updated; was:");
            Serial.print(previousTime.timestamp(DateTime::TIMESTAMP_FULL));
            Serial.print(", now:");
            Serial.println(DateTime(devicetime).timestamp(DateTime::TIMESTAMP_FULL));
            
            // repeat timer
            updateDelay.start(DelayNormal);
        }
        // not calling ntp time frequently, stop releases resources
        udp.stop();
    }

    if(serialDelay.justFinished()){
      serialDelay.repeat();
      sendTimeViaSerial();
    }

    if (displayDelay.justFinished()) {
      displayDelay.repeat();
      drawTime();
    }
}
 
void sendTimeViaSerial(){
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
      if(wifiDelay.justFinished()){
        WiFi.begin(ssid, pwd);
        wifiDelay.restart(); 
      }
    }
    
    Serial.println("Connected.");
    printWiFiStatus();
}
 
unsigned long getNTPtime(const char* address) { 
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
        while(serverTimeout.isRunning()) {
          if(udp.parsePacket()){
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
            const unsigned long seventyYears = 2208988800UL;
            // subtract seventy years:
            unsigned long epoch = secsSince1900 - seventyYears;

            return epoch;
 
            // adjust time for timezone offset in secs +/- from UTC
            // WA time offset from UTC is +8 hours (28,800 secs)
            // + East of GMT
            // - West of GMT
            long tzOffset = 28800UL;
 
            // WA local time 
            unsigned long adjustedTime;
            return adjustedTime = epoch + tzOffset;
        }
        else {
            // were not able to parse the udp packet successfully
            // clear down the udp connection
            udp.stop();
            return 0; // zero indicates a failure
        }
    }
    else {
        // network not connected
        return 0;
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
    Serial.printf("[WiFi-event] event: %d\n", event);

    switch (event) {
        case SYSTEM_EVENT_WIFI_READY: 
            Serial.println("WiFi interface ready");
            break;
        case SYSTEM_EVENT_SCAN_DONE:
            Serial.println("Completed scan for access points");
            break;
        case SYSTEM_EVENT_STA_START:
            Serial.println("WiFi client started");
            break;
        case SYSTEM_EVENT_STA_STOP:
            Serial.println("WiFi clients stopped");
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            Serial.println("Connected to access point");
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            Serial.println("Disconnected from WiFi access point");
            break;
        case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
            Serial.println("Authentication mode of access point has changed");
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            Serial.print("Obtained IP address: ");
            Serial.println(WiFi.localIP());
            break;
        case SYSTEM_EVENT_STA_LOST_IP:
            Serial.println("Lost IP address and IP address is reset to 0");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
            Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_FAILED:
            Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
            Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_PIN:
            Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
            break;
        case SYSTEM_EVENT_AP_START:
            Serial.println("WiFi access point started");
            break;
        case SYSTEM_EVENT_AP_STOP:
            Serial.println("WiFi access point  stopped");
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            Serial.println("Client connected");
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            Serial.println("Client disconnected");
            break;
        case SYSTEM_EVENT_AP_STAIPASSIGNED:
            Serial.println("Assigned IP address to client");
            break;
        case SYSTEM_EVENT_AP_PROBEREQRECVED:
            Serial.println("Received probe request");
            break;
        case SYSTEM_EVENT_GOT_IP6:
            Serial.println("IPv6 is preferred");
            break;
        case SYSTEM_EVENT_ETH_START:
            Serial.println("Ethernet started");
            break;
        case SYSTEM_EVENT_ETH_STOP:
            Serial.println("Ethernet stopped");
            break;
        case SYSTEM_EVENT_ETH_CONNECTED:
            Serial.println("Ethernet connected");
            break;
        case SYSTEM_EVENT_ETH_DISCONNECTED:
            Serial.println("Ethernet disconnected");
            break;
        case SYSTEM_EVENT_ETH_GOT_IP:
            Serial.println("Obtained IP address");
            break;
        default: break;
    }
}

void SetAlarm(DateTime alarm, RTC_SAMD51::Alarm_Match match){
//      DateTime alarm = DateTime(now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second() + 15);
    rtc.setAlarm(0,alarm);
    rtc.enableAlarm(0, match);
//    rtc.enableAlarm(0, rtc.MATCH_HHMMSS);
    rtc.attachInterrupt(alarmMatch);
}

void alarmMatch(uint32_t flag){
    Serial.println("Alarm Match!");
}

// Draw the time on the display
void drawTime() {
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
          tft.setTextColor(0x39C4, TFT_BLACK);        // Set colour to grey to dim colon
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
