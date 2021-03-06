# WioTerminalAlarmClock
An Alarm Clock implemented on the Seeed Studio Wio Terminal device in Arduino

WioTerminalAlarmClock for Seeed Wio Terminal based on the examples in the Seeed studio Wio Terminal wiki - a great learning resource btw

The aim is to have a little alarm-clock with multiple alarms that will be useful for my workplace as a shift/break buzzer source

## Requirements:
 - get the time automatically over the network using NTP
 - adjust to the local time and cope with daylight savings time
 - display the time
 - display the status of the network connection
 - display the status of the NTP synchronisation
 - track multiple alarm times, which may repeat on every day, every weekday and so on
 - at each alarm time make a buzz and give an output signal suitable for PA or Tannoy

## Progress so far:
 - uses serial to send status, mostly for debugging purposes, with a 30 second timeout to connect on power-up
 - displays the configuration progress on the TFT
 - gets the SSID and PSK from file WiFi.json on SD card
 - connects to a WiFi network
 - requests the time using NTP
 - displays the NTP synch status (Failed, Synchronised, Corrected)
 - displays the time (in UTC)

## To Do:
 - get local timezone and DST settings from SD card
 - display local time
 - implement alarms
