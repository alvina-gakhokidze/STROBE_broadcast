// /*
//   Rui Santos & Sara Santos - Random Nerd Tutorials
//   Complete project details at https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/
//   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.  
//   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// */

// #include <Arduino.h>
// #include <WiFi.h>
// #include <esp_wifi.h>


// void setup(){

//   delay(3000);
//   Serial.begin(115200);

//   Serial.print("ESP Board MAC Address: ");

//   WiFi.mode(WIFI_STA);
//   WiFi.begin();
//   uint8_t baseMac[6];
//   esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
//   if (ret == ESP_OK) {
//       Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
//                   baseMac[0], baseMac[1], baseMac[2],
//                   baseMac[3], baseMac[4], baseMac[5]);
//   } 
//   else {
//       Serial.println("Failed to read MAC address");
//   }

// }
 
// void loop(){

// }
