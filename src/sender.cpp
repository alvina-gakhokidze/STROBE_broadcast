/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define MAC_ADDRESS_SIZE 6
#define NUM_BOOLS 5
#define NUM_FLOATS 4


/********************* START OF USER ADJUSTABLE CODE*********************************/

#define NUM_ADDRESSES 16 // NUMBER OF STROBES YOU ARE BROADCASTING TO - CHANGE IF NEEDED


// you can declare up to 20 addresses total. MAC address will be printed in form  XX:XX:XX:XX:XX:XX -> convert into: {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX } ; 
uint8_t broadcastAddress1[] = {0x28, 0x37, 0x2f, 0xe0, 0x95, 0xf8}; 
uint8_t broadcastAddress2[] = {0x28, 0x37, 0x2f, 0xe0, 0xaa, 0x1c}; 
uint8_t broadcastAddress3[] = {0x28, 0x37, 0x2f, 0xe0, 0x96, 0x20}; 
uint8_t broadcastAddress4[] = {0x28, 0x37, 0x2f, 0xe0, 0xaa, 0x6c}; 
uint8_t broadcastAddress5[] = {0x28, 0x37, 0x2f, 0xe0, 0xaa, 0x0c};
uint8_t broadcastAddress6[] = {0x28, 0x37, 0x2f, 0xe0, 0xaa, 0x60};
uint8_t broadcastAddress7[] = {0x28, 0x37, 0x2f, 0xe0, 0x95, 0xd0};
uint8_t broadcastAddress8[] = {0x98, 0x3d, 0xae, 0x64, 0xc9, 0x88};
uint8_t broadcastAddress9[] = {0x28, 0x37, 0x2f, 0xe0, 0x96, 0x44};
uint8_t broadcastAddress10[] = {0x28, 0x37, 0x2f, 0xe0, 0xaa, 0x64};
uint8_t broadcastAddress11[] = {0x28, 0x37, 0x2f, 0xe0, 0xaa, 0x24};
uint8_t broadcastAddress12[] = {0x28, 0x37, 0x2f, 0xe0, 0xaa, 0x20};
uint8_t broadcastAddress13[] = {0x98, 0x3d, 0xae, 0x64, 0xc9, 0xa0};
uint8_t broadcastAddress14[] = {0x28, 0x37, 0x2f, 0xe0, 0x96, 0x18};
uint8_t broadcastAddress15[] = {0x98, 0x3d, 0xae, 0x64, 0xc9, 0x64};
uint8_t broadcastAddress16[] = {0x28, 0x37, 0x2f, 0xe0, 0xa9, 0xe0};


uint8_t* broadcastList[] = {broadcastAddress1, broadcastAddress2, broadcastAddress3, 
    broadcastAddress4, broadcastAddress5, 
    broadcastAddress6, broadcastAddress7, broadcastAddress8,
    broadcastAddress9, broadcastAddress10,
    broadcastAddress11, broadcastAddress12, broadcastAddress13,
    broadcastAddress14, broadcastAddress15, broadcastAddress16}; // if there are more address, add them to this list


/********************* END OF USER ADJUSTABLE CODE*********************************/

const byte numChars = 64;
char receivedChars[numChars];
char tempChars[numChars];
int conditionInputs[NUM_BOOLS];
float parameterInputs[NUM_FLOATS];
boolean newData = false;


typedef struct wifiParams {
    bool redLEDOn;
    bool blueLEDOn;
    bool manualMode;
    bool powerOn;
    bool frequencyOn;
    float redLEDFrequency;
    float redLEDPower;
    float blueLEDFrequency;
    float blueLEDPower;
} wifiParams;

wifiParams outData;

esp_now_peer_info_t peerInfo;


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}



bool addPeer(esp_now_peer_info_t * peerData, uint8_t broadcastAddress[], int num)
{
    Serial.print("Packet to: ");
    char macStr[18];
    // Copies the sender mac address to a string
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            broadcastAddress[0], broadcastAddress[1], broadcastAddress[2], broadcastAddress[3], broadcastAddress[4], broadcastAddress[5]);
    Serial.println(macStr);
    Serial.printf("Registering peer: %d\n", num);
    memcpy(peerData->peer_addr, broadcastAddress, MAC_ADDRESS_SIZE);

    if (esp_now_add_peer(peerData) != ESP_OK)
    {
        Serial.printf("Failed to add peer %d\n", num);
        return false;
    }
    Serial.printf("Added peer: %d\n", num);
    return true;
}


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars, ",");
    conditionInputs[0] = atoi(strtokIndx);     // convert this part to an integer
    

    for(int i=1; i < NUM_BOOLS; i++)
    {
        strtokIndx = strtok(NULL, ",");
        conditionInputs[i] = atoi(strtokIndx);     // convert this part to an integer
    }

    for(int i=0; i < NUM_FLOATS; i++)
    {
        strtokIndx = strtok(NULL, ",");
        parameterInputs[i] = atof(strtokIndx);     // convert this part to a float
    }

}

void showParsedData() {
    Serial.printf("Red LED On: %d\n", conditionInputs[0]);
    Serial.printf("Blue LED On: %d\n", conditionInputs[1]);
    Serial.printf("Manual Mode: %d\n", conditionInputs[2]);
    Serial.printf("Power On: %d\n", conditionInputs[3]);
    Serial.printf("Frequency On: %d\n", conditionInputs[4]);
    Serial.printf("Red LED Frequency: %lf\n", parameterInputs[0]);
    Serial.printf("Red LED Power: %lf\n", parameterInputs[1]);
    Serial.printf("Blue LED Frequency: %lf\n", parameterInputs[2]);
    Serial.printf("Blue LED Power: %lf\n", parameterInputs[3]);
}

 
void setup() {

    delay(3000);
    Serial.begin(115200);
    
    WiFi.mode(WIFI_STA);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    esp_now_register_send_cb(OnDataSent);
    
    // register peer
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    for(int i=0; i < NUM_ADDRESSES; i++)
    {
        addPeer(&peerInfo, broadcastList[i], i+1);
    }


    
}
 
void loop() {

    //first we check if we got input
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
        // this temporary copy is necessary to protect the original data
        //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        showParsedData();
        newData = false;


        outData.redLEDOn = conditionInputs[0];
        outData.blueLEDOn = conditionInputs[1];
        outData.manualMode = conditionInputs[2];
        outData.powerOn = conditionInputs[3];
        outData.frequencyOn = conditionInputs[4];
        outData.redLEDFrequency = parameterInputs[0];
        outData.redLEDPower = parameterInputs[1];
        outData.blueLEDFrequency = parameterInputs[2];
        outData.blueLEDPower = parameterInputs[3];
        
        

        for(int i=0; i < NUM_ADDRESSES; i++)
        {
            //Serial.printf("sending to peer: %d\n", i+1);
            esp_err_t result = esp_now_send(broadcastList[i], (uint8_t *) &outData, sizeof(wifiParams));
            
            if (result == ESP_OK) {
                Serial.println("Sent with success");
            }
            else {
                Serial.println("Error sending the data");
            }
            unsigned long time1 = millis();
            while(millis() - time1 <300);
        }

    }

    unsigned long time1 = millis();
    while(millis() - time1 <300);

}
