/*  Rui Santos & Sara Santos - Random Nerd Tutorials
    THIS EXAMPLE WAS TESTED WITH THE FOLLOWING HARDWARE:
      REGULAR ESP32 Dev Board + 2.8 inch 240x320 TFT Display: https://makeradvisor.com/tools/2-8-inch-ili9341-tft-240x320/ and https://makeradvisor.com/tools/esp32-dev-board-wi-fi-bluetooth/
      SET UP INSTRUCTIONS: https://RandomNerdTutorials.com/esp32-tft/
    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
    The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
    Complete project details: https://RandomNerdTutorials.com/esp32-tft-touchscreen-on-off-button-ili9341-arduino/
*/

#include <SPI.h>

#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x34, 0x85, 0x18, 0xa1, 0xc8, 0x78};

void EspNowTask(void *pvParameters); 
char *mac_to_str(char *buffer, uint8_t *mac); 
void on_sent(const uint8_t *mac_addr, esp_now_send_status_t status); 
void on_receive(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len);
void on_receive(const uint8_t *mac, const uint8_t *incomingData, int len); 

struct display_out_t {
    uint16_t id; // ID from the display unit. Gets from display unit (w)
    uint16_t setAmount; // The set amount configured by the user. Gets from display unit (w) 
    uint16_t setLength; // The set length configured by the user. Gets from display unit (w)
    bool flagStartStop; // Start/Stop flag. Defines if it should stop or continue working. Gets from display unit (R/W) (1 = start, 0 = stop)
    bool flagAbort; // Abort flag. Defines if the job should get aborted. Gets from display unit (w) (1 = abort, 0 = no abort)
} display_outgoing;

struct display_in_t {
    uint16_t id; // Sends to display unit (W)
    uint16_t processedAmount; // Sends to display unit (W) 
    bool flagStartStop; // Sends to display unit (W) (1 = start, 0 = stop)
    bool flagAbort; // Sends to display unit (W) (1 = abort, 0 = no abort)
    bool runOut; // Sends to display unit (W) (1 = run out, 0 = no run out)
} display_incoming; 


/*  Install the "TFT_eSPI" library by Bodmer to interface with the TFT Display - https://github.com/Bodmer/TFT_eSPI
    *** IMPORTANT: User_Setup.h available on the internet will probably NOT work with the examples available at Random Nerd Tutorials ***
    *** YOU MUST USE THE User_Setup.h FILE PROVIDED IN THE LINK BELOW IN ORDER TO USE THE EXAMPLES FROM RANDOM NERD TUTORIALS ***
    FULL INSTRUCTIONS AVAILABLE ON HOW CONFIGURE THE LIBRARY: https://RandomNerdTutorials.com/esp32-tft/   */
#include <TFT_eSPI.h>

// Install the "XPT2046_Touchscreen" library by Paul Stoffregen to use the Touchscreen - https://github.com/PaulStoffregen/XPT2046_Touchscreen
// Note: this library doesn't require further configuration
#include <XPT2046_Touchscreen.h>

TFT_eSPI tft = TFT_eSPI();

// Touchscreen pins
#define XPT2046_IRQ 36   // T_IRQ
#define XPT2046_MOSI 32  // T_DIN
#define XPT2046_MISO 39  // T_OUT
#define XPT2046_CLK 25   // T_CLK
#define XPT2046_CS 33    // T_CS

SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define FONT_SIZE 2
#define FONT_BIG 3

// Button position and size
#define FRAME_X 40
#define FRAME_Y 150
#define FRAME_W 250
#define FRAME_H 70

//4 adjust bottons pos/size
#define ADJ_W 80
#define ADJ_L 80
#define ADJL 10
#define ADJR SCREEN_WIDTH-ADJL-ADJ_W
#define ADJT 20
#define ADJB SCREEN_HEIGHT-ADJT-ADJ_W

//start button pos/size
#define START_H 60
#define START_Y ADJB+10
#define START_X ADJL+80+10
#define START_W SCREEN_WIDTH-20-ADJ_W-20-ADJ_W


//cont button pos/size
#define CONT_X FRAME_X
#define CONT_Y FRAME_Y
#define CONT_W (FRAME_W/2)-5
#define CONT_H FRAME_H

//abort button pos/size
#define ABORT_X FRAME_X+(FRAME_W/2)+5
#define ABORT_Y FRAME_Y
#define ABORT_W (FRAME_W/2)-5
#define ABORT_H FRAME_H

// Red zone size
#define REDBUTTON_X FRAME_X-10
#define REDBUTTON_Y FRAME_Y
#define REDBUTTON_W FRAME_W
#define REDBUTTON_H FRAME_H

// Green zone size
#define GREENBUTTON_X FRAME_X -10
#define GREENBUTTON_Y FRAME_Y
#define GREENBUTTON_W 120
#define GREENBUTTON_H FRAME_H


//Stop button
#define STOPBUTTON_X 

// LED Pin
#define LED_GREEN 16

// Touchscreen coordinates: (x, y) and pressure (z)
int x, y, z;

// Stores current button state
int buttonState = 0;
bool cont = false;
int lng =15;
int qty =10;
int prog=0;
int runtime=0;
int state=0;
int systime=0;
int stateo=0;
bool receive=0;
bool first=0;
bool touch=0;
int prodcounter=0;
bool sawstop=0;



// Print Touchscreen info about X, Y and Pressure (Z) on the Serial Monitor
void printTouchToSerial(int touchX, int touchY, int touchZ) {
  Serial.print("X = ");
  Serial.print(touchX);
  Serial.print(" | Y = ");
  Serial.print(touchY);
  Serial.print(" | Pressure = ");
  Serial.print(touchZ);
  Serial.println();
}


// Draw button frame
void drawFrame() {
  tft.fillRect(FRAME_X-10, FRAME_Y, FRAME_W+20, FRAME_H, TFT_BLACK);
}

void clearscreen(){
  tft.fillRect(0,0, SCREEN_WIDTH, SCREEN_HEIGHT, TFT_BLACK);
}



void drawscreensetup() {
  clearscreen();
  tft.fillRect(ADJR, ADJT, ADJ_W, ADJ_L, TFT_LIGHTGREY);
  tft.fillRect(ADJL, ADJT, ADJ_W, ADJ_L, TFT_LIGHTGREY);
  tft.fillRect(ADJL, ADJB, ADJ_W, ADJ_L, TFT_LIGHTGREY);
  tft.fillRect(ADJR, ADJB, ADJ_W, ADJ_L, TFT_LIGHTGREY);
  tft.fillRect(START_X, START_Y, START_W, START_H, TFT_GREEN);

  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(FONT_BIG);
  tft.setTextDatum(MC_DATUM);

  tft.drawString("UP", (ADJL+ADJ_W/2), (ADJT+ADJ_W/2));
  tft.drawString("UP", (ADJR+ADJ_W/2), (ADJT+ADJ_W/2));
  tft.drawString("DOWN", (ADJL+ADJ_W/2), (ADJB+ADJ_W/2));
  tft.drawString("DOWN", (ADJR+ADJ_W/2), (ADJB+ADJ_W/2));
  tft.drawString("START", (SCREEN_WIDTH/2), (START_Y+START_H/2));

  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(FONT_SIZE);
  tft.setTextDatum(MC_DATUM);

  tft.drawString("QTY", (ADJL+ADJ_W/2), (SCREEN_HEIGHT/2));
  tft.drawString("LNG", (ADJR+ADJ_W/2), (SCREEN_HEIGHT/2));
  tft.drawString("QTY:", ((SCREEN_WIDTH*2)/5), (80));
  tft.drawString("LNG:", ((SCREEN_WIDTH*2)/5), (100));
  tft.drawString(String(qty), ((SCREEN_WIDTH*3)/5), 80);
  tft.drawString(String(lng) + "mm", ((SCREEN_WIDTH*3)/5), 100);
}
  
void drawscreenprod(){
  clearscreen();
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(3);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(String(prog) + "/" + String(qty), (SCREEN_WIDTH/2), 100);
  tft.drawString(String(lng) + "mm", (SCREEN_WIDTH/2), 130);
  tft.setTextSize(3);
  tft.drawString("IN PRODUCTION:", (SCREEN_WIDTH/2), (60));

  tft.fillRect(FRAME_X, FRAME_Y, FRAME_W, FRAME_H, TFT_RED);
  
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(3);
  tft.drawString("PAUSE", (SCREEN_WIDTH/2), (FRAME_Y+FRAME_H/2));
}

void drawscreenpause(){
  clearscreen();
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(4);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("PAUSED", (SCREEN_WIDTH/2), (50));
  tft.setTextSize(3);
  tft.drawString(String(prog) + "/" + String(qty), (SCREEN_WIDTH/2), 100);
  tft.drawString(String(lng) + "mm", (SCREEN_WIDTH/2), 130);

  tft.fillRect(CONT_X, CONT_Y, CONT_W, CONT_H, TFT_GREEN);
  tft.fillRect(ABORT_X, ABORT_Y, ABORT_W, ABORT_H, TFT_RED);
  
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(3);
  tft.drawString("CONT", (CONT_X+(60)), (CONT_Y+(CONT_H/2)));
  tft.drawString("ABORT", (ABORT_X+(60)), (ABORT_Y+(ABORT_H/2)));
}

void drawscreencomplete(){
  clearscreen();
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(4);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("TASK", (SCREEN_WIDTH/2), (90));
  tft.drawString("COMPLETE", (SCREEN_WIDTH/2), (130));
  tft.fillRect(FRAME_X, FRAME_Y, FRAME_W, FRAME_H, TFT_GREEN);
  tft.setTextSize(3);
  tft.setTextColor(TFT_BLACK);
  tft.drawString("RETURN", (SCREEN_WIDTH/2), (FRAME_Y+FRAME_H/2));
}

void drawscreenerror(){
  clearscreen();
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.fillRect(FRAME_X, FRAME_Y, FRAME_W, FRAME_H, TFT_RED);
  tft.setTextSize(4);
  tft.drawString("ERROR", (SCREEN_WIDTH/2), (35));
  tft.setTextSize(2);
  tft.drawString("A FATAL ERROR HAS OCURRED", (SCREEN_WIDTH/2), (60));
  tft.drawString("RESOLVE THE ISSUE BELOW ", (SCREEN_WIDTH/2), (75));
  tft.drawString("AND THEN PRESS RETURN", (SCREEN_WIDTH/2), (90));
  tft.setTextColor(TFT_BLACK);

  tft.drawString("RETURN", (SCREEN_WIDTH/2), (FRAME_Y+(FRAME_H/2)));  
}


void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // Just in case
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(on_sent);
  esp_now_register_recv_cb(on_receive);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESP-NOW Initialized");
}

void on_receive(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
  ESP_LOGI("ESP_NOW", "Received message from " MACSTR, MAC2STR(esp_now_info->src_addr));

  // Print raw data (safe way for strings)
  printf("Raw message: %.*s\n", data_len, data);

  // Check if received data is the correct size
  if (data_len == sizeof(display_in_t)) {
    memcpy(&display_incoming, data, sizeof(display_in_t));
    
    Serial.println("Received structured data:");
    Serial.printf("ID: %d, Processed: %d, Start: %d, Abort: %d, RunOut: %d\n",
                  display_incoming.id,
                  display_incoming.processedAmount,
                  display_incoming.flagStartStop,
                  display_incoming.flagAbort,
                  display_incoming.runOut);
    
    receive = 1; 
  } else {
    ESP_LOGW("ESP_NOW", "Received data size mismatch: expected %d, got %d", sizeof(display_in_t), data_len);
  }
}


char *mac_to_str(char *buffer, uint8_t *mac)
{
    // sprintf(buffer, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    // below is another way to do this
    sprintf(buffer, MACSTR, MAC2STR(mac));
    return buffer;
}
 

void on_sent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  char buffer[13];
  switch (status)
  {
  case ESP_NOW_SEND_SUCCESS:
    ESP_LOGI("MAC_ADDRESS", "message sent to %s", mac_to_str(buffer, (uint8_t *)mac_addr));
    break;
  case ESP_NOW_SEND_FAIL:
    ESP_LOGE("MAC_ADDRESS", "message sent to %s failed", mac_to_str(buffer, (uint8_t *)mac_addr));
    break;
  }
}


void sendDisplayData() {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&display_outgoing, sizeof(display_outgoing));
  if (result == ESP_OK) {
    Serial.println("Data sent successfully");
  } else {
    Serial.print("Error sending data: ");
    Serial.println(result);
  }
}

void on_receive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&display_incoming, incomingData, sizeof(display_in_t));
  Serial.println("Received data:");
  Serial.printf("ID: %d, Processed: %d, Start: %d, Abort: %d, RunOut: %d\n",
                display_incoming.id,
                display_incoming.processedAmount,
                display_incoming.flagStartStop,
                display_incoming.flagAbort,
                display_incoming.runOut);
    // You can now use display_incoming values to update the screen
  receive = 1; 
}

void setup() {
  Serial.begin(115200);

  // Start the SPI for the touchscreen and init the touchscreen
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  // Set the Touchscreen rotation in landscape mode
  // Note: in some displays, the touchscreen might be upside down, so you might need to set the rotation to 3: touchscreen.setRotation(3);
  touchscreen.setRotation(3);

  // Start the tft display
  tft.init();
  // Set the TFT display rotation in landscape mode
  tft.setRotation(1);

  // Clear the screen before writing to it
  tft.fillScreen(TFT_BLACK);

  // Draw button 

  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, LOW);

  // Init ESP-NOW
  initEspNow();

  display_outgoing.setAmount=5;
  display_outgoing.setLength=100;
  display_outgoing.flagStartStop=1;
  display_outgoing.flagAbort=0;

  drawscreensetup();
}

void loop() {
  stateo=state;
  
  if (touchscreen.tirqTouched() && touchscreen.touched()) {
    // Get Touchscreen points
    TS_Point p = touchscreen.getPoint();
    // Calibrate Touchscreen points with map function to the correct width and height
    x = map(p.x, 200, 3700, 1, SCREEN_WIDTH );
    y = map(p.y, 240, 3800, 1, SCREEN_HEIGHT);

    touch=1;
  }
  else{
    touch=0;
    x=0;
    y=0;
  }

  if(display_incoming.runOut){
    state=4;
    //display_outgoing.flagStartStop=0;
    //sendDisplayData();
    prog=0;
  }
  if((display_incoming.flagAbort)&&!sawstop){
    state=4;
    //display_outgoing.flagStartStop=0;
    //sendDisplayData();
  }

  switch (state){
  case 0:                                                             //SETUP
    if(first){
      drawscreensetup();
      sawstop=0;
    }
    if(!cont&&touch){
      if ((x>ADJL) && (x<=(ADJL+ADJ_W))) {
        if ((y > ADJT) && (y <= (ADJT+ADJ_W))) {
          Serial.println("qty increased");
          if (qty<100){
            qty=qty+1;
          }
        }
      }
      if ((x>ADJL) && (x<=(ADJL+ADJ_W))) {
        if ((y > ADJB) && (y <= (ADJB+ADJ_W))) {
          Serial.println("qty decreased");
          if(qty>1){
            qty=qty-1;
          }
        }
      }
      if ((x>ADJR) && (x<=(ADJR+ADJ_W))) {
        if ((y > ADJT) && (y <= (ADJT+ADJ_W))) {
          Serial.println("lng increased");
          if (lng<999){
            lng=lng+1;
          }
        }
      }
      if ((x>ADJR) && (x<=(ADJR+ADJ_W))) {
        if ((y > ADJB) && (y <= (ADJB+ADJ_W))) {
          Serial.println("lng decreased");
          if (lng>10){
            lng=lng-1;
          }
        }
      }
      tft.fillRect(SCREEN_WIDTH/2, 60, ADJR-SCREEN_WIDTH/2, 60, TFT_BLACK);
      tft.drawString(String(qty), ((SCREEN_WIDTH*3)/5), 80);
      tft.drawString(String(lng) + "mm", ((SCREEN_WIDTH*3)/5), 100);

      if ((x>START_X) && (x<=(START_X+START_W))) {
        if ((y > START_Y) && (y <= (START_Y+START_H))) {
          Serial.println("state: production");
          state=1;
          prog=0;
          display_outgoing.setAmount=qty;
          display_outgoing.setLength=lng;
          display_outgoing.flagStartStop=1;
          sendDisplayData();
        }
      }      
    }
    break;

    case 1:                                                                                 //PRODUCTION

    if(first){
      drawscreenprod();
    }
    


    if(receive){
      prog=display_incoming.processedAmount;
      tft.setTextDatum(MC_DATUM);
      tft.setTextColor(TFT_WHITE);
      tft.setTextSize(3);
      tft.fillRect(0, 85, SCREEN_WIDTH, 30, TFT_BLACK);
      tft.drawString(String(prog) + "/" + String(qty), (SCREEN_WIDTH/2), 100);
    }
    

    if(!cont){
      if ((x>FRAME_X) && (x<=(FRAME_X+FRAME_W))) {
        if ((y > FRAME_Y) && (y <= (FRAME_Y+START_H))) {
          Serial.println("state: pause");
          state=2;
          display_outgoing.setAmount=qty;
          display_outgoing.flagStartStop=0;
          sendDisplayData();

        }
      }
    }
    if(prog>=qty){
      state=3;
      display_outgoing.flagStartStop=0;
      sendDisplayData();
    }
    break;

    case 2:                                                                           //PAUSE
    if(first){
      drawscreenpause();
    }  
    if(!cont){
      if ((x>CONT_X) && (x<=(CONT_X+CONT_W))) {
        if ((y > CONT_Y) && (y <= (CONT_Y+CONT_H))) {
          Serial.println("state: production");
          state=1;
          display_outgoing.flagStartStop=1;
          sendDisplayData();
        }
      }
      if ((x>ABORT_X) && (x<=(ABORT_X+ABORT_W))) {
        if ((y > ABORT_Y) && (y <= (ABORT_Y+ABORT_H))) {
          Serial.println("state: setup");
          prog=0;
          state=0;
          display_outgoing.flagStartStop=0;
          sendDisplayData();
        }
      }
    }
    break;

    case 3:                                                             //COMPLETE
    if(first){
      drawscreencomplete();
    }
    if(!cont){
      if ((x>FRAME_X) && (x<=(FRAME_X+FRAME_W))) {
        if ((y > FRAME_Y) && (y <= (FRAME_Y+START_H))) {
          Serial.println("state: setup");
          state=0;
        }
      }
    }
    break;
    
    case 4:                                                                                //ERROR
    if(first){
      drawscreenerror();
      if(display_incoming.runOut){
        tft.fillRect(0, 100, SCREEN_WIDTH, 50, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(3);
        tft.drawString("MATERIAL RUN OUT", (SCREEN_WIDTH/2), (125));
      }
      if(display_incoming.flagAbort){
        tft.fillRect(0, 100, SCREEN_WIDTH, 50, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(3);
        tft.drawString("FINGER WEG", (SCREEN_WIDTH/2), (125));
      }
    }
    
    if(!cont){
      if ((x>FRAME_X) && (x<=(FRAME_X+FRAME_W))) {
        if ((y > FRAME_Y) && (y <= (FRAME_Y+START_H))) {
          Serial.println("state: production");
          if(sawstop){
            state=0;
            sawstop=0;
          }
          else if(display_incoming.runOut){
            state=1;
          }
        }
      }
    }
  }


  if(touch){
    cont=1;
  }
  else{
    cont=0;
  }
  if (!(stateo==state)){
    first=1;
  }
  else{
    first=0;
  }  
  if(display_incoming.flagAbort){
    sawstop=1;
  }
  receive=0;
  delay(50);

}


