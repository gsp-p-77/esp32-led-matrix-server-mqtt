/*
 * Project: LED Matrix Server for platform.io
 *
 * Use case: General use case
 * - Use 8x32 4 in one Dot Matrix display to display messages requested via MQTT (node RED)
 * as AZDelivery MAX7219 8x32 4 in 1 Dot Matrix LED Display Modul
 * My special use case: Display blood sugar from my Nightscout server, temperature, date and time and random news
 * 
 * HW requirements:
 * - AZDelivery MAX7219 8x32 4 in 1 Dot Matrix LED Display Modul (https://www.amazon.de/dp/B079HVW652?ref=ppx_yo2ov_dt_b_fed_asin_title)
 * - AZDelivery ESP32 Dev Kit C V4 NodeMCU WLAN WiFi Development Board 
 * Optional:
 * - Raspberry PI as NodeRed server and MQTT message broker
 * - Internet WLAN Router
 * 
 * Initial base: Example from AzDelivery, providing the scrollText() function (commented out all other demo functions from AzDelivery)
*/


#include <MD_MAX72xx.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

//RingBuffer
#define BUFFER_SIZE 20       // Maximum number of messages in the buffer
#define MAX_MSG_LENGTH 255   // Maximum length of each message
#define JSON_PAYLOAD_LENGTH 1024

typedef struct
{
  char buffer [MAX_MSG_LENGTH];
  int duration_1s;
}led_message;

struct RingBuffer {
    char msg_buffer[BUFFER_SIZE][MAX_MSG_LENGTH];
    uint8_t msg_delay_1s_buffer[BUFFER_SIZE];
    int head;  // Points to the next position to write
    int tail;  // Points to the next position to read
    int count; // Number of elements in the buffer
};

RingBuffer ringBuffer = {{0}, 0, 0, 0};

// Function to add a message to the ring buffer
bool addMessage(RingBuffer *rb, const char *msg, uint8_t msg_delay1s) {
    if (rb->count == BUFFER_SIZE) {
        // Buffer is full
        return false;
    }
    strncpy(rb->msg_buffer[rb->head], msg, MAX_MSG_LENGTH - 1);    
    rb->msg_buffer[rb->head][MAX_MSG_LENGTH - 1] = '\0'; // Null-terminate
    rb->msg_delay_1s_buffer[rb->head] = msg_delay1s;
    rb->head = (rb->head + 1) % BUFFER_SIZE;
    rb->count++;
    return true;
}

// Function to get a message from the ring buffer
bool getMessage(RingBuffer *rb, char *msgOut, uint8_t *msg_delay_1s) {
    if (rb->count == 0) {
        // Buffer is empty
        return false;
    }
    strncpy(msgOut, rb->msg_buffer[rb->tail], MAX_MSG_LENGTH);
    *msg_delay_1s = rb->msg_delay_1s_buffer[rb->tail];
    rb->tail = (rb->tail + 1) % BUFFER_SIZE;
    rb->count--;
    return true;
}




//MQTT
String clientID="ESP32-";
const char* mqtt_server = "192.168.178.36";
const char* mqtt_user="";
const char* mqtt_password="";
WiFiClient espClient;
PubSubClient client(espClient);

//LED config
#define PRINT(s, x) { Serial.print(F(s)); Serial.print(x); }
#define PRINTS(x) Serial.print(F(x))
#define PRINTD(x) Serial.println(x, DEC)
#define PRINT(s, x)
#define PRINTS(x)
#define PRINTD(x)
//#define HARDWARE_TYPE MD_MAX72XX::ICSTATION_HW
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define CLK_PIN   18 // VSPI_SCK
#define DATA_PIN  23 // VSPI_MOSI
#define CS_PIN    5  // VSPI_SS
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
#define DELAYTIME 100 // in milliseconds

// WiFi login parameters - network name and password
const char ssid[] = "FRITZ!Box 7530 RR";
const char password[] = "06420304028449282342";

// WiFi Server object and parameters
WiFiServer server(80);

void scrollText(char *p) {
  uint8_t charWidth;
  uint8_t cBuf[8]; // this should be ok for all built-in fonts
  PRINTS("\nScrolling text");
  mx.clear();
  while (*p != '\0') {
    charWidth = mx.getChar(*p++, sizeof(cBuf) / sizeof(cBuf[0]), cBuf);
    // allow space between characters
    for (uint8_t i=0; i<=charWidth; i++) {
      mx.transform(MD_MAX72XX::TSL);
      if (i < charWidth) {
        mx.setColumn(0, cBuf[i]);
      } 
      delay(DELAYTIME);
    }
  }
}
/*
void rows() {
  mx.clear();
  for (uint8_t row=0; row<ROW_SIZE; row++) {
    mx.setRow(row, 0xff); delay(2*DELAYTIME);
    mx.setRow(row, 0x00);
  }
}

void columns() {
  mx.clear();
  for (uint8_t col=0; col<mx.getColumnCount(); col++) {
    mx.setColumn(col, 0xff); delay(DELAYTIME/MAX_DEVICES);
    mx.setColumn(col, 0x00);
  }
}

void stripe() {
  const uint16_t maxCol = MAX_DEVICES*ROW_SIZE;
  const uint8_t stripeWidth = 10;
  mx.clear();
  for (uint16_t col=0; col<maxCol + ROW_SIZE + stripeWidth; col++) {
    for(uint8_t row=0; row < ROW_SIZE; row++) {
      mx.setPoint(row, col-row, true);
      mx.setPoint(row, col-row - stripeWidth, false);
    }
    delay(DELAYTIME);
  }
}

void spiral() {
  int rmin = 0, rmax = ROW_SIZE-1;
  int cmin = 0, cmax = (COL_SIZE*MAX_DEVICES)-1;
  mx.clear();
  while ((rmax > rmin) && (cmax > cmin)) {
    for (int i=cmin; i<=cmax; i++) { // do row
      mx.setPoint(rmin, i, true); delay(DELAYTIME/MAX_DEVICES);
    }
    rmin++;
    for (uint8_t i=rmin; i<=rmax; i++) { // do column
      mx.setPoint(i, cmax, true); delay(DELAYTIME/MAX_DEVICES);
    }
    cmax--;

    for (int i=cmax; i>=cmin; i--) { // do row
      mx.setPoint(rmax, i, true); delay(DELAYTIME/MAX_DEVICES);
    }
    rmax--;

    for (uint8_t i=rmax; i>=rmin; i--) { // do column
      mx.setPoint(i, cmin, true); delay(DELAYTIME/MAX_DEVICES);
    }
    cmin++;
  }
}

void bounce() {
  const int minC = 0;
  const int maxC = mx.getColumnCount()-1;
  const int minR = 0;
  const int maxR = ROW_SIZE-1;
  int nCounter = 0;
  int r = 0, c = 2;
  int8_t dR = 1, dC = 1; // delta row and column
  mx.clear();
  while (nCounter++ < 200) {
    mx.setPoint(r, c, false);
    r += dR; c += dC;
    mx.setPoint(r, c, true);
    delay(DELAYTIME/2);
    if ((r == minR) || (r == maxR)) {
      dR = -dR;
    }
    if ((c == minC) || (c == maxC)) {
      dC = -dC;
    }
  }
}

void intensity() {
  uint8_t row;
  mx.clear();
  for (int8_t i=0; i<=MAX_INTENSITY; i++) {
    mx.control(MD_MAX72XX::INTENSITY, i);
    mx.setRow(0, 0xff); delay(DELAYTIME*10);
  }
  mx.control(MD_MAX72XX::INTENSITY, 8);
}

void transformation() {
  uint8_t arrow[COL_SIZE] = {
  0b00001000,
  0b00011100,
  0b00111110,
  0b01111111,
  0b00011100,
  0b00011100,
  0b00111110,
  0b00000000 };
  MD_MAX72XX::transformType_t t[] = {
    MD_MAX72XX::TSL, MD_MAX72XX::TSL, MD_MAX72XX::TSL, MD_MAX72XX::TSL,
    MD_MAX72XX::TSL, MD_MAX72XX::TSL, MD_MAX72XX::TSL, MD_MAX72XX::TSL,
    MD_MAX72XX::TSL, MD_MAX72XX::TSL, MD_MAX72XX::TSL, MD_MAX72XX::TSL,
    MD_MAX72XX::TSL, MD_MAX72XX::TSL, MD_MAX72XX::TSL, MD_MAX72XX::TSL,
    MD_MAX72XX::TFLR,
    MD_MAX72XX::TSR, MD_MAX72XX::TSR, MD_MAX72XX::TSR, MD_MAX72XX::TSR,
    MD_MAX72XX::TSR, MD_MAX72XX::TSR, MD_MAX72XX::TSR, MD_MAX72XX::TSR,
    MD_MAX72XX::TSR, MD_MAX72XX::TSR, MD_MAX72XX::TSR, MD_MAX72XX::TSR,
    MD_MAX72XX::TSR, MD_MAX72XX::TSR, MD_MAX72XX::TSR, MD_MAX72XX::TSR,
    MD_MAX72XX::TRC,
    MD_MAX72XX::TSD, MD_MAX72XX::TSD, MD_MAX72XX::TSD, MD_MAX72XX::TSD,
    MD_MAX72XX::TSD, MD_MAX72XX::TSD, MD_MAX72XX::TSD, MD_MAX72XX::TSD,
    MD_MAX72XX::TFUD,
    MD_MAX72XX::TSU, MD_MAX72XX::TSU, MD_MAX72XX::TSU, MD_MAX72XX::TSU,
    MD_MAX72XX::TSU, MD_MAX72XX::TSU, MD_MAX72XX::TSU, MD_MAX72XX::TSU,
    MD_MAX72XX::TINV,
    MD_MAX72XX::TRC, MD_MAX72XX::TRC, MD_MAX72XX::TRC, MD_MAX72XX::TRC,
    MD_MAX72XX::TINV };
  mx.clear();
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);

  for (uint8_t j=0; j<mx.getDeviceCount(); j++) {
    mx.setBuffer(((j+1)*COL_SIZE)-1, COL_SIZE, arrow);
  }

  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
  delay(DELAYTIME);
  mx.control(MD_MAX72XX::WRAPAROUND, MD_MAX72XX::ON);
  // one tab
  for (uint8_t i=0; i<(sizeof(t)/sizeof(t[0])); i++) {
    mx.transform(t[i]); delay(DELAYTIME*4);
  }
  mx.control(MD_MAX72XX::WRAPAROUND, MD_MAX72XX::OFF);
}

void showCharset() {
  mx.clear();
  mx.update(MD_MAX72XX::OFF);
  for (uint16_t i=0; i<256; i++) {
    mx.clear(0);
    mx.setChar(COL_SIZE-1, i);
    if (MAX_DEVICES >= 3) {
      char hex[3];
      sprintf(hex, "%02X", i);
      mx.clear(1); mx.setChar((2*COL_SIZE)-1, hex[1]);
      mx.clear(2); mx.setChar((3*COL_SIZE)-1, hex[0]);
    }
    mx.update();
    delay(DELAYTIME*2);
  }
  mx.update(MD_MAX72XX::ON);
}
*/
static void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    clientID += String(random(0xffff), HEX);
    if (client.connect(clientID.c_str(), mqtt_user, mqtt_password))
    {
      Serial.println("connect to MQTT");
      client.subscribe("ledMessageRequest");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

static void callback(char *topic, byte *message, unsigned int length)
{   
    if (String(topic) == "ledMessageRequest") 
    {
      char jsonPayload[JSON_PAYLOAD_LENGTH];  // Adjust size based on expected JSON length
      length = (length >= sizeof(jsonPayload)) ? (sizeof(jsonPayload) - 1) : length;
      strncpy(jsonPayload, (char *)message, length);
      jsonPayload[length] = '\0'; // Null-terminate the message

      StaticJsonDocument<JSON_PAYLOAD_LENGTH> doc;  // Adjust size as necessary
      DeserializationError error = deserializeJson(doc, jsonPayload);

      if (error) {
          Serial.print("Failed to parse JSON: ");
          Serial.println(error.f_str());
          return;
      }

      // Extract elements from the JSON object
      const char* messageTemp = doc["message"];
      int delay = doc["delay"];

      if (!message || delay < 0) {
          Serial.println("Invalid JSON: Missing 'message' or 'delay'.");
          return;
      }

      // Add the message to the ring buffer
      if (!addMessage(&ringBuffer, messageTemp, delay)) {
          Serial.println("Ring buffer full! Dropping message.");
      } else {
          // Optionally store the delay if needed for later processing
          Serial.print("Message added with delay: ");
          Serial.println(delay);
      }
    }
}

void mqttWiFiDemoApp_init(void)
{
    enum 
    {
        wifiM_NOT_INITIALIZED,
        wifiM_INITIALIZED
    }wifiM_state = wifiM_NOT_INITIALIZED;

  if (wifiM_state == wifiM_NOT_INITIALIZED)
  {
    Serial.println("Connect to my WIFI");
    WiFi.begin(ssid, password);

    byte cnt = 0;

    while(WiFi.status() != WL_CONNECTED){
        delay(1000);
        Serial.print(".");
        cnt++;

        if(cnt>30){
        ESP.restart();
        }
    } 
    Serial.println(WiFi.localIP());
    wifiM_state = wifiM_INITIALIZED;

    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);    
  }
}

void mqttWiFiDemoApp_backGroundTask(void)
{
  if (!client.connected())
  {
    reconnect();
  }

  if (!client.loop())
  {
    client.connect("ESP32MQTT");
  }  
}


void setup()
{
  Serial.begin(115200);
  while(!Serial)
  {
  //wait
  }
  
  Serial.println("Starting ESP32...");

  mx.begin();
  mqttWiFiDemoApp_init();
}


void loop()
{
    uint64_t gSystemTimer1msLastSnapshot = millis();
  	bool delay_done = true;
    static char message[MAX_MSG_LENGTH];
    static uint8_t delay_1s = 0;

    if ((millis() - gSystemTimer1msLastSnapshot) >= delay_1s * 1000)
    {
      gSystemTimer1msLastSnapshot = millis();
      delay_done = true;

    }
    
    mqttWiFiDemoApp_backGroundTask();

    if (delay_done)
    {
      // Check if there is a message to process
      if (getMessage(&ringBuffer, message, &delay_1s)) {
          delay_done = false;
          Serial.println("Printing message: " + String(message));
          scrollText(message);
      }
    }
  
  /*

  scrollText("Graphics");
  
  rows();
  
  stripe();
  bounce();
  spiral();
  intensity();
  transformation();
  showCharset();
*/
}

