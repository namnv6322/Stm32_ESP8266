#include<ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <SoftwareSerial.h>

const byte RX = 12;
const byte TX = 13;
SoftwareSerial mySerial = SoftwareSerial(RX, TX);
long lastUART = 0;
void Read_Uart();    // UART STM
//----Thay đổi thành thông tin của bạn---------------
const char* ssid = "House";      //Wifi connect
const char* password = "0947779331";   //Password
const char* mqtt_server = "00223d1dc96f4e34a761a5b5ee950faa.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "namnv1"; //User
const char* mqtt_password = "matkhau63"; //Password
//--------------------------------------------------
WiFiClientSecure espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
//------------Connect to MQTT Broker-----------------------------
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientID =  "ESPClient-";
    clientID += String(random(0xffff),HEX);
    if (client.connect(clientID.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe("testTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
//-----Call back Method for Receiving MQTT massage---------
void callback(char* topic, byte* payload, unsigned int length) {
  String incommingMessage = "";
  for(int i=0; i<length;i++) incommingMessage += (char)payload[i];
  Serial.println("Massage arived ["+String(topic)+"]"+incommingMessage);
  if (incommingMessage.equals("{nhap_hang}")) mySerial.print("1");
  else if (incommingMessage.equals("{dat_hang}")) mySerial.print("2");
  else if (incommingMessage.equals("{san_pham_1}")) mySerial.print("3");
  else if (incommingMessage.equals("{san_pham_2}")) mySerial.print("4");
  else if (incommingMessage.equals("{san_pham_3}")) mySerial.print("5");
  else if (incommingMessage.equals("{san_pham_4}")) mySerial.print("6");
  else if (incommingMessage.equals("{san_pham_5}")) mySerial.print("7");
  else if (incommingMessage.equals("{san_pham_6}")) mySerial.print("8");
  else if (incommingMessage.equals("{stop}")) mySerial.print("0");
}
//-----Method for Publishing MQTT Messages---------
void publishMessage(const char* topic, String payload, boolean retained){
  if(client.publish(topic,payload.c_str(),true))
    Serial.println("Message published ["+String(topic)+"]: "+payload);
}


void setup() {
  Serial.begin(9600);
  while(!Serial) delay(1);
  setup_wifi();
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  mySerial.begin(115200);
}
unsigned long timeUpdata=millis();
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  Read_Uart();
}
void Read_Uart()
{
  String st = "";
  while (mySerial.available())
  {
    char inChar = (char)mySerial.read();
    st +=  inChar;
    publishMessage("App", st, true);
    Serial.println("Nhan : " +st);
  }
}