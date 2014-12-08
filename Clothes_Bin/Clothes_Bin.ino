#include <RFduinoBLE.h>
#include <Float.h>

#define GSM_Power 2
#define Ultra_Power 3
#define Ultra_Sig 6
#define txPin 1 
#define rxPin 0
#define DEBUG false
#define SENSITIVITY 10 //Sensitivity of sensor in CM
#define STABLE_SAMPLE_COUNT 3 //Number of samples before setting base level
#define START_SAMPLE_DELAY 3
#define enableBLESend false
#define enableGSMSend true

class BinProperties{
  private:
    BinProperties(){};//Static Constructor
    BinProperties(BinProperties const&);// Don't Implement
    void operator=(BinProperties const&); // Don't implement
    char* deviceIMEI = "TESTUNIT";
    char* davidPhone = "+12267916646";
    char* twilioPhone = "+17806664662";
    uint64_t sensorTimeoutLength = MINUTES(20);
    float lastReading = FLT_MIN;
    float messageThreshold = 20;
    void setBaseLevel(){
      //wait until steady state distance is achieved
      float temp = RFduino_temperature(CELSIUS) - 5; //offset device heating
      float change = FLT_MAX;
      float lastDistance = FLT_MAX;
      float epsilon = SENSITIVITY;
      float distance = 0.0;
      int stableSampleCount = 0;
      while(stableSampleCount < STABLE_SAMPLE_COUNT){
        distance = Ultra_Sample(temp);
        if(lastDistance == FLT_MAX){
          lastDistance = distance;
        }
        //Calculate change
        float delta = distance - lastDistance;
        change = ((delta)>0?(delta):-(delta));
        lastDistance = distance;
        if(DEBUG){
          Serial.printf("Finding Base Level: \r\n Dist:%f Change:%f StableSampleCount: %d \r\n", distance, change, stableSampleCount);
          Serial.flush();
        }
        RFduino_ULPDelay( SECONDS(START_SAMPLE_DELAY));
        if(change < epsilon){
          stableSampleCount = stableSampleCount + 1;
          continue;
        }
        stableSampleCount = 0;
      }
      this->baseLevel = distance;
      Serial.printf("Base Level Established: %f \r\n", this->baseLevel);
      Serial.flush();
    }
  public:
    bool isActive = false;
    float baseLevel;//Empty Distance in CM
    float currentLevel;//Distance in CM
    float currentUsage;//Usage in %
    float temp;

  static BinProperties& getBinProperties(){
      static BinProperties  instance; // Guaranteed to be destroyed.
      return instance;// Instantiated on first use.
  }
  /**
  uint64_t id = getDeviceID();
  Serial.print("Device ID:");
  Serial.print(getdeviceIMEILow(), HEX);
  Serial.print(getdeviceIMEIHigh(), HEX);
  Serial.printf("\r\nTemp= %f Dist= %f\r\n", temp, distance);
  */
  float getCurrentUsage(){
    Serial.flush();
    this->temp = RFduino_temperature(CELSIUS) - 5;
    this->currentLevel = Ultra_Sample(this->temp);
    this->currentUsage = ((this->baseLevel-this->currentLevel)/this->baseLevel)*100.0;
    Serial.flush();
    return this->currentUsage;
  }
  
  void initBin(){
//TODO: properly get IMEI
//      Power_On_GSM();
//      Serial.flush();
//      this->deviceIMEI = getDeviceIMEI();
//      Serial.flush();
//      Power_Off_GSM();
//      Serial.flush();
      if(DEBUG==true){
        this->sensorTimeoutLength = SECONDS(10);
      }
      this->setBaseLevel();
      this->isActive = true;
      //Establish the base level
      //Advertise sensor to network
      //Advertise bluetooth to verify on app-side
  }

  void sendTestMessageSerial(char *number, char* message){
    Serial.printf("Target Number: %s\r\n", number);
    Serial.printf("%s\r\n",message);
    Serial.flush();
  }
  
  char* statusMessage(){
    char message[1000];
    cleanBuffer(message,1000);
    if(DEBUG==true){
      Serial.printf("\r\n{DeviceID:\"%s\",Temp:%f,Level:%f,Dist:%f,BaseLevel:%f}\r\n",
            this->deviceIMEI, this->temp, this->currentUsage, this->currentLevel, this->baseLevel);
      Serial.flush();
    }
    int d = sprintf(message, "{DeviceID:'%s',Temp:%d,Level:%d,Dist:%d,BaseLevel:%d}",
            this->deviceIMEI, (int)(this->temp*100), (int)(this->currentUsage*100), (int)(this->currentLevel*100), (int)(this->baseLevel*100));
    return message;
  }
  
  void mainLoop(){
    while(true){
      RFduino_ULPDelay(this->sensorTimeoutLength);
      this->currentUsage = this->getCurrentUsage();
      char* message = this->statusMessage();
      if(enableBLESend == true){
        RFduinoBLE.sendFloat(this->temp);
        RFduinoBLE.sendFloat(this->currentLevel);
      }
      if(DEBUG == true){
        Serial.print("SENSED VALUE\r\n--------\r\n");
        this->sendTestMessageSerial(this->davidPhone,message);
        Serial.flush();
      }
      //Change is below threshold, and there is still more than 20CM in the bin
      //Do not report
      if((this->lastReading - this->currentLevel) < this->messageThreshold && (this->currentLevel >= 20 && this->lastReading != this->currentLevel) ){
        continue;
      }
      this->lastReading = this->currentLevel;
      if(DEBUG == true){
        Serial.print("\r\nMESSAGE BODY\r\n--------\r\n");
        this->sendTestMessageSerial(this->davidPhone,message);
        Serial.flush();
      }
      // David Skinner 226 791 6646
      if(DEBUG == false && enableGSMSend == true){
        Power_On_GSM();
        sendSMS(this->twilioPhone, message);
        Power_Off_GSM();
      }
    }
  }
  void beginOperation(){
    while(true){
      Serial.print("Starting Operation\r\n");
      this->lastReading = this->baseLevel;
      //Send first message - Registration
      if(DEBUG == false && enableGSMSend == true){
        this->currentUsage = this->getCurrentUsage();
        char* message = this->statusMessage();
        char initialMessage[1000];
        sprintf(initialMessage,"Register Bin: %s\r\n %s",this->deviceIMEI,message);
        Power_On_GSM();
        sendSMS(this->davidPhone, initialMessage);
        Power_Off_GSM();
        Power_On_GSM();
        sendSMS(this->twilioPhone, initialMessage);
        Power_Off_GSM();
      }
      this->mainLoop();
    }
  }


};

//returns distance in cm taking into account temperature
float Ultra_Sample(float temp)
{
  float c = 331.3 + 0.606 * temp;
  float duration = 0;
  for (int i = 0; i <= 5; i++) //averaging to reduce error
  {
    pinMode(Ultra_Sig, OUTPUT);
    digitalWrite(Ultra_Sig, LOW);
    delayMicroseconds(2);
    digitalWrite(Ultra_Sig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Ultra_Sig, LOW);
    pinMode(Ultra_Sig, INPUT);
    duration += pulseIn(Ultra_Power, HIGH, 20000);
    delay(10);
  }
  digitalWrite(Ultra_Power, LOW);
  pinMode(Ultra_Power, INPUT);
  duration /= 5 * 1000000; //averages and microseconds to seconds
  return (duration * c / 2) * 100;
}

void cleanBuffer(char *buffer, int count)
{
  for (int i = 0; i < count; i++) {
    buffer[i] = '\0';
  }
}

int readBuffer(char *buffer, int count, unsigned int timeOut)
{
  int i = 0;
  unsigned long timerStart, timerEnd;
  timerStart = millis();
  while (1) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\r' || c == '\n') c = '$';
      buffer[i++] = c;
      if (i > count - 1)break;
    }
    if (i > count - 1)break;
    timerEnd = millis();
    if (timerEnd - timerStart > 1000 * timeOut) {
      break;
    }
  }
  delay(500);
  while (Serial.available()) {  // display the other thing..
    Serial.read();
  }
  return 0;
}

//check SIMCard status
int checkSIM()
{
  char gprsBuffer[32];
  int count = 0;
  cleanBuffer(gprsBuffer, 32);
  while (count < 3) {
    Serial.print("AT+CPIN?\r\n");
    readBuffer(gprsBuffer, 32, 5);
    Serial.print(gprsBuffer);
    if ((NULL != strstr(gprsBuffer, "+CPIN: READY"))) {
      break;
    }
    count++;
    delay(1000);
  }
  if (count == 3) {
    return -1;
  }
  return 0;
}

char* getDeviceIMEI(){
  sendSync();
  char imeiBuffer[15];
  int count = 0;
  cleanBuffer(imeiBuffer, 15);
  Serial.print("AT+GSN=?\r\n");
  Serial.flush();
  while (count < 3) {
    Serial.print("AT+GSN\r\n");
    readBuffer(imeiBuffer, 15, 5);
    Serial.printf("%s",imeiBuffer);
    if (imeiBuffer[0] != '\0') {
      break;
    }
    count++;
    delay(1000);
  }
  return imeiBuffer;
}

//Toggle GSM module power
void Power_On_GSM() {
  pinMode(GSM_Power, OUTPUT);
  //Serial.print("Power On GSM");
  digitalWrite(GSM_Power, HIGH);
  delay(200);
  digitalWrite(GSM_Power, LOW); //enable power on module
  delay(1200);
  digitalWrite(GSM_Power, HIGH);
  for (int i = 0; i < 12; i++) {
    Serial.print("AT\r\n");//syncronize transmission
    delay(1000);
  }
//  Serial.flush();
//  if(0 != checkSIM()) Serial.print("ERROR:checkSIMStatus\r\n\r\n");
  Serial.flush();
  //Serial.flush();
  delay(1000);
}

void Power_Off_GSM(){
    digitalWrite(GSM_Power, HIGH);
    delay(200);
    digitalWrite(GSM_Power, LOW); //disable power on module
    delay(1200);
    pinMode(GSM_Power, INPUT);
}

void sendSync(){
  for (int i = 0; i < 12; i++) {
    Serial.print("AT\r\n");//syncronize transmission
    delay(1000);
  }
}

void sendSMS(char *number, char* message)
{
  Serial.flush();
  Serial.print("AT+CMGF=1\r\n");  // switch to message mode
  delay(1000);
  Serial.flush();
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"\r\n", number);
  Serial.print(cmd);
  delay(1000);
  Serial.printf("%s",message);
  Serial.print((char)26);
  
  delay(6000);
}


void setup() {
  pinMode(txPin, OUTPUT);
  pinMode(rxPin, INPUT);
  digitalWrite(txPin, HIGH);
  digitalWrite(rxPin, HIGH);
  Serial.begin(9600);
  RFduinoBLE.deviceName = "Bin Monitor";
  RFduinoBLE.advertisementData = "Bin Monitor";
  // start the BLE stack
  RFduinoBLE.begin();
  // get the bin class for the first time
  BinProperties& bin = BinProperties::getBinProperties();
  bin.initBin();
  bin.beginOperation();
}

void loop(){
  
}
