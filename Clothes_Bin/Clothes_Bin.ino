#include <RFduinoBLE.h>
#include <Float.h>

#define GSM_Power 2
#define Ultra_Power 3
#define Ultra_Sig 6
#define txPin 1 
#define rxPin 0
#define DEBUG true
#define SENSITIVITY 5 //Sensitivity of sensor in CM
#define STABLE_SAMPLE_COUNT 3 //Number of samples before setting base level
#define START_SAMPLE_DELAY 3

class BinProperties{
  private:
    BinProperties(){};//Static Constructor
    BinProperties(BinProperties const&);// Don't Implement
    void operator=(BinProperties const&); // Don't implement
    char* deviceIMEI;
    bool enableGSMSend = false;
    bool enableBLESend = false;
    char* davidPhone = "+12267916646";
    char* twilioPhone = "+17806664662";
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
    float baseLevel = 0.0;//Empty Distance in CM
    float currentLevel = 0.0;//Distance in CM
    float currentUsage = 0.0;//Usage in %
    float temp;

  static BinProperties& getBinProperties(){
      static BinProperties  instance; // Guaranteed to be destroyed.
      return instance;// Instantiated on first use.
  }
  /**
  uint64_t id = getdeviceIMEI();
  Serial.print("Device ID:");
  Serial.print(getdeviceIMEILow(), HEX);
  Serial.print(getdeviceIMEIHigh(), HEX);
  Serial.printf("\r\nTemp= %f Dist= %f\r\n", temp, distance);
  */
  float getCurrentUsage(){
    this->temp = RFduino_temperature(CELSIUS) - 5;
    this->currentLevel = Ultra_Sample(this->temp);
    this->currentUsage = (this->currentLevel/this->baseLevel)*100.0;
    return this->currentUsage;
  }
  
  void initBin(){
      Power_On_GSM();
      Serial.flush();
      this->deviceIMEI = getDeviceIMEI();
      Serial.flush();
      Power_Off_GSM();
      Serial.flush();
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
    char message[500];
    int d = sprintf(message, "{deviceIMEI:\"%s\",Temp:%f,Level:%f,Dist:%f}",
            this->deviceIMEI,
            this->temp,
            this->currentUsage,
            this->currentLevel);
    Serial.printf("{deviceIMEI:\"%s\",Temp:%f,Level:%f,Dist:%f}",
            this->deviceIMEI,
            this->temp,
            this->currentUsage,
            this->currentLevel);
    Serial.flush();
    return message;
  }
  
  void mainLoop(){
    while(true){
      this->getCurrentUsage();
      if(this->enableBLESend){
        RFduinoBLE.sendFloat(this->temp);
        RFduinoBLE.sendFloat(this->currentLevel);
      }
      char* message = this->statusMessage();
      if(DEBUG == true){
        Serial.print("CALLING TEST MESSAGE");
        this->sendTestMessageSerial(this->davidPhone,message);
      }
      // David Skinner 226 791 6646
      if(DEBUG == false && this->enableGSMSend == true){
        Power_On_GSM();
        Serial.flush();
        sendSMS(this->davidPhone, message);
        Power_Off_GSM();
      }
      RFduino_ULPDelay( SECONDS(30) );
    }
  }
  void beginOperation(){
    while(true){
      Serial.print("Starting Operation\r\n");
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
  Serial.flush();
  if(0 != checkSIM()) Serial.print("ERROR:checkSIMStatus\r\n\r\n");
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
  sendSync();
  Serial.print("AT+CMGF=1\r\n");  // switch to message mode
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
