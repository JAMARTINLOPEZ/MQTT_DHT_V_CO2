#define ARM_MATH_CM0PLUS

#include <SPI.h>
#include  <ADE9153A.h>
#include <ADE9153AAPI.h>

/*Basic initializations*/
ADE9153AClass ade9153A;
#define SPI_SPEED 1000000     //SPI Speed
#define CS_PIN 8              //8-->Arduino Zero. 15-->ESP8266 
#define ADE9153A_RESET_PIN 4  //On-board Reset Pin
#define USER_INPUT 5          //On-board User Input Button Pin
#define LED 6                 //On-board LED pin

EnergyRegs energyVals;  //Energy register values are read and stored in EnergyRegs structure
PowerRegs powerVals;    //Metrology data can be accessed from these structures
RMSRegs rmsVals;  
PQRegs pqVals;
AcalRegs acalVals;
Temperature tempVal;

void readandwrite(void);
void resetADE9153A(void);

int ledState = LOW;
int inputState = LOW;
unsigned long lastReport = 0;
const long reportInterval = 2000;
const long blinkInterval = 500;

void setup() {
  /*Pin and serial monitor setup*/
  pinMode(LED, OUTPUT);
  pinMode(USER_INPUT, INPUT);
  pinMode(ADE9153A_RESET_PIN, OUTPUT);
  digitalWrite(ADE9153A_RESET_PIN, HIGH);  
  Serial.begin(115200);

  resetADE9153A();            //Reset ADE9153A for clean startup
  delay(1000);
  /*SPI initialization and test*/
  bool commscheck = ade9153A.SPI_Init(SPI_SPEED,CS_PIN); //Initialize SPI
  if (!commscheck) {
    Serial.println("ADE9153A Shield not detected. Plug in Shield and reset the Arduino");
    while (!commscheck) {     //Hold until arduino is reset
      delay(1000);
    }
  }
  
  ade9153A.SetupADE9153A(); //Setup ADE9153A according to ADE9153AAPI.h
  delay(500); 
}

void loop() {
  /*Main loop*/
  /*Returns metrology to the serial monitor and waits for USER_INPUT button press to run autocal*/
  unsigned long currentReport = millis();
  
  if ((currentReport - lastReport) >= reportInterval){
    lastReport = currentReport;
    readandwrite();
  }
  
  inputState = digitalRead(USER_INPUT);

  if (inputState == LOW) {
    Serial.println("Autocalibrating Current Channel");
    ade9153A.StartAcal_AINormal();
    runLength(20);
    ade9153A.StopAcal();
    Serial.println("Autocalibrating Voltage Channel");
    ade9153A.StartAcal_AV();
    runLength(40);
    ade9153A.StopAcal();
    delay(100);
    
    ade9153A.ReadAcalRegs(&acalVals);
    ade9153A.ApplyAcal(acalVals.AICC, acalVals.AVCC);

    Serial.println("Autocalibration Complete");
    delay(2000);
  }
}

void readandwrite()
{

 /*Read and Print Specific Register using ADE9153A SPI Library */
  //Serial.println(ade9153A.SPI_Read_32(REG_AIRMS),HEX); // AIRMS
  
 /*Read and Print WATT Register using ADE9153A Read Library*/
  ade9153A.ReadPowerRegs(&powerVals);    //Template to read Power registers from ADE9000 and store data in Arduino MCU
  ade9153A.ReadRMSRegs(&rmsVals);
  ade9153A.ReadPQRegs(&pqVals);
  ade9153A.ReadTemperature(&tempVal);
  
  Serial.print("RMS Current:\t");        
  Serial.print(rmsVals.CurrentRMSValue/1000); 
  Serial.println(" A");
  
  Serial.print("RMS Voltage:\t");        
  Serial.print(rmsVals.VoltageRMSValue/1000);
  Serial.println(" V");
  
  Serial.print("Active Power:\t");        
  Serial.print(powerVals.ActivePowerValue/1000);
  Serial.println(" W");
  
  Serial.print("Reactive Power:\t");        
  Serial.print(powerVals.FundReactivePowerValue/1000);
  Serial.println(" VAR");
  
  Serial.print("Apparent Power:\t");        
  Serial.print(powerVals.ApparentPowerValue/1000);
  Serial.println(" VA");
  
  Serial.print("Power Factor:\t");        
  Serial.println(pqVals.PowerFactorValue);
  
  Serial.print("Frequency:\t");        
  Serial.print(pqVals.FrequencyValue);
  Serial.println(" Hz");
  
  Serial.print("Temperature:\t");        
  Serial.print(tempVal.TemperatureVal);
  Serial.println(" degC");

  Serial.println("");
  Serial.println("");
}
void resetADE9153A(void)
{
 digitalWrite(ADE9153A_RESET_PIN, LOW);
 delay(100);
 digitalWrite(ADE9153A_RESET_PIN, HIGH);
 delay(1000);
 Serial.println("Reset Done");
}

void runLength(int seconds)
{
  unsigned long startTime = millis();
  
  while (millis() - startTime < (seconds*1000)){
    digitalWrite(LED, HIGH);
    delay(blinkInterval);
    digitalWrite(LED, LOW);
    delay(blinkInterval);
  }  
}

