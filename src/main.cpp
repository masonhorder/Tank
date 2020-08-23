#include <Arduino.h>

// tank goes on the right side - red antenna




//   TTTTTTTTTTTTTTTTTTTTTTTTTTTTTT                                                     kk
//                 TT                                                                   kk
//                 TT                                                                   kk       kk
//                 TT                  aaaaaaaaaaaaaa                                   kk     kk
//                 TT                               aaa                                 kk   kk
//                 TT                                aa                                 kk kk
//                 TT                                aa         nn nnnnnnnnnnn          kkkk
//                 TT                   aaaaaaaaaaaaaaa         nnnn        nnn         kk kk
//                 TT                  aa            aa         nn           nn         kk   kk
//                 TT                  aa            aa         nn           nn         kk     kk
//                 TT                  aa            aa         nn           nn         kk       kk
//                 TT                  aa            aa         nn           nn         kk         kk
//                 TT                   aaaaaaaaaaaaaa          nn           nn         kk           kk



//                                   ##
//                                     ##
//                                       ##
//                                         ##          
//                                           ##
//        ######################################
//                                          ##
//                                        ##  
//                                      ##
//                                    ##
//                                  ##


///////////////
// LIBRARIES //
///////////////

// libraries do result in some warnings - the program still works

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <SparkFun_TB6612.h>


///////////////





#define RF69_FREQ 915.0

// who am i? (server address)
#define MY_ADDRESS     1


#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4

unsigned long lastConnection = 0;



#define connectionLed 13


#define redRGBPin 10
#define greenRGBPin 11
#define blueRGBPin 12


#define VBATPIN A7

#define PWMA 19
#define AIN2 13
#define AIN1 16
#define BIN1 17
#define BIN2 18
#define PWMB 6
#define STBY 15


const int offsetA = 1;
const int offsetB = 1;

int rightMotorDriveSpeed;
int leftMotorDriveSpeed;

Motor rightMotor = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor leftMotor = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


int connectionStatus = 2;






float rtdata[100];








// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);


int16_t packetnum = 0;  // packet counter, we increment per xmission


void rgbColor(int redColor, int greenColor, int blueColor) {
  analogWrite( redRGBPin, redColor);
  analogWrite( greenRGBPin, greenColor);
  analogWrite( blueRGBPin, blueColor);
}

void connectionLedOn() {
  digitalWrite(connectionLed,HIGH);
}

void connectionLedOff() {
  digitalWrite(connectionLed,LOW);
}



void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(connectionLed, OUTPUT);  

  pinMode(redRGBPin, OUTPUT); 
  pinMode(greenRGBPin, OUTPUT); 
  pinMode(blueRGBPin, OUTPUT); 

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather Addressed RFM69 RX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}


// Dont put this on the stack:
uint8_t data[] = "And hello back to you";
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void loop() {
  
  
  if (rf69_manager.available()){
    connectionLedOn();
    connectionStatus = 1;
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (rf69_manager.recvfromAck(buf, &len, &from)) {
      buf[len] = 0; // zero out remaining string
      
      Serial.print("Got packet from #"); Serial.print(from);
      Serial.print(" [RSSI :");
      Serial.print(rf69.lastRssi());
      Serial.print("] : ");
      Serial.println((char*)buf);
      memcpy(rtdata,(char*)buf,sizeof(buf));

      // for (byte i = 0; i < 7; i++){
      //   //Serial.println(*(float *)&buf[i]);
      //   //Serial.print('\t');
      //   Serial.println(rtdata[i]);
      // }

      Serial.print(rtdata[0]);
      Serial.print(", ");
      Serial.print(rtdata[1]);
      Serial.print(", ");
      Serial.println(rtdata[2]);
      lastConnection = millis();

      leftMotorDriveSpeed = (rtdata[1]/200)*255;
      rightMotorDriveSpeed = (rtdata[0]/200)*255*-1;
      Serial.println(leftMotorDriveSpeed);
      Serial.println(rightMotorDriveSpeed);


      leftMotor.drive(leftMotorDriveSpeed);

      rightMotor.drive(rightMotorDriveSpeed);

      // Serial.print(rightMotorDriveSpeed);
      // Serial.println(leftMotorDriveSpeed);




      // Send a reply back to the originator client
      if (!rf69_manager.sendtoWait(data, sizeof(data), from)){
        Serial.println("Sending failed (no ack)");
        connectionLedOff();
        connectionStatus = 0;
      }
    }
  }
  
  else if(lastConnection == 0){
    connectionLedOff(); 
    connectionStatus = 0;
  }
  else if(millis() - lastConnection > 5000){
    connectionLedOff(); 
    connectionStatus = 0;
  }

  float measuredvbat = analogRead(VBATPIN);

  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  if(measuredvbat > 3.9){
    rgbColor(0,255,0);
    // rgbColor(0,255,255);
  }
  else if(measuredvbat > 3.7){
    rgbColor(255,166,0);
  }
  else if(measuredvbat <= 3.7){
    rgbColor(255,0,0);
  }
}











































































