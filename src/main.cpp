#include <Arduino.h>

#define PHSensorPin  34    //dissolved oxygen sensor analog output pin to arduino mainboard
#define VREF 3.3    //for arduino uno, the ADC reference is the AVCC, that is 5.0V(TYP)
#define OFFSET 0.00  //zero drift compensation

#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    //store the analog value in the array, readed from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;

float averageVoltage,phValue;


void setup() {
  Serial.begin(9600);
  pinMode(PHSensorPin,INPUT);
  pinMode(2, OUTPUT); // LED bawaan ESP32 terhubung ke GPIO 2
  // put your setup code here, to run once:
}

int getMedianNum(int bArray[], int iFilterLen)
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      {
      bTab[i] = bArray[i];
      }
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++)
      {
      for (i = 0; i < iFilterLen - j - 1; i++)
          {
        if (bTab[i] > bTab[i + 1])
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

void loop() {
  // put your main code here, to run repeatedly:
   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 30U)     //every 30 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(PHSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT)
         analogBufferIndex = 0;
   }
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 1000U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
      {
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      }
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4096.0; // read the value more stable by the median filtering algorithm
      phValue = 3.5 * averageVoltage + OFFSET;
      Serial.print("Voltage:");
      Serial.print(averageVoltage,2);
      Serial.print("   pH value:");
      Serial.println(phValue,2);
      digitalWrite(2, HIGH);
      delay(1000); // tunggu 1 detik
      digitalWrite(2, LOW); // matikan LED
   }
}