#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define PHSensorPin 34
#define VREF 3.3
#define OFFSET 0.00
#define SCOUNT 30

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage, phValue;

SemaphoreHandle_t xMutex;

void readAnalogTask(void *pvParameters)
{
  static unsigned long analogSampleTimepoint = millis();
  while (1)
  {
    if (millis() - analogSampleTimepoint > 30U)
    {
      analogSampleTimepoint = millis();
      analogBuffer[analogBufferIndex] = analogRead(PHSensorPin);
      analogBufferIndex++;
      if (analogBufferIndex == SCOUNT)
        analogBufferIndex = 0;
    }
    vTaskDelay(10); // Adjust the delay based on the desired task frequency
  }
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
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


void printDataTask(void *pvParameters)
{
  static unsigned long printTimepoint = millis();
  while (1)
  {
    if (millis() - printTimepoint > 1000U)
    {
      printTimepoint = millis();
      if (xSemaphoreTake(xMutex, portMAX_DELAY))
      {
        for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
        {
          analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
        }
        xSemaphoreGive(xMutex);

        averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;
        phValue = 3.5 * averageVoltage + OFFSET;

        Serial.print("Voltage:");
        Serial.print(averageVoltage, 2);
        Serial.print("   pH value:");
        Serial.println(phValue, 2);

        digitalWrite(2, HIGH);
        delay(1000);
        digitalWrite(2, LOW);
      }
    }
    vTaskDelay(10); // Adjust the delay based on the desired task frequency
  }
}


void setup()
{
  Serial.begin(9600);
  pinMode(PHSensorPin, INPUT);
  pinMode(2, OUTPUT);

  xMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
      readAnalogTask,
      "ReadAnalogTask",
      10000,
      NULL,
      1,
      NULL,
      0);

  xTaskCreatePinnedToCore(
      printDataTask,
      "PrintDataTask",
      10000,
      NULL,
      1,
      NULL,
      1);
}


void loop()
{
  // Empty, as all functionality is in tasks
}
