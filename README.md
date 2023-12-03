**DFRobot SEN0249 pH Sensor with ESP32**
This project utilizes the DFRobot SEN0249 pH Sensor with an ESP32 microcontroller to measure pH levels. The code is written in C++ using the PlatformIO framework within the Arduino environment. The project includes two FreeRTOS tasks to read analog data from the sensor and print the pH value.
**Hardware Setup**
- Connect the DFRobot SEN0249 pH Sensor to the ESP32 board.
- Connect the sensor's analog output to pin 34 on the ESP32.
- Connect the sensor's reference voltage to the ESP32's 3.3V pin.
- Connect the sensor's ground to the ESP32's ground.
**Code Explanation**
***Libraries Used***
- Arduino.h: Standard Arduino library for general functionalities.
- freertos/FreeRTOS.h: FreeRTOS library for multitasking.
***Constants***
- PHSensorPin: The pin to which the analog output of the pH sensor is connected (pin 34).
- VREF: The reference voltage for analog-to-digital conversion (3.3V).
- OFFSET: An offset value for pH calibration (0.00).
- SCOUNT: Number of samples for median filtering (30).
***Variables***
- analogBuffer: Buffer to store analog readings from the pH sensor.
- analogBufferTemp: Temporary buffer for median filtering.
- analogBufferIndex: Index to keep track of the current position in the analog buffer.
- averageVoltage: Calculated average voltage from the analog readings.
- phValue: Calculated pH value based on the average voltage.
- xMutex: FreeRTOS semaphore to protect critical sections of code.
**Tasks**
1. Read Analog Task (readAnalogTask):
   - Reads analog data from the pH sensor every 30 milliseconds.
   - Stores the data in the circular buffer analogBuffer.
2. Print Data Task (printDataTask):
   - Prints the pH value every second.
   - Uses a median filter to smooth out noise in the analog readings.
   - Uses a semaphore (xMutex) to protect the critical section when accessing the analog buffer.
**Functions**
- getMedianNum: Calculates the median value of an array.
**Setup**
- Initializes serial communication, pin modes, and creates FreeRTOS tasks.
- Task priorities are set to ensure proper execution.
- The loop function is empty, as all functionality is implemented using FreeRTOS tasks.
**Build and Upload**
1. Open the project in PlatformIO within the Arduino environment.
2. Connect the ESP32 to your computer.
3. Build and upload the code to the ESP32.
**Notes**
- Adjust the vTaskDelay values in both tasks to modify the task frequencies.
- Calibration may be required based on the specific characteristics of the pH sensor.
