#include <Wire.h>
#define LCD_DLUGOSC_LINII 16
#include <kd_LCD.h>
#include <kd_SimpleButton.h>
#include <INA226_WE.h>
#define I2C_ADDRESS 0x40

int interruptPin = 2;
volatile bool event = false;

LiquidCrystal_I2C lcd(0x27, 2, LCD_DLUGOSC_LINII);
kd_DebounceButton tareButton(4);

INA226_WE ina226(I2C_ADDRESS);
// INA226_WE ina226 = INA226_WE(); // Alternative: sets default address 0x40

uint16_t power[60];
uint32_t currentTime;
uint8_t currentSecond;
uint8_t previousSecond;
uint8_t secondsFromStart;
uint8_t readingIdx;
bool setTare;

void setup() {
  Serial.begin(115200);
  Serial.println("setup start");
  Wire.begin();
  ina226.init();
  Serial.println("ina setup");
  lcd_init();
  Serial.println("lcd setup");
  lcd_wyczysc_wszystko();
  tareButton.begin();
  memset(power, 9, sizeof(power));
  currentSecond = previousSecond = 0;
  secondsFromStart = 0;
  readingIdx = 0;
  setTare = false;

  /* Set Number of measurements for shunt and bus voltage which shall be averaged
  * Mode *     * Number of samples *
  AVERAGE_1            1 (default)
  AVERAGE_4            4
  AVERAGE_16          16
  AVERAGE_64          64
  AVERAGE_128        128
  AVERAGE_256        256
  AVERAGE_512        512
  AVERAGE_1024      1024
  */
  ina226.setAverage(AVERAGE_16);

  /* Set conversion time in microseconds
     One set of shunt and bus voltage conversion will take: 
     number of samples to be averaged x conversion time x 2
     
     * Mode *         * conversion time *
     CONV_TIME_140          140 µs
     CONV_TIME_204          204 µs
     CONV_TIME_332          332 µs
     CONV_TIME_588          588 µs
     CONV_TIME_1100         1.1 ms (default)
     CONV_TIME_2116       2.116 ms
     CONV_TIME_4156       4.156 ms
     CONV_TIME_8244       8.244 ms  
  */
  ina226.setConversionTime(CONV_TIME_8244); // Conversion ready after conversion time x number of averages x 2
  
  /* Set measure mode
  POWER_DOWN - INA219 switched off
  TRIGGERED  - measurement on demand
  CONTINUOUS  - Continuous measurements (default)
  */
  //ina226.setMeasureMode(CONTINUOUS); // choose mode and uncomment for change of default
  
  /* Set Current Range
    * Mode *   * Max Current *
     MA_400          400 mA
     MA_800          800 mA (default)
  */
  //ina226.setCurrentRange(MA_800); // choose gain and uncomment for change of default
  
  /* If the current values delivered by the INA226 differ by a constant factor
     from values obtained with calibrated equipment you can define a correction factor.
     Correction factor = current delivered from calibrated equipment / current delivered by INA226
  */
  ina226.setCorrectionFactor(0.89830);
  
//   Serial.println("INA226 Current Sensor Example Sketch - Continuous_Alert_Controlled");
  
  attachInterrupt(digitalPinToInterrupt(interruptPin), alert, FALLING);

  ina226.enableConvReadyAlert(); // an interrupt will occur on interrupt pin when conversion is ready
}

void loop() {
  currentTime = millis();
  currentSecond = (currentTime/1000) % 60;
  if (currentSecond != previousSecond) {
    Serial.print("cur sec ");Serial.println(currentSecond);
      previousSecond = currentSecond;
      power[currentSecond] = 0;
      if (secondsFromStart < 60) {
          secondsFromStart++;
      }
      readingIdx = 0;
      printResults();
  }
  if (tareButton.isPressed()) {
    setTare = true;
  }

  if(event) {
    ina226.readAndClearFlags(); // reads interrupt and overflow flags and deletes them 
    power[currentSecond] = (ina226.getBusPower()+0.5
        + power[currentSecond]*readingIdx)/(readingIdx+1);
    readingIdx++;
    event = false;
    attachInterrupt(digitalPinToInterrupt(interruptPin), alert, FALLING);
  }
}

/*
VXX.X wXXXXsXXXX
aXXXX xXXXXmXXXX
*/
char line[LCD_DLUGOSC_LINII+1];

void printValueToLine(int idx, uint16_t v, char c) {
    if (v < 9999) {
        snprintf(line+idx, 6, "%c%4u", c, v);
    } else {
        // snprintf(line+idx, 5, "%c%2u.%u", toupper(c), v/1000, 
            // (v-(v/1000)*1000)/100);
        snprintf(line+idx, 6, "%c%2.1f", (char)toupper(c), double(v/1000));
    }
}

uint16_t getAvgOf(int seconds) {
  uint32_t sum = 0;
  for (int i = 0; i < seconds; ++i) {
    sum += power[(currentSecond + 59 - i) % 60];
  }
  sum /= seconds;
  return sum;
}

float tare_mA = 0.0;
float tare_mW = 0.0;
void printResults() {
  if (ina226.overflow) {
      lcd_pisz(0, "=== OVERFLOW ===");
      return;
  }
  double mV = ina226.getShuntVoltage_mV() + ina226.getBusVoltage_V()*1000;
  double V = mV / 1000.0;
  double mA = ina226.getCurrent_mA();
  double mW = ina226.getBusPower();
  Serial.print("mA=");Serial.println(mA);
  // double r = mV / mA;
  // Serial.print("R="); Serial.println(r);
  if (setTare) {
    if (tare_mA == mA) { // values from INA are up to 0.04
      tare_mA = 0.0;
      tare_mW = 0.0;
    } else {
      tare_mA = mA;
      tare_mW = mW;
    }
    setTare = false;
  }
  mA -= tare_mA;
  mW -= tare_mW;

  char cV = 'V';
  if (tare_mA != 0.0) {
    cV = 'O';
  }
  if (V < 10.0) {
    snprintf(line, 7, "%c%4.2f ", cV, V);
  } else {
    snprintf(line, 7, "%c%4.1f ", cV, V);
  }
  printValueToLine(6, mW, 'w');
  if (secondsFromStart >= 5) {
    printValueToLine(11, getAvgOf(5), 's');
  }
  lcd_pisz_str(0, line);

  if (mA < 10.0) {
    snprintf(line, 7, "a%4.2f ", mA);
  } else if (mA < 100.0) {
    snprintf(line, 7, "a%4.1f ", mA);
  } else if (mA < 1000.0) {
    snprintf(line, 7, "A.%3u ", (unsigned)mA);
  } else if (mA < 10000.0) {
    snprintf(line, 7, "A%4.2f ", mA/1000.0);
  } else {
    snprintf(line, 7, "A%4.1f ", mA/1000.0);
  }
  if (secondsFromStart >= 30) {
    printValueToLine(6, getAvgOf(30), 'x');
  }
  if (secondsFromStart >= 60) {
    printValueToLine(11, getAvgOf(59), 'm');
  }
  lcd_pisz_str(1, line);
}

void alert() {
  event = true;
  detachInterrupt(2);
}