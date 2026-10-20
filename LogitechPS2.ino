#include <libmaple/iwdg.h>
#include "dwt.h"
#include <USBComposite.h>

//#define DEBUG
//#define BENCHMARK // I measured about 125 Hz

//#define USE_LED
#define MIN_LITTLE_RUMBLE 64
#define BIG_RUMBLE_OUT_MIN 170

static const uint32_t cyclesPerUS = (SystemCoreClock / 1000000ul);
// 16MHz nrf24e1, 4 oscillator clocks per machine cycle, so 0.25 microseconds per machine cycle
static const uint32_t nrfCycles = (cyclesPerUS / 4);
static const uint32_t markCycleMin = 15 * nrfCycles;
static const uint32_t oneCycleMin = 15 * nrfCycles / 2;
static const uint32_t TIMEOUT = nrfCycles * 40;
static const uint32_t rumbleBitCycles = nrfCycles * 30;
static const uint32_t rumbleWaitCycles = nrfCycles * 20;

const uint32_t watchdogSeconds = 3;
 
#ifdef BENCHMARK
# define BENCHMARK_PIN PB12
int benchmarkState = 0;
#endif

#define PRESSURE_SENSITIVE_COUNT 12

const uint8_t pressureSensitiveToXBox[PRESSURE_SENSITIVE_COUNT] =
{
  XBOX_DRIGHT, XBOX_DLEFT, XBOX_DUP, XBOX_DDOWN,
  XBOX_Y, XBOX_X, XBOX_B, XBOX_A,
  XBOX_LSHOULDER, XBOX_RSHOULDER,
  0,0
};

union pressure_u {
  struct pressureButtons_s {
    uint8_t dpad_right;
    uint8_t dpad_left;
    uint8_t dpad_up;
    uint8_t dpad_down;
    
    uint8_t tri;
    uint8_t o;
    uint8_t x;
    uint8_t sqr;
    
    uint8_t l1;
    uint8_t r1;
    uint8_t l2;
    uint8_t r2;
  } buttons;
  uint8_t values[PRESSURE_SENSITIVE_COUNT];
};

struct inputData {
  uint8_t unused;
  uint8_t rx;
  uint8_t ry;
  uint8_t lx;
  uint8_t ly;
  union pressure_u pressure;
  uint8_t digitalButtons;
  uint8_t checksum;
};

#define EEPROM_MIN_VALUE 0
#define EEPROM_MAX_VALUE 20

pressure_u maxAtIdle;
pressure_u maxValues;
pressure_u minValues;

// byte 13: front (near user) and back connector: L1 (upper)
// byte 14: front (near user) and back connector: R1 (upper)
// byte 15: front and middle connectors: L2 (lower) -- bad
// byte 16: front and middle connectors: R2 (lower)
// byte 17: digital
// byte 18: checksum

#define LED PC13  
#define COMM PB11
#define PACKET_SIZE 19
struct inputData input;
uint32_t commMask;
volatile uint32_t* commReceive;
volatile uint32_t* commSend;
#ifdef USE_LED
gpio_dev* ledPort;
uint32_t ledPin;
#endif

#define CALIBRATE_COUNT 10

#define DEBOUNCE_MILLIS      40

uint32_t vibrateReleaseTime = 0;
uint8_t prevButtons = 0xFF; // active low

uint32_t sumLX = 0;
uint32_t sumLY = 0;
uint32_t sumRX = 0;
uint32_t sumRY = 0;
uint32_t dataCount = 0;

int32_t centerLX = 128;
int32_t centerLY = 128;
int32_t centerRX = 128;
int32_t centerRY = 128;

int32_t scaleLX = 128;
int32_t scaleLY = 128;
int32_t scaleRX = 128;
int32_t scaleRY = 128;

uint8_t rumbleBig = 0;
uint8_t rumbleLittle = 0;
bool vibrate = true;
uint32_t changedSettingTime = 0;
const uint32_t changedSettingWriteDelayMillis = 2000;

USBXBox360 XBox360;

void initDWT() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= 1;
}

void rumble(uint8_t left, uint8_t right) {
  if (left == 0)
    rumbleBig = 0;
  else
    rumbleBig = BIG_RUMBLE_OUT_MIN + ((255-BIG_RUMBLE_OUT_MIN) * left + 128)/ 255;
  rumbleBig = left;
  rumbleLittle = right;
}

void setup() {
  EEPROM8_init();
  for (int i=0; i<PRESSURE_SENSITIVE_COUNT; i++) {
    minValues.values[i] = EEPROM8_getValue(EEPROM_MIN_VALUE+i);
    maxValues.values[i] = EEPROM8_getValue(EEPROM_MAX_VALUE+i);
    if (minValues.values[i] == 0)
      minValues.values[i] = 1;
    if (maxValues.values[i] < minValues.values[i])
      maxValues.values[i] = minValues.values[i];
  }
  iwdg_init(IWDG_PRE_256, watchdogSeconds*156);
  initDWT();
#ifdef USE_LED  
  pinMode(LED,OUTPUT);
  digitalWrite(LED,1);
  ledPort = digitalPinToPort(LED);
  ledPin = PIN_MAP[LED].gpio_bit;
#endif
  pinMode(COMM,INPUT_PULLDOWN);
#ifdef BENCHMARK
  pinMode(BENCHMARK_PIN,OUTPUT);
#endif  
  commReceive = &(digitalPinToPort(COMM)->regs->IDR);
  commSend = &(digitalPinToPort(COMM)->regs->ODR); // communicate via pullups
  commMask = digitalPinToBitMask(COMM);
#ifdef DEBUG
  Serial.begin();
  while (!Serial) ;
#else  
  USBComposite.setProductString("LogitechPS2ToXBox360");
  XBox360.begin();
  XBox360.setManualReportMode(true);
  XBox360.setRumbleCallback(rumble);
#endif  
}

bool readData() {
  uint8_t bitMask = 1;  
  uint32_t pos = 0;
  uint8_t* data = (uint8_t*)&input;
  data[0] = 0;
  for (pos=0;pos<PACKET_SIZE;) {
    DWT->CYCCNT = 0;
    while (0 == (*commReceive & commMask)) {
      if (DWT->CYCCNT > TIMEOUT) {
        return false; 
      }
    }
    DWT->CYCCNT = 0;
    while ((*commReceive & commMask)) {
      if (DWT->CYCCNT > TIMEOUT) {
        return false;
      }
    }
    if (DWT->CYCCNT > oneCycleMin) {
      data[pos] |= bitMask;
    }
    bitMask <<= 1;
    if (bitMask == 0) {
      pos++;
      if (pos < PACKET_SIZE)
        data[pos] = 0;
      bitMask = 1;
    }
  }
  uint8_t sum = 0;
  for (pos=0;pos<PACKET_SIZE-1;pos++) {
    sum += data[pos];
  }
  if (sum != input.checksum) {
    return false;
  }

  return true;
}

int32_t getScale(int32_t center) 
{
  if (255 - center < center)
    return 255-center;
  return center;
}

int32_t scaleAxis(uint8_t datum, int32_t center, int32_t scale)
{
  int32_t x = (datum-center) * 32768 / scale;
  if (x < -32768)
    return -32768;
  else if (x > 32767)
    return 32767;  
  else
    return x;
}

uint8_t getMinFromMax(uint8_t v)
{
  if (v == 0)
    return 3;
  if (v < 40)
    return v + 5;
  return v + 10;
}

void writeRumble() {
  uint8_t big;
  uint8_t little;
  if (vibrate) {
    big = rumbleBig;
    little = rumbleLittle;
  }
  else {
    big = 0;
    little = 0;
  }
  *commSend &= ~commMask;
  DWT->CYCCNT = 0;
  while (DWT->CYCCNT < rumbleWaitCycles);
  DWT->CYCCNT = 0;
  *commSend |= commMask;
  while (DWT->CYCCNT < rumbleBitCycles / 2);
  DWT->CYCCNT = 0;
  if (little >= MIN_LITTLE_RUMBLE) 
    *commSend |= commMask;
  else
    *commSend &= ~commMask;
  while (DWT->CYCCNT < rumbleBitCycles);
  DWT->CYCCNT = 0;
  for (uint8_t mask = 0x80 ; mask ; mask >>= 1) {
    while (DWT->CYCCNT < rumbleBitCycles);
    DWT->CYCCNT = 0;
    if (big & mask)
      *commSend |= commMask;
    else
      *commSend &= ~commMask;
  }
  while (DWT->CYCCNT < rumbleBitCycles);
  *commSend &= ~commMask;
}

uint8_t scaleSlider(uint8_t p,uint8_t minValue,uint8_t maxValue) {
  if (p < minValue)
    return 0;
  return (p - (minValue-1)) * (unsigned int)255 / (maxValue - (minValue-1));
}

void writeData() {
#ifdef DEBUG
  char s[50];
  for (int i=0;i<sizeof(input);i++) {
    uint8_t x = ((uint8_t*)&input)[i];
    sprintf(s, "%02x ", x);
    Serial.write(s); 
  }
  Serial.write("\n");
#endif
  if (dataCount < CALIBRATE_COUNT) {
    if (dataCount == 0 && (input.digitalButtons & 0x10)) {
      for (int i=0; i < PRESSURE_SENSITIVE_COUNT ; i++) {
        minValues.values[i] = 1;
        maxValues.values[i] = 1;
      }
      changedSettingTime = millis();
    }
    sumLX += input.lx;
    sumRX += input.rx;
    sumLY += input.ly;
    sumRY += input.ry;
    for (int i=0; i<PRESSURE_SENSITIVE_COUNT; i++) {
      if (input.pressure.values[i] > maxAtIdle.values[i]) {
        maxAtIdle.values[i] = input.pressure.values[i];
        changedSettingTime = millis();
      }
    }
    dataCount++;
    if (dataCount == CALIBRATE_COUNT) {
      centerLX = (sumLX + CALIBRATE_COUNT/2) / CALIBRATE_COUNT;
      centerRX = (sumRX + CALIBRATE_COUNT/2) / CALIBRATE_COUNT;
      centerLY = (sumLY + CALIBRATE_COUNT/2) / CALIBRATE_COUNT;
      centerRY = (sumRY + CALIBRATE_COUNT/2) / CALIBRATE_COUNT;
      scaleLX = getScale(centerLX);
      scaleRX = getScale(centerRX);
      scaleLY = getScale(centerLY);
      scaleRY = getScale(centerRY);
      for (int i=0; i<PRESSURE_SENSITIVE_COUNT; i++) {
        uint8_t v = getMinFromMax(maxAtIdle.values[i]);
        if (v > minValues.values[i]) {
          minValues.values[i] = v;
          changedSettingTime = millis();
        }
        if (v > maxValues.values[i]) {
          maxValues.values[i] = v;          
          changedSettingTime = millis();
        }
#ifdef DEBUG
        sprintf(s,"%d: %d %d %d", i, maxAtIdle.values[i], minValues.values[i], maxValues.values[i]);
        Serial.println(s);
#endif        
                        
      }
    }
    return;
  }
#ifdef DEBUG
  sprintf(s,"%d,%d %d,%d", scaleAxis(input.lx, centerLX, scaleLX),scaleAxis(input.ly, centerLY, scaleLY),scaleAxis(input.rx, centerRX, scaleRX),scaleAxis(input.ry, centerRY, scaleRY));
  Serial.println(s);
#endif    
  uint16_t buttons = 0;
  for (int i=0; i<PRESSURE_SENSITIVE_COUNT; i++) {
    uint8_t p = input.pressure.values[i];
    if (p > maxValues.values[i]) {
#ifdef DEBUG
      sprintf(s, "update %d->%u", i,p);
      Serial.println(s);
#endif      
      maxValues.values[i] = p;
      changedSettingTime = millis();
    }
    unsigned xbox = pressureSensitiveToXBox[i];
    if (xbox && p >= minValues.values[i]) {
      buttons |= 1 << (xbox-1);
    }
  }
  if (0 == (input.digitalButtons & (1<<3)))
    buttons |= 1 << (XBOX_START-1);
  if (0 == (input.digitalButtons & 1)) // select
    buttons |= 1 << (XBOX_BACK-1);
  if (0 == (input.digitalButtons & 0x10))
    buttons |= 1 << (XBOX_GUIDE-1);
  if (0 == (input.digitalButtons & (1<<1)))
    buttons |= 1 << (XBOX_L3-1);
  if (0 == (input.digitalButtons & (1<<2)))
    buttons |= 1 << (XBOX_R3-1);
  if ((input.digitalButtons ^ prevButtons) & 0x20) {
    // vibrate changed
    if (input.digitalButtons & 0x20) {
      vibrateReleaseTime = millis();
    }
    else {
      if (millis() >= vibrateReleaseTime + DEBOUNCE_MILLIS) {
        vibrate = !vibrate;
      }        
    }
  }
  prevButtons = input.digitalButtons;

#ifdef DEBUG
  sprintf(s,"buttons %04x", buttons);    
  Serial.println(s);
#else
  XBox360.buttons(buttons);
  XBox360.X(scaleAxis(input.lx, centerLX, scaleLX));
  XBox360.Y(scaleAxis(input.ly, centerLY, scaleLY));
  XBox360.XRight(scaleAxis(input.rx, centerRX, scaleRX));
  XBox360.YRight(scaleAxis(input.ry, centerRY, scaleRY));
  XBox360.sliderLeft(scaleSlider(input.pressure.buttons.l2,minValues.buttons.l2,maxValues.buttons.l2));
  XBox360.sliderRight(scaleSlider(input.pressure.buttons.r2,minValues.buttons.r2,maxValues.buttons.r2));
  XBox360.send();
#endif

  if (changedSettingTime > 0 && millis() >= changedSettingTime + changedSettingWriteDelayMillis) {
    for (int i=0; i<PRESSURE_SENSITIVE_COUNT; i++) {
#ifdef DEBUG
      sprintf(s,"store %d", i);    
      Serial.println(s);
#endif      
      EEPROM8_storeValue(EEPROM_MIN_VALUE+i, minValues.values[i]);
      EEPROM8_storeValue(EEPROM_MAX_VALUE+i, maxValues.values[i]);
    }
    changedSettingTime = 0;
  }
}

void loop() {
  iwdg_feed();
  if (*commReceive & commMask) {
      DWT->CYCCNT = 0;
      while (*commReceive & commMask); // iwdg_feed()?
      if (DWT->CYCCNT < markCycleMin) {
        // too short for mark
        return;
      }
      bool success;
      noInterrupts();
#ifdef USE_LED      
      gpio_write_bit(ledPort, ledPin, 0);
#endif
      success = readData(); 
      if (success) {         
        writeRumble();
        interrupts();
        writeData();
#ifdef BENCHMARK
        digitalWrite(BENCHMARK_PIN, (benchmarkState = !benchmarkState));
#endif
      }
      else {
        interrupts();
      }
#ifdef USE_LED      
      gpio_write_bit(ledPort, ledPin, 1);
#endif      
  }
}
