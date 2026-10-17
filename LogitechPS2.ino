#include <libmaple/iwdg.h>
#include "dwt.h"
#include <SegaController.h>
#include <USBXBox360.h>

//#define DEBUG
//#define BENCHMARK // I measured about 125 Hz

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


struct inputData {
  uint8_t unused;
  uint8_t rx;
  uint8_t ry;
  uint8_t lx;
  uint8_t ly;
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
  uint8_t digitalButtons;
  uint8_t checksum;
};

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
gpio_dev* ledPort;
uint32_t ledPin;

#define CALIBRATE_COUNT 10

#define ANALOG_MINIMUM 1

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

uint8_t maxR1 = 1;
uint8_t maxL1 = 1;
uint8_t maxR2 = 1;
uint8_t maxL2 = 1;

uint8_t maxUp = 0;
uint8_t maxLeft = 0;
uint8_t maxRight = 0;
uint8_t maxDown = 0;

uint8_t minUp = 0;
uint8_t minLeft = 0;
uint8_t minRight = 0;
uint8_t minDown = 0;

uint8_t rumbleBig = 0;
uint8_t rumbleLittle = 0;

USBXBox360 XBox360;

inline int16_t range8u16s(uint16_t x) {
  // TODO: fix
  return (((int32_t)(uint32_t)x - 128) * 32767 + 64) / 128;
}

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
  iwdg_init(IWDG_PRE_256, watchdogSeconds*156);
  initDWT();
  pinMode(LED,OUTPUT);
  digitalWrite(LED,1);
  pinMode(COMM,INPUT_PULLDOWN);
#ifdef BENCHMARK
  pinMode(BENCHMARK_PIN,OUTPUT);
#endif  
  commReceive = &(digitalPinToPort(COMM)->regs->IDR);
  commSend = &(digitalPinToPort(COMM)->regs->ODR); // communicate via pullups
  commMask = digitalPinToBitMask(COMM);
  ledPort = digitalPinToPort(LED);
  ledPin = PIN_MAP[LED].gpio_bit;
#ifdef DEBUG
  Serial.begin();
  while (!Serial) ;
  Serial.println("Hello");
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
    return ANALOG_MINIMUM;
  if (v < 40)
    return v + 5;
  return v + 10;
}

void writeRumble() {
  *commSend &= ~commMask;
  DWT->CYCCNT = 0;
  while (DWT->CYCCNT < rumbleWaitCycles);
  DWT->CYCCNT = 0;
  *commSend |= commMask;
  while (DWT->CYCCNT < rumbleBitCycles / 2);
  DWT->CYCCNT = 0;
  if (rumbleLittle >= MIN_LITTLE_RUMBLE) 
    *commSend |= commMask;
  else
    *commSend &= ~commMask;
  while (DWT->CYCCNT < rumbleBitCycles);
  DWT->CYCCNT = 0;
  for (uint8_t mask = 0x80 ; mask ; mask >>= 1) {
    while (DWT->CYCCNT < rumbleBitCycles);
    DWT->CYCCNT = 0;
    if (rumbleBig & mask)
      *commSend |= commMask;
    else
      *commSend &= ~commMask;
  }
  while (DWT->CYCCNT < rumbleBitCycles);
  *commSend &= ~commMask;
}

void writeData() {
#ifdef DEBUG
  for (int i=0;i<sizeof(input);i++) {
    uint8_t x = ((uint8_t*)&input)[i];
    char s[4];
    sprintf(s, "%02x ", x);
    Serial.write(s); 
  }
  Serial.write("\n");
#endif
  if (dataCount < CALIBRATE_COUNT) {
    sumLX += input.lx;
    sumRX += input.rx;
    sumLY += input.ly;
    sumRY += input.ry;
    if (input.dpad_up > maxUp)
      maxUp = input.dpad_up;
    if (input.dpad_down > maxDown)
      maxDown = input.dpad_down;
    if (input.dpad_left > maxLeft)
      maxLeft = input.dpad_left;
    if (input.dpad_right > maxRight)
      maxRight = input.dpad_right;
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
      minUp = getMinFromMax(maxUp);
      minDown = getMinFromMax(maxDown);
      minLeft = getMinFromMax(maxLeft);
      minRight = getMinFromMax(maxRight);
    }
    return;
  }
  if (input.r1 > maxR1)
    maxR1 = input.r1;
  if (input.l1 > maxL1)
    maxL1 = input.l1;
  if (input.r2 > maxR2)
    maxR2 = input.r2;
  if (input.l2 > maxL2)
    maxL2 = input.l2;
#ifdef DEBUG
  char s[50];
  sprintf(s,"%d,%d %d,%d", scaleAxis(input.lx, centerLX, scaleLX),scaleAxis(input.ly, centerLY, scaleLY),scaleAxis(input.rx, centerRX, scaleRX),scaleAxis(input.ry, centerRY, scaleRY));
  Serial.println(s);
#endif    
  uint16_t buttons = 0;
  if (input.dpad_left >= minLeft)
    buttons |= 1 << (XBOX_DLEFT-1);
  if (input.dpad_right >= minRight)
    buttons |= 1 << (XBOX_DRIGHT-1);
  if (input.dpad_up >= minUp)
    buttons |= 1 << (XBOX_DUP-1);
  if (input.dpad_down >= minDown)
    buttons |= 1 << (XBOX_DDOWN-1);
  if (input.tri >= ANALOG_MINIMUM)
    buttons |= 1 << (XBOX_Y-1);
  if (input.sqr >= ANALOG_MINIMUM)
    buttons |= 1 << (XBOX_X-1);
  if (input.o >= ANALOG_MINIMUM)
    buttons |= 1 << (XBOX_B-1);
  if (input.x >= ANALOG_MINIMUM)
    buttons |= 1 << (XBOX_A-1);
  if (input.l1 >= ANALOG_MINIMUM)
    buttons |= 1 << (XBOX_LSHOULDER-1);
  if (input.r1 >= ANALOG_MINIMUM)
    buttons |= 1 << (XBOX_RSHOULDER-1);
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
#ifdef DEBUG
  sprintf(s,"buttons %04x", buttons);    
  Serial.println(s);
  return;
#endif
  XBox360.buttons(buttons);
  XBox360.X(scaleAxis(input.lx, centerLX, scaleLX));
  XBox360.Y(scaleAxis(input.ly, centerLY, scaleLY));
  XBox360.XRight(scaleAxis(input.rx, centerRX, scaleRX));
  XBox360.YRight(scaleAxis(input.ry, centerRY, scaleRY));
  XBox360.sliderLeft( (input.l2 * 255 + maxL2/2) / maxL1 );
  XBox360.sliderRight( (input.r2 * 255 + maxR2/2) / maxR1 );
  XBox360.send();
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
      gpio_write_bit(ledPort, ledPin, 0);
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
      gpio_write_bit(ledPort, ledPin, 1);
  }
}
