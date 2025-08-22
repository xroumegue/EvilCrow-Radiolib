#include <Arduino.h>
#include <FreeRTOS_Shell.h>
#include <RadioLib.h>
#include <SPI.h>

#ifndef SPI_CLK_FREQ
#define SPI_CLK_FREQ 1000000
#endif

static const int spiClk = SPI_CLK_FREQ;

#define CC1101_MOD_SCK    14
#define CC1101_MOD_MISO   12
#define CC1101_MOD_MOSI   13

#define CC1101_MOD1_CSN    5
#define CC1101_MOD1_GDO0   2
#define CC1101_MOD1_GDO2   4
#define CC1101_MOD1_IRQ   CC1101_MOD1_GDO0
#define CC1101_MOD1_GPIO  CC1101_MOD1_GDO2

#define CC1101_MOD2_CSN   27
#define CC1101_MOD2_GDO0  25
#define CC1101_MOD2_GDO2  26
#define CC1101_MOD2_IRQ   CC1101_MOD2_GDO0
#define CC1101_MOD2_GPIO  CC1101_MOD2_GDO2

#define USE_RADIO2

SPIClass spi0 = SPIClass(HSPI);
#ifdef USE_RADIO1
CC1101 radio =  new Module(CC1101_MOD1_CSN, CC1101_MOD1_IRQ, RADIOLIB_NC, CC1101_MOD1_GPIO,
                          spi0, SPISettings(spiClk, MSBFIRST, SPI_MODE0));
#endif
#ifdef USE_RADIO2
CC1101 radio =  new Module(CC1101_MOD2_CSN, CC1101_MOD2_IRQ, RADIOLIB_NC, CC1101_MOD2_GPIO,
                          spi0, SPISettings(spiClk, MSBFIRST, SPI_MODE0));
#endif

static const uint32_t subghz_frequency_list[] = {
  /* 300 - 348 */
  300000000,
  302757000,
  303875000,
  303900000,
  304250000,
  307000000,
  307500000,
  307800000,
  309000000,
  310000000,
  312000000,
  312100000,
  312200000,
  313000000,
  313850000,
  314000000,
  314350000,
  314980000,
  315000000,
  318000000,
  330000000,
  345000000,
  348000000,
  350000000,

  /* 387 - 464 */
  387000000,
  390000000,
  418000000,
  430000000,
  430500000,
  431000000,
  431500000,
  433075000, /* LPD433 first */
  433220000,
  433420000,
  433657070,
  433889000,
  433920000, /* LPD433 mid */
  434075000,
  434176948,
  434190000,
  434390000,
  434420000,
  434620000,
  434775000, /* LPD433 last channels */
  438900000,
  440175000,
  464000000,
  467750000,

  /* 779 - 928 */
  779000000,
  868350000,
  868400000,
  868800000,
  868950000,
  906400000,
  915000000,
  925000000,
  928000000,
};

typedef struct
{
  uint32_t frequency_coarse;
  int rssi_coarse;
  uint32_t frequency_fine;
  int rssi_fine;
} FrequencyRSSI;

const int rssi_threshold = -75;


static const uint32_t uartRxTimeout = 1;
static const uint32_t uartBaudrate = 115200;

void FreeRTOS_ShellOutput(const char *buffer, int length) {
  Serial.write(buffer, length);
}

//void serialEvent(void) {
void FreeRTOS_Shell_cb(void) {
  while (Serial.available() > 0) {
    uint8_t incomingByte = Serial.read();
    FreeRTOS_ShellIRQHandle(incomingByte);
  }
}

void FreeRTOS_Shell_init(void) {
  Serial.setRxBufferSize(512);
  Serial.begin(uartBaudrate);
  while (!Serial) {
  };

  Serial.setRxTimeout(uartRxTimeout);
  Serial.onReceive(FreeRTOS_Shell_cb, true);
}

void toggleLED(void * parameter){
  int led = 32;
  pinMode(led, OUTPUT);
  for(;;){ // infinite loop

    // Turn the LED on
    digitalWrite(led, HIGH);

    // Pause the task for 500ms
    vTaskDelay(1500 / portTICK_PERIOD_MS);

    // Turn the LED off
    digitalWrite(led, LOW);

    // Pause the task again for 500ms
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void setup() {
  xTaskCreate(
    toggleLED,    // Function that should be called
    "Toggle LED",   // Name of the task (for debugging)
    1008,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    NULL             // Task handle
  );

  xTaskCreate(
    FreeRTOS_Shell,
    "Shell",
    2048,
    NULL,
    10,
    NULL
  );

#if 0
#ifdef USE_RADIO1
#ifndef USE_RADIO2
  pinMode(CC1101_MOD2_CSN, OUTPUT);
  digitalWrite(CC1101_MOD2_CSN, HIGH);
#endif
  spi0.begin(CC1101_MOD_SCK, CC1101_MOD_MISO, CC1101_MOD_MOSI, CC1101_MOD1_CSN);
  Serial.print(F("[CC1101] Initializing Module 1 ... "));
#endif

#ifdef USE_RADIO2
#ifndef USE_RADIO1
  pinMode(CC1101_MOD1_CSN, OUTPUT);
  digitalWrite(CC1101_MOD1_CSN, HIGH);
#endif

  spi0.begin(CC1101_MOD_SCK, CC1101_MOD_MISO, CC1101_MOD_MOSI, CC1101_MOD2_CSN);
  Serial.print(F("[CC1101] Initializing Module 2 ... "));
#endif

  delay(100);


  radio.begin();
  Serial.println("Frequency Analyzer Started!");
#endif
}

void loop() {
#if 0
  int rssi;
  FrequencyRSSI frequency_rssi = {
    .frequency_coarse = 0, .rssi_coarse = -100, .frequency_fine = 0, .rssi_fine = -100
  };

  // First stage: coarse scan
  radio.setRxBandwidth(650); // 58, 68, 81, 102, 116, 135, 162, 203, 232, 270, 325, 406, 464, 541, 650 and 812 kHz    (81kHz seems to work best for me)
  size_t array_size = sizeof(subghz_frequency_list) / sizeof(subghz_frequency_list[0]);
  for (size_t i = 0; i < array_size; i++) {
    uint32_t frequency = subghz_frequency_list[i];
    if (frequency != 467750000 && frequency != 464000000 && frequency != 390000000 && frequency != 312000000 && frequency != 312100000 && frequency != 312200000 && frequency != 440175000) {
      radio.setFrequency((float)frequency / 1000000.0);
      radio.receiveDirect();
      delay(2);
      rssi = radio.getRSSI();

      if (frequency_rssi.rssi_coarse < rssi) {
        frequency_rssi.rssi_coarse = rssi;
        frequency_rssi.frequency_coarse = frequency;
      }
    }
  }

  // Second stage: fine scan
  if (frequency_rssi.rssi_coarse > rssi_threshold) {
    // for example -0.3 ... 433.92 ... +0.3 step 20KHz
    radio.setRxBandwidth(58);
    for (uint32_t i = frequency_rssi.frequency_coarse - 300000; i < frequency_rssi.frequency_coarse + 300000; i += 20000) {
      uint32_t frequency = i;
      radio.setFrequency((float)frequency / 1000000.0);
      radio.receiveDirect();
      delay(2);
      rssi = radio.getRSSI();

      if (frequency_rssi.rssi_fine < rssi) {
        frequency_rssi.rssi_fine = rssi;
        frequency_rssi.frequency_fine = frequency;
      }
    }
  }

  // Deliver results fine
  if (frequency_rssi.rssi_fine > rssi_threshold) {
    Serial.printf("FINE        Frequency: %.2f  RSSI: %d\n", (float)frequency_rssi.frequency_fine / 1000000.0, frequency_rssi.rssi_fine);
  }

  // Deliver results coarse
  else if (frequency_rssi.rssi_coarse > rssi_threshold) {
    Serial.printf("COARSE      Frequency: %.2f  RSSI: %d\n", (float)frequency_rssi.frequency_coarse / 1000000.0, frequency_rssi.rssi_coarse);
  }
  #endif
  delay(10);
}

