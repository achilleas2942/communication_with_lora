#include "LoRaWan_APP.h"
#include "Arduino.h"

/* Link to library https://github.com/HelTecAutomation/Heltec_ESP32
   For LoRa Basic Tools settings in Arduino IDE bar have no effect
   all settings need to be set manually in code

   For radio/radioEvents detailed docummentation check radio library located in
   Arduino\packages\Heltec-esp32\hardware\esp32\0.0.7\libraries\LoraWan102\src\radio
*/

#define RF_FREQUENCY 868000000 // Hz (as discussed with Marios)
/*
Set according to region, for EU 868MHz.
*/

#define DEVICE_ID 02
/*
As lora is broadcasting transmition form of identification can turn out
useful for device differentiation
*/

#define LORA_BANDWIDTH 0 // [0: 125 kHz,
                         //  1: 250 kHz,
                         //  2: 500 kHz,
                         //  3: Reserved]
/*
Bigger transmition bandwidth improves transfer rate, but reduces the range.
*/

#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
/*
Bigger spreading factor improves the range, but also increase the latency.
*/

#define LORA_CODINGRATE 1 // [1: 4/5,
                          //  2: 4/6,
                          //  3: 4/7,
                          //  4: 4/8]
/*
Increasing coding rate decreases chance of dropping the packet,
but reduces data rate.
*/

#define LORA_PREAMBLE_LENGTH 8 // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0  // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define BUFFER_SIZE 40 // Define the payload size here

char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

int16_t rssi, rxSize;

bool lora_idle = true;

void setup()
{
  Serial.begin(115200); // Update this to match the LattePanda's baud rate
  Mcu.begin();

  rssi = 0;

  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
}

void loop()
{
  if (lora_idle)
  {
    lora_idle = false;
    Serial.println("into RX mode");
    Radio.Rx(0);
  }
  Radio.IrqProcess();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  rssi = rssi;
  rxSize = size;
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();
  Serial.printf("\r\nreceived packet \"%s\", rssi %d , length %d\r\n", rxpacket, rssi, rxSize);
  Serial.write(payload, size); // Write payload and size to serial connection for communication with LattePanda
  lora_idle = true;
}

