#include "LoRaWan_APP.h"
#include "Arduino.h"

/* Link to library https://github.com/HelTecAutomation/Heltec_ESP32
   For LoRa Basic Tools settings in Arduino IDE bar have no effect
   all settings need to be set manually in code

   For radio/radioEvents detailed docummentation check radio library located in
   Arduino\packages\HeTltec-esp32\hardware\esp32\0.0.7\libraries\LoraWan102\src\radio
*/

#define RF_FREQUENCY 868000000 // Hz
/*
Set according to region, for EU 868MHz.
*/

#define DEVICE_ID 01
/*
As lora is broadcasting transmition form of identification can turn out
useful for device differentiation
*/

#define TX_OUTPUT_POWER 14 // dBm
                           //[14,17,22]

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

#define BUFFER_SIZE 30 // Define the payload size here

char txpacket[BUFFER_SIZE];

double txNumber;

bool lora_idle = true;

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);

void setup()
{
  Serial.begin(115200); // Update this to match the LattePanda's baud rate
  Mcu.begin();

  txNumber = 0;

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
}

void loop()
{
  if (lora_idle == true)
  {
    while (Serial.available() > 0)
    {
      char receivedChar = Serial.read();
      txpacket[strlen(txpacket)] = receivedChar;
    }

    if (strlen(txpacket) > 0)
    {
      txNumber += 0.01;
      Serial.printf("\r\nsending packet \"%s\" , length %d\r\n", txpacket, strlen(txpacket));
      Radio.Send((uint8_t *)txpacket, strlen(txpacket));
      txpacket[0] = '\0'; // Clear the packet buffer
      lora_idle = false;
    }
  }
  Radio.IrqProcess();
}

void OnTxDone(void)
{
  Serial.println("TX done......");
  lora_idle = true;
}

void OnTxTimeout(void)
{
  Radio.Sleep();
  Serial.println("TX Timeout......");
  lora_idle = true;
}
