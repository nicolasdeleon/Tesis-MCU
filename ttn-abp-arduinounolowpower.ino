#include <LowPower.h>

/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define DEBUG 0 // Define 0 in production

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0xA1, 0x0A, 0xF8, 0xC8, 0x60, 0x7C, 0xA1, 0xF8, 0xC9, 0x6B, 0xB3, 0x85, 0xE7, 0x04, 0xFD, 0x7B };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0xAF, 0x93, 0x06, 0x8C, 0x60, 0x6F, 0xC0, 0x2C, 0x9A, 0x42, 0x69, 0x6A, 0xCF, 0xE3, 0x8B, 0xC6 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x260C7167 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = {0x00, 0x23, 0x00, 0x00};
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60*3;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, LMIC_UNUSED_PIN},
};

// SENSOR CONSTANTS DEFINITIONS

// These constants won't change. They're used to give names to the pins used:
const int ADC_VOLTAGE_RANGE = 5;
const float STEP_VALUE = ADC_VOLTAGE_RANGE / 1024.0; // Esto es en V
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
// const int analogOutPin = 9; // Analog output pin that the LED is attached to
const int digitalPin = 8;

const int minValue = 100;  //cuando el sensor esta 100% al aire. Este valor sale de la experiencia propia
const int maxValue = 390;  //cuando el sensor esta 100% en agua. Este valor sale de la experiencia propia
const float range = maxValue - minValue;

const int sensorMinValue = 1000; //el valor minimo que devuelve el sensor es de 1000mv
const int sensorMaxValue = 2500; //el valor maximo que devuelve el sensor es de 2500mv
const float sensorRange = sensorMaxValue - sensorMinValue;

const float stepValue = sensorRange/range;


// Variables will change:
int digitalState = LOW;         // digitalState used to set the pin

void onEvent (ev_t ev) {
    #if DEBUG
    Serial.print(os_getTime());
    Serial.print(": ");
    #endif
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            #if DEBUG
            Serial.println(F("EV_SCAN_TIMEOUT"));
            #endif
            break;
        case EV_BEACON_FOUND:
            #if DEBUG
            Serial.println(F("EV_BEACON_FOUND"));
            #endif
            break;
        case EV_BEACON_MISSED:
            #if DEBUG
            Serial.println(F("EV_BEACON_MISSED"));
            #endif
            break;
        case EV_BEACON_TRACKED:
            #if DEBUG
            Serial.println(F("EV_BEACON_TRACKED"));
            #endif
            break;
        case EV_JOINING:
            #if DEBUG
            Serial.println(F("EV_JOINING"));
            #endif
            break;
        case EV_JOINED:
            #if DEBUG
            Serial.println(F("EV_JOINED"));
            #endif
            break;
        case EV_RFU1:
            #if DEBUG
            Serial.println(F("EV_RFU1"));
            #endif
            break;
        case EV_JOIN_FAILED:
            #if DEBUG
            Serial.println(F("EV_JOIN_FAILED"));
            #endif
            break;
        case EV_REJOIN_FAILED:
            #if DEBUG
            Serial.println(F("EV_REJOIN_FAILED"));
            #endif
            break;
        case EV_TXCOMPLETE:
            #if DEBUG
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            #endif
            for (int i=0; i<int(TX_INTERVAL/8); i++) {
              // Use library from https://github.com/rocketscream/Low-Power
              LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
              delay(1000);
            }
            if (LMIC.txrxFlags & TXRX_ACK)
              #if DEBUG
              Serial.println(F("Received ack"));
              #endif
            if (LMIC.dataLen) {
              #if DEBUG
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
              #endif
            }
            // Schedule next transmission
            os_setCallback(&sendjob, do_send);
            break;
        case EV_LOST_TSYNC:
            #if DEBUG
            Serial.println(F("EV_LOST_TSYNC"));
            #endif
            break;
        case EV_RESET:
            #if DEBUG
            Serial.println(F("EV_RESET"));
            #endif
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            #if DEBUG
            Serial.println(F("EV_RXCOMPLETE"));
            #endif
            break;
        case EV_LINK_DEAD:
            #if DEBUG
            Serial.println(F("EV_LINK_DEAD"));
            #endif
            break;
        case EV_LINK_ALIVE:
            #if DEBUG
            Serial.println(F("EV_LINK_ALIVE"));
            #endif
            break;
         default:
            #if DEBUG
            Serial.println(F("Unknown event"));
            #endif
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
      #if DEBUG
      Serial.println(F("OP_TXRXPEND, not sending"));
      #endif
    } else {
      // Prepare upstream data transmission at the next possible time.
      // Read information from sensor
      int sensorValueInMiliVolts = readHumiditySensor();
      memcpy((uint8_t*) mydata, (uint8_t*) &sensorValueInMiliVolts, sizeof(int));

      LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
      #if DEBUG
      Serial.println(F("Packet queued"));
      #endif
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


int readHumiditySensor() {
    // Send pulse to read sensor
    float sensorValue = 0;        // value read from the pot
    float mappedValue = 0;

    float discardFirstMess = analogRead(analogInPin);
    delay(1000);
    digitalWrite(digitalPin, HIGH);
    delay(10);
    sensorValue = analogRead(analogInPin);
    digitalWrite(digitalPin, LOW);

    // Read value from analog input
    mappedValue = map(sensorValue, 0.0, 1023.0, 0.0, 3227);
         
    // print the results to the Serial Monitor:
    #if DEBUG
    Serial.print("sensor = ");
    Serial.println(sensorValue);
    Serial.print("sensor mV = ");
    Serial.println(mappedValue);
    #endif

    return (int) mappedValue;
  }

void setup() {
    #if DEBUG
    Serial.begin(115200);
    Serial.println(F("Starting"));
    Serial.println(F("Init Sensor"));
    #endif

    // Set up In/Out Pins and other definitions
    pinMode(digitalPin, OUTPUT);


    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF10, 14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
