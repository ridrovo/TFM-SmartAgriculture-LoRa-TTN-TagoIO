  #include <lmic.h>
  #include <hal/hal.h>
  #include <SPI.h>
  #include <TinyGPS.h>
  #include <SoftwareSerial.h>

// include the DHT11 Sensor Library
  #include "DHT.h"
  #include <DHT_U.h>

  TinyGPS gps;
  SoftwareSerial ss(4, 3); // Arduino RX, TX to conenct to GPS module.


// DHT digital pin and sensor type
   #define DHTPIN A5
   #define DHTTYPE DHT11
// init. DHT
   DHT dht(DHTPIN, DHTTYPE);

  unsigned long chars;
  unsigned short sentences, failed;
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
   #ifdef COMPILE_REGRESSION_TEST
    #define FILLMEIN 0
   #else
    #warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
    #define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
   #endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
   static const u1_t PROGMEM APPEUI[8]={ 0x25, 0x12, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
   void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
   static const u1_t PROGMEM DEVEUI[8]={ 0x9E, 0xC5, 0x24, 0xD3, 0x56, 0xBE, 0x39, 0x00 };
   void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
   static const u1_t PROGMEM APPKEY[16] = { 0x8A, 0x10, 0x78, 0xBC, 0xDE, 0x33, 0x00, 0xE2, 0x08, 0x7D, 0xC0, 0x33, 0x89, 0x12, 0x1F, 0x87 };
   void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}



/* These callbacks are only used in over-the-air activation, so they are
  left empty here (we cannot leave them out completely unless
   DISABLE_JOIN is set in config.h, otherwise the linker will complain).*/

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).

   #define TX_INTERVAL 30


//static void smartdelay(unsigned long ms);


   static osjob_t sendjob;
 
// Pin mapping
   const lmic_pinmap lmic_pins = {
        .nss = 10,
        .rxtx = LMIC_UNUSED_PIN,
        .rst = 5, // LMIC_UNESED_PIN (before change)
        .dio = {2, 3, LMIC_UNUSED_PIN}, // 2 6 7 (before change)
   };


void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(F(": "));
        switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED - Connected to TTN"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            Serial.println(F(""));
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    ini:
    // Check if there is not a current TX/RX job running
          uint32_t  tem = dht.readTemperature(false)*100;
          Serial.print("Temperature: ");
          Serial.print((tem/100),DEC);
          //float humidity = dht.read(DHTPIN);
          uint32_t  hum = dht.readHumidity(false)*100;
          Serial.print(" - Humidity: ");
          Serial.println((hum/100),DEC);
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else if (tem!=0 || hum!=0) {
          byte payload[15];
          float flat, flon, falt;
          long unsigned int hdopGPS;
          unsigned long age;

          gps.f_get_position(&flat, &flon);
          hdopGPS == (gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
          flat    == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;//save six decimal places          
          flon    == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;//save six decimal places
          falt    == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : falt, 2;//save two decimal places
          
          Serial.print(F("Latitude: "));
          Serial.print(flat);
          Serial.print(F(" - Longitude: "));
          Serial.print(flon);
          Serial.print(F(" - Altitude: "));
          Serial.print(falt);
          Serial.print(F(" - HDOP: "));
          Serial.println(hdopGPS);
          
          int32_t latitude  = flat    * 10000;
          int32_t longitude = flon    * 10000;
          int32_t altituded = falt    * 10000;
          int32_t HDOP      = hdopGPS / 10;
          
          payload[0] = highByte(tem);
          payload[1] = lowByte(tem);
          payload[2] = highByte(hum);
          payload[3] = lowByte(hum);
          payload[4] = latitude;
          payload[5] = latitude >> 8;
          payload[6] = latitude >> 16;
          payload[7] = longitude;
          payload[8] = longitude >> 8;
          payload[9] = longitude >> 16;
          payload[10]= altituded;
          payload[11]= altituded >> 8;
          payload[12]= altituded >> 16;
          payload[13]= HDOP;
          payload[14]= HDOP >> 8;
          payload[15]= HDOP >> 16;
          
          LMIC_setTxData2(1,  (uint8_t*)payload, sizeof(payload), 0);
          //LMIC_setTxData2(1,  (uint8_t*)coords, sizeof(coords), 0);
          Serial.println(F("Packet queued"));
    }else{
    // Next TX is scheduled after TX_COMPLETE event.
    goto ini;
    }
}

   void setup() {
        delay(5000);
        Serial.begin(9600);
        dht.begin();
        ss.begin(9600);
        Serial.println(F("Starting - LoRa/GPS - Temperature&Humidity/DHT11"));
        smartdelay(2000);
       
        #ifdef VCC_ENABLE
        // For Pinoccio Scout boards
        pinMode(VCC_ENABLE, OUTPUT);
        digitalWrite(VCC_ENABLE, HIGH);
        delay(1000);
        #endif
    
        // LMIC init
        os_init();
        // Reset the MAC state. Session and pending data transfers will be discarded.
        LMIC_reset();
        LMIC.dn2Dr = DR_SF9;
        // Start job (sending automatically starts OTAA too)
        do_send(&sendjob);
        //smartdelay(2000);
   }


    static void smartdelay(unsigned long ms)
    {
      unsigned long start = millis();
      do 
      {
        while (ss.available())
          gps.encode(ss.read());
      } while (millis() - start < ms);
    }

void loop() {       
    os_runloop_once();
    Serial.flush();
    ss.flush();
}
