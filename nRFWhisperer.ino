// nRFLoader - a sketch for testing nRF24 modules, and to enable
// OTA upgrade of devices running the nRFLoader bootloader
//
// (c) 2022-2025, Andrew Williams
// -------------------------
// Hardware Setup - Uses and LCDKEYPAD with a 2x16 LCD
// and an ESP24L01 Radio connected to SPI and pins 3&2
// Serial Port: Baud 115200
// ----------------------------------------------------
#include <SPI.h>
#include "RF24.h"
#include <printf.h>

#include <Wire.h>

#define LED_TEST
// For unknown reasons, typedef structures must come from external .h file
#include "lcd_menu.h"
#include "rf_debug.h"

// Serial Configuration ====
// =========================
#define BAUDRATE (115200)

// nRF Radio Hardware configuration ====
// on SPI bus plus pins 9 & 10
// =====================================
RF24 radio(3,2);


// LCD Hardware configuration ====
// ===============================
#ifdef USE_I2C_LCD
  #include <LiquidCrystal_I2C.h>
  // I2C LCD adapter
  LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3, POSITIVE);
#else
  #include <LiquidCrystal.h>
  const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
  LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
#endif


// Macro to clear existing line on the LCD
#define clearLine() lcd.print("                ");

// Keypad Driver ====
// using the Labrat's Lights Keypad Driver
// ==================
#include <LRL_Key.h>
static LRL_Key keypad;
#define getkey() keypad.getKey()


// Status LEDs
#define LED_GREEN (A1)
#define LED_BLUE  (A2)
#define LED_AMBER (A3)


// Global Parameters ------------------------------------------
// ------------------------------------------------------------
#define LOG_NORMAL  (0x00)
#define LOG_VERBOSE (0x01)


// gGlobal Variables
//
uint8_t gLogLevel = LOG_NORMAL;

// For ease of copying, storing the nRF address (3 bytes)
// as the three LSB of a uint32_t
typedef uint32_t tDeviceId;

// Addresses for this Device (Server) and the client device
tDeviceId addr_server= 0xC0DEC1; // Primary broadcast and receive for the nRFLoader
tDeviceId addr_p2p   = 0xC0DE02; // Pipe 2 - dedicated P2P address
tDeviceId addr_data  = 0xC0DE42;

tDeviceId addr_client= 0x123456; // Client device address

// nRF Radio Frequency
uint8_t gRFchan;
uint8_t gNRF_message[32];

// Timestamp for any timeout event
unsigned long gWaitTimeout=0;
unsigned long gHeartBeatTimeout=0;

tSelectItem cRFRateMenu[] = {
  {"1Mbps",0},
  {"2Mbps",1}
};
uint8_t indexRFRate= 1; //Default 2MBps

tSelectItem cRFChanMenu[] = {
  {"2470         ",70},
  {"2472         ",72},
  {"2474         ",74},
  {"2476         ",76},
  {"2478         ",78},
  {"2480 (Legacy)",80},
  {"2482         ",82}
};
uint8_t indexRFChan = 6; // Default to 2482

// A firmware write takes 3 nRF messages to complete (setup/write/bind)
// This structure maintains a cached copy of the transaction, in case
// of failure, and a need to re-transmit.
struct sWriteCmd {
  uint8_t addrL;
  uint8_t addrH;
  bool erase;
  uint8_t csum;
  uint8_t data[32];
} gWriteCmd;

// Serial Mode..
//    IDLE  - waiting on first byte of SYNC command
//    SYNCED - waiting for packet if (0x01 .. 0x03) all else back to IDLE
//    Parsing Packet type 1 - Setup <AddrL><AddrH><csum><erase>
//    Parsing Packet type 2 - 32 bytes of payload
//    Parsing Packet type 3 - 'R' = RESET
//    Parsing Packet type 4 - Audit <AddrL><AddrH><sizeL><sizeH><csumL><csumH><store>
//    Parsing Packet type 5 - (DeviceId)
//    Parsing Packet type 42 - SYNC request .. if see SYNC cmd, reply with SYNC response

// Counter for clocking bytes in from the Serial port
uint8_t gREAD_counter = 0;

// STATE = W4SERIAL | W4NRF | MODE
#define STATE_IDLE   (0x80|0x40|0x00)
#define STATE_LOG    (0x80|0x40|0x01)
#define STATE_DEVID  (0x80|0x40|0x02)
#define STATE_BIND   (0x00|0x40|0x03)
#define STATE_HXHDR  (0x80     |0x04)
#define STATE_HXDATA (0x80     |0x05)
#define STATE_SETUP  (0x00|0x40|0x06)
#define STATE_WRITE  (0x00|0x40|0x07)
#define STATE_COMMIT (0x00|0x40|0x08)
#define STATE_FINISH (0x80|0x00|0x09)
#define STATE_AUDIT  (0x00|0x40|0x0A)
#define STATE_RESET  (0x80|0x00|0x0B)

uint8_t gState = STATE_IDLE;
#define W4Radio(x) ((x&0x40)>0)
#define W4Serial(x) ((x&0x80)>0)
#define mode(x)    (x&0x0F)

#define major 0
#define minor 2

// Preventing Double reading of keypresses
int last_key = KEY_NONE;

// Eye Candy - idle display
char info_line[17];   // Text on 2nd line (changes every 2 seconds)
unsigned long splash_timeout = 0; // Timeout value for IDLE anmation
uint8_t phase = 0;                // Offset of string being displayed
uint8_t dir = 1;                  // Directino of travel for animation

// Splash Page: Initial strings during Arduino setup()
void splash() {
  // Splash Page
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("_nRF  WHISPERER_"));
  sprintf(info_line," LabRat  Lights ");
  lcd.setCursor(0,1);
  lcd.print(info_line);
}

// void idleRadio()
// Common code to restore the radio to an idle state
//   Pipe 1 incoming broadcasts
//   Pipe 2 incoming P2P channel

void idleRadio() {
      radio.openReadingPipe(1,addr_server);
      radio.openReadingPipe(2,addr_p2p);
}

// Configure the NRF24L01
//
void configRadio() {
      radio.setChannel(gRFchan);
      radio.setPayloadSize(32);
      radio.setAddressWidth(3);
      radio.setAutoAck(false);
      radio.setRetries(0x0F,0x0F); // Not time critical -so wait longer
      radio.setDataRate(RF24_2MBPS);
      radio.setCRCLength(RF24_CRC_16);
      radio.setPALevel(RF24_PA_LOW);
      idleRadio();
      radio.maskIRQ(true, true, true);
}

// ** EYE CANDY ** Customer Heart Icons for the Hitachi Display
// custom characters for the heartbeat
byte Heart[] = {
  B00000,
  B01010,
  B11111,
  B11111,
  B01110,
  B00100,
  B00000,
  B00000
};

byte Heart2[] = {
  B00000,
  B01010,
  B10101,
  B10001,
  B01010,
  B00100,
  B00000,
  B00000
};

byte BAR2[] = { B00000, B00000, B00000, B00000, B00000, B00000, B11111, B11111};
byte BAR3[] = { B00000, B00000, B00000, B00000, B00000, B11111, B11111, B11111};
byte BAR4[] = { B00000, B00000, B00000, B00000, B11111, B11111, B11111, B11111};
byte BAR5[] = { B00000, B00000, B00000, B11111, B11111, B11111, B11111, B11111};
byte BAR6[] = { B00000, B00000, B11111, B11111, B11111, B11111, B11111, B11111};
byte BAR7[] = { B00000, B11111, B11111, B11111, B11111, B11111, B11111, B11111};

uint8_t barmap[9] = {0x20, 0x5F,0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0xFF}; // 0x00 -> 8 bars (9 entries)
// ---------------
// Arduino Setup :
// ---------------
void setup() {
  // Serial port for communication with the python nrfLoad.py application
  Serial.begin(BAUDRATE);
  // Debugging Setup
  printf_begin();

  // I2C LCD
  lcd.begin(16,2);

  #ifdef USE_I2C_LCD
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  #endif

  lcd.createChar(0, Heart);  // Blinking Heart Icon
  lcd.createChar(1, Heart2);
  lcd.createChar(2, BAR2);
  lcd.createChar(3, BAR3);
  lcd.createChar(4, BAR4);
  lcd.createChar(5, BAR5);
  lcd.createChar(6, BAR6);
  lcd.createChar(7, BAR7);
  splash();

  // Default gRFchan - update to be an option to be passed in.
  gRFchan=cRFChanMenu[indexRFChan].value;
  addr_client = 0x00;
  gState = STATE_IDLE;

  // Setup the NRF Radio sub-system
  radio.begin();
  configRadio();
  radio.startListening(); // Note  ENRXADDR_P0 = 0
  delay(1000);
  Serial.println("nRFWhisperer Serial Console");
}

// Ugly Debug Code
void dumpMsg(uint8_t * msg,unsigned char num_param) {
  char tempstr[21];
  switch (num_param) {
    case 1:snprintf(tempstr,20,"%2.2X",msg[0]); break;
    case 2:snprintf(tempstr,20,"%2.2X%2.2X",msg[0],msg[1]); break;
    case 3:snprintf(tempstr,20,"%2.2X%2.2X%2.2X",msg[0],msg[1],msg[2]); break;
    case 4:snprintf(tempstr,20,"%2.2X%2.2X%2.2X%2.2X",msg[0],msg[1],msg[2],msg[3]); break;
    case 5:snprintf(tempstr,20,"%2.2X%2.2X%2.2X%2.2X%2.2X",msg[0],msg[1],msg[2],msg[3],msg[4]); break;
    case 7:snprintf(tempstr,20,"%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X",msg[0],msg[1],msg[2],msg[3],msg[4],msg[5],msg[6]); break;
    case 8:snprintf(tempstr,20,"%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X",msg[0],msg[1],msg[2],msg[3],msg[4],msg[5],msg[6],msg[7]); break;
    default: snprintf(tempstr,20,"Unsupported");break;
  }
  lcd.print(tempstr);
}

bool SendSetup(){
  uint8_t * msg = gNRF_message;
  bool retCode = false;

        // Build the message
        msg[0] = 0x80; // String to erase
        msg[1] = gWriteCmd.addrL;
        msg[2] = gWriteCmd.addrH;
        msg[3] = gWriteCmd.erase; // No ERASE

        // Send the message
        radio.stopListening(); // Ready to Write - EN_RXADDRP0 = 1
        radio.setAutoAck(0,true);

        retCode =  radio.write(msg,32); // Want to get AA working here

        // Continue listening
        radio.setAutoAck(0,false);
        radio.startListening(); // EN_RXADDRP0 = 0

        lcd.setCursor(0,1);
        if (gLogLevel == LOG_VERBOSE) {
          //0200 Erase :.0..
          //0123456789ABCDEF
          dumpMsg(&(msg[2]),1);
          dumpMsg(&(msg[1]),1);

          lcd.print(" Erase  : ");
          lcd.print(retCode);
          lcd.print(" ");
        } else {
          lcd.print(">>>             ");
        }

        return retCode;
}

bool SendReboot() {
   uint8_t * msg = gNRF_message;
   bool retCode =false;

        radio.stopListening();
        radio.setAutoAck(0,true);

        msg[0] = 0x86;

        if (radio.write(msg,32)) { // Want to get AA working here
          retCode= true;
        }

        gState = STATE_DEVID;
        radio.setAutoAck(0,false);
        radio.startListening();
        if (gLogLevel == LOG_VERBOSE) {
          lcd.setCursor(0,1);
          lcd.print("Reset           ");
        }
        return retCode;
}

bool SendWrite() {
  uint8_t * msg = gNRF_message;
  bool retCode = false;

        radio.stopListening();
        radio.setAutoAck(0,true);

        msg[0] = 0x81; // String to erase
        memcpy(&(msg[1]),gWriteCmd.data,30);

        if (radio.write(msg,32)) { // Want to get AA working here
          retCode = true;
        }
        radio.setAutoAck(0,false);
        radio.startListening();


        if (gLogLevel == LOG_VERBOSE) {
          lcd.setCursor(5,1);
          //0200 Write :.0..
          //0123456789ABCDEF
          lcd.print("Write  : ");
          lcd.print(retCode);
          lcd.print(" ");
        } else {
          lcd.setCursor(0,1);
          lcd.print("     >>>        ");
        }
        return retCode;
}

bool SendCommit() {
  uint8_t * msg = gNRF_message;
  bool retCode = false;

        radio.stopListening();
        radio.setAutoAck(0,true);

        msg[0] = 0x82;
        msg[1] = 1;
        msg[2] = gWriteCmd.csum;
        memcpy(&(msg[3]),&(gWriteCmd.data[30]),2);

        if (radio.write(msg,32)) { // Want to get AA working here
          retCode = true;
        }

        radio.setAutoAck(0,false);
        radio.startListening();

        if (gLogLevel == LOG_VERBOSE) {
          lcd.setCursor(5,1);
          //0200 Commit :.0.
          //0123456789ABCDEF
          lcd.print("Commit : ");
          lcd.print(retCode);
          lcd.print(" ");
        }else {
          lcd.setCursor(0,1);
          lcd.print("          >>>   ");
        }
        return retCode;
}

bool SendAudit() {
  uint8_t * msg = gNRF_message;
  bool retCode = false;

        radio.stopListening();
        radio.setAutoAck(0,true);

        msg[0] = 0x83;
        memcpy(&(msg[1]),gWriteCmd.data,7);

        if (radio.write(msg,32)) { // Want to get AA working here
          retCode = true;
        }

        radio.setAutoAck(0,false);
        radio.startListening();
        if (gLogLevel == LOG_VERBOSE) {
          lcd.setCursor(0,1);
          lcd.print("Audit..         ");
        } else {
          lcd.setCursor(14,1);
          lcd.print("||");
        }
        return retCode;
}

bool SendHeartBeat() {
  uint8_t * msg = gNRF_message;
  bool retCode = false;

        radio.stopListening();
        radio.setAutoAck(0,true);

        msg[0] = 0x84;
        msg[1] = 0x2A; // 42 .. the ultimate question;
        msg[2] = millis()&0xff; // ensure unique packets

        if (radio.write(msg,32)) { // Want to get AA working here
          retCode = true;
        }

        radio.setAutoAck(0,false);
        radio.startListening();

        return retCode;
}

bool sendBindRequest() {
     uint8_t * msg = gNRF_message;
     bool retCode = false;

         msg[0] = 0x87;

        // Allocate a pipe and send that address to the client
        // (for now use the default)
        // Format <0x87><DevId0><DevId1><DevId2><P2P0><P2P1><P2P2>

        memcpy(&(msg[1]),&addr_client,3);
        memcpy(&(msg[4]),&addr_p2p,3);

        msg[16]=millis()&0xff;

        radio.stopListening();   // Ready to write EN_RXADDR_P0 = 1
        radio.openWritingPipe(addr_client);
        delay(1);
        radio.setAutoAck(0,false);// As a broadcast first ...
        gWaitTimeout=millis();
        //delay(500);
        retCode = radio.write(msg,32); // Want to get AA working here

        radio.setAutoAck(2,true);// From now on ... BOUND to P2P
        radio.startListening(); // EN_RXADDR_P0 = 0

    #ifdef DEBUG
        lcd.setCursor(0,1);

        if(gLogLevel == LOG_VERBOSE) {
          uint32_t tempAddr;
          radio.qryAddrReg(0x0C,(char *)&tempAddr);
          lcd.print(">>");
          lcd.print(tempAddr,HEX);
        }
    #endif
        return retCode;
}

// Timeout values for various states
unsigned char heart_beat_count = 0;
unsigned long serial_timeout = 0;
unsigned char retry_count = 0;
uint8_t poll_count = 0;
// --------------------------------
// NRF_GPIO handler - toggle GPIO on Rx packets
// Allow for timing calculations
// --------------------------------
uint8_t pollRadio () {
  uint8_t pipe;
  uint8_t inbuf[32];

  if (radio.available(&pipe)) {
    uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
    radio.read(inbuf, bytes);               // fetch payload from FIFO
    if ((pipe == 0)|| (pipe==7)) {
      // Ignore - we aren't listening on 0
    }

    //DEBUG CODE - Print out any received messages
    #ifdef DEBUG
        lcd.setCursor(0,1);
        lcd.print("R");
        lcd.print(pipe);
        lcd.print("[");
        dumpMsg(inbuf,5);
    #endif
    if (inbuf[0] != 0x85) {
       heart_beat_count = 0; // receive ANYTHING other than BEACON will reset the heartbeat
       gHeartBeatTimeout = millis();
    }

    switch (gState) {
      case STATE_IDLE:
        // If I see an 0x88 .. add to scrolling list
        if (inbuf[0] == 0x88) { // This is a BIND request
          // Display for giggles
          // Display String -
        }
        break;
      case STATE_BIND:
        if (inbuf[0] == 0x88) { // This is a BIND request
          // Display for giggles
          sendBindRequest(); // Broadcast BOUND request
        }
        if (inbuf[0] == 0x87) { // This is a BIND reply
          gState = STATE_HXHDR;
          radio.openWritingPipe(addr_client);
          delay(1);
          Serial.write(0x01);
          gWaitTimeout=millis();
        }
        break;
      case STATE_SETUP:
        if (inbuf[0] == 0x80) { // This is a SETUP response
          if (inbuf[1] == 0x01) {
            // ACK
            gState = STATE_HXHDR; // Back to Parsing input?
            gWaitTimeout=millis();
            Serial.write(0x01);
            Serial.flush();
          } else {
            // NACK - re-send the SETUP request
            SendSetup();
          }
        }
        break;
      case STATE_WRITE:
        if (inbuf[0] == 0x81) { // This is a WRITE response
          if (inbuf[1] == 0x01) {
            // ACK
            //Build and send the COMMIT request
             gState = STATE_COMMIT;
             while (!SendCommit());
          } else {
            // NACK - re-send the SETUP request
            while (!SendWrite());
          }
        }
        break;
      case STATE_COMMIT:
        if (inbuf[0] == 0x82) { // This is a COMMIT response
          if (inbuf[1] == 0x01) {
            // ACK
             gState = STATE_HXHDR;
             gWaitTimeout=millis();
             Serial.write(0x01);
             Serial.flush();
          } else {
            // NACK - re-send the SETUP request
            gState = STATE_SETUP;
            SendSetup();
          }
        }
        break;
      case STATE_AUDIT:
        // Check return code
        if (inbuf[0] == 0x83) { // This is a SETUP response
          if (inbuf[1] == 0x01) {
             // ACK
             gState = STATE_HXHDR; // Wait on RESET
             gWaitTimeout=millis();
             Serial.write(0x01);
             Serial.flush();
          } else {
            // NACK - error out
             gState = STATE_DEVID;
             Serial.write(0x04);
             Serial.flush();
          }
        }
        break;
      default: // Unexpected reply
        break;
    }// switch (gState)
  } else {
     // On Radio Message Timeout..
     if (W4Radio(gState)) {
        if (gState!=STATE_IDLE) {
           // If failure to hear back after 5 retries
           uint8_t RETRY_LIMIT = 5;
           if (gState == STATE_BIND) {
              RETRY_LIMIT=20;
           }
           if (retry_count > RETRY_LIMIT) {
            // Exit with error
            idleRadio();
            addr_client =0;
            gState = STATE_IDLE;
            retry_count = 0;
            Serial.write(0x07);
            splash();
           }

           // Has there been seven mised HeartBeats?
           if (heart_beat_count == 7) {
             idleRadio();
           }

           if (millis()-gWaitTimeout > 1000) {
             switch (gState) {
                case STATE_BIND:
                  sendBindRequest();
                  break;
                case STATE_SETUP:
                  SendSetup();
                  break;
                case STATE_WRITE:
                  SendWrite();
                  break;
                case STATE_COMMIT:
                  SendCommit();
                  break;
                default:
                  // do nothing
                  break;
             } // switch gState
             retry_count++;
             gWaitTimeout= millis();
          } //1 second Timeout
        }// Not STATE_IDLE
     } //W4Radio
  } //No Radio Payload
} // pollRadio


char  poll_idle[4] ={'-','\\','|','/'};
char poll_bound[4]={0,1,0,' '};

void DisplayState() {
   lcd.home (); // set cursor to 0,0

   if (gState==STATE_IDLE) {
     show_idle();
     return;
   }
   lcd.print("nRF:");

   switch (gState) {
    case STATE_IDLE:
    case STATE_LOG:
    case STATE_DEVID:
        lcd.print("                ");
        break;
    case STATE_BIND:{
          char temp[17];
          lcd.print("Searching  ");
               //nRF:56789ABCDE
          lcd.setCursor(0,1);
          sprintf(temp," device: %6.6lX ",addr_client);
          //0123456789ABCDEF
          // device: 1D0002
          lcd.print(temp);
        }
        break;
    default:
          lcd.print("F/W Upload");
               //nRF:56789ABCDE
          break;
    }

   // Show the "I'm Alive" animation
   lcd.setCursor(15,0);
   /* Show the polling cursor */
   poll_count = (poll_count+1)%32;
   if ((heart_beat_count==0) && (mode(gState)>mode(STATE_BIND))){
      lcd.print(poll_bound[poll_count/8]);
   } else {
      lcd.print(poll_idle[poll_count/8]);
   }
}

// Helper routine for parsing HEX to binary
unsigned char x2i(char input) {
    if (input >= '0' && input <= '9') {
        return input- '0';
    } else if (input >= 'A' && input <= 'F') {
        return input -('A' - 10);
    } else if (input >= 'a' && input <= 'f') {
        return input- ('a' - 10);
    }
    return -1;
}

void show_idle() {
  if ((millis()-splash_timeout) >100) {
    // Clear previous Character
    lcd.setCursor(phase%16,1);
    lcd.write(info_line[phase%16]);
    phase = (phase+dir);
    // Show ICON
    lcd.setCursor(phase,1);
    lcd.write(255);

    phase %=16;

    //
    if (phase == 15) {
      dir=-1;
      sprintf(info_line,"  version  %d.%d  ",major,minor);
      delay(2000);
    }
    if (phase == 0) {
      dir=1;
      sprintf(info_line," LabRat  Lights ");
                       //0123456789ABCDEF
      delay(2000);
    }
    splash_timeout = millis();
  }
}

void pollSerial() {
  int inch = 0x00;

  if ( Serial.available() ) {
    inch = Serial.read();
    serial_timeout = millis();
    if (gState==STATE_IDLE) {
         // Only thing we do is wait on the SYNC
         if (inch == 0x42){
            gState = STATE_LOG;
            Serial.write(0x42);
         }
    } else {
      if (gREAD_counter == 0) { // No data pending
        switch (gState) {
          case STATE_LOG:
            if (inch==0x06){
              gREAD_counter = 1;
            } // Intentional FALL through
          case STATE_DEVID: // Wait for type 05 record
            if (inch==0x05) {// nRF target device ID
                  gREAD_counter = 3;
                  gState = STATE_DEVID;
              }
              break;
          case STATE_HXHDR: {
              switch (inch)  { // Parse record and update state accordingly
                 case 0x01 :
                    // Stay in STATE_HXHDR state
                    gREAD_counter = 4;
                    break;
                 case 0x02 :
                    // Change to parsing 32 byte payload
                    gState=STATE_HXDATA;
                    gREAD_counter = 32;
                    break;
                 case 0x03 :
                    gState = STATE_RESET;
                    gREAD_counter = 1;
                    break;
                 case 0x04 :
                    gState = STATE_FINISH; // This is the AUDIT request
                    gREAD_counter = 7;
                    break;
                 default:
                   // Invalid Record type
                   Serial.write(0x02);
                   gState = STATE_IDLE;
                   splash();
                   break;
              }// switch (inch)
              break;
            } // case STATE_HXHDR
            break;
          } // switch gState
      } else {
        // Count is not at ZERO
        switch(gState) {
          case STATE_HXHDR:
           switch (gREAD_counter) {
               case 4: gWriteCmd.addrL = inch; break;
               case 3: gWriteCmd.addrH = inch; break;
               case 2: gWriteCmd.csum  = inch; break;
               case 1: gWriteCmd.erase = (inch == 'E'); break;
               default: break;
           }
           gREAD_counter--;
           if (gREAD_counter == 0) {
               SendSetup();
               retry_count = 0;
               gState = STATE_SETUP;
           }
           break;
        case STATE_HXDATA:
           gWriteCmd.data[32-gREAD_counter] = inch;
           gREAD_counter--;
           if (gREAD_counter == 0) {
               SendWrite();
               gState = STATE_WRITE;
           }
           break;
        case STATE_RESET:
           gREAD_counter--;
           if (gREAD_counter == 0) {
              SendReboot();
              Serial.write(0x01);
              Serial.flush();
              gState = STATE_IDLE;
              delay(2000);
              splash();
              addr_client =0;
           }
           break;
        case STATE_FINISH:
           gWriteCmd.data[7-gREAD_counter] = inch;
           gREAD_counter--;
           if (gREAD_counter == 0) {
               SendAudit();
               gState=STATE_AUDIT; // Wait for AUDIT ACK
           }
           break;
        case STATE_DEVID:
           addr_client=addr_client<<8 | (inch&0xFF);
           gREAD_counter--;
           if (gREAD_counter == 0) {
               // Attempt to Bind to the client
               gState = STATE_BIND; // Wait for BIND ACK
               sendBindRequest();
           }
           break;
        case STATE_LOG:
           gLogLevel = (inch>0);
           gREAD_counter--;
           if (gREAD_counter == 0) {
               // Attempt to Bind to the client

               gState = STATE_DEVID; // Wait for BIND
               //Serial.write(gLogLevel+0x10);
               Serial.write(0x01); // Confirm message
           }
           break;
        default:
           break;  // Invalid state

        } // Switch
      } // Else Count !=0
    } // Not IDLE
  } else { // No Serial Available
     // Is Serial expected?
     if (W4Serial(gState) && (gState != STATE_IDLE)) {
       if (millis()-gHeartBeatTimeout > 1500) {
         if (mode(gState) > mode(STATE_BIND)) {
           SendHeartBeat();
           heart_beat_count++;
           gHeartBeatTimeout=millis();
           lcd.setCursor(0,1);
           lcd.print(F("- signal lost  -"));
         }
       }
       if ((millis()-serial_timeout)>10000) {
            Serial.write(0x06); // Timeout on reading from file
            Serial.flush();
            gState = STATE_IDLE;
            addr_client =0;
            lcd.setCursor(4,0);
            lcd.print(millis()-serial_timeout);
            splash();
       }
     }
  }
}

// Tx Test : Broadcast an identity packet
//                  0  1  2  3  4   5 6  7    
char msg_88[32]={0x88,04,02,00,0x1D,5,0,0x80,32,0,0,0,0,0,0,0,0,0,0,82,2,0};

void tx_test (int key) {
  int inch = KEY_NONE;
  unsigned long timestamp= 0;
  uint8_t char_count = 8;

  lcd.setCursor(0,0);
  lcd.print("Transmit Test   ");
  lcd.setCursor(0,1);
  clearLine();
  lcd.setCursor(0,1);

  while (inch != KEY_SELECT) {
     inch = getkey();
     if (millis() - timestamp > 1000) {
        timestamp = millis();

        radio.stopListening();
        radio.openWritingPipe(addr_server);
        radio.write(msg_88,32);
        radio.startListening();
        lcd.write(0xBB);
	char_count--;
        if (char_count==0) {
           lcd.setCursor(0,1);
           clearLine();
           lcd.setCursor(0,1);
        }
     }
  }
}

int radio_snoop (int key) {
  int inch = KEY_NONE;

  lcd.setCursor(0,0);
  lcd.print("Channel Snoop   ");
  lcd.setCursor(0,1);
  lcd.print("0 1 2 3 4 5 6 7   ");

  // Listen on both the
  radio.stopListening();   // Ready to write EN_RXADDR_P0 = 1
  radio.openReadingPipe(0,addr_server);
  radio.openReadingPipe(1,addr_data);
  radio.startListening();

  uint8_t pcount=0;
  uint32_t timeout[8];

  while (inch != KEY_SELECT) {
    inch = getkey();
    if (inch >=0) {
       // Keypress
    }
    uint8_t pipe;
    uint8_t inbuf[32];

    if (radio.available(&pipe)) {
       pipe=pipe&0x07;
       uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
       radio.read(inbuf, bytes);               // fetch payload from FIFO
       if (pipe==1) {
          if (inbuf[0] == 0x00) {
             pcount=(pcount+1)%44;
             if (pcount==0) {
                lcd.setCursor((pipe*2)+1,1);
                lcd.write((uint8_t)1);
                timeout[pipe]=millis()+1000;
             }
          }
       } else {
          lcd.setCursor((pipe*2)+1,1);
          lcd.write((uint8_t)0);
          timeout[pipe]=millis()+1000;
       }
    }
    for (int i=0;i<8;i++) {
       if (timeout[i] < millis()){
          lcd.setCursor(1+(i*2),1);
          lcd.print(" ");
       }
    }
  }
}

int device_snoop (int key) {
  int inch = KEY_NONE;

  lcd.setCursor(0,0);
  lcd.print("Devices:   WNRF ");
  lcd.setCursor(0,1);
  clearLine();

  // Listen on both the
  radio.stopListening();   // Ready to write EN_RXADDR_P0 = 1
  radio.openReadingPipe(0,addr_server);
  radio.closeReadingPipe(1);
  radio.startListening();

  uint8_t pcount=0;
  uint32_t timeout[2];

  while (inch != KEY_SELECT) {
    inch = getkey();
    if (inch >=0) {
       // Keypress
    }
    uint8_t pipe;
    uint8_t inbuf[32];

    if (radio.available(&pipe)) {
       pipe=pipe&0x07;
       uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
       radio.read(inbuf, bytes);               // fetch payload from FIFO
       if (pipe==0) {
          if (inbuf[0] == 0x85) {
             lcd.setCursor(15,0);
             lcd.write((uint8_t)0);
             timeout[pipe]=millis()+1000;
          }
          if (inbuf[0] == 0x88) {
                lcd.setCursor(0,1);
                for (int i=0;i<3;i++) {
                  if (inbuf[4-i]<16) {
                      lcd.print("0");
                  }
                  lcd.print(inbuf[4-i],HEX);
                }
                timeout[1]=millis()+1000;
          }
       }
    }
    if (timeout[0] < millis()){
       lcd.setCursor(15,0);
       lcd.print(" ");
    }
    if (timeout[1] < millis()){
       lcd.setCursor(0,1);
       clearLine();
    }
  }
}

const PROGMEM tRFRegInfo regInfo[]={
   {"NRF_CONFIG",1,0},
   {"EN_AA",     1,1},
   {"EN_RXADDR", 1,2},
   {"SETUP_AW",  1,3},
   {"SETUP_RETR",1,4},
   {"RF_CH",     1,5},
   {"RF_SETUP",  1,6},
   {"NRF_STATUS",1,7},
   {"OBSERVE_TX",1,8},
   {"CD",        1,9},
   {"RXA0",5,10},
   {"RXA1",5,15},
   {"RXA2",1,20},
   {"RXA3",1,21},
   {"RXA4",1,22},
   {"RXA5",1,23},
   {"TXA", 5,24},
   {"RXPW0",  1,29},
   {"RXPW1",  1,30},
   {"RXPW2",  1,31},
   {"RXPW3",  1,32},
   {"RXPW4",  1,33},
   {"RXPW5",  1,34},
   {"FIFO_STAT",1,35},
   {"DYNPD",1, 36},
   {"FEATURE",1, 37},
   {"ce_pin",2, 38},
   {"csn_pin",2,40},
   {"SPI_SPEED",1,42}
};

int radio_sanity_test (int key) {
  int inch = KEY_NONE;
  uint8_t rf_registers[64];
  uint8_t dispIndex = 0;
  tRFRegInfo cache;
  uint32_t* map = (uint32_t*)rf_registers;

  lcd.setCursor(0,1);
  radio.encodeRadioDetails((uint8_t*) &(rf_registers[0]));
  lcd.print("RESULT:");
  if ((map[0]==0xFFFFFFFF) && (map[1]==0xFFFFFFFF)) {
     lcd.print("FAIL");
  } else {
    Serial.print("DEBUG:");
    Serial.print(map[0],HEX);
    Serial.print(",");
    Serial.print(map[1],HEX);
    Serial.println(" = PASS");
     lcd.print("PASS");
  }

  while (inch != KEY_SELECT) {
    inch = getkey();
    if (inch >=0) {
        memcpy_P(&cache,&(regInfo[dispIndex]),sizeof(tRFRegInfo));
        lcd.setCursor(0,1);
        clearLine();
        lcd.setCursor(0,1);
        lcd.print(cache.name);
        lcd.print(":0x");
        for (int i=0;i<cache.size;i++) {
           if (rf_registers[cache.offset+i]<16) {
              lcd.print("0");
           }
           lcd.print(rf_registers[cache.offset+i],HEX);
        }
        if (inch == KEY_UP) {
          dispIndex=(dispIndex+1) % 29;
        }
        if (inch==KEY_DOWN) {
          dispIndex=(dispIndex+28)%29;
        }
    }
    // Pause if there is something to see
  }
  return 0;
}

#ifdef LED_TEST
int led_test ( int key ) {
  int inch = KEY_NONE;
  lcd.setCursor(0,1);

  uint8_t cache = PORTC;
  uint8_t led_count = 0;

  while (inch != KEY_SELECT) {
    inch = getkey();
    // Pause if there is something to see
    if (inch>=0) {
        if (inch < KEY_NONE) {
          led_count=(led_count+1)%8;

          lcd.setCursor(14,1);  // Debug code .. but looks nice.
          lcd.write(led_count);

          digitalWrite(LED_GREEN, (led_count&0x01));
          digitalWrite(LED_BLUE,  (led_count&0x02));
          digitalWrite(LED_AMBER, (led_count&0x04));
          delay(200);
        }
    }
  }
  digitalWrite(LED_GREEN,cache&0x01);
  digitalWrite(LED_BLUE, cache&0x02);
  digitalWrite(LED_AMBER,cache&0x03);
  return 0;
}
#endif


int keypad_test (int  key) {
    int error=0;

    int inch = KEY_NONE;
    lcd.setCursor(0,1);
    lcd.print("PRESSED KEY: \x5B \x5D");
    while (inch != KEY_SELECT) {
       inch = getkey();
       //Serial.print("GETKEY returned: ");
       //Serial.println(inch);
       lcd.setCursor(14,1);
       switch (inch) {
          case KEY_NONE:   lcd.write(" ");   Serial.println("None");break;
          case KEY_LEFT:   lcd.write("\x7F");Serial.println("LEFT");break;
          case KEY_RIGHT:  lcd.write("\x7E");Serial.println("RIGHT");break;
          case KEY_UP:     lcd.write("^");   Serial.println("UP");  break;
          case KEY_DOWN:   lcd.write("v");   Serial.println("DOWN"); break;
          case KEY_SELECT: lcd.write("$");   Serial.println("SELECT");break;
       }
    }
    return error;
}


int onSerialUpload(int  key) {
  int done=false;

  while (!done) {
   done = (getkey()==KEY_SELECT);

      // Are there any Radio Payload Pending?
    if (W4Radio(gState)) {
       pollRadio();
    }

    // Is there any incoming Serial Pending?
    if (W4Serial(gState)) { // In a state that requires reading the file?
       pollSerial();
    }

    DisplayState();
  }
  return 1;
}

int selectHandler(int  key) {
  Serial.println("SelectHandler Invoked");
  return 1;
}


// ----------------------------------------------------------------
// Frequency Scanner :
// ----------------------------------------------------------------
#define NUM_CHAN (sizeof(cRFChanMenu)/sizeof(tSelectItem))
uint8_t  freqCount[NUM_CHAN]   = {0,0,0,0,0,0,0}; // Counter for each channel
uint16_t freqHistory[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // 1 bit per channel
uint8_t  rollingIndex = 0;    // Index of channel being updated (for the history)

uint8_t freqOffset = 0;
int freqScanner(int key) {
  Serial.println("Frequency Scanner Invoked");

  int inch = KEY_NONE;
  int CountDown= 16;
  bool changeRange=1;

  /* Setup */
  while (inch != KEY_SELECT) {
    uint16_t pollvalue = 0x00;
    inch = getkey();

    switch (inch) {
      case KEY_RIGHT:
         if (freqOffset)  {
          freqOffset-=12;
          changeRange = 1;
         }
         break;
      case KEY_LEFT:
        if (freqOffset<12*5) {
          freqOffset+=12;
          changeRange=1;
        }
      default: // Do nothing
         break;
    }

   if (changeRange) {
      char tempstr[16];

      sprintf(tempstr,"%d|",2470-freqOffset);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.write(tempstr);
      lcd.setCursor(12,0);
      sprintf(tempstr,"| %2.2d",82-freqOffset);
      lcd.write(tempstr);
      lcd.setCursor(4,1);
      lcd.write("|       |MHz");
      changeRange = 0;
      for (int i=0;i<16;i++) {
        freqHistory[i] =0 ;
        CountDown = 16;
      }

      for (int i=0;i<NUM_CHAN;i++) {
        freqCount[i]=0;
      }
   }
    // Poll the Radio
    for (int i=0; i<NUM_CHAN;i++) {
      radio.setChannel(cRFChanMenu[i].value-freqOffset);

      // Listen for a little
      radio.startListening();
      delayMicroseconds(128);
      radio.stopListening();

      // Did we get a carrier?
      if ( radio.testCarrier() ) {
        pollvalue |= (1<<i);
        ++freqCount[i];
      }
    }

    if (CountDown) CountDown--;
    freqHistory[rollingIndex] = pollvalue; // Store History
    rollingIndex = (rollingIndex+1) % 16;

    // Display the results
    lcd.setCursor(5,0);
    for (int i=0;i<NUM_CHAN;i++) {
      int reading = freqCount[i]*2; // scalling to 2x looks better

      if (reading>8) {
        lcd.write(barmap[reading-8]);
      } else {
        lcd.write(barmap[0]);
      }
    }

    lcd.setCursor(5,1);
    for (int i=0;i<NUM_CHAN;i++) {
      int reading = freqCount[i]*2;
      if (reading>8) {
        lcd.write(255);
      } else {
        lcd.write(barmap[reading]);
      }
    }


    if (CountDown ==0) {
      uint8_t prevIndex = (rollingIndex+1)%16;

      if (freqHistory[prevIndex]) {
        for (int i=0;i<NUM_CHAN;i++) {
          if (freqHistory[prevIndex] & (1<<i)) {
            freqCount[i]--;
          }
        }
      }
    }
  }
}

//
// Selection description and pMenuAction to call for it.
// If the selection callback implements a sub menu then
// "subMenu" is set to true and the callback will receive
// key presses.
//

int dispRFRate(bool display) {
  if (display) {
    lcd.print(cRFRateMenu[indexRFRate].option);
  }
  return indexRFRate;
}


int dispRFChan(bool display) {
  if (display) {
    lcd.print(cRFChanMenu[indexRFChan].option);
  }
  return indexRFChan;
}

#define MENU_SIZE(x)  (sizeof(x)/sizeof(tMenuItem))
#define SELECT_SIZE(x) (sizeof(x)/sizeof(tSelectItem))

tSelectMenu rfRateMenu={ dispRFRate, &indexRFRate, cRFRateMenu };
tSelectMenu rfChanMenu={ dispRFChan, &indexRFChan, cRFChanMenu };

tMenuItem SetupMenu[] = {
   {"RF Rate:",      MENU_TYPE_SELECT,selectHandler,(sMenuItem*)&rfRateMenu,SELECT_SIZE(cRFRateMenu)},
   {"RF Channel:",   MENU_TYPE_SELECT,selectHandler,(sMenuItem*)&rfChanMenu,SELECT_SIZE(cRFChanMenu)}
};

tMenuItem RadioMenu[] = {
  {"Sanity Check    ", MENU_TYPE_ACTION, radio_sanity_test, NULL, 0},
  {"TX Test         ", MENU_TYPE_ACTION, tx_test, NULL, 0},
  {"RX Snoop        ", MENU_TYPE_ACTION, radio_snoop, NULL, 0},
  {"Dev Snoop       ", MENU_TYPE_ACTION, device_snoop, NULL, 0}
};

tMenuItem MainMenu[] = {
  {"Setup           ",MENU_TYPE_MENU,NULL,SetupMenu,MENU_SIZE(SetupMenu)},
  {"Radio Test      ",MENU_TYPE_MENU,NULL,RadioMenu,MENU_SIZE(RadioMenu)},
  {"Keypad Test     ",MENU_TYPE_ACTION,keypad_test,NULL,0},
 #ifdef LED_TEST
  {"LED Test        ",MENU_TYPE_ACTION,led_test,NULL,0},
 #endif
  {"Serial Upload   ",MENU_TYPE_ACTION,onSerialUpload,NULL,0},
  {"Frequency Scan  ",MENU_TYPE_ACTION, freqScanner,NULL,0}
};


int runMenu(tMenuItem *menu, uint8_t menu_size, uint8_t startIndex, uint8_t startSubIndex) {
  int inch =0x00;
  int done = false;
  // Active Menu/Sub-Menu indexes
  int current = startIndex;
  uint8_t sub_index = startSubIndex;
  uint8_t menuSize = 0;
  menuSize = menu[current].menuSize;

  while (!done) {
    int key=getkey();
    if (key != last_key) {
      // There is a key status change
      last_key = key;
      switch(key) {
        case KEY_SELECT:
          switch(menu[current].menuType) {
            case MENU_TYPE_MENU: {
                   if (menu[current].pSubMenu) {
                      Serial.print("Invoking Sub-Function: startIndex=");
                      if (menu[current].pSubMenu->menuType == MENU_TYPE_SELECT) {
                        uint8_t optionIndex = menu[current].pSubMenu[sub_index].pSelectMenu->pGetDefault(false);
                        Serial.println(optionIndex);
                        int result = runMenu(menu[current].pSubMenu,menu[current].menuSize,sub_index,optionIndex);
                         *(menu[current].pSubMenu[sub_index].pSelectMenu->selectVar) = result;
                         Serial.print("Result Variable:");
                         Serial.println(result);
                         Serial.print("DEBUG ADDR");
                         Serial.println((long) menu[current].pSubMenu[sub_index].pSelectMenu->selectVar,HEX);
                      } else {
                         Serial.println(0);
                         runMenu(menu[current].pSubMenu,menu[current].menuSize,sub_index,0);
                      }
                      Serial.println("Done with the sub-menu.. uh.. how do we leave?");
                   } else {
                    Serial.println("Error:No sub-menu defined");
                   }
               }
               break;
            case MENU_TYPE_ACTION:
               {
                  uint8_t optionIndex = sub_index;
                  Serial.print("ACTION KEY");
                  Serial.println(menu[current].title);
                  Serial.print("Option:");
                  Serial.println(optionIndex);
                  if (menu[current].pMenuAction)
                  done = menu[current].pMenuAction(key);
               }
               break;
            case MENU_TYPE_SELECT:
              Serial.print("SELECT KEY: return=");
              Serial.println(sub_index);
              return sub_index; //current; // return index of current selection
              break;
            default:
               Serial.println("Error - unknown menu type");
               break;
          }

          // On return from Select or Sub-Menu - need to clear/redraw the second line?
          lcd.setCursor(0,1);
          clearLine();
          break;
        case KEY_LEFT:
           if (menu[current].menuType != MENU_TYPE_SELECT) {
               current= (current+(menu_size-1))%menu_size;
               sub_index = 0;
               menuSize = menu[current].menuSize;
           }
           break;
        case KEY_RIGHT:
           if (menu[current].menuType != MENU_TYPE_SELECT) {
               current= (current+1)%menu_size;
               menuSize = menu[current].menuSize;
               sub_index= 0;
           }
           break;
        case KEY_UP:
           sub_index = (sub_index+(menuSize-1))%menuSize;
           break;
        case KEY_DOWN:
           sub_index = (sub_index+1)%menuSize;
           break;
        case KEY_NONE:
        default: continue;
      }
      lcd.setCursor(0,0);
      lcd.print(menu[current].title);
      Serial.println(menu[current].title);
      lcd.setCursor(1,1);
      clearLine();

      if (menu[current].menuType == MENU_TYPE_SELECT) {
         lcd.setCursor(1,1);
         lcd.print("[");
         lcd.print(menu[current].pSelectMenu->list[sub_index].option);
         lcd.print("]");
      } else {
        if (menu[current].pSubMenu) {
          lcd.setCursor(1,1);
          lcd.print(menu[current].pSubMenu[sub_index].title);
          // Display currently selected option
          if (menu[current].pSubMenu[sub_index].menuType == MENU_TYPE_SELECT) {
            menu[current].pSubMenu[sub_index].pSelectMenu->pGetDefault(true);
          }
        }
      }
    }
  }
  return done;
}
// Main Loop
//    - Check for Serial Input
//    - Determine Parsing of Serial based on character received
//    -
//
void loop() {
  int retCode = 0x00;
  retCode = runMenu(MainMenu, MENU_SIZE(MainMenu),0,0);
}
