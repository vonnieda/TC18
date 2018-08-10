/*
* TC18 by Jason von Nieda <jason@vonnieda.org>
* License: CC BY-SA
* 
* Board Configuration
*  Arduino Duemilanove or Diecimila, ATmega328P or ATmega168 @ 16MHz
*  Arduino will set the brown out detect too high on burning the bootloader
*  so it has to be changed to 1.8 manually.
*  VBB feedback       Analog 0 (PC0)
*    VBB with 100k to 1k voltage divider
*  SCK to MAX6921     Digital 13 (PB5)
*  MOSI to MAX6921    Digital 11 (PB3)
*  PPS                Digital 8 (PB0)
*  SW2 (ENC_B)        Digital 7 (PD7)
*  SW1 (ENC_A)        Digital 6 (PD6)
*  SW3 (ENC_SW)       Digital 5 (PD5)
*  LOAD               Digital 4 (PD4)
*  BOOST              Digital 3 (PD3)
*  SDA                Analog 4 (PC4)
*  SCL                Analog 5 (PC5)
*  RXD                Digital 0 (PD0)
*  TXD                Digital 1 (PD1)
*/


// Used only for testing a board after programming
//#define TESTMODE     1

#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <Wire.h>
#include <EEPROM.h>

#define DS3231_ADDR   0b01101000

#define VFDLOAD_DDR   DDRD
#define VFDLOAD_PORT  PORTD
#define VFDLOAD       PORTD4

#define BOOST_DDR     DDRD
#define BOOST_PORT    PORTD
#define BOOST         PORTD3

#define PPS_DDR       DDRB
#define PPS_PIN       PINB
#define PPS           PORTB0

#define ENC_A_DDR     DDRD
#define ENC_A_PIN     PIND
#define ENC_A_PORT    PORTD
#define ENC_A         PORTD6

#define ENC_B_DDR     DDRD
#define ENC_B_PIN     PIND
#define ENC_B_PORT    PORTD
#define ENC_B         PORTD7

#define ENC_SW_DDR    DDRD
#define ENC_SW_PIN    PIND
#define ENC_SW_PORT   PORTD
#define ENC_SW        PORTD5

#define VFD_SEG_A     0b00000001
#define VFD_SEG_B     0b00000010
#define VFD_SEG_C     0b00000100
#define VFD_SEG_D     0b00001000
#define VFD_SEG_E     0b00010000
#define VFD_SEG_F     0b00100000
#define VFD_SEG_G     0b01000000
#define VFD_SEG_H     0b10000000

#define VFD_DIGIT_0   0b000000001
#define VFD_DIGIT_1   0b000000010
#define VFD_DIGIT_2   0b000000100
#define VFD_DIGIT_3   0b000001000
#define VFD_DIGIT_4   0b000010000
#define VFD_DIGIT_5   0b000100000
#define VFD_DIGIT_6   0b001000000
#define VFD_DIGIT_7   0b010000000
#define VFD_DIGIT_8   0b100000000

#define MENU_TIMEOUT  10000

static const byte vfd_font[] = {
  /* 0 - 9 */
  VFD_SEG_A | VFD_SEG_B | VFD_SEG_C | VFD_SEG_D | VFD_SEG_E | VFD_SEG_F,
  VFD_SEG_B | VFD_SEG_C,
  VFD_SEG_A | VFD_SEG_B | VFD_SEG_D | VFD_SEG_E | VFD_SEG_G,
  VFD_SEG_A | VFD_SEG_B | VFD_SEG_C | VFD_SEG_D | VFD_SEG_G,
  VFD_SEG_B | VFD_SEG_C | VFD_SEG_F | VFD_SEG_G,
  VFD_SEG_A | VFD_SEG_C | VFD_SEG_D | VFD_SEG_F | VFD_SEG_G,
  VFD_SEG_A | VFD_SEG_C | VFD_SEG_D | VFD_SEG_E | VFD_SEG_F | VFD_SEG_G,
  VFD_SEG_A | VFD_SEG_B | VFD_SEG_C,
  VFD_SEG_A | VFD_SEG_B | VFD_SEG_C | VFD_SEG_D | VFD_SEG_E | VFD_SEG_F | VFD_SEG_G,
  VFD_SEG_A | VFD_SEG_B | VFD_SEG_C | VFD_SEG_F | VFD_SEG_G,
  
  /* a - z */
  VFD_SEG_A | VFD_SEG_B | VFD_SEG_C | VFD_SEG_D | VFD_SEG_E | VFD_SEG_G,
  VFD_SEG_C | VFD_SEG_D | VFD_SEG_E | VFD_SEG_F | VFD_SEG_G,
  VFD_SEG_D | VFD_SEG_E | VFD_SEG_G,
  VFD_SEG_B | VFD_SEG_C | VFD_SEG_D | VFD_SEG_E | VFD_SEG_G,
  VFD_SEG_A | VFD_SEG_B | VFD_SEG_D | VFD_SEG_E | VFD_SEG_F | VFD_SEG_G,
  VFD_SEG_A | VFD_SEG_E | VFD_SEG_F | VFD_SEG_G,
  VFD_SEG_A | VFD_SEG_B | VFD_SEG_C | VFD_SEG_D | VFD_SEG_F | VFD_SEG_G,
  VFD_SEG_C | VFD_SEG_E | VFD_SEG_F | VFD_SEG_G,
  VFD_SEG_B | VFD_SEG_C,
  VFD_SEG_B | VFD_SEG_C | VFD_SEG_D | VFD_SEG_E,
  VFD_SEG_A | VFD_SEG_C | VFD_SEG_E | VFD_SEG_F | VFD_SEG_G,
  VFD_SEG_D | VFD_SEG_E | VFD_SEG_F,
  VFD_SEG_A | VFD_SEG_C | VFD_SEG_E | VFD_SEG_G,
  VFD_SEG_C | VFD_SEG_E | VFD_SEG_G,
  VFD_SEG_C | VFD_SEG_D | VFD_SEG_E | VFD_SEG_G,
  VFD_SEG_A | VFD_SEG_B | VFD_SEG_E | VFD_SEG_F | VFD_SEG_G,
  VFD_SEG_A | VFD_SEG_B | VFD_SEG_C | VFD_SEG_D | VFD_SEG_G | VFD_SEG_H,
  VFD_SEG_E | VFD_SEG_G,
  VFD_SEG_A | VFD_SEG_C | VFD_SEG_D | VFD_SEG_F | VFD_SEG_G,
  VFD_SEG_D | VFD_SEG_E | VFD_SEG_F | VFD_SEG_G,
  VFD_SEG_C | VFD_SEG_D | VFD_SEG_E,
  VFD_SEG_C | VFD_SEG_D | VFD_SEG_E,
  VFD_SEG_A | VFD_SEG_C | VFD_SEG_D | VFD_SEG_E,
  VFD_SEG_B | VFD_SEG_C | VFD_SEG_E | VFD_SEG_F | VFD_SEG_G,
  VFD_SEG_B | VFD_SEG_C | VFD_SEG_D | VFD_SEG_F | VFD_SEG_G,
  VFD_SEG_A | VFD_SEG_B | VFD_SEG_D | VFD_SEG_E | VFD_SEG_G,
};

static const char *month_abbrevs[] = {
  "Jan",
  "Feb",
  "Mar",
  "Apr",
  "May",
  "Jun",
  "Jul",
  "Aug",
  "Sep",
  "Oct",
  "Nov",
  "Dec",
};

volatile byte vfd_data[9];
volatile byte vfd_digit;
// Digit dwell is used to adjust the brightness of individual digits on
// the display. Used to compensate for tube manufacturing differences.
// Increasing the value of a position causes the refresh loop to dwell on
// that digit for additional cycles causing it's brightness to increase.
// Use sparingly. This slows the refresh of the entire display.
// No need to change here, there is a menu option and it is saved
// to EEPROM.
volatile byte vfd_digit_dwell[] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0
};
volatile byte vfd_digit_dwell_count;
volatile unsigned long spi_buffer;
volatile byte spi_buffer_count;

// Default boost voltage used during startup, this is quickly replaced by the
// config data
float voltage_target = 30.00;
// The allowable voltage +/- drift before the ADC interrupt makes adjustements
static const float voltage_drift = 3.00;
// The current voltage as measured by the ADC
float voltage;
// The number of times the voltage has been sampled. This is used to limit
// the frequency of updates to the boost PWM.
byte voltage_sample;

volatile char encoder_debounce = -1;
volatile char encoder_pos;
volatile char encoder_pressed;

volatile byte hour, minute, second, year, month, date;
volatile unsigned int millisecond;
volatile float temperature;

// Used in many main loop functions to format text for display
char display_buf[20];

// Configurable options, pulled from EEPROM at startup and stored
// when changed.
char display_mode;
char brightness;
char bank_mode;

void setup() {
  Serial.begin(19200);
  
  BOOST_DDR |= _BV(BOOST);

  // Timer 2 is used to create the waveform for the boost controller
  // Timer 2 Waveform Generation Mode 3 Fast PWM
  // Compare Output Mode Clear OC2B on Compare Match, set OC2B at BOTTOM
  TCCR2A = _BV(WGM21) | _BV(WGM20) | _BV(COM2B1);
  // Clock Select No Prescaling
  TCCR2B = _BV(CS20);
  // Initial value for boost timer
  OCR2B = 230;
  
  // Timer 1 is used to refresh the VFD display
  // Set defaults in case Arduino screws them up
  TCCR1A = 0;
  // Clock Select No Prescaling
  TCCR1B = _BV(CS10);
  // Interrupt Enable
  TIMSK1 = _BV(TOIE1);
  
  // Internal 1.1V Voltage Reference, Single Ended Input on ADC0
  ADMUX = _BV(REFS1) | _BV(REFS0);
  // ADC Enable, ADC Auto Trigger Enable, ADC Interrupt Enable
  // ADC Prescaler 128
  ADCSRA |= _BV(ADEN) | _BV(ADATE) | _BV(ADIE) 
    | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  // ADC Start Conversion (starts free running mode)
  ADCSRA |= _BV(ADSC);
  
  // Enable SS, MOSI and SCK as outputs, required by SPI
  DDRB |= _BV(PORTB2) | _BV(PORTB3) | _BV(PORTB5);
  // SPI Enable, Master Mode, Interrupt Enable, FOSC/16
  SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0) | _BV(SPIE);
 
  VFDLOAD_DDR |= _BV(VFDLOAD);
  
  // Enable Pin Change Interrupt 0 PCINT0 (PB0) [PPS]
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT0);
  
  // Set ENC_A, ENC_B and ENC_SW pins to input
  ENC_A_DDR &= ~(_BV(ENC_A));
  ENC_B_DDR &= ~(_BV(ENC_B));
  ENC_SW_DDR &= ~(_BV(ENC_SW));
  
  // Turn on internal pullups for ENC_A, ENC_B, ENC_SW
  ENC_A_PORT |= _BV(ENC_A);
  ENC_B_PORT |= _BV(ENC_B);
  ENC_SW_PORT |= _BV(ENC_SW);
  
  // Enable Pin Change Interrupt 2 for PCINT22 (PD6) [ENC_A], PCINT21 (PD5) [ENC_SW]
  PCMSK2 |= _BV(PCINT21);
  PCMSK2 |= _BV(PCINT22);
  PCICR |= _BV(PCIE2);
  
  ds3231_setup();
  
  load_config();
}

void ds3231_setup() {
  Wire.begin();
  // set the control register
  ds3231_set_register(0x0e, 0b00000000);
}

byte dec_to_bcd(int dec) {
  return ((dec % 10) | ((dec / 10) << 4));
}

int bcd_to_dec(byte bcd) {
  return (((bcd >> 4) * 10) + (bcd & 0x0f));
}

void ds3231_set_register(byte address, byte value) {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission();
}

byte ds3231_get_register(byte address) {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDR, 1);
  while (Wire.available() < 1);
  return Wire.read();
}

void ds3231_set_time(byte hour, byte minute, byte second) {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x00);
  Wire.write(dec_to_bcd(second));
  Wire.write(dec_to_bcd(minute));
  Wire.write(dec_to_bcd(hour));
  Wire.endTransmission();
}

void ds3231_get_time(byte *hour, byte *minute, byte *second) {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDR, 3);
  while (Wire.available() < 3);
  byte s = Wire.read();
  byte m = Wire.read();
  byte h = Wire.read();
  *second = bcd_to_dec(s);
  *minute = bcd_to_dec(m);
  *hour = bcd_to_dec(h);
}

void ds3231_set_date(byte year, byte month, byte date) {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x04);
  Wire.write(dec_to_bcd(date));
  Wire.write(dec_to_bcd(month));
  Wire.write(dec_to_bcd(year));
  Wire.endTransmission();
}

void ds3231_get_date(byte *year, byte *month, byte *date) {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x04);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDR, 3);
  while (Wire.available() < 3);
  byte d = Wire.read();
  byte m = Wire.read();
  byte y = Wire.read();
  *date = bcd_to_dec(d);
  *month = bcd_to_dec(m & 0b00011111);
  *year = bcd_to_dec(y);
}

float ds3231_get_temp() {
  byte msb = ds3231_get_register(0x11);
  byte lsb = ds3231_get_register(0x12);
  float c = msb + ((lsb >> 6) * 0.25f);
  return c;
}

/*
  Load the given VFD data into the SPI buffer and start the transfer.
  This sends the first byte of the data. The rest is sent from the
  SPI interrupt.
*/
static inline void vfd_send(unsigned long d) {
  spi_buffer = d << 8;
  spi_buffer_count = 3;
  
  SPDR = (spi_buffer >> 24);
  spi_buffer <<= 8;
  spi_buffer_count--;
}

/*
  Refresh the VFD by showing the next digit
*/
static inline void vfd_refresh() {
  if (vfd_digit_dwell_count--) {
    return;
  }
  vfd_digit_dwell_count = vfd_digit_dwell[8 - vfd_digit];
  uint32_t d = vfd_data[8 - vfd_digit];
  d <<= 9;
  d |= (1 << vfd_digit);
  vfd_send(d);
  vfd_digit++;
  if (vfd_digit > 8) {
    vfd_digit = 0;
  }
}
  
/*
  Map the given char to the correct VFD segment byte
  using the font table and some internals.
   a
 f   b  
   g  
 e   c   
   d
       h   
*/
byte vfd_map_char(char c) {
  if (c >= 'a' && c <= 'z') {
    return vfd_font[c - 'a' + 10]; 
  }
  else if (c >= 'A' && c <= 'Z') {
    return vfd_font[c - 'A' + 10];
  }
  else if (c >= '0' && c <= '9') {
    return vfd_font[c - '0']; 
  }
  else if (c == ' ') {
    return 0; 
  }
  else if (c == '-') {
    return VFD_SEG_G;
  }
  else if (c == '.') {
    return VFD_SEG_H;
  }
  else if (c == '!') {
    return VFD_SEG_B | VFD_SEG_C | VFD_SEG_H;
  }
  else if (c == '?') {
    return VFD_SEG_A | VFD_SEG_B | VFD_SEG_E | VFD_SEG_G | VFD_SEG_H;
  }
  else if (c == ',') {
    return VFD_SEG_C;
  }
  else if (c == -1) {
    return VFD_SEG_G | VFD_SEG_H;
  }
  else {
    return 0; 
  }
}

/*
  Print the given string to the VFD
*/
void vfd_set_string(char *s, int offset) {
  int slen = strlen(s);
  
  for (int i = offset; i < offset + 8; i++) {
    if (i < 0 || i > slen - 1) {
      vfd_data[i - offset + 1] = 0;
    }
    else {
      vfd_data[i - offset + 1] = vfd_map_char(s[i]);
    }
  }
}

void vfd_scroll_string(char *s, int del) {
  int slen = strlen(s);
  for (int i = -7; i < slen; i++) {
    vfd_set_string(s, i);
    delay(del);
  }
}

int serial_readline(char *s, int length) {
  int ch;
  do {
    ch = Serial.read();
    *s++ = ch;
  } while (ch != '\n' && ch != '\r' && Serial.available() > 0);
  *s = NULL;
  return 0;
}

/*
  Handles the ADC conversion complete interrupt. ADC is used to
  measure the output of the voltage boost circuit and adjusts it
  to stay near the target voltage by adjusting the PWM width
  going to the boost MOSFET.
*/
ISR(ADC_vect) {
  byte low = ADCL;
  byte high = ADCH;
  if (voltage_sample++ > 20) {
    voltage_sample = 0;
    int adc = (high << 8) | low;
    voltage = 1.066f / (0x3ff - 1) * adc * 112.344;
    // We don't let the boost generator go below 10 or above 230
    // Those equate to roughly VCC and 60v so allowing it to
    // go too high can start blowing up components.
    if (voltage > voltage_target + voltage_drift && OCR2B > 10) {
      OCR2B--;
    }
    else if (voltage < voltage_target - voltage_drift && OCR2B < 230) {
      OCR2B++;
    }
  }
}

/*
  Timer fires approx. every 1ms and handles refreshing the VFD,
  debouncing inputs and updating the milliscond time register.
*/
ISR(TIMER1_OVF_vect) {
  // fire approx every 1ms
  TCNT1 = 0xffff - 16000;
  
  millisecond++;
  
  if (encoder_debounce >= 0) {
    encoder_debounce--;
  }
  
  if (encoder_debounce == 0) {
    int encoder_a = (ENC_A_PIN & _BV(ENC_A));
    int encoder_b = (ENC_B_PIN & _BV(ENC_B));
    int encoder_sw = (ENC_SW_PIN & _BV(ENC_SW));
    
    if (!encoder_sw) {
      encoder_pressed++;
    }
    
    if (!encoder_a) {
      encoder_pos += encoder_b ? -1 : +1;
    }
    PCIFR |= _BV(PCIF2);
    PCICR |= _BV(PCIE2);
  }
  
  vfd_refresh();
}

/*
  Picks up interrupts from any of the encoder pins changing and
  starts the debounce procedure. 
*/
ISR(PCINT2_vect) {
  encoder_debounce = 2;
  PCICR &= ~(_BV(PCIE2));
  PCIFR |= _BV(PCIF2);
}

/*
  Handles interrupts from the PPS (pulse per second) interrupt
  updating all the time registers
*/
ISR(PCINT0_vect) {
  if (PPS_PIN & _BV(PPS)) {
    // we have to enable interrupts because the Wire library uses them
    sei();
    ds3231_get_time((byte *) &hour, (byte *) &minute, (byte *) &second);
    ds3231_get_date((byte *) &year, (byte *) &month, (byte *) &date);
    temperature = ds3231_get_temp();
    millisecond = 0;
  }
}

/*
  Handles the completion of a SPI transfer and either latches
  the data or starts the transfer of the next byte
*/
ISR(SPI_STC_vect) {
  if (spi_buffer_count) {
    SPDR = (spi_buffer >> 24);
    spi_buffer <<= 8;
    spi_buffer_count--;
  }
  else {
    // latch data
    VFDLOAD_PORT |= _BV(VFDLOAD);
    VFDLOAD_PORT &= ~_BV(VFDLOAD);
  }
}

/*
  Wait the given number of milliseconds (ms) for any input from the encoder.
  If there is input the enc_moved and enc_pressed references are filled in
  and the function returns true. If no input is detected before the
  timer expires the function returns false.
  This function clears the global input counters in either case.
*/
boolean wait_for_input(int ms, char *enc_pos, boolean *enc_pressed) {
  unsigned long m = millis() + ms;
  while (!encoder_pressed && !encoder_pos && (millis() < m));
  *enc_pos = encoder_pos;
  *enc_pressed = encoder_pressed;
  encoder_pressed = 0;
  encoder_pos = 0;
  return *enc_pressed || *enc_pos;
}

void load_config() {
  byte b;

  b = EEPROM.read(0);
  if (b == 255) {
    b = 0;
  }
  display_mode = b;
  
  b = EEPROM.read(1);
  if (b == 255) {
    b = 2;
  }
  brightness = b;
  voltage_target = 20.0 + ((brightness + 1) * 3.75);
  
  b = EEPROM.read(2);
  if (b == 255) {
    b = 1;
  }
  bank_mode = b;
  
  for (byte i = 0; i < 9; i++) {
    b = EEPROM.read(3 + i);
    if (b == 255) {
      b = 0;
    }
    vfd_digit_dwell[i] = b;
  }
  
}

void save_config() {
  EEPROM.write(0, display_mode);
  EEPROM.write(1, brightness);
  EEPROM.write(2, bank_mode);
  for (byte i = 0; i < 9; i++) {
    EEPROM.write(3 + i, vfd_digit_dwell[i]);
  }
}

/*
  In setting the time we will display the current time with first the hours, then minutes, then seconds
  flashing once per second. The encoder turning increments or decrements the currently flashing item
  and encoder button causes the next item to become flashing. A push with seconds flashing returns.
*/
void set_time() {
  int h = hour, m = minute, s = second;
  
  char enc_pos = 0;
  boolean enc_pressed = 0;
  
  boolean flash = false;
  
  int hms_current = 0;
  int *hms[] = {
    &h,
    &m,
    &s
  };

  while (1) {
    for (int i = 0; i < 3; i++) {
      if (i == hms_current && flash) {
        sprintf(display_buf + (i * 3), "  :");
      }
      else {
        sprintf(display_buf + (i * 3), "%02d:", *hms[i]);
      }
    }
    
    vfd_set_string(display_buf, 0);

    if (wait_for_input(500, &enc_pos, &enc_pressed)) {
      if (enc_pressed) {
        hms_current++;
        if (hms_current > 2) {
          ds3231_set_time(h, m, s);
          break;
        }
      }
      else {
        *hms[hms_current] += enc_pos;
        if (hms_current == 0 && *hms[hms_current] > 23) {
          *hms[hms_current] = 0;
        }
        else if (hms_current == 0 && *hms[hms_current] < 0) {
          *hms[hms_current] = 23;
        }
        else if (hms_current == 1 && *hms[hms_current] > 59) {
          *hms[hms_current] = 0;
        }
        else if (hms_current == 1 && *hms[hms_current] < 0) {
          *hms[hms_current] = 59;
        }
        else if (hms_current == 2 && *hms[hms_current] > 59) {
          *hms[hms_current] = 0;
        }
        else if (hms_current == 2 && *hms[hms_current] < 0) {
          *hms[hms_current] = 59;
        }
      }
      flash = false;
    }
    else {
      flash = !flash;
    }
  }
}

void set_date() {
  int m = month, d = date, y = year;
  
  char enc_moved = 0;
  boolean enc_pressed = 0;

  do {
    m += enc_moved;
    if (m < 1) {
      m = 12;
    }
    else if (m > 12) {
      m = 1;
    }
    if (enc_pressed) {
      ds3231_set_date(y, m, d);
      break;
    }
    sprintf(display_buf, "Mo %s", month_abbrevs[m - 1]);
    vfd_set_string(display_buf, 0);
  } while (wait_for_input(MENU_TIMEOUT, &enc_moved, &enc_pressed));
  
  enc_moved = 0;
  enc_pressed = 0;
  do {
    d += enc_moved;
    if (d < 1) {
      d = 31;
    }
    else if (d > 31) {
      d = 1;
    }
    if (enc_pressed) {
      ds3231_set_date(y, m, d);
      break;
    }
    sprintf(display_buf, "Day %d", d);
    vfd_set_string(display_buf, 0);
  } while (wait_for_input(MENU_TIMEOUT, &enc_moved, &enc_pressed));
  
  enc_moved = 0;
  enc_pressed = 0;
  do {
    y += enc_moved;
    if (y < 0) {
      y = 99;
    }
    else if (y > 99) {
      y = 0;
    }
    if (enc_pressed) {
      ds3231_set_date(y, m, d);
      break;
    }
    sprintf(display_buf, "Yr 20%02d", y);
    vfd_set_string(display_buf, 0);
  } while (wait_for_input(MENU_TIMEOUT, &enc_moved, &enc_pressed));
}

void set_display_mode() {
  char *modes[] = {
    "hh mm ss",
    "hhmm  DD",
    "hhmm TTF",
    "MMM DD",
    "tttttttt",
    "hhmm  vv",
  };
  
  char enc_moved = 0;
  boolean enc_pressed = 0;
  
  char orig_display_mode = display_mode;

  do {
    display_mode += enc_moved;
    if (display_mode < 0) {
      display_mode = 5;
    }
    else if (display_mode > 5) {
      display_mode = 0;
    }
    if (enc_pressed) {
      save_config();
      orig_display_mode = display_mode;
      break;
    }
    vfd_set_string(modes[display_mode], 0);
  } while (wait_for_input(MENU_TIMEOUT, &enc_moved, &enc_pressed));
  
  display_mode = orig_display_mode;
}

void set_brightness() {
  char enc_moved = 0;
  boolean enc_pressed = 0;

  char orig_brightness = brightness;

  do {
    brightness += enc_moved;
    if (brightness < 0) {
      brightness = 0;
    }
    else if (brightness > 7) {
      brightness = 7;
    }
    
    vfd_data[0] = 0;
    for (int i = 1; i < 9; i++) {
      if (i == 1) {
        vfd_data[i] = VFD_SEG_F | VFD_SEG_E;
      }
      else if (i == 8) {
        vfd_data[i] = VFD_SEG_B | VFD_SEG_C;
      }
      else {
        vfd_data[i] = 0;
      }
      vfd_data[i] |= (brightness == i - 1 ? VFD_SEG_G : 0);
    }
    
    voltage_target = 20.0 + ((brightness + 1) * 3.75);
    
    if (enc_pressed) {
      save_config();
      orig_brightness = brightness;
      break;
    }
    
  } while (wait_for_input(MENU_TIMEOUT, &enc_moved, &enc_pressed));
  
  brightness = orig_brightness;
  voltage_target = 20.0 + ((brightness + 1) * 3.75);
}

void set_bank_mode() {
  char *modes[] = {
    "Off",
    "On",
  };
  
  char enc_moved = 0;
  boolean enc_pressed = 0;
  
  char orig_bank_mode = bank_mode;

  do {
    bank_mode += enc_moved;
    if (bank_mode < 0) {
      bank_mode = 1;
    }
    else if (bank_mode > 1) {
      bank_mode = 0;
    }
    if (enc_pressed) {
      save_config();
      orig_bank_mode = bank_mode;
      break;
    }
    sprintf(display_buf, "Bank %s", modes[bank_mode]);
    vfd_set_string(display_buf, 0);
  } while (wait_for_input(MENU_TIMEOUT, &enc_moved, &enc_pressed));
  
  bank_mode = orig_bank_mode;
}

void set_dwell() {
  char enc_moved = 0;
  boolean enc_pressed = 0;

  byte digit = 0;
  
  do {
    if (vfd_digit_dwell[digit] + enc_moved > 9) {
      vfd_digit_dwell[digit] = 9;
    }
    else if (vfd_digit_dwell[digit] + enc_moved < 0) {
      vfd_digit_dwell[digit] = 0;
    }
    else {
      vfd_digit_dwell[digit] += enc_moved;
    }
    
    
    if (enc_pressed) {
      if (++digit > 8) {
        save_config();
        break;
      }
    }

    for (byte i = 0; i < 9; i++) {
      vfd_data[i] = VFD_SEG_A | VFD_SEG_B | VFD_SEG_C | VFD_SEG_D | VFD_SEG_E | VFD_SEG_F | VFD_SEG_G;
    }
    vfd_data[digit] |= VFD_SEG_H;

    
  } while (wait_for_input(MENU_TIMEOUT, &enc_moved, &enc_pressed));

  vfd_data[0] = 0;
}

void main_menu() {
  /*
    TODO
    Change Display to Dsp Mode, add Time Fmt, add Date Fmt, make Bank a Dsp Mode option 
    along with Time, Date. Maybe. Kinda nice to have date/time formats too.
    Also add transition: Scroll, Switch
  */
  char *items[] = {
    "Set Time",
    "Set Date",
    "Display",
    "Britenes",
    "Dwell",
    "Bank",
    "Exit",
  };
  int items_length = 7;
  
  int index = 0;
  char enc_moved = 0;
  boolean enc_pressed = 0;

  do {
    index += enc_moved;
    if (index < 0) {
      index = items_length - 1;
    }
    else if (index > items_length - 1) {
      index = 0;
    }
    if (enc_pressed) {
      if (index == 0) {
        set_time();
        break;
      }
      else if (index == 1) {
        set_date();
        break;
      }
      else if (index == 2) {
        set_display_mode();
        break;
      }
      else if (index == 3) {
        set_brightness();
        break;
      }
      else if (index == 4) {
        set_dwell();
      }
      else if (index == 5) {
        set_bank_mode();
        break;
      }
      else {
        break;
      }
    }
    vfd_set_string(items[index], 0);
  } while (wait_for_input(MENU_TIMEOUT, &enc_moved, &enc_pressed));
}

float c_to_f(float c) {
  return (((9.0 / 5.0) * c) + 32.0);
}

void test_loop() {
  sprintf(display_buf, "%d:%d:%d %d/%d/%d %d %d", hour, minute, second, date, month, year, (int) voltage, (int) c_to_f(temperature));
  Serial.println(display_buf);
  delay(1000);
}

/*
  The main loop of the program is responsible for displaying the current time, checking for input
  from the encoder or button and running the menu system for the user to interact with the clock.
*/
void loop() {
  #ifdef TESTMODE
  test_loop();
  return;
  #endif
  char enc_moved = 0;
  boolean enc_pressed = 0;
  for (;;) {
    // This effectively defines the refresh rate for the clock data. We wait for input
    // for 10 ms and then refresh the clock.
    if (wait_for_input(10, &enc_moved, &enc_pressed)) {
      main_menu();
    }
    switch (display_mode) {
      case 0: {
        sprintf(display_buf, "%02d:%02d:%02d", hour, minute, second);
        vfd_data[0] = 0;
        break;
      }
      case 1: {
        sprintf(display_buf, "%02d%02d  %02d", hour, minute, date);
        vfd_data[0] = (second % 2 ? VFD_SEG_H : 0);
        break;
      }
      case 2: {
        sprintf(display_buf, "%02d%02d %dF", hour, minute, (int) c_to_f(temperature));
        vfd_data[0] = (second % 2 ? VFD_SEG_H : 0);
        break;
      }
      case 3: {
        sprintf(display_buf, "%s %02d", month_abbrevs[month - 1], date);
        vfd_data[0] = 0;
        break;
      }
      case 4: {
        uint64_t t = (24L * 60L * 60L * 1000L) - ((hour * 60L * 60L * 1000L) + (minute * 60L * 1000L) + (second * 1000L) + millisecond);
        sprintf(display_buf, "%08lu", t);
        vfd_data[0] = 0;
        break;
      }
      case 5: {
        sprintf(display_buf, "%02d%02d  %02d", hour, minute, (int) voltage);
        vfd_data[0] = (second % 2 ? VFD_SEG_H : 0);
        break;
      }
    }
    if (bank_mode) {
      if (second % 12 < 4) {
          sprintf(display_buf, "%02d-%02d-%02d", month, date, year);
          vfd_data[0] = 0;
      }
      else if (second % 12 < 8) {
          sprintf(display_buf, "%dF %dC", (int) c_to_f(temperature), (int) temperature);
          vfd_data[0] = 0;
      }
    }
    vfd_set_string(display_buf, 0);
  }
}



