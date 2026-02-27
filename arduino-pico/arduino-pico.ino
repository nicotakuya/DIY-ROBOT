// DIY-ROBOT
// for Raspberry Pi Pico(Arduino-Pico)
// and TFT-LCD(M154-240240-RGB ST7789)
// and Motor driver(TC78H653FTG)
// and NeoPixel(RGBLED PL9823-F5)
// (C)2026 takuya matsubara

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "8x8font.h"
#include <Adafruit_NeoPixel.h>
// https://github.com/adafruit/Adafruit_NeoPixel

#define ATM0177B3A 1  // 1.7inch TFT Display
#define SSD1306    3  // 0.9inch OLED Display
#define M154240240 4  // 1.5inch TFT Display M154-240240-RGB

//#define DISP_TYPE    ATM0177B3A   // DISPLAY TYPE
#define DISP_TYPE    M154240240   // DISPLAY TYPE

#define DISP_ROTATE  0 // TURN SCREEN(0=normal / 1=turn90 / 2=turn180 / 3=turn270)

#define SOFT_SPI  0 // (0=hardware / 1=software)

#if DISP_TYPE==ATM0177B3A
#define VRAMWIDTH   128  // width[pixel]
#define VRAMHEIGHT  160  // height[pixel]
#define FONTSIZE    2
#define VRSPZOOM    3    // sprite zoom
#endif

#if DISP_TYPE==SSD1306
#define VRAMWIDTH   128 // width[pixel]
#define VRAMHEIGHT  64  // height[pixel]
#define FONTSIZE    1
#define VRSPZOOM    1   // sprite zoom
#endif

#if DISP_TYPE==M154240240
#define VRAMWIDTH   240  // width[pixel]
#define VRAMHEIGHT  240  // height[pixel]
#define FONTSIZE    3
#define VRSPZOOM    3    // sprite zoom
#endif

#define VRAMSIZE   (VRAMWIDTH*VRAMHEIGHT*2) 

#if DISP_ROTATE==0
#define VRAMXRANGE VRAMWIDTH
#define VRAMYRANGE VRAMHEIGHT
#endif
#if DISP_ROTATE==1
#define VRAMYRANGE VRAMWIDTH
#define VRAMXRANGE VRAMHEIGHT
#endif
#if DISP_ROTATE==2
#define VRAMXRANGE VRAMWIDTH
#define VRAMYRANGE VRAMHEIGHT
#endif
#if DISP_ROTATE==3
#define VRAMYRANGE VRAMWIDTH
#define VRAMXRANGE VRAMHEIGHT
#endif
#define VRAMXMAX  (VRAMXRANGE-1)
#define VRAMYMAX  (VRAMYRANGE-1)

//const int SDCARD_MISO = 16;
//const int SDCARD_MOSI = 19;
//const int SDCARD_CS = 17;
//const int SDCARD_SCK = 18;
#define SDCARD_MISO   16
#define SDCARD_MOSI   19
#define SDCARD_CS     17
#define SDCARD_SCK    18

#define GPIO_VBAT     28
#define GPIO_VSYS     29
#define GPIO_REF      26
#define GPIO_MOTOR1A   4
#define GPIO_MOTOR1B   5
#define GPIO_MOTOR2A   6
#define GPIO_MOTOR2B   7
#define GPIO_BUTTON   22
#define GPIO_NEOPIXEL 20
#define GPIO_SOUND    21
#define MOTOR_FREQ 2000  // PWM freq

#define NUMPIXELS 1   // NeoPixel ring size
Adafruit_NeoPixel pixels(NUMPIXELS, GPIO_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// 周波数テーブル
PROGMEM const int freq_table[] = {
    262,  // C (ド) 4
    277,  // C#
    294,  // D (レ)
    311,  // D#
    330,  // E (ミ)
    349,  // F (ファ) 
    370,  // F#
    392,  // G (ソ) 
    415,  // G# 
    440,  // A (ラ) 
    466,  // A# 
    494,  // B (シ) 
    523   // C (ド) 5
};

PROGMEM const char notes1[] = {0,2,4,5,7,9,11,12,99};
PROGMEM const char notes2[] = {12,11,9,7,5,4,2,1,99};
PROGMEM const char note_do[] = {0,99};
PROGMEM const char note_re[] = {2,99};
PROGMEM const char note_mi[] = {4,99};
PROGMEM const char note_fa[] = {5,99};
PROGMEM const char note_so[] = {7,99};
PROGMEM const char note_ra[] = {9,99};
PROGMEM const char note_si[] = {11,99};

unsigned char vram[VRAMSIZE];
int putch_x = 0;
int putch_y = 0;
unsigned int putch_color = 0xffff;
int putch_zoom = 1;

void disp_init(void);
void disp_update(void);
void tft_sendcmd_n_byte(unsigned char cmd ,const unsigned char *p ,char cnt);
void vram_cls(void);
unsigned int vram_point(int x,int y);
void vram_pset(int x,int y,unsigned int color);
void vram_line(int x1 ,int y1 ,int x2 ,int y2 ,unsigned int color);
void vram_fill(int x1 ,int y1 ,int x2 ,int y2 ,unsigned int color);
void vram_locate(int textx, int texty);
void vram_textcolor(unsigned int color);
void vram_putch(unsigned char ch);
void vram_putstr(unsigned char *p);
void vram_putdec(unsigned int num);
void vram_puthex(unsigned int num);
void vram_scroll(int xd,int yd);
void vram_spput(int x,int y, int num,unsigned int color);
void vram_spclr(int x,int y);
void spi_sendbyte(unsigned char data);
unsigned int color16bit(int r,int g,int b);

// ABS
int fnc_abs(int a)
{
  if(a<0)a = -a;
  return (a);
}

// SGN
int fnc_sgn(int a)
{
  if(a<0)return(-1);
  return (1);
}

// 画面キャプチャ
void capture(void)
{
  unsigned int color;
  int x,y,incomingByte;

  // request
  if (Serial.available() == 0) return;
  incomingByte = Serial.read();
  if(incomingByte != 0x43)return;
  // responce
  Serial.write(VRAMXRANGE);
  Serial.write(VRAMYRANGE);
  for(y=0; y<VRAMYRANGE; y++){
    for(x=0; x<VRAMXRANGE; x++){
      color = vram_point(x,y);
      Serial.write(color >> 8);
      Serial.write(color & 0xff);
      Serial.flush();
    }
  }
}

#if DISP_TYPE==ATM0177B3A
#define TFTCLK    14   // clock
#define TFTMOSI   15   // TX
//#define TFTMISO   16   // RX
#define TFTCS     13   // chip select
#define TFTCD     12   // command/data

#define SOFTWARE_RESET            0x01
#define SLEEP_OUT                 0x11
#define PARTIAL_MODE_ON           0x12
#define NORMAL_DISPLAY_MODE_ON    0x13
#define DISPLAY_INVERSION_OFF     0x20
#define DISPLAY_INVERSION_ON      0x21
#define GAMMA_SET                 0x26
#define DISPLAY_ON                0x29
#define COLUMN_ADDRESS_SET        0x2A
#define PAGE_ADDRESS_SET          0x2B
#define MEMORY_WRITE              0x2C
#define MEMORY_ACCESS_CONTROL     0x36
#define INTERFACE_PIXEL_FORMAT    0x3A
#define FRAME_RATE_CONTROL_1      0xB1
#define DISPLAY_INVERSION_CONTROL 0xB4
#define DISPLAY_FUCTION_SET       0xB6
#define POWER_CONTROL_1           0xC0
#define POWER_CONTROL_2           0xC1
#define VCOM_CONTROL_1            0xC5
#define VCOM_OFFSET_CONTROL       0xC7
#define GAM_R_SEL                 0xF2

// INITIALIZE
void disp_init(void)
{
  pinMode(TFTMOSI, OUTPUT);
  pinMode(TFTCLK, OUTPUT);
  pinMode(TFTCD, OUTPUT);
  pinMode(TFTCS, OUTPUT);
  digitalWrite(TFTMOSI, HIGH);
  digitalWrite(TFTCLK, LOW);
  digitalWrite(TFTCD, HIGH);
  digitalWrite(TFTCS, HIGH);

#if SOFT_SPI==0
  SPI1.setCS(TFTCS);
  SPI1.setSCK(TFTCLK);
  SPI1.setTX(TFTMOSI);
//  SPI1.setRX(TFTMISO);
  SPI1.begin();
  SPI1.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
#endif

  delay(500);
  tft_sendcmd(SOFTWARE_RESET);
  delay(500);
  tft_sendcmd(SLEEP_OUT);
  delay(5);
  tft_sendcmd_byte(INTERFACE_PIXEL_FORMAT, 0x05);  //16bit
  tft_sendcmd_byte(GAMMA_SET,0x04);
  tft_sendcmd_byte(GAM_R_SEL,0x01);
  tft_sendcmd(NORMAL_DISPLAY_MODE_ON);
  tft_sendcmd_word(DISPLAY_FUCTION_SET,0xff06);
  tft_sendcmd_word(FRAME_RATE_CONTROL_1,0x0802);
  tft_sendcmd_byte(DISPLAY_INVERSION_CONTROL,0x07);
  tft_sendcmd_word(POWER_CONTROL_1,0x0A02);
  tft_sendcmd_byte(POWER_CONTROL_2,0x02);
  tft_sendcmd_word(VCOM_CONTROL_1,0x5063);
  tft_sendcmd_byte(VCOM_OFFSET_CONTROL,0x00);
  tft_sendcmd_long(COLUMN_ADDRESS_SET,VRAMWIDTH-1); 
  tft_sendcmd_long(PAGE_ADDRESS_SET,VRAMHEIGHT-1); 
  tft_sendcmd(DISPLAY_ON);
  delay(25);
}

//
void spi_sendbyte(unsigned char data)
{
#if SOFT_SPI==0
  SPI1.transfer(data);
#else
  unsigned char bitmask;
  digitalWrite(TFTCLK, LOW);
  bitmask = 0x80;
  while(bitmask){
    digitalWrite(TFTMOSI, ((bitmask & data) != 0));
    digitalWrite(TFTCLK, HIGH);
    digitalWrite(TFTCLK, LOW);
    bitmask >>= 1;
  }
#endif
}

//
void tft_sendcmd(unsigned char data)
{
  digitalWrite(TFTCD,LOW);
  digitalWrite(TFTCS,LOW);
  spi_sendbyte(data);
  digitalWrite(TFTCS,HIGH);
  digitalWrite(TFTCD,HIGH);
}

//
void tft_sendcmd_byte(unsigned char cmd,unsigned char data)
{
  tft_sendcmd(cmd);
  digitalWrite(TFTCS,LOW);
  spi_sendbyte(data);
  digitalWrite(TFTCS,HIGH);
  delay(1);
}

//
void tft_sendcmd_word(unsigned char cmd,unsigned int data)
{
  tft_sendcmd(cmd);
  digitalWrite(TFTCS,LOW);
  spi_sendbyte((unsigned char)(data >> 8));
  spi_sendbyte((unsigned char)(data & 0xff));
  digitalWrite(TFTCS,HIGH);
  delay(1);
}

//
void tft_sendcmd_long(unsigned char cmd,unsigned long data)
{
  tft_sendcmd(cmd);
  digitalWrite(TFTCS,LOW);
  spi_sendbyte((unsigned char)(data >> 24));
  spi_sendbyte((unsigned char)((data >> 16)& 0xff));
  spi_sendbyte((unsigned char)((data >> 8)& 0xff));
  spi_sendbyte((unsigned char)(data & 0xff));
  digitalWrite(TFTCS,HIGH);
  delay(1);
}

// SEND VRAM to DISPLAY 
void disp_update(void)
{
  unsigned int totalcnt;
  unsigned char *ptr;
  
  tft_sendcmd(MEMORY_WRITE);
  digitalWrite(TFTCS,LOW);
  ptr = vram;
  totalcnt = VRAMSIZE;
  while(totalcnt--){
    spi_sendbyte(*ptr++);
  }
  digitalWrite(TFTCS,HIGH);
  delay(1);
//    capture();
}
#endif

//
#if DISP_TYPE==SSD1306

#define OLEDADDR  (0x78 >> 1) // SSD1306 I2C address

#define SET_CONTRAST_CONTROL  0x81
#define SET_CHARGE_PUMP       0x8D
#define SET_ADDRESSING_MODE   0x20
#define SET_DISPLAY_STARTLINE 0x40
#define SET_SEGMENT_REMAP     0xA1
#define SET_ENTIRE_DISPLAY    0xA4
#define SET_DISPLAY_NORMAL    0xA6
#define SET_MULTIPLEX_RATIO   0xA8
#define SET_DISPLAY_ON        0xAF
#define SET_COM_OUTPUT_SCAN   0xC8
#define SET_DISPLAY_OFFSET    0xD3
#define SET_OSCILLATOR_FREQ   0xD5
#define SET_COM_PINS_HARDWARE 0xDA
#define SET_COLUMN_ADDRESS    0x21
#define SET_PAGE_ADDRESS      0x22

//
void oled_command(unsigned char data)
{
  Wire1.beginTransmission(OLEDADDR);
  Wire1.write(0b10000000);
  Wire1.write(data);             
  Wire1.endTransmission();
}

//
void oled_command2(unsigned char data1,unsigned char data2)
{
  Wire1.beginTransmission(OLEDADDR);
  Wire1.write(0b00000000);
  Wire1.write(data1);             
  Wire1.write(data2);             
  Wire1.endTransmission();
}

// INITIALIZE
void disp_init(void)
{
  Wire1.setSDA(18);
  Wire1.setSCL(19);
  Wire1.setClock(2000000);  
  Wire1.begin();

  delay(50);
  oled_command2(SET_MULTIPLEX_RATIO , 0x3F);
  oled_command2(SET_DISPLAY_OFFSET,0);
  oled_command(SET_DISPLAY_STARTLINE);
  oled_command(SET_COM_OUTPUT_SCAN);
  oled_command(SET_SEGMENT_REMAP);
  oled_command2(SET_COM_PINS_HARDWARE, 0x12);
  oled_command2(SET_CONTRAST_CONTROL , 0x80);
  oled_command(SET_ENTIRE_DISPLAY);
  oled_command(SET_DISPLAY_NORMAL);
  oled_command2(SET_OSCILLATOR_FREQ  , 0x80);
  oled_command2(SET_ADDRESSING_MODE  ,0); 
  oled_command2(SET_CHARGE_PUMP , 0x14);
  oled_command(SET_DISPLAY_ON);
  delay(10);
}

//   SEND VRAM to DISPLAY 
void disp_update(void){
  int i,j,x,y;
  unsigned char *ptr,*ptr2;
  unsigned char work;

  Wire1.beginTransmission(OLEDADDR);
  Wire1.write(0b00000000);
  Wire1.write(SET_COLUMN_ADDRESS);
  Wire1.write(0);       // start column
  Wire1.write(VRAMWIDTH-1); // end column
  Wire1.write(SET_PAGE_ADDRESS);
  Wire1.write(0);           // start page
  Wire1.write((VRAMHEIGHT/8)-1); // end page
  Wire1.endTransmission();

  x=0;
  y=0;
  ptr = vram;
  while(y < VRAMHEIGHT){  
    Wire1.beginTransmission(OLEDADDR);
    Wire1.write(0b01000000);

    for(i=0; i<8; i++){
      ptr2 = ptr;
      work = 0;
      for(j=0; j<8; j++){  
        work >>= 1;
        if(*ptr2)work |= 0x80;
        ptr2 += VRAMWIDTH*2;
      }
      Wire1.write(work);
      x++;
      ptr += 2;
    }
    Wire1.endTransmission();
    if(x >= VRAMWIDTH){
      x=0;
      y+=8;
      ptr = vram + (y*VRAMWIDTH*2);
    }
  }
}
//capture();
#endif

#if DISP_TYPE==M154240240
#define TFTCLK    14   // clock
#define TFTMOSI   15   // TX
//#define TFTMISO   16   // RX
#define TFTCS     13   // chip select
#define TFTCD     12   // command/data

// INITIALIZE
void disp_init(void)
{
  pinMode(TFTMOSI, OUTPUT);
  pinMode(TFTCLK, OUTPUT);
  pinMode(TFTCD, OUTPUT);
  pinMode(TFTCS, OUTPUT);
  digitalWrite(TFTMOSI, HIGH);
  digitalWrite(TFTCLK, LOW);
  digitalWrite(TFTCD, HIGH);
  digitalWrite(TFTCS, HIGH);

#if SOFT_SPI==0
  SPI1.setCS(TFTCS);
  SPI1.setSCK(TFTCLK);
  SPI1.setTX(TFTMOSI);
//  SPI1.setRX(TFTMISO);
  SPI1.begin();
  SPI1.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
#endif

  delay(500);
  tft_sendcmd(0x01); // SWRESET (01h): Software Reset 
  delay(200);
  tft_sendcmd(0x11); // SLPOUT (11h): Sleep Out 
  delay(100);
  tft_sendcmd_byte(0x3A, 0x55); // COLMOD (3Ah): Interface Pixel Format
  tft_sendcmd_byte(0x36, 0x00); // MADCTL (36h): Memory Data Access Control 
  tft_sendcmd(0x21);  // INVON (21h): Display Inversion On 
  tft_sendcmd(0x29);  // DISPON (29h): Display On
  delay(200);
  tft_sendcmd_long(0x2A,VRAMWIDTH-1);  // CASET (2Ah): Column Address Set
  tft_sendcmd_long(0x2B,VRAMHEIGHT-1); // RASET (2Bh): Row Address Set
  delay(25);
}

//
void spi_sendbyte(unsigned char data)
{
#if SOFT_SPI==0
  SPI1.transfer(data);
#else
  unsigned char bitmask;
  digitalWrite(TFTCLK, LOW);
  bitmask = 0x80;
  while(bitmask){
    digitalWrite(TFTMOSI, ((bitmask & data) != 0));
    digitalWrite(TFTCLK, HIGH);
    digitalWrite(TFTCLK, LOW);
    bitmask >>= 1;
  }
#endif
}

//
void tft_sendcmd(unsigned char data)
{
  digitalWrite(TFTCD,LOW);
  digitalWrite(TFTCS,LOW);
  spi_sendbyte(data);
  digitalWrite(TFTCS,HIGH);
  digitalWrite(TFTCD,HIGH);
}

//
void tft_sendcmd_byte(unsigned char cmd,unsigned char data)
{
  tft_sendcmd(cmd);
  digitalWrite(TFTCS,LOW);
  spi_sendbyte(data);
  digitalWrite(TFTCS,HIGH);
  delay(1);
}

//
void tft_sendcmd_long(unsigned char cmd,unsigned long data)
{
  tft_sendcmd(cmd);
  digitalWrite(TFTCS,LOW);
  spi_sendbyte((unsigned char)(data >> 24));
  spi_sendbyte((unsigned char)((data >> 16)& 0xff));
  spi_sendbyte((unsigned char)((data >> 8)& 0xff));
  spi_sendbyte((unsigned char)(data & 0xff));
  digitalWrite(TFTCS,HIGH);
  delay(1);
}

// SEND VRAM to DISPLAY 
void disp_update(void)
{
  unsigned int totalcnt;
  unsigned char *ptr;
  
  tft_sendcmd(0x2C); //RAMWR (2Ch): Memory Write
  digitalWrite(TFTCS,LOW);
  ptr = vram;
  totalcnt = VRAMSIZE;
  while(totalcnt--){
    spi_sendbyte(*ptr++);
  }
  digitalWrite(TFTCS,HIGH);
  delay(1);
//    capture();
}
#endif

// CALC. COLOR
unsigned int color16bit(int r,int g,int b)
{
// RRRRRGGGGGGBBBBB
// blue :bit4 -bit0 (0-31)
// green:bit10-bit5 (0-63)
// red  :bit15-bit11(0-31)
  r >>= 3;
  g >>= 2;
  b >>= 3;
  return(((unsigned int)r << 11)+(g << 5)+b);
}

// CLEAR VRAM
void vram_cls(void)
{
  long i;
  unsigned char *ptr;

  ptr = vram;
  i = VRAMSIZE;
  while(i--){
    *ptr++ = 0;
  }
}

// GET PIXEL
unsigned int vram_point(int x,int y)
{
  int i;
  unsigned int color;
  unsigned char *ptr;

#if DISP_ROTATE==1
  i=x;
  x=(VRAMWIDTH-1)-y;
  y=i;
#endif
#if DISP_ROTATE==2
  x=(VRAMWIDTH-1)-x;
  y=(VRAMHEIGHT-1)-y;
#endif
#if DISP_ROTATE==3
  i=x;
  x=y;
  y=(VRAMHEIGHT-1)-i;
#endif

  if(x<0)return(0);
  if(y<0)return(0);
  if(x>=VRAMWIDTH)return(0);
  if(y>=VRAMHEIGHT)return(0);

  ptr = vram;
  ptr += ((long)y*(VRAMWIDTH*2)) + (x*2);
  color =  *ptr << 8;
  ptr++;
  color += *ptr;
  return(color);
}

// DRAW PIXEL
void vram_pset(int x,int y,unsigned int color)
{
  int i;
  unsigned char *ptr;

#if DISP_ROTATE==1
  i=x;
  x=(VRAMWIDTH-1)-y;
  y=i;
#endif
#if DISP_ROTATE==2
  x=(VRAMWIDTH-1)-x;
  y=(VRAMHEIGHT-1)-y;
#endif
#if DISP_ROTATE==3
  i=x;
  x=y;
  y=(VRAMHEIGHT-1)-i;
#endif

  if(x<0)return;
  if(y<0)return;
  if(x>=VRAMWIDTH)return;
  if(y>=VRAMHEIGHT)return;
  ptr = vram;
  ptr += ((long)y*(VRAMWIDTH*2)) + (x*2);

  *ptr++ = color >> 8;   //high
  *ptr = color & 0xff;   //low 
}

// BOX FILL
void vram_fill(int x1 ,int y1 ,int x2 ,int y2 ,unsigned int color)
{
  int x,y;
  for(y=y1; y<=y2; y++){
    for(x=x1; x<=x2; x++){
      vram_pset(x, y ,color); //ドット描画
    }
  }
}

// DRAW LINE
void vram_line(int x1 ,int y1 ,int x2 ,int y2 ,unsigned int color)
{
  int xd;    // X2-X1座標の距離
  int yd;    // Y2-Y1座標の距離
  int xs=1;  // X方向の1pixel移動量
  int ys=1;  // Y方向の1pixel移動量
  int e;

  xd = x2 - x1;  // X2-X1座標の距離
  if(xd < 0){
    xd = -xd;  // X2-X1座標の絶対値
    xs = -1;    // X方向の1pixel移動量
  }
  yd = y2 - y1;  // Y2-Y1座標の距離
  if(yd < 0){
    yd = -yd;  // Y2-Y1座標の絶対値
    ys = -1;    // Y方向の1pixel移動量
  }
  vram_pset (x1, y1 ,color); //ドット描画
  e = 0;
  if( yd < xd ) {
    while( x1 != x2) {
      x1 += xs;
      e += (2 * yd);
      if(e >= xd) {
        y1 += ys;
        e -= (2 * xd);
      }
      vram_pset(x1, y1 ,color); //ドット描画
    }
  }else{
    while( y1 != y2) {
      y1 += ys;
      e += (2 * xd);
      if(e >= yd) {
        x1 += xs;
        e -= (2 * yd);
      }
      vram_pset(x1, y1 ,color); //ドット描画
    }
  }
}

// LOCATE
void vram_locate(int textx, int texty)
{
  putch_x = textx;
  putch_y = texty;
}

// TEXT COLOR
void vram_textcolor(unsigned int color)
{
  putch_color = color;
}

// TEXT ZOOM
void vram_textzoom(int zoom)
{
  putch_zoom = zoom;
}

// PRINT CHARACTER
void vram_putch(unsigned char ch)
{
  char i,j;
  unsigned char bitdata;
  int idx,x,y;

  if(ch =='\n')putch_x += VRAMXMAX;
  if(putch_x > (VRAMXRANGE-(8*putch_zoom))){
    putch_x = 0;
    putch_y += 8*putch_zoom;
    y = (VRAMYRANGE-(8*putch_zoom));
    if(putch_y > y){
      vram_scroll(0,putch_y - y);
      putch_y = y;
    }
  }  
  if(ch < 0x20)return;
  if(ch >= 0x80)return;

  ch -= 0x20;
  idx = ((int)ch * 8);
  for(i=0 ;i<8 ;i++) {
    bitdata = font[idx];
    idx++;
    for(j=0; j<8; j++){
      if(bitdata & 0x80){
        x=putch_x+(j*putch_zoom);
        y=putch_y+(i*putch_zoom);
        vram_fill(x,y,x+putch_zoom-1,y+putch_zoom-1,putch_color);
      }
      bitdata <<= 1;
    }
  }
  putch_x += 8*putch_zoom;
}

// PRINT STRING
void vram_putstr(unsigned char *p)
{
  while(*p != 0){
    vram_putch( *p++ );
  }
}

// PRINT DECIMAL
void vram_putdec(unsigned int num)
{
  unsigned char ch;
  unsigned int shift=10000;
  
  while(shift > 0){
    ch = (num / shift) % 10;
    ch += '0';
    vram_putch(ch);
    shift /= 10;
  }
}

// PRINT VOLT
void vram_putvolt(unsigned int num)
{
  vram_putch((num / 1000)+'0');
  vram_putch('.');
  vram_putch(((num / 100) % 10)+'0');
  vram_putch(((num / 10) % 10)+'0');
  vram_putch('V');
}

// PRINT HEXADECIMAL
void vram_puthex(unsigned char num)
{
  unsigned char ch;
  ch = (num >> 4)+'0';
  if(ch > '9') ch += ('A'-('9'+1));
  vram_putch( ch);

  ch = (num & 0xf)+'0';
  if(ch > '9') ch += ('A'-('9'+1));
  vram_putch( ch);
}

// SCROLL
void vram_scroll(int xd,int yd)
{
  int x,y;
  unsigned int color;

  for(y=0;y<VRAMYRANGE;y++){
    for(x=0;x<VRAMXRANGE;x++){
      color = vram_point(x+xd, y+yd);
      vram_pset(x,y,color);
    }
  }
}

// BCDを10進に変換
unsigned char bcd_to_num(unsigned char num){
  return((num >> 4)*10 + (num & 0xf));
}

// 10進をBCDに変換
unsigned char num_to_bcd(unsigned char num){
  unsigned char numhigh,numlow;
  numhigh = num / 10;
  numlow = num % 10;
  return((numhigh << 4) + numlow);
}

// 
int str_length(unsigned char *str)
{
  int l;
  for(l=0;;l++){
    if(*str == 0)break;
    str++;
  }
  return(l);
}

//----
void print_text(int x, int y, unsigned char *str)
{
  int l;
  l = str_length(str);
  vram_fill(x,y,x+l*24,y+24,0);
  vram_locate(x,y);
  vram_putstr(str);
  disp_update();
}

//----SDカードスロット初期化
void sdcard_init(void)
{
  SPI.setRX(SDCARD_MISO);
  SPI.setTX(SDCARD_MOSI);
  SPI.setSCK(SDCARD_SCK);

  if (!SD.begin(SDCARD_CS)) { // error
    return;
  }
}

//----
void sdcard_test(void)
{
  char filename[] = "example.txt";
  File myFile;
  char tempstr[20];

  print_text(0,0*24,(unsigned char *)"SD CARD");
  print_text(0,1*24,(unsigned char *)"WRITE");
  button_wait();
  myFile = SD.open(filename, FILE_WRITE);
  myFile.println("Hello!");
  myFile.close();

  if (SD.exists(filename)) {
    print_text(0,2*24,(unsigned char *)"EXIST");
  } else {
    print_text(0,2*24,(unsigned char *)"NOT EXIST");
  }

  print_text(0,3*24,(unsigned char *)"READ");
  button_wait();
  myFile = SD.open(filename);
  if (!myFile) {
    // read error    
    return;
  }
  for(int i=0; i<20; i++){
    if(!myFile.available()){
      tempstr[i] = 0;
      break;
    }
    tempstr[i] = myFile.read();
  }
  myFile.close();
  print_text(0,4*24,(unsigned char *)tempstr);

  print_text(0,5*24,(unsigned char *)"DELETE");
  button_wait();
  SD.remove(filename);
  button_wait();
}

//----ボタン初期化
void button_init(void)
{
  pinMode(GPIO_BUTTON, INPUT_PULLUP);
}

//----
void button_wait(void)
{
  int x,y;
  x = 0;
  y = 240-24;

  vram_fill(x,y,x+9*24,y+24,color16bit(128,0,0));
  vram_locate(x,y);
  vram_putstr((unsigned char *)"PUSH BTN.");
  disp_update();

  while(1){
    if (digitalRead(GPIO_BUTTON) == LOW){
      break;
    }
    delay(20);
  }
  vram_fill(x,y,x+9*24,y+24,0);
  disp_update();
}

//----AD初期化
void adc_init(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
  analogReadResolution(12); // 0～4095
}

//---- バッテリチェック
void bat_check(void) {
  int adcount;
  int vsys;
  int vbat;
  int x,y;

  adcount = analogRead(GPIO_VSYS);
  vsys = (int)((long)adcount * 3300 / 4095);
  vsys *= 3;  // 分圧しているぶん3倍

  adcount = analogRead(GPIO_VBAT);
  vbat = (int)((long)adcount * 3300 / 4095);
  vbat *= 2;  // 分圧しているぶん2倍

  x=0;
  y=0;
  vram_fill(x,y,x+24*10,y+24*2,color16bit(0,0,0));
  vram_locate(x,y);
  vram_putstr((unsigned char *)"VSYS:");
  vram_putvolt(vsys);
  vram_putstr((unsigned char *)"VBAT:");
  vram_putvolt(vbat);
  disp_update();

  if(vbat < 1800){
    print_text(0,2*24,(unsigned char *)"UNDER 1.8V");
    sound_play((char *)notes2);
    button_wait();    
  }
  if(vbat > 5500){
    print_text(0,2*24,(unsigned char *)"OVER 5.5V");
    sound_play((char *)notes2);
    button_wait();    
  }
}

//----サウンド初期化
void sound_init(void)
{
  pinMode(GPIO_SOUND, OUTPUT);
}

//----
void sound_end(void)
{
  digitalWrite(GPIO_SOUND, LOW); // PWM停止
  pinMode(GPIO_SOUND, INPUT);
}

//----
void sound_play(char *p_note)
{
  char note;

  sound_init();

  while(1){
    note = *p_note++;
    if (note == 99)break;

    analogWriteFreq(freq_table[note]);
    analogWriteRange(1024);
    analogWrite(GPIO_SOUND, 512); // デューティ比 50%
    delay(150);
    analogWrite(GPIO_SOUND, 0); // デューティ比 0%
  }
  sound_end();
}

//----NeoPixel初期化
void neopixel_init() {
  pixels.begin(); // INITIALIZE NeoPixel strip object
  pixels.clear(); // Set all pixel colors to 'off'
  delay(1);
  neopixel_set(0,0,0);
}

//----NeoPixel点灯
void neopixel_set(unsigned char r,unsigned char g,unsigned char b)
{
  pixels.setPixelColor(0, pixels.Color(g, r, b));
  pixels.show();   // 
}

//----モータ初期化
void motor_init()
{
  pinMode(GPIO_MOTOR1A, OUTPUT);
  pinMode(GPIO_MOTOR1B, OUTPUT);
  pinMode(GPIO_MOTOR2A, OUTPUT);
  pinMode(GPIO_MOTOR2B, OUTPUT);
  analogWriteFreq(MOTOR_FREQ);
  analogWriteRange(1024);

  analogWrite(GPIO_MOTOR1A, 0); // デューティ比 0%
  analogWrite(GPIO_MOTOR1B, 0); // デューティ比 0%
  analogWrite(GPIO_MOTOR2A, 0); // デューティ比 0%
  analogWrite(GPIO_MOTOR2B, 0); // デューティ比 0%
}

//----
void motor_end(void)
{
  digitalWrite(GPIO_MOTOR1A, LOW); // PWM停止
  digitalWrite(GPIO_MOTOR1B, LOW); // PWM停止
  digitalWrite(GPIO_MOTOR2A, LOW); // PWM停止
  digitalWrite(GPIO_MOTOR2B, LOW); // PWM停止

  pinMode(GPIO_MOTOR1A, INPUT);
  pinMode(GPIO_MOTOR1B, INPUT);
  pinMode(GPIO_MOTOR2A, INPUT);
  pinMode(GPIO_MOTOR2B, INPUT);
}

//----モータ制御
void motor_control(int num, int power)
{
  int duty_a = 0;
  int duty_b = 0;

  if(power > 0){
    if(power > 100) power = 100;
    duty_a = (int)((long)power * 1023/100);
    duty_b = 0;
  }else if(power < 0){
    if(power < -100) power = -100;
    power = -power;
    duty_a = 0;
    duty_b = (int)((long)power * 1023/100);
  }
  // デューティ比
  if(num==1){
    analogWrite(GPIO_MOTOR1A, duty_a); 
    analogWrite(GPIO_MOTOR1B, duty_b);
  }else{
    analogWrite(GPIO_MOTOR2A, duty_a); 
    analogWrite(GPIO_MOTOR2B, duty_b); 
  }
}

//----モータペア
void motor_pair(int left_power,int right_power)
{
  motor_control(1,right_power);
  motor_control(2,left_power);
}

//----
void  motor_test1()
{
  int mot_num;
  int mot_pow=40;

  button_wait();
  motor_init();

  for(mot_num=1; mot_num<3; mot_num++){
    print_text(0,3*24,(unsigned char *)"FOW_");
    motor_control(mot_num,mot_pow);
    delay(2000);
    print_text(0,3*24,(unsigned char *)"STOP");
    motor_control(mot_num,0);
    delay(1000);
    print_text(0,3*24,(unsigned char *)"BACK");
    motor_control(mot_num,-mot_pow);
    delay(2000);
    print_text(0,3*24,(unsigned char *)"STOP");
    motor_control(mot_num,0);
    delay(1000);
  }

  motor_end();
}
//----
void motor_test()
{ 
  int mot_pow=40;

  while(1){
    motor_init();
    print_text(0,4*24,(unsigned char *)"MOTOR TEST");
    button_wait();
    bat_check();

    print_text(0,5*24,(unsigned char *)"FOW_");
    motor_pair(mot_pow,mot_pow);
    delay(2000);
    print_text(0,5*24,(unsigned char *)"STOP");
    motor_pair(0,0);
    delay(1000);

    print_text(0,5*24,(unsigned char *)"BACK");
    motor_pair(-mot_pow,-mot_pow);
    delay(2000);
    print_text(0,5*24,(unsigned char *)"STOP");
    motor_pair(0,0);
    delay(1000);

    print_text(0,5*24,(unsigned char *)"TURN");
    motor_pair(mot_pow,-mot_pow);
    delay(2000);
    print_text(0,5*24,(unsigned char *)"STOP");
    motor_pair(0,0);
    delay(1000);

    print_text(0,5*24,(unsigned char *)"TURN");
    motor_pair(-mot_pow,mot_pow);
    delay(2000);
    print_text(0,5*24,(unsigned char *)"STOP");
    motor_pair(0,0);
    delay(1000);
    motor_end();
    sound_play((char *)notes1);
  }
}

//----LINE TRACE ROBOT
void linetrace()
{
#define THRESHOLD 0xA0
    int cnt;
    int mot_pow;

    mot_pow = 40;
    motor_init();
    print_text(0,0,(unsigned char *)"LINE TRACE");
    button_wait();
    bat_check();
    neopixel_set(99,99,99);

    while(1){
        cnt = adc0_get();
        if( cnt < THRESHOLD){
            motor_pair(mot_pow,0);
        }else{
            motor_pair(0,mot_pow);
        }
        delay(10);
    }
}

//----センサー
int adc0_get()
{
#define LOOPMAX  5   // 測定回数
#define DEVIDE  50   // 

    int total;
    int i;

    delay(1);
    total = 0;
    for(i=0;i< LOOPMAX;i++){
        total += analogRead(GPIO_REF);
        delay(1);
    }
    total /= DEVIDE;
    if(total > 255)total=255;
    return(total);
}

//----反射光センサー
void ref_test()
{
  int cnt;
  int x,y;
  print_text(0,0,(unsigned char *)"REF TEST");
  button_wait();
  neopixel_set(99,99,99);
  while(1)
  {
    cnt = adc0_get();
    x=0;
    y=24*2;
    vram_fill(x,y,x+2*24,y+24,color16bit(0,0,0));
    vram_locate(x,y);
    vram_puthex((unsigned char)cnt);
    disp_update();
    delay(200);
  }
}

//----カラーセンサー
void color_test()
{
  int x,y;
  int red,green,blue;
  int red_only,green_only,blue_only;

  print_text(0,0,(unsigned char *)"COLOR TEST");
  button_wait();
  while(1){
    neopixel_set(150,0,0 );  // R
    red = adc0_get();

    neopixel_set(0,230,0 );  // G
    green = adc0_get();

    neopixel_set(0,0,230 );  // B
    blue = adc0_get();

    neopixel_set(0,0,0);  // OFF

    x=0;
    y=1*24;
    vram_fill(x,y,x+8*24,y+4*24,color16bit(red,green,blue));
    vram_locate(x+0   ,y+24*3);
    vram_puthex((unsigned char)red);
    vram_locate(x+24*3,y+24*3);
    vram_puthex((unsigned char)green);
    vram_locate(x+24*6,y+24*3);
    vram_puthex((unsigned char)blue);

    red_only   = red - ((green + blue)/2);
    green_only = green - ((blue + red)/2);
    blue_only  = blue - ((green + red)/2);
    x=0;
    y=8*24;
    if(red_only > 0x20){
      vram_fill(0*24,7*24,8*24,8*24,color16bit(255,0,0));
      print_text(x,y,(unsigned char *)"RED__");
      sound_play((char *)note_do);
    }else if (green_only > 0x20){
      vram_fill(0*24,7*24,8*24,8*24,color16bit(0,255,0));
      print_text(x,y,(unsigned char *)"GREEN");
      sound_play((char *)note_fa);
    }else if (blue_only > 0x20){
      vram_fill(0*24,7*24,8*24,8*24,color16bit(0,0,255));
      print_text(x,y,(unsigned char *)"BLUE_");
      sound_play((char *)note_si);
    }else{
      vram_fill(0*24,7*24,8*24,8*24,color16bit(0,0,0));
      print_text(x,y,(unsigned char *)"_____");
    }
  }
}

//----
void setup(void)
{
//  Serial.begin(115200);
//  while(!Serial);
  button_init();
  sdcard_init();
  adc_init();
  disp_init();
  neopixel_init();
  vram_textzoom(FONTSIZE);
}

//----
void loop()
{
//  ref_test();   // 反射光センサー
//  color_test(); // カラーセンサー
//  linetrace();  // ライントレース
  motor_test(); // モーターテスト
//  sdcard_test();  // SDカード読み書き
}
