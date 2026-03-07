# DIY-ROBOT
# for Raspberry Pi Pico(MicroPython)
# and OLED(SSD1306) or TFT-LCD(M154-240240-RGB ST7789)
# and Motor driver(TC78H653FTG)
# and NeoPixel(RGBLED PL9823-F5)
# (C)2026 takuya matsubara

import neopixel
import time
import random
import math
from machine import SPI
from machine import I2C
from machine import ADC
from machine import Pin
from machine import PWM
from micropython import const

SSD1306    = const(0)  # 0.96inch OLED Display
M154240240 = const(1)  # 1.5inch TFT Display M154-240240-RGB

# ディスプレイに合わせて書き換える
DISP_TYPE  = const(SSD1306)      # DISPLAY TYPE
#DISP_TYPE  = const(M154240240)   # DISPLAY TYPE

if DISP_TYPE == M154240240:
    VRAM_WIDTH = 240   # X解像度
    VRAM_HEIGHT = 240  # Y解像度
    FONTSIZE = 3       # フォント拡大率
else:
    VRAM_WIDTH = 128   # X解像度
    VRAM_HEIGHT = 64   # Y解像度
    FONTSIZE = 1       # フォント拡大率

FONTPIXEL = FONTSIZE*8
VRAM_SIZE = VRAM_WIDTH * VRAM_HEIGHT * 2
vram = bytearray(VRAM_SIZE)
def_posx = 0
def_posy = 0
def_tcolor = 0xFFFF

# 周波数テーブル
freq_table =(
    262,  # C (ド)
    277,  # C#
    294,  # D (レ)
    311,  # D#
    330,  # E (ミ)
    349,  # F (ファ)
    370,  # F#
    392,  # G (ソ)
    415,  # G# 
    440,  # A (ラ)
    466,  # A# 
    494,  # B (シ)
    523   # C (ド)
)

notes1 = (0,2,4,5,7,9,11,12)
notes2 = (12,11,9,7,5,4,2,1)
note_do = (0,99)
note_re = (2,99)
note_mi = (4,99)
note_fa = (5,99)
note_so = (7,99)
note_ra = (9,99)
note_si = (11,99)

GPIO_OLEDSDA = const(2)
GPIO_OLEDSCL = const(3)
GPIO_TFTCS   = const(13)
GPIO_TFTDC   = const(12)
GPIO_TFTSCK  = const(14)
GPIO_TFTTX   = const(15)
GPIO_VSYS    = const(29)
GPIO_VBAT    = const(28)
GPIO_REF     = const(26)
GPIO_MOTOR1A = const(4)
GPIO_MOTOR1B = const(5)
GPIO_MOTOR2A = const(6)
GPIO_MOTOR2B = const(7)
GPIO_BUTTON  = const(22)
GPIO_NEOPIXEL = const(20)
GPIO_SOUND   = const(21)

#font data 0x20-0x7F
font = (
 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 
,0x18,0x18,0x18,0x18,0x18,0x00,0x18,0x00 
,0x6c,0x6c,0x24,0x48,0x00,0x00,0x00,0x00 
,0x28,0x28,0xfe,0x28,0xfe,0x28,0x28,0x00 
,0x10,0x7e,0x90,0x7c,0x12,0xfc,0x10,0x00 
,0x42,0xa4,0x48,0x10,0x24,0x4a,0x84,0x00 
,0x30,0x48,0x48,0x30,0x4a,0x84,0x7a,0x00 
,0x18,0x18,0x08,0x10,0x00,0x00,0x00,0x00 
,0x18,0x20,0x40,0x40,0x40,0x20,0x18,0x00 
,0x30,0x08,0x04,0x04,0x04,0x08,0x30,0x00 
,0x92,0x54,0x38,0xfe,0x38,0x54,0x92,0x00 
,0x10,0x10,0x10,0xfe,0x10,0x10,0x10,0x00 
,0x00,0x00,0x00,0x00,0x30,0x30,0x60,0x00 
,0x00,0x00,0x00,0xfe,0x00,0x00,0x00,0x00 
,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00 
,0x00,0x02,0x04,0x08,0x10,0x20,0x40,0x00 
,0x7c,0x82,0x8a,0x92,0xa2,0x82,0x7c,0x00 
,0x10,0x30,0x10,0x10,0x10,0x10,0x38,0x00 
,0xfc,0x02,0x02,0x3c,0x40,0x80,0xfe,0x00 
,0xfc,0x02,0x02,0xfc,0x02,0x02,0xfc,0x00 
,0x18,0x28,0x48,0x88,0xfe,0x08,0x08,0x00 
,0xfe,0x80,0x80,0xfc,0x02,0x02,0xfc,0x00 
,0x7e,0x80,0x80,0xfc,0x82,0x82,0x7c,0x00 
,0xfe,0x02,0x04,0x08,0x10,0x20,0x40,0x00 
,0x7c,0x82,0x82,0x7c,0x82,0x82,0x7c,0x00 
,0x7c,0x82,0x82,0x7e,0x02,0x02,0xfc,0x00 
,0x30,0x30,0x00,0x00,0x30,0x30,0x00,0x00 
,0x30,0x30,0x00,0x00,0x30,0x30,0x60,0x00 
,0x08,0x10,0x20,0x40,0x20,0x10,0x08,0x00 
,0x00,0x00,0x7e,0x00,0x7e,0x00,0x00,0x00 
,0x40,0x20,0x10,0x08,0x10,0x20,0x40,0x00 
,0x7c,0x82,0x82,0x0c,0x10,0x00,0x10,0x00 
,0x7c,0x82,0x02,0x3e,0x42,0x42,0x3c,0x00 
,0x38,0x44,0x82,0x82,0xfe,0x82,0x82,0x00 
,0xf0,0x88,0x88,0xfc,0x82,0x82,0xfc,0x00 
,0x7c,0x82,0x80,0x80,0x80,0x82,0x7c,0x00 
,0xf8,0x84,0x82,0x82,0x82,0x84,0xf8,0x00 
,0xfe,0x80,0x80,0xfe,0x80,0x80,0xfe,0x00 
,0xfe,0x80,0x80,0xfe,0x80,0x80,0x80,0x00 
,0x7c,0x82,0x80,0x8e,0x82,0x82,0x7c,0x00 
,0x82,0x82,0x82,0xfe,0x82,0x82,0x82,0x00 
,0x38,0x10,0x10,0x10,0x10,0x10,0x38,0x00 
,0x38,0x10,0x10,0x10,0x10,0x90,0x60,0x00 
,0x88,0x90,0xa0,0xc0,0xa0,0x90,0x88,0x00 
,0x80,0x80,0x80,0x80,0x80,0x80,0xfe,0x00 
,0x82,0xc6,0xc6,0xaa,0xaa,0x92,0x82,0x00 
,0x82,0xc2,0xa2,0x92,0x8a,0x86,0x82,0x00 
,0x7c,0x82,0x82,0x82,0x82,0x82,0x7c,0x00 
,0xfc,0x82,0x82,0xfc,0x80,0x80,0x80,0x00 
,0x7c,0x82,0x82,0x82,0xba,0x84,0x7a,0x00 
,0xfc,0x82,0x82,0xfc,0x88,0x84,0x82,0x00 
,0x7e,0x80,0x80,0x7c,0x02,0x02,0xfc,0x00 
,0xfe,0x10,0x10,0x10,0x10,0x10,0x10,0x00 
,0x82,0x82,0x82,0x82,0x82,0x82,0x7c,0x00 
,0x82,0x82,0x44,0x44,0x28,0x28,0x10,0x00 
,0x82,0x92,0xaa,0xaa,0xc6,0xc6,0x82,0x00 
,0x82,0x44,0x28,0x10,0x28,0x44,0x82,0x00 
,0x82,0x44,0x28,0x10,0x10,0x10,0x10,0x00 
,0xfe,0x04,0x08,0x10,0x20,0x40,0xfe,0x00 
,0x18,0x10,0x10,0x10,0x10,0x10,0x18,0x00 
,0x44,0x28,0x10,0x7c,0x10,0x7c,0x10,0x00 
,0x30,0x10,0x10,0x10,0x10,0x10,0x30,0x00 
,0x10,0x28,0x44,0x00,0x00,0x00,0x00,0x00 
,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0x00 
,0x18,0x18,0x08,0x10,0x00,0x00,0x00,0x00 
,0x00,0x7c,0x02,0x7e,0x82,0x82,0x7e,0x00 
,0x00,0x80,0x80,0xfc,0x82,0x82,0xfc,0x00 
,0x00,0x7c,0x82,0x80,0x80,0x82,0x7c,0x00 
,0x00,0x02,0x02,0x7e,0x82,0x82,0x7e,0x00 
,0x00,0x7c,0x82,0x82,0xfe,0x80,0x7e,0x00 
,0x00,0x38,0x20,0xfc,0x20,0x20,0x20,0x00 
,0x00,0x7c,0x82,0x82,0x7e,0x02,0x7c,0x00 
,0x00,0x80,0x80,0xfc,0x82,0x82,0x82,0x00 
,0x00,0x10,0x00,0x10,0x10,0x10,0x38,0x00 
,0x00,0x10,0x00,0x10,0x10,0x90,0x60,0x00 
,0x00,0x80,0x88,0x90,0xe0,0x90,0x88,0x00 
,0x80,0x80,0x80,0x80,0x80,0x80,0xc0,0x00 
,0x00,0xec,0x92,0x92,0x92,0x92,0x92,0x00 
,0x00,0xf8,0x84,0x82,0x82,0x82,0x82,0x00 
,0x00,0x7c,0x82,0x82,0x82,0x82,0x7c,0x00 
,0x00,0xfc,0x82,0x82,0xfc,0x80,0x80,0x00 
,0x00,0x7e,0x82,0x82,0x7e,0x02,0x02,0x00 
,0x00,0x98,0xa0,0xc0,0x80,0x80,0x80,0x00 
,0x00,0x7e,0x80,0x7c,0x02,0x02,0xfc,0x00 
,0x00,0x10,0xfe,0x10,0x10,0x10,0x10,0x00 
,0x00,0x82,0x82,0x82,0x82,0x82,0x7c,0x00 
,0x00,0x82,0x44,0x44,0x28,0x28,0x10,0x00 
,0x00,0x92,0xaa,0xaa,0xaa,0xaa,0x44,0x00 
,0x00,0x84,0x48,0x30,0x30,0x48,0x84,0x00 
,0x00,0x84,0x48,0x30,0x10,0x10,0x60,0x00 
,0x00,0xfc,0x08,0x10,0x20,0x40,0xfc,0x00 
,0x08,0x10,0x10,0x20,0x10,0x10,0x08,0x00 
,0x10,0x10,0x10,0x00,0x10,0x10,0x10,0x00 
,0x10,0x08,0x08,0x04,0x08,0x08,0x10,0x00 
,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x00 
,0x81,0x42,0x24,0x18,0x18,0x24,0x42,0x81 
)

#----16bit color
def color16bit(r,g,b):
    r >>= 3
    g >>= 2
    b >>= 3
    return (r << 11)+(g << 5)+b

#---- clear vram
def vram_cls():
    for i in range(VRAM_WIDTH*VRAM_HEIGHT*2):
        vram[i] = 0

#----M154-240240-RGB
#---- (TFT)コマンド送信
def tft_command(cmd):
    tftdc.low()  # command
    tftcs.low()  # enable
    tftspi.write(cmd.to_bytes(1, "big"))
    tftcs.high() # disable

#---- (TFT)データ送信
def tft_data(dat):
    tftdc.high() # data 
    tftcs.low()  # enable
    tftspi.write(dat.to_bytes(1, "big"))
    tftcs.high() # disable
    
#---- (TFT)初期化
def tft_init():
    global tftspi,tftcs,tftdc
    
    # polarity=0 phase=0(クロックlow=アイドル、データ立ち上がり=サンプリング)
    # 1Sec / Serial clock cycle 16 ns = 62500000 Hz
    tftspi = SPI(1, baudrate=16000000,sck=Pin(GPIO_TFTSCK), mosi=Pin(GPIO_TFTTX))
    tftcs = Pin(GPIO_TFTCS, mode=Pin.OUT)
    tftdc = Pin(GPIO_TFTDC, mode=Pin.OUT)
    tftcs.high() # disable 
    time.sleep(0.5)
    tft_command(0x01) # SWRESET (01h): Software Reset 
    time.sleep(0.2)
    tft_command(0x11) # SLPOUT (11h): Sleep Out 
    time.sleep(0.1)
    tft_command(0x3A)  # COLMOD (3Ah): Interface Pixel Format
    tft_data(0x55)
    #  b4-6:101=65K-color / 110=262K-color
    #  b0-2:101=16bit / 011=12bit / 110=18bit
    tft_command(0x36) # MADCTL (36h): Memory Data Access Control 
    tft_data(0x00)
    tft_command(0x21) # INVON (21h): Display Inversion On 
    tft_command(0x29) # DISPON (29h): Display On
    time.sleep(0.2)
    tft_command(0x2A)    # CASET (2Ah): Column Address Set
    tft_data(0x00)
    tft_data(0x00)
    tft_data((VRAM_WIDTH-1) >> 8)
    tft_data((VRAM_WIDTH-1) & 0xff)

    tft_command(0x2B)    # RASET (2Bh): Row Address Set
    tft_data(0x00)
    tft_data(0x00)
    tft_data((VRAM_WIDTH-1) >> 8)
    tft_data((VRAM_WIDTH-1) & 0xff)
    time.sleep(0.1)

#---- (TFT)画面更新(for Pico/W)
def tft_update():
    tft_command(0x2C)   # RAMWR (2Ch): Memory Write
    tftdc.high() # data
    tftcs.low()  # enable
    # メモリアロケーションエラーを回避するため分割して転送
    ptr = 0
    division = VRAM_WIDTH*2*16  #
    while ptr < VRAM_SIZE:
        tftspi.write(bytes(vram[ptr:ptr+division]))
        ptr += division
       
    tftcs.high() # disable 

#---- (TFT)画面更新(for Pico2/2W)
#def tft_update():
#    tft_command(0x2C)   # RAMWR (2Ch): Memory Write
#    tftdc.high() # data
#    tftcs.low()  # enable
#    tftspi.write(bytes(vram))
#    tftcs.high() # disable 

#----(OLED)コマンド送信byte
def oled_command( data):
    send = bytearray(2)
    send[0] = 0x80 # control(single + command)
    send[1] = data             
    i2cport.writeto(OLED_ADDR, send)

#----(OLED)コマンド送信word
def oled_command2( data1, data2):
    send = bytearray(3)
    send[0] = 0x00 # control(Continuation + command)
    send[1] = data1             
    send[2] = data2             
    i2cport.writeto(OLED_ADDR, send)

#----(OLED)初期化
def oled_init():
    global i2cport,OLED_ADDR
    OLED_ADDR = const(0x78 >> 1)  # SSD1306 slave address

    SET_CONTRAST_CONTROL  = const(0x81)
    SET_CHARGE_PUMP       = const(0x8D)
    SET_ADDRESSING_MODE   = const(0x20)
    SET_DISPLAY_STARTLINE = const(0x40)
    SET_SEGMENT_REMAP     = const(0xA1)
    SET_ENTIRE_DISPLAY    = const(0xA4) 
    SET_DISPLAY_NORMAL    = const(0xA6)
    SET_MULTIPLEX_RATIO   = const(0xA8)
    SET_DISPLAY_ON        = const(0xAF)
    SET_COM_OUTPUT_SCAN   = const(0xC8)
    SET_DISPLAY_OFFSET    = const(0xD3)
    SET_OSCILLATOR_FREQ   = const(0xD5)
    SET_COM_PINS_HARDWARE = const(0xDA)

    i2cport = I2C(1, scl=Pin(GPIO_OLEDSCL), sda=Pin(GPIO_OLEDSDA), freq=2000000)
    time.sleep(0.05)
    oled_command2(SET_MULTIPLEX_RATIO , 0x3F)  # multiplex ratio
    oled_command2(SET_DISPLAY_OFFSET,0)
    oled_command(SET_DISPLAY_STARTLINE)  # starting address of display RAM
    oled_command(SET_COM_OUTPUT_SCAN)
    oled_command(SET_SEGMENT_REMAP)  # column address and the segment driver
    oled_command2(SET_COM_PINS_HARDWARE, 0x12)
    oled_command2(SET_CONTRAST_CONTROL , 0x80)
    oled_command(SET_ENTIRE_DISPLAY) # entire display “ON” stage
    oled_command(SET_DISPLAY_NORMAL)
    oled_command2(SET_OSCILLATOR_FREQ  , 0x80)  
    oled_command2(SET_ADDRESSING_MODE  ,0) 
    oled_command2(SET_CHARGE_PUMP , 0x14)  # Enable charge pump
    oled_command(SET_DISPLAY_ON)
    time.sleep(1)

#----(OLED) 画面更新
def oled_update():
    SET_COLUMN_ADDRESS    = const(0x21)
    SET_PAGE_ADDRESS      = const(0x22)

    send = bytearray(7)
    send[0] = 0x00  # control(Continuation + command)
    send[1] = SET_COLUMN_ADDRESS
    send[2] = 0       # start column
    send[3] = VRAM_WIDTH-1 # end column
    send[4] = SET_PAGE_ADDRESS
    send[5] = 0           # start page
    send[6] = int(VRAM_HEIGHT/8)-1 # end page
    i2cport.writeto(OLED_ADDR,send)
    x = 0
    y = 0
    ptr = 0
    while y < VRAM_HEIGHT: 
        send = bytearray(9)
        send[0] = 0x40 # control(Continuation + data)
        # 8 Bytes
        for i in range(8):
            ptr2 = ptr
            work = 0
            # pack 8pixel
            for j in range(8):
                work >>= 1
                if vram[ptr2]: work |= 0x80
                ptr2 += VRAM_WIDTH*2

            send[i+1] = work
            x += 1
            ptr += 2

        i2cport.writeto(OLED_ADDR,send)
        if x >= VRAM_WIDTH:
            x = 0
            y += 8
            ptr = y*VRAM_WIDTH*2

#----表示初期化
def disp_init():
    if DISP_TYPE  == M154240240:
        tft_init()
    else:
        oled_init()

#    vram_locate(0, 0)
#    vram_textcolor(color16bit(255,255,255))

#----表示更新
def disp_update():
    if DISP_TYPE  == M154240240:
        tft_update()
    else:
        oled_update()

#----PRINT STRING
def vram_putstr(textstr):
    textbytes = textstr.encode('utf-8')
    for i in range(len(textbytes)):
        vram_putch( textbytes[i] )

#----PRINT DECIMAL
def vram_putdec(num):
    shift=10000
    while shift > 0:
        ch = int(num / shift) % 10
        ch += 0x30
        vram_putch(ch)
        shift = int(shift/10)

#----POINT
def vram_point(x,y):
    if x < 0:return 0
    if y < 0:return 0
    if x >= VRAM_WIDTH:return 0
    if y >= VRAM_HEIGHT:return 0
    ptr = (x + (y*VRAM_WIDTH)) * 2
    return (vram[ptr] << 8)+vram[ptr+1]

#----PSET
def vram_pset(x,y,c):
    if x < 0:return
    if y < 0:return
    if x >= VRAM_WIDTH:return
    if y >= VRAM_HEIGHT:return
    ptr = (x + (y*VRAM_WIDTH)) * 2
    vram[ptr] = c >> 8
    vram[ptr+1] = c & 0xff

#----DRAW LINE
def vram_line(x1 ,y1 ,x2 ,y2 ,c):
    xs = 1  # X方向の1pixel移動量
    ys = 1  # Y方向の1pixel移動量

    xd = x2 - x1  # X2-X1座標の距離
    if xd < 0:
        xd = -xd  # X2-X1座標の絶対値
        xs = -1    # X方向の1pixel移動量
  
    yd = y2 - y1  # Y2-Y1座標の距離
    if yd < 0:
        yd = -yd  # Y2-Y1座標の絶対値
        ys = -1    # Y方向の1pixel移動量

    vram_pset (x1, y1 ,c)
    e = 0
    if yd < xd:
        while x1 != x2:
            x1 += xs;
            e += (2 * yd)
            if e >= xd:
                y1 += ys
                e -= (2 * xd)

            vram_pset(x1, y1 ,c)
    else:
        while y1 != y2:
            y1 += ys
            e += (2 * xd)
            if e >= yd:
                x1 += xs;
                e -= (2 * yd)

            vram_pset(x1, y1 ,c)

#----BOX FILL
def vram_fill(x1 ,y1 ,x2 ,y2 ,color):
  for y in range(y1,y2,1):
    for x in range(x1,x2,1):
      vram_pset(x, y ,color)

#----TEXT COLOR
def vram_textcolor(newcolor):
    global def_tcolor
    def_tcolor = newcolor

#----LOCATE
def vram_locate(newx,newy):
    global def_posx,def_posy
    def_posx = newx
    def_posy = newy

#----SCROLL
def vram_scroll(xd,yd):
  for y in range(VRAM_HEIGHT):
    for x in range(VRAM_WIDTH):
      color = vram_point(x+xd, y+yd)
      vram_pset(x,y,color)

#----PUT CHARA
def vram_putch(ch):
    global def_posx,def_posy

    if ch < 0x20:
        if ch == 10:
            def_posx += VRAM_WIDTH
        return

    if def_posx+FONTPIXEL > VRAM_WIDTH:
        def_posx = 0
        def_posy += FONTPIXEL

    if def_posy+FONTPIXEL > VRAM_HEIGHT:
        def_posy = VRAM_HEIGHT - FONTPIXEL        
        vram_scroll(0,FONTPIXEL)

    ptr = (ch - 0x20) * 8
    for i in range(8):
        bitdata = font[ptr]
        ptr += 1
        for j in range(8):
            if bitdata & 0x80:
                tx = def_posx+(j*FONTSIZE)
                ty = def_posy+(i*FONTSIZE)
                for y1 in range(FONTSIZE):
                    for x1 in range(FONTSIZE):
                        vram_pset(tx+x1,ty+y1,def_tcolor);
            bitdata <<= 1
    def_posx += FONTPIXEL

#----テキスト表示
def print_text(x,y,s):
    l = len(s)
    vram_fill(x,y,x+(l*FONTPIXEL),y+FONTPIXEL,0)
    vram_locate(x, y)
    vram_putstr(s)
    disp_update()

#----ボタン初期化
def button_init():
    global button
    button = Pin(GPIO_BUTTON, Pin.IN, Pin.PULL_UP)

#----ボタン入力待ち
def button_wait():
    x = 0
    y = VRAM_HEIGHT - FONTPIXEL
    vram_fill(x,y,x+10*FONTPIXEL,y+FONTPIXEL,color16bit(255,255,255))
    vram_locate(x,y)
    vram_textcolor(color16bit(0,0,0))
    vram_putstr(" PUSH BTN.")
    disp_update()
    vram_textcolor(color16bit(255,255,255))
    # ボタンが押されるまで待つ
    while True:
        if button.value() == 0:break
        time.sleep(0.02)

    vram_fill(x,y,x+10*FONTPIXEL,y+FONTPIXEL,0)
    disp_update()
    # ボタンが離されるまで待つ
    while True:
        if button.value() == 1:break
        time.sleep(0.02)

#----画面表示テスト
def disp_test():
    print_text(0,0,"DISP TEST")
    button_wait()
    vram_locate(0,FONTPIXEL*1)
    c = 0x20
    while True:    
        vram_putch(c)
        disp_update()
        c += 1
        if c>127:c=32

#----AD初期化
def adc_init():
    global chipled,adc2,adc3,adc0
    chipled = Pin("LED", Pin.OUT)
    chipled.off()
    adc3 = ADC(Pin(GPIO_VSYS))
    adc2 = ADC(Pin(GPIO_VBAT))
    adc0 = ADC(Pin(GPIO_REF))

#----単発のバッテリチェック
def bat_check():
    vsys = adc3.read_u16() * 3.3 / 65535
    vsys *= 3

    vbat = adc2.read_u16() * 3.3 / 65535
    vbat *= 2

    tempstr = "VSYS:"+"{:.2f}V".format(vsys)
    print_text(0,FONTPIXEL*1,tempstr)

    tempstr = "VBAT:"+"{:.2f}V".format(vbat)
    print_text(0,FONTPIXEL*2,tempstr)

    if vbat < 1.8:
        print_text(0, FONTPIXEL*4,"UNDER 1.8V")
        sound_play(notes2)
        button_wait()

    if vbat > 5.5:
        print_text(0, FONTPIXEL*4,"OVER 5.5V")
        sound_play(notes2)
        button_wait()

#----連続的なバッテリチェック
def bat_test():
    print_text(0,0,"BATTERY")
    button_wait()
    while 1:
        bat_check()
        time.sleep(2)

#----モーター初期化
def motor_init():
    global pwm_mot_1a,pwm_mot_1b,pwm_mot_2a,pwm_mot_2b 
    MOTOR_FREQ = const(2000)  # PWM freq

    pwm_mot_1a = PWM(Pin(GPIO_MOTOR1A))
    pwm_mot_1b = PWM(Pin(GPIO_MOTOR1B))
    pwm_mot_2a = PWM(Pin(GPIO_MOTOR2A))
    pwm_mot_2b = PWM(Pin(GPIO_MOTOR2B))

    pwm_mot_1a.freq(MOTOR_FREQ)
    pwm_mot_1b.freq(MOTOR_FREQ)
    pwm_mot_1a.duty_u16(0)     # パルス幅
    pwm_mot_1b.duty_u16(0)     # パルス幅

    pwm_mot_2a.freq(MOTOR_FREQ)
    pwm_mot_2b.freq(MOTOR_FREQ)
    pwm_mot_2a.duty_u16(0)     # パルス幅
    pwm_mot_2b.duty_u16(0)     # パルス幅

#----モーター終了
def motor_end():
    global pwm_mot_1a,pwm_mot_1b,pwm_mot_2a,pwm_mot_2b 

    # PWM停止
    pwm_mot_1a.deinit()
    pwm_mot_1b.deinit()
    pwm_mot_2a.deinit()
    pwm_mot_2b.deinit()

    Pin(GPIO_MOTOR1A, Pin.IN)
    Pin(GPIO_MOTOR1B, Pin.IN)
    Pin(GPIO_MOTOR2A, Pin.IN)
    Pin(GPIO_MOTOR2B, Pin.IN)

#----モータ制御(モータ番号,パワー -100～+100/停止は0)
def motor_control(num, power):
    pwm_a = 0
    pwm_b = 0
    if power > 0:        # 正転
        if power > 100: power = 100
        pwm_a = int(power * 65535/100)
        pwm_b = 0
    elif power < 0:      # 逆転
        if power < -100: power = -100
        power = -power
        pwm_a = 0
        pwm_b = int(power * 65535/100)

    # パルス幅
    if(num==1):
        pwm_mot_1a.duty_u16(pwm_a)
        pwm_mot_1b.duty_u16(pwm_b)
    else:
        pwm_mot_2a.duty_u16(pwm_a)
        pwm_mot_2b.duty_u16(pwm_b)

#----モータペア制御
def motor_pair(power_left,power_right):
    motor_control(1,power_right)
    motor_control(2,power_left)

#----モーター単体のテスト
def motor_test1():
    mot_pow = 50
    while True:
        motor_init()
        print_text(0,0,"MOTOR TEST")
        button_wait()
        bat_check()
        for i in range(2):
            mot_num = 1 + i

            print_text(0,FONTPIXEL*4,"MOTOR:"+"{:d}".format(mot_num))
            print_text(0,FONTPIXEL*5,"POWER:"+"{:d}".format(mot_pow))

            print_text(0,FONTPIXEL*6,"FORWARD")
            motor_control(mot_num,mot_pow)
            time.sleep(2)

            print_text(0,FONTPIXEL*6,"STOP___")
            motor_control(mot_num,0)
            time.sleep(1)

            print_text(0,FONTPIXEL*6,"BACK   ")
            motor_control(mot_num,-mot_pow)
            time.sleep(2)

            print_text(0,FONTPIXEL*6,"STOP___")
            motor_control(mot_num,0)
            time.sleep(1)
        motor_end()
        sound_play(notes1)

#----ロボットの移動テスト
def motor_test():
    mot_pow = 50
    while 1:
        motor_init()
        print_text(0,0,"MOTOR TEST")
        button_wait()
        bat_check()

        print_text(0,FONTPIXEL*5,"POWER:"+"{:d}".format(mot_pow))

        print_text(0,FONTPIXEL*6,"FORWARD")
        motor_pair(mot_pow,mot_pow)
        time.sleep(2)
        motor_pair(0,0)
        time.sleep(0.5)
        print_text(0,FONTPIXEL*6,"BACK   ")
        motor_pair(-mot_pow,-mot_pow)
        time.sleep(2)
        motor_pair(0,0)
        time.sleep(1)

        print_text(0,FONTPIXEL*6,"TURN R ")
        motor_pair(mot_pow,-mot_pow)
        time.sleep(3)
        motor_pair(0,0)
        time.sleep(1)

        print_text(0,FONTPIXEL*6,"TURN L ")
        motor_pair(-mot_pow,mot_pow)
        time.sleep(3)
        motor_pair(0,0)
        time.sleep(1)

        print_text(0,FONTPIXEL*7,"DONE!")
        motor_end()  # 

        sound_play(notes1)

#----LINE TRACE ROBOT
def linetrace():
    THRESHOLD = const(0xA0)  # しきい値
    mot_pow = 50
    motor_init()
    print_text(0,0,"LINE TRACE")
    button_wait()
    bat_check()
    neopixel_set(99,99,99)

    while True:
        cnt = adc0_get()
        if cnt < THRESHOLD:
            motor_pair(mot_pow,0)
        else:
            motor_pair(0,mot_pow)
        time.sleep(0.010)

#----サウンド初期化
def sound_init():
    global pwm_sound
    pwm_sound = PWM(Pin(GPIO_SOUND))
    pwm_sound.duty_u16(0)     # パルス幅 0%

#----サウンド停止
def sound_end():
    global pwm_sound
    pwm_sound.deinit()
    Pin(GPIO_SOUND, Pin.IN)

#----サウンド再生
def sound_play(notes):
    sound_init()
    for note in notes:
        if(note == 99):break
        pwm_sound.freq(freq_table[note]) # 周波数
        pwm_sound.duty_u16(32768)     # パルス幅50%
        time.sleep(0.15)

    pwm_sound.duty_u16(0)     # パルス幅 0%
    sound_end()

#----サウンドテスト
def sound_test():
    print_text(0,0,"SOUND TEST")
    while True:
        button_wait()
        sound_play(notes1)

#----NeoPixel初期化
def neopixel_init():
    global np
    np = neopixel.NeoPixel(Pin(GPIO_NEOPIXEL), 8)
    np[0] = (0,0,0)  #
    np.write()

#----NeoPixel終了
def neopixel_end():
    global np
    np.deinit()
    Pin(GPIO_NEOPIXEL, Pin.IN)

#----NeoPixel点灯
def neopixel_set(r,g,b):
    np[0] = (g,r,b)
    np.write()

#----Neopixelのテスト
def neopixel_test():
    print_text(0,0,"NEOPIXEL")
    button_wait()
    while True:
        print_text(0,FONTPIXEL*2,"RED__")
        neopixel_set(128,0,0)  # R
        time.sleep(2)

        print_text(0,FONTPIXEL*2,"GREEN")
        neopixel_set(0,128,0)  # G
        time.sleep(2)

        print_text(0,FONTPIXEL*2,"BLUE_")
        neopixel_set(0,0,128)  # B
        time.sleep(2)

        print_text(0,FONTPIXEL*2,"OFF__")
        neopixel_set(0, 0, 0)   # 
        time.sleep(2)

#----センサー入力 戻り値0～255
def adc0_get():
    LOOPMAX = const(5)   # 測定回数
    DEVIDE = const(800)  # 割り算の定数
    time.sleep(0.001)
    total = 0
    for i in range(LOOPMAX):
        total += adc0.read_u16()
        time.sleep(0.001)

    total = int(total / DEVIDE)
    if total > 255:total=255
    return total

#----反射光センサーのテスト
def ref_test():
    print_text(0,0,"REF TEST")
    button_wait()
    neopixel_set(99,99,99)
    while True:
        cnt = adc0_get()
        tempstr = " {:02X}".format(cnt)
        print_text(0,FONTPIXEL*2,tempstr)
        time.sleep(0.2)

#----カラーセンサーのテスト
def color_test():
    print_text(0,0,"COLOR TEST")
    button_wait()
    while True:
        neopixel_set(150,0,0 )  # R
        red = adc0_get()

        neopixel_set(0,230,0 )  # G
        green = adc0_get()

        neopixel_set(0,0,230 )  # B
        blue = adc0_get()

        neopixel_set(0,0,0 )  # OFF

        x=0
        y=1*FONTPIXEL
        vram_fill(x,y,x+8*FONTPIXEL,y+FONTPIXEL,color16bit(red,green,blue))

        x=0
        y=2*FONTPIXEL
        vram_fill(x,y,x+8*FONTPIXEL,y+FONTPIXEL,0)
        tempstr = "{:02X}".format(red)
        tempstr += " {:02X}".format(green)
        tempstr += " {:02X}".format(blue)
        vram_locate(x,y)
        vram_putstr(tempstr)

        red_only   = red - int((green + blue)/2)
        green_only = green - int((blue + red)/2)
        blue_only  = blue - int((green + red)/2)
        x=0
        y=7*FONTPIXEL
        if red_only > 0x20:
            vram_fill(0,6*FONTPIXEL,8*FONTPIXEL,7*FONTPIXEL,color16bit(255,0,0))
            print_text(x,y,"RED__")
            sound_play(note_do);
        elif green_only > 0x20:
            vram_fill(0,6*FONTPIXEL,8*FONTPIXEL,7*FONTPIXEL,color16bit(0,255,0))
            print_text(x,y,"GREEN")
            sound_play(note_fa);
        elif blue_only > 0x20:
            vram_fill(0,6*FONTPIXEL,8*FONTPIXEL,7*FONTPIXEL,color16bit(0,0,255))
            print_text(x,y,"BLUE_")
            sound_play(note_si);
        else:
            vram_fill(0,6*FONTPIXEL,8*FONTPIXEL,7*FONTPIXEL,color16bit(0,0,0))
            print_text(x,y,"_____")

#---- setup
time.sleep(0.5)
adc_init()
button_init()
disp_init()
neopixel_init()

#---- main
#disp_test()   # 画面表示テスト
#bat_test()    # 連続的なバッテリチェック
#neopixel_test() # NeoPixelテスト
motor_test()  # ロボットの移動テスト
#motor_test1() # モーター単体のテスト
#sound_test()  # サウンドテスト
#ref_test()    # 反射光センサーのテスト
#color_test()  # カラーセンサーのテスト
#linetrace()   # ライントレース
