# DIY-ROBOT
# for Raspberry Pi Pico(MicroPython)
# and TFT-LCD(M154-240240-RGB ST7789)
# and Motor driver(TC78H653FTG)
# and NeoPixel(RGBLED PL9823-F5)
# (C)2026 takuya matsubara

import neopixel
import time
import random
import math
from machine import SPI
from machine import ADC
from machine import Pin
from machine import PWM
from micropython import const

# 周波数テーブル
freq_table =(
    262,  # C (ド) 4
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
    523   # C (ド) 5
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

VRAM_WIDTH = const(240)
VRAM_HEIGHT = const(240)
VRAM_SIZE = const(VRAM_WIDTH * VRAM_HEIGHT * 2)
vram = bytearray(VRAM_SIZE)

GPIO_TFTCS = const(13)
GPIO_TFTDC = const(12)
GPIO_TFTSCK = const(14)
GPIO_TFTTX = const(15)

GPIO_VSYS = const(29)
GPIO_VBAT = const(28)
GPIO_REF  = const(26)
GPIO_MOTOR1A = const(4)
GPIO_MOTOR1B = const(5)
GPIO_MOTOR2A = const(6)
GPIO_MOTOR2B = const(7)
GPIO_BUTTON = const(22)
GPIO_NEOPIXEL = const(20)
GPIO_SOUND = const(21)

def_posx = 0
def_posy = 0
def_tcolor = 0
FONTSIZE = const(3)

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

# 16bit color
def color16bit(r,g,b):
    r >>= 3
    g >>= 2
    b >>= 3
    return (r << 11)+(g << 5)+b

# clear vram
def vram_cls():
    for i in range(VRAM_WIDTH*VRAM_HEIGHT*2):
        vram[i] = 0

# コマンド送信
def tft_command(cmd):
    tftdc.low()  # command
    tftcs.low()  # enable
    tftspi.write(cmd.to_bytes(1, "big"))
    tftcs.high() # disable

# データ送信
def tft_data(dat):
    tftdc.high() # data 
    tftcs.low()  # enable
    tftspi.write(dat.to_bytes(1, "big"))
    tftcs.high() # disable
    
# TFT初期化
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

    vram_locate(0, 0)
    vram_textcolor(color16bit(255,255,255))

# print string
def vram_putstr(textstr):
    textbytes = textstr.encode('utf-8')
    for i in range(len(textbytes)):
        vram_putch( textbytes[i] )

# PRINT DECIMAL
def vram_putdec(num):
    shift=10000
    while shift > 0:
        ch = int(num / shift) % 10
        ch += 0x30
        vram_putch(ch)
        shift = int(shift/10)

# POINT
def vram_point(x,y):
    if x < 0:return 0
    if y < 0:return 0
    if x >= VRAM_WIDTH:return 0
    if y >= VRAM_HEIGHT:return 0
    ptr = (x + (y*VRAM_WIDTH)) * 2
    return (vram[ptr] << 8)+vram[ptr+1]
    
# PSET
def vram_pset(x,y,c):
    if x < 0:return
    if y < 0:return
    if x >= VRAM_WIDTH:return
    if y >= VRAM_HEIGHT:return
    ptr = (x + (y*VRAM_WIDTH)) * 2
    vram[ptr] = c >> 8
    vram[ptr+1] = c & 0xff

# draw line
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

# box fill
def vram_fill(x1 ,y1 ,x2 ,y2 ,color):
  for y in range(y1,y2,1):
    for x in range(x1,x2,1):
      vram_pset(x, y ,color)

# text color
def vram_textcolor(newcolor):
    global def_tcolor
    def_tcolor = newcolor

# locate
def vram_locate(newx,newy):
    global def_posx,def_posy
    def_posx = newx
    def_posy = newy

# scroll
def vram_scroll(xd,yd):
  for y in range(VRAM_HEIGHT):
    for x in range(VRAM_WIDTH):
      color = vram_point(x+xd, y+yd)
      vram_pset(x,y,color)

# put chara
def vram_putch(ch):
    global def_posx,def_posy

    if ch < 0x20:
        if ch == 10:
            def_posx += VRAM_WIDTH
        return

    if def_posx+(8*FONTSIZE) > VRAM_WIDTH:
        def_posx = 0
        def_posy += 8*FONTSIZE

    if def_posy+(8*FONTSIZE) > VRAM_HEIGHT:
        def_posy = VRAM_HEIGHT - (8*FONTSIZE)        
        vram_scroll(0,8*FONTSIZE)

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
    def_posx += 8*FONTSIZE

# Display(for Pico/W)
def disp_update():
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

# Display(for Pico2/2W)
#def disp_update():
#    tft_command(0x2C)   # RAMWR (2Ch): Memory Write
#    tftdc.high() # data
#    tftcs.low()  # enable
#    tftspi.write(bytes(vram))
#    tftcs.high() # disable 

#----
def print_text(x,y,s):
    l = len(s)
    vram_fill(x,y,x+l*24,y+24,0)
    vram_locate(x, y)
    vram_putstr(s)
    disp_update()

#----ボタン初期化
def button_init():
    global button
    button = Pin(GPIO_BUTTON, Pin.IN, Pin.PULL_UP)

#----
def button_wait():
    x = 0
    y = 240-24
    vram_fill(x,y,x+9*24,y+24,color16bit(128,0,0))
    vram_locate(x,y)
    vram_putstr("PUSH BTN.")
    disp_update()
    while True:
        if button.value() == 0:break
        time.sleep(0.02)

    vram_fill(x,y,x+9*24,y+24,0)
    disp_update()

#----AD初期化
def adc_init():
    global chipled,adc2,adc3,adc0
    chipled = Pin("LED", Pin.OUT)
    chipled.off()
    adc3 = ADC(Pin(GPIO_VSYS))
    adc2 = ADC(Pin(GPIO_VBAT))
    adc0 = ADC(Pin(GPIO_REF))

#----バッテリチェック
def bat_check():
    vsys = adc3.read_u16() * 3.3 / 65535
    vsys *= 3

    vbat = adc2.read_u16() * 3.3 / 65535
    vbat *= 2

    tempstr = "VSYS:"+"{:.2f}V".format(vsys)
    print_text(0,24*2,tempstr)

    tempstr = "VBAT:"+"{:.2f}V".format(vbat)
    print_text(0,24*3,tempstr)

    if vbat < 1.8:
        print_text(0, 24*5,"UNDER 1.8V")
        sound_play(notes2)
        button_wait()

    if vbat > 5.5:
        print_text(0, 24*5,"OVER 5.5V")
        sound_play(notes2)
        button_wait()

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

#----モーター停止
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

#----モータ制御        
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

#----
def motor_test1():
    mot_pow = 40
    mot_num = 1
    while True:
        print_text(0,0,"MOTOR TEST")
        button_wait()
        bat_check()

        print_text(0,24*5,"MOTOR:"+"{:d}".format(mot_num))
        print_text(0,24*7,"POWER:"+"{:d}".format(mot_pow))

        print_text(0,24*6,"FORWARD")
        motor_control(mot_num,mot_pow)
        time.sleep(2)

        print_text(0,24*6,"STOP___")
        motor_control(mot_num,0)
        time.sleep(1)

        print_text(0,24*6,"BACK   ")
        motor_control(mot_num,-mot_pow)
        time.sleep(2)

        print_text(0,24*6,"STOP___")
        motor_control(mot_num,0)
        time.sleep(1)

        if mot_num == 1:
            mot_num = 2
        else:
            mot_num = 1

#----
def motor_test():
    mot_pow = 50
    while 1:
        motor_init()
        print_text(0,0,"MOTOR TEST")
        button_wait()
        bat_check()

        print_text(0,24*7,"POWER:"+"{:d}".format(mot_pow))

        print_text(0,24*6,"FORWARD")
        motor_pair(mot_pow,mot_pow)
        time.sleep(2)
        motor_pair(0,0)
        time.sleep(0.5)
        print_text(0,24*6,"BACK   ")
        motor_pair(-mot_pow,-mot_pow)
        time.sleep(2)
        motor_pair(0,0)
        time.sleep(1)

        print_text(0,24*6,"TURN R ")
        motor_pair(mot_pow,-mot_pow)
        time.sleep(3)
        motor_pair(0,0)
        time.sleep(1)

        print_text(0,24*6,"TURN L ")
        motor_pair(-mot_pow,mot_pow)
        time.sleep(3)
        motor_pair(0,0)
        time.sleep(1)

        print_text(0,24*9,"DONE!")
        motor_end()  # 

        sound_play(notes1)

#----LINE TRACE ROBOT
def linetrace():
    THRESHOLD = const(0xA0)
    mot_pow = 40
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

#----NeoPixel初期化
def neopixel_init():
    global np
    np = neopixel.NeoPixel(Pin(GPIO_NEOPIXEL), 8)
    np[0] = (0,0,0)  #
    np.write()

#----
def neopixel_end():
    global np
    np.deinit()
    Pin(GPIO_NEOPIXEL, Pin.IN)

#----NeoPixel点灯
def neopixel_set(r,g,b):
    np[0] = (g,r,b)
    np.write()

#----センサー
def adc0_get():
    LOOPMAX = const(5)   # 測定回数
    DEVIDE = const(800)  # 
    time.sleep(0.001)
    total = 0
    for i in range(LOOPMAX):
        total += adc0.read_u16()
        time.sleep(0.001)

    total = int(total / DEVIDE)
    if total > 255:total=255
    return total

#----反射光センサー
def ref_test():
    print_text(0,0,"REF TEST")
    button_wait()
    neopixel_set(99,99,99)
    while True:
        cnt = adc0_get()
        tempstr = " {:02X}".format(cnt)
        print_text(0,24*2,tempstr)
        time.sleep(0.2)

#----カラーセンサー
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
        y=1*24
        vram_fill(x,y,x+8*24,y+4*24,color16bit(red,green,blue))
        tempstr = "{:02X}".format(red)
        tempstr += " {:02X}".format(green)
        tempstr += " {:02X}".format(blue)
        vram_locate(x,y+24*3)
        vram_putstr(tempstr)

        red_only   = red - int((green + blue)/2)
        green_only = green - int((blue + red)/2)
        blue_only  = blue - int((green + red)/2)
        x=0
        y=8*24
        if red_only > 0x20:
            vram_fill(0*24,7*24,8*24,8*24,color16bit(255,0,0))
            print_text(x,y,"RED__")
            sound_play(note_do);
        elif green_only > 0x20:
            vram_fill(0*24,7*24,8*24,8*24,color16bit(0,255,0))
            print_text(x,y,"GREEN")
            sound_play(note_fa);
        elif blue_only > 0x20:
            vram_fill(0*24,7*24,8*24,8*24,color16bit(0,0,255))
            print_text(x,y,"BLUE_")
            sound_play(note_si);
        else:
            vram_fill(0*24,7*24,8*24,8*24,color16bit(0,0,0))
            print_text(x,y,"_____")

#---- setup
time.sleep(2) 
adc_init()
button_init()
tft_init()
neopixel_init()

#---- main
motor_test() # モーターテスト
#ref_test()  # 反射光センサー
#color_test() # カラーセンサー
#linetrace()  # ライントレース

