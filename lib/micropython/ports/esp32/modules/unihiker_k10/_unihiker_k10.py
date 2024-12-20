import gc
from k10_base import pin, button,Screen,Camera, aht20, Light, accelerometer,Pin,WiFi,MqttClient,Mic,Speaker,TF_card,hid,keycode,pins_k10,I2C,i2c
from neopixel import NeoPixel
from machine import Servo
from hcsr04 import HCSR04
from ds18x20 import DS18X20

import onewire
import task_handler
import time
import machine
import dht as _dht
import os
from machine import Timer
from mpython import  dfrobot_global_extio_timer_handler
gc.collect()
#sd_cs  = machine.Pin(40, machine.Pin.OUT)

class rgb_board():
    def __init__(self,pin=None):
        self.my_rgb = NeoPixel(Pin(46, Pin.OUT), n=10, bpp=3, timing=1)
        self.bright = 9

    def write(self,num=-1,R=0,G=0,B=0,color=None):
        #如果传入了color，则听color的
        if num > 2:
            num = 2
        if color == None:
            pass
        else:
            R = (color >> 16) & 0xFF
            G = (color >> 8) & 0xFF
            B = (color ) & 0xFF
        self.r = int(R/(10-self.bright))
        self.g = int(G/(10-self.bright))
        self.b = int(B/(10-self.bright))
        if num == -1:
            for i in range(3):
                self.my_rgb[i] = (self.r, self.g, self.b)
                self.my_rgb.write()
                time.sleep(0.001)
        else:
            #K10的灯顺序需要调一下
            self.my_rgb[2-num] = (self.r, self.g, self.b)
            self.my_rgb.write()
            time.sleep(0.001)
    def brightness(self,bright=9):
        if bright <= 9 and bright >= 0:
            self.bright = bright
            
        
    def clear(self):
        self.my_rgb.fill((0,0,0))
        self.my_rgb.write()
        time.sleep(0.001)



class neopixel():
    def __init__(self, io, n, bpp=3, timing=1):
        '''
        if isinstance(io, pin):
            self.pin = pins_k10[io.pin_num]
        else:
            self.pin = io
        '''
        if io > 20:
            self.pin = io
        else:
            self.pin = pins_k10[io]
        #pins_k10[pin]
        self.my_rgb = NeoPixel(Pin(self.pin, Pin.OUT), n, bpp, timing)
        self.bright = 9

    def brightness(self,bright=9):
        if bright <= 9 and bright >= 0:
            self.bright = bright

    def write(self,begin,end,R=0,G=0,B=0):
        self.r = int(R/(10-self.bright))
        self.g = int(G/(10-self.bright))
        self.b = int(B/(10-self.bright))
        for i in range(begin,end+1):
            self.my_rgb[i] = (self.r, self.g, self.b)
            self.my_rgb.write()
            time.sleep(0.001)        
        
    def clear(self):
        self.my_rgb.fill((0,0,0))
        self.my_rgb.write()
        time.sleep(0.001)
'''
继承舵机
'''
class servo(Servo):
    def __init__(self,pin):
        super().__init__(pin)
    
    def angle(self, value):
        value = int(value)
        if(value<0):
            value = 0
        if(value>180):
            value = 180
        self.write_angle(value)

'''
DS18b20
'''
class ds18b20():
    def __init__(self, pin):
        self._pin = pins_k10[pin]
        self.dat = Pin(self._pin)
   
    def read(self):
        # create the onewire object
        self.ds = DS18X20(onewire.OneWire(self.dat))
        # scan for devices on the bus
        roms = self.ds.scan()
        # print('found devices:', roms)
        self.ds.convert_temp()
        time.sleep(0.75)
        temp = self.ds.read_temp(roms[0])
        # print(temp,end='℃\n ')
        return temp

class sen0388(object):
    def __init__(self, d_pin, echo_timeout_us=500*2*30):
        """
        trigger_pin: Output pin to send pulses
        echo_pin: Readonly pin to measure the distance. The pin should be protected with 1k resistor
        echo_timeout_us: Timeout in microseconds to listen to echo pin. 
        By default is based in sensor limit range (4m)
        """
        self.echo_timeout_us = echo_timeout_us
        self._pin = d_pin
        # Init trigger pin (out)
        self.trigger = Pin(self._pin, mode=Pin.OUT, pull=None)
        self.trigger.value(0)

        # Init echo pin (in)
        #self.echo = Pin(echo_pin, mode=Pin.IN, pull=None)
        self.__limit = 500    #cm

    def _send_pulse_and_wait(self):
        self.trigger = Pin(self._pin, mode=Pin.OUT, pull=None)
        self.trigger.value(0) # Stabilize the sensor
        time.sleep_us(5)
        self.trigger.value(1)
        # Send a 10us pulse.
        time.sleep_us(10)
        self.trigger.value(0)
        # try:
        self.echo = Pin(self._pin, mode=Pin.IN, pull=None)
        pulse_time = machine.time_pulse_us(self.echo, 1, self.echo_timeout_us)
        return pulse_time

    def distance_mm(self):
        """
        Get the distance in milimeters without floating point operations.
        """
        pulse_time = self._send_pulse_and_wait()

        # To calculate the distance we get the pulse_time and divide it by 2 
        # (the pulse walk the distance twice) and by 29.1 becasue
        # the sound speed on air (343.2 m/s), that It's equivalent to
        # 0.34320 mm/us that is 1mm each 2.91us
        # pulse_time // 2 // 2.91 -> pulse_time // 5.82 -> pulse_time * 100 // 582 
        if pulse_time != (-1 or -2):
            mm = pulse_time * 100 // 582
            return mm
        else:
            return self.__limit*10

    def distance_cm(self):
        """
        Get the distance in centimeters with floating point operations.
        It returns a float
        """
        pulse_time = self._send_pulse_and_wait()

        # To calculate the distance we get the pulse_time and divide it by 2 
        # (the pulse walk the distance twice) and by 29.1 becasue
        # the sound speed on air (343.2 m/s), that It's equivalent to
        # 0.034320 cm/us that is 1cm each 29.1us
        if pulse_time != (-1 or -2):
            cms = (pulse_time / 2) / 29.1
            return cms
        else:
            return self.__limit

class ultrasonic(HCSR04):
    def __init__(self, trig=1, echo=0):
        self._trig = pins_k10[trig]
        self._echo = pins_k10[echo]
        if(trig != echo):
            self.dev = HCSR04(trigger_pin=self._trig, echo_pin=self._echo)
        else:
            #使用SEN0388
            self.dev = sen0388(d_pin=self._trig)


    def distance(self):
        return self.dev.distance_cm()



class _dht11():
    def __init__(self, pin):
        self._pin = pins_k10[pin]
        self.dht11 = _dht.DHT11(Pin(self._pin))
        self.dht11.measure()
    def read(self):
        self.dht11.measure()
        return self.dht11.temperature(),self.dht11.humidity()
    
dht11_old_pin = None
dht11_thing = None
def dht(pin):
    global dht11_old_pin,dht11_thing
    if dht11_old_pin != pin:
        dht11_thing = _dht11(pin)
        dht11_old_pin = pin
    return dht11_thing

'''
KIT0176 HX
'''

class hx711(object):
    REG_DATA_GET_RAM_DATA      = 0x66  #Get sensor raw data
    REG_DATA_GET_CALIBRATION   = 0x67  #Gets the automatic calibration value
    REG_DATA_SET_CALIBRATION   = 0x68  #Obtain peeling position
    REG_DATA_GET_PEEL_FLAG     = 0x69  #Module initialization
    REG_SET_CAL_THRESHOLD      = 0x71  #Set the calibration trigger threshold
    REG_SET_TRIGGER_WEIGHT     = 0x72  #Set calibration weight
    _calibration = 2210.0
    _offset = 0
    def __init__(self, address = 0x64, i2c=i2c):
        self._i2c = i2c
        self._address = address
        self._i2c.writeto(self._address, bytearray([0x70,0x65]))
        self._offset = self.average(10)

    def peel(self):
        self._offset = self.average(10)
        self._i2c.writeto(self._address, bytearray([0x73,0x00]))
        return self._offset

    def peel_flag(self):
        self._i2c.writeto(self._address, bytearray([self.REG_DATA_GET_PEEL_FLAG]))
        data = self._i2c.readfrom(self._address,1)
        if(data[0] == 0x01 or data[0] == 129):
            return 1
        elif data[0] == 0x02:
            return 2
        else:
            return 0
    def set_calibration(self ,value):
      '''!
        @fn set_calibration
        @brief Set calibration value
        @param value the calibration value
      '''
      self._offset = self.average(15)
      self._calibration = value

    def get_calibration(self):
        '''!
            @fn get_calibration
            @brief get calibration value 
            @return return the read calibration value
        '''
        self._i2c.writeto(self._address, bytearray([self.REG_DATA_GET_CALIBRATION]))
        data = self._i2c.read_reg(self._address,4)
        aa= bytearray(data) 
        return struct.unpack('>f', aa)
    
    def average(self,times):
        sum = 0
        for i in range(times):
            #
            data = self.get_value()
            if data == 0 :
                times = times -1
            else:
                sum = sum + data
        if(times == 0):
            times =1
        return  sum/times
    def get_value(self):
        self._i2c.writeto(self._address, bytearray([0x66]))
        time.sleep(0.022)
        data = self._i2c.readfrom(0x64, 4)
        value = 0
        if(data[0] == 0x12):
            value = (data[1]<<16) | (data[2]<<8) | data[3]
        else:
            return 0
        return value^0x800000
    
    def read_weight(self,times):
        '''!
        @fn read_weight
        @brief Get the weight of the object
        @param times Take the average several times
        @return return the read weight value, unit: g
        '''
        value = self.average(times)
        time.sleep(0.05)
        ppFlag = self.peel_flag()
        if ppFlag == 1:
            self._offset = self.average(times)
        elif ppFlag == 2:
            b = self.get_calibration()
            self._calibration = b[0]
        return ((value - self._offset)/self._calibration) 
    
    def common_measure(self):
        return self.read_weight(10)

'''
重力传感器
'''
class force(object):
    def __init__(self, sda=20, scl=19):
        self._zero_scale = 0
        _sda = pins_k10[sda]
        _scl = pins_k10[scl]
        if(sda==20 or scl==19):
            self.i2c = i2c
            self.dev = hx711(address=0x64, i2c=self.i2c)
        else:
            self.i2c = I2C(scl=Pin(_scl), sda=Pin(_sda), freq=400000)
            time.sleep_ms(100)
            self.dev = hx711(address=0x64, i2c=self.i2c)

    def zero(self):
        self._zero_scale = self.dev.peel()

    def read(self,mass=True):
        tmp = self.dev.common_measure()
        if(tmp!=None):
            if(mass):
                # g  
                return round(tmp,2)
            else:
                #1000克(g)受到9.80665牛顿(N)
                m = round((tmp*9.80665)/1000,2)
                return round(m,2)
        else:
            return None


class _myTimer(object):
    def __init__(self, temp, acc=None, exio=None):
        self.tim = Timer(18)
        self.tim.init(period=50, mode=Timer.PERIODIC, callback=self.timer_callback)
        self.num = 0
        self.temp = temp
        self.acc = acc
        self.exio = exio
        pass
    def timer_callback(self,_):
        #50毫秒进行一次IO扩展的操作
        self.num += 1
        dfrobot_global_extio_timer_handler(_)
        #100ms执行另外一个任务
        if self.num % 2 == 0:
            pass
        #1000ms读取一次温湿度
        if self.num % 20 == 0:
            if isinstance(self.temp, aht20):
                self.temp.measure()
        if self.num >= 20:
            self.num = 0
temp_humi = aht20()
k10_measure_timer = _myTimer(temp = temp_humi)

screen = Screen()
camera = Camera()



light = Light()
acce = accelerometer()

rgb = rgb_board()
wifi = WiFi()
mqttclient = MqttClient()
mic = Mic()
speaker = Speaker()
tf_card = TF_card()

#th = task_handler.TaskHandler(timer_id = 14)
th = task_handler.TaskHandler()

#用来定时读取需要频繁读取的东西(IO扩展、温湿度、加速度计等)
