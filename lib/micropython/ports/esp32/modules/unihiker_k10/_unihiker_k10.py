from k10_base import pin,button,temp_humi
from k10_base import Light,Mic,Speaker,TF_card,Screen,Camera,WiFi,MqttClient,Timer,time,Pin
from k10_base import k10_i2c, pins_remap_k10
from neopixel import NeoPixel
from machine import Servo,I2C
import machine,onewire, struct,task_handler
from ds18x20 import DS18X20
from hcsr04 import HCSR04
from dht import DHT11


'''
六轴的驱动类
'''
class Accelerometer(object):
    SHANK = 0          # Shake gesture
    SCREEN_UP = 1       # Screen facing up
    SCREEN_DOWN = 2     # Screen facing down
    TILT_LEFT = 3       # Tilt left
    TILT_RIGHT = 4      # Tilt right
    TILT_FORWARD = 5    # Tilt forward
    TILT_BACK = 6       # Tilt backward
    GESTURE_NONE = 7     # No gesture detected
    def __init__(self) :
        self.shake_status = False
        self._i2c = k10_i2c
        self._addr = 0x19
        self._gesture = self.GESTURE_NONE
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0
        self._begin()
        self._measure()

    def _begin(self):
        buf = self._read_bytes(0x24, 1)
        data = buf[0]
        data |= 0x08
        self._writeReg(0x24, data)

        data = 0x00
        data |= 0x40
        data |= 0x03
        data |= 0x0C
        data |= 0x38
        self._writeReg(0x30, data)

        buf = self._read_bytes(0x21, 1)
        data = buf[0]
        data |= 0x81
        self._writeReg(0x21, data)
        self._writeReg(0x32, 0x60)
        self._writeReg(0x33, 0x02)

        buf = self._read_bytes(0x22, 1)
        data = buf[0]
        data |= 0x40
        self._writeReg(0x21, data)

        buf = self._read_bytes(0x24, 1)
        data = buf[0]
        data |= 0x02
        self._writeReg(0x24, data)

        buf = self._read_bytes(0x25, 1)
        data = buf[0]
        data |= 0x02
        self._writeReg(0x25, data)

        data = 0x00
        data |= 0xc0
        data |= 0x3f
        self._writeReg(0x34, data)

        buf = self._read_bytes(0x21, 1)
        data = buf[0]
        data |= 0xfd
        self._writeReg(0x21, data)

        self._writeReg(0x36, 0x18)
        self._writeReg(0x37, 0x02)

        buf = self._read_bytes(0x25, 1)
        data = buf[0]
        data |= 0x20
        self._writeReg(0x21, data)

        self._read_bytes(0x23, 1)
        self._writeReg(0x23, 0x88)
        self._writeReg(0x20, 0x27)


    def _read_bytes(self, reg, size):
        rslt = self._i2c.readfrom_mem(self._addr, reg, size)
        return rslt
    def _writeReg(self, reg, value):
        self._i2c.writeto_mem(self._addr, reg, value.to_bytes(1, 'little'))
    def _measure(self):
        tempbuf = self._read_bytes(0x27,1)
        if (tempbuf[0] & 0x0F) == 0x0F:
            accbuf = self._read_bytes(0xA8, 6)
            self.X = (accbuf[1]<<8 | accbuf[0]) >> 4
            self.Y = (accbuf[3]<<8 | accbuf[2]) >> 4
            self.Z = (accbuf[5]<<8 | accbuf[4]) >> 4
            if (self.X & 0x800) == 0x800:
                self.X -= 4096
            if (self.Y & 0x800) == 0x800:
                self.Y -= 4096
            if (self.Z & 0x800) == 0x800:
                self.Z -= 4096
            self.X = self.X / 1024.0
            self.Y = self.Y / 1024.0
            self.Z = self.Z / 1024.0
        tempbuf = self._read_bytes(0x35,1)

        if(tempbuf[0] & 0x60) == 0x60:
            self._gesture = self.SCREEN_DOWN
        elif (tempbuf[0] & 0x50) == 0x50:
            self._gesture = self.SCREEN_UP
        elif (tempbuf[0] & 0x41) == 0x41:
            self._gesture = self.TILT_LEFT
        elif (tempbuf[0] & 0x42) == 0x42:
            self._gesture = self.TILT_RIGHT
        elif (tempbuf[0] & 0x44) == 0x44:
            self._gesture = self.TILT_FORWARD
        elif (tempbuf[0] & 0x48) == 0x48:
            self._gesture = self.TILT_BACK
        elif (tempbuf[0] != 0):
            self._gesture = self.SHANK
       

    def x(self):
        #self.X = _accelerometer.get_x()
        return self.X

    def y(self):
        #self.Y = _accelerometer.get_y()
        return self.Y

    def z(self):
        #self.Z = _accelerometer.get_z()
        return self.Z

    def shake(self):
        return self.shake_status

'''
为了兼容上层API使用做的类
'''
class accelerometer(object):
    def __init__(self):
        self.tim_count = 0
        self._is_shaked = False
        self._last_x = 0
        self._last_y = 0
        self._last_z = 0
        self._count_shaked = 0
        self._i2c = k10_i2c
        devices = self._i2c.scan()
        #0x19是K10的加速度计地址
        if 0x19 in devices:
            #K10
            self.tim = Timer(17)
            self.tim.init(period=100, mode=Timer.PERIODIC, callback=self.timer_callback)
            self.accel_sensor = Accelerometer()
        else:
            #K10box
            self.tim = None
            self.accel_sensor = None

    def timer_callback(self,_):
        self.tim_count += 1 
        try:
            self.accelerometer_callback()
        except Exception as e:
            print(str(e))
        if(self.tim_count==200):
            self.tim_count = 0

    def accelerometer_callback(self):
        '''加速度计'''
        if self._is_shaked:
            self._count_shaked += 1
            if self._count_shaked == 5: 
                self._count_shaked = 0
                self.accel_sensor.shake_status = False
        self.accel_sensor._measure()
        x=self.accel_sensor.x(); y=self.accel_sensor.y();z=self.accel_sensor.z()
        if self._last_x == 0 and self._last_y == 0 and self._last_z == 0:
            self._last_x = x; 
            self._last_y = y; 
            self._last_z = z; 
            self.accel_sensor.shake_status = False
            return
        diff_x = x - self._last_x; diff_y = y - self._last_y; diff_z = z - self._last_z
        self._last_x = x; self.last_y = y; self._last_z = z
        if self._count_shaked > 0: 
            return
        self._is_shaked = (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 1)
        if self._is_shaked: 
            self.accel_sensor.shake_status = True
        
    def X(self):
        return self.accel_sensor.x()
    
    def Y(self):
        return self.accel_sensor.y()
    
    def Z(self):
        return self.accel_sensor.z()
    
    def read_x(self):
        return self.accel_sensor.x()
    
    def read_y(self):
        return self.accel_sensor.y()
    
    def read_z(self):
        return self.accel_sensor.z()
    
    def shake(self):
        return self.accel_sensor.shake()
    def gesture(self):
        return self.accel_sensor._gesture
    def status(self,status=""):
        if status is "forward":
            if self.gesture() == self.accel_sensor.TILT_FORWARD:
                return True
            else:
                return False
        elif status is "back":
            if self.gesture() == self.accel_sensor.TILT_BACK:
                return True
            else:
                return False
        elif status is "left":
            if self.gesture() == self.accel_sensor.TILT_LEFT:
                return True
            else:
                return False
        elif status is "right":
            if self.gesture() == self.accel_sensor.TILT_RIGHT:
                return True
            else:
                return False
        elif status is "up":
            if self.gesture() == self.accel_sensor.SCREEN_UP:
                return True
            else:
                return False
        elif status is "down":
            if self.gesture() == self.accel_sensor.SCREEN_DOWN:
                return True
            else:
                return False
        else:
            return False

'''
k10的板载2812灯的控制类(灯的定义顺序是反的)
'''
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

'''
外置舵机驱动类
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
外置18B20温度传感器驱动类
'''
class ds18b20(object):
    def __init__(self, pin):
        self._pin = pins_remap_k10[pin]
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

'''
外置重量传感器hx711驱动类
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
    def __init__(self, address = 0x64, i2c=k10_i2c):
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
        _sda = pins_remap_k10[sda]
        _scl = pins_remap_k10[scl]
        if(sda==20 or scl==19):
            self.i2c = k10_i2c
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

'''
外置超声波驱动类
'''
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
        self._trig = pins_remap_k10[trig]
        self._echo = pins_remap_k10[echo]
        if(trig != echo):
            self.dev = HCSR04(trigger_pin=self._trig, echo_pin=self._echo)
        else:
            #使用SEN0388
            self.dev = sen0388(d_pin=self._trig)


    def distance(self):
        return self.dev.distance_cm()

'''
外置dht11温湿度传感器驱动类
'''
class dht(object):
    def __init__(self,pin):
        self._pin = pins_remap_k10[pin]
        self._dht = DHT11(Pin(self._pin))
        self._dht.measure()
    def read(self):
        self._dht.measure()
        return self._dht.temperature(),self._dht.humidity()

light = Light()
mic = Mic()
speaker = Speaker()
tf_card = TF_card()
screen = Screen()
camera = Camera()
wifi = WiFi()
mqttclient = MqttClient()
acce = accelerometer()
rgb = rgb_board()

th = task_handler.TaskHandler()