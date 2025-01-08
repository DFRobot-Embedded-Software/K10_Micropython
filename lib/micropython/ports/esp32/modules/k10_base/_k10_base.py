import gc
from machine import Pin,PWM,ADC,Timer,I2C,I2S,SPI,SDCard
from micropython import schedule
import lvgl as lv
import time,vfs,camera,lcd_bus,ili9341,network,ubinascii
from umqtt.robust import MQTTClient as MQTT

gc.collect()

'''
K10的引脚操作类
K10分为原生esp32S3的引脚和IO扩展芯片的引脚
引脚编号大于等于50的是IO扩展芯片的引脚
'''
# K10的引脚映射   P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15, P16, P17, P18, P19, P20    
pins_remap_k10 = (1, 2,  57, 66, 65, 64, 63, -1, 60, 61, 62,  52,  53,  54,  55,  56,  -1,  -1,  -1,  48,  47)
k10_i2c = I2C(0, scl=48, sda=47, freq=100000)
'''
定时读取IO扩展的所有IO口，这样可以避免频繁操作不同IO口时，频繁的去读取
也是为了同步解决所有IO都设置为了输入时，因为是浮空状态会频繁的产生中断去读，就会导致I2C总线上拥挤
'''
__extio_dfrobot_value = bytearray([0x00, 0x00])  # 确保变量在全局范围内初始化
__dfrobot_buttons = {}

def dfrobot_global_irq_handler_timer():
    # 遍历所有按钮，根据引脚状态处理
    try:
        #time.sleep_ms(10)  # 防抖处理
        if not __dfrobot_buttons:
            pass
        else:
            for btn in __dfrobot_buttons.values():
                btn.check_state()
    except KeyboardInterrupt:
        Button._pin.irq(handler=None)

def dfrobot_global_extio_timer_handler(_):
    global __extio_dfrobot_value
    k10_i2c.writeto(0x20, bytearray([0x00]))
    __extio_dfrobot_value = k10_i2c.readfrom(0x20, 2)
    dfrobot_global_irq_handler_timer()

class PinMode(object):
    IN = 1
    OUT = 2
    PWM = 3
    ANALOG = 4
    OUT_DRAIN = 5

class extIO(object):
    """
    引脚拓展模块控制类

    :param i2c: I2C实例对象,默认i2c=i2c.
    输入端口寄存器
    地址：0x00和0x01
    功能：读取16个IO引脚的输入电平
    0x00对应P00-P07，0x01对应P10-P17

    输出端口寄存器
    地址：0x02和0x03
    功能：控制16个IO引脚的输出电平
    0x02对应P00-P07，0x03对应P10-P17

    极性反转寄存器
    地址：0x04，0x05
    功能：反转输入信号的逻辑电平
    0x04对应P00-P7，0x05对应P10-P17
    暂不使用

    配置寄存器
    地址：0x06和0x07
    功能：配置引脚为输入或输出
    0x06对应P00-P07，0x07对应P10-P17
    设置为1时为输入，为0时为输出
    """
    def __init__(self, pin, mode=PinMode.OUT):
        self._i2c = k10_i2c
        self._addr = 0x20
        self._mode = mode
        self._pin = pin
        _real_pin = 0
        if pin >= 60:
            self._i2c.writeto(self._addr,bytearray([0x07]))
            _real_pin = pin -60
        else:
            self._i2c.writeto(self._addr,bytearray([0x06]))
            _real_pin = pin - 50
        _mode_old = self._i2c.readfrom(self._addr, 1)[0]
        _mode_new = 0
        if self._mode == PinMode.IN:
            _mode_new = _mode_old | (1 << _real_pin)
        elif self._mode == PinMode.OUT:
            _mode_new = _mode_old & ~(1 << _real_pin)
        if pin >= 60:
            self._i2c.writeto(self._addr,bytearray([0x07, _mode_new]))
        else:
            self._i2c.writeto(self._addr,bytearray([0x06, _mode_new]))
    def value(self,value=None):
        if value == None:
            return self.readIO()
        else:
            return self.writeIO(value)
    def readIO(self):
        #因为扩展IO有A、B按键、需要定时读取IO状态，所以这里直接获取全局buf即可
        _real_pin = 0
        dat = 0
        if self._pin >= 60:
            _real_pin = self._pin -60
            dat = __extio_dfrobot_value[1]
        else:
            _real_pin = self._pin - 50
            dat = __extio_dfrobot_value[0]
        return (dat >> _real_pin) & 0x01
    
    def writeIO(self,value):
        _real_pin = 0
        if self._pin >= 60:
            _real_pin = self._pin - 60
            _reg = bytearray([0x03])
        else:
            _real_pin = self._pin - 50
            _reg = bytearray([0x02])
        self._i2c.writeto(self._addr, _reg)
        _status_old = self._i2c.readfrom(self._addr, 1)[0]
        _status_new = 0
        if value == 1:
            _status_new = _status_old | (1 << _real_pin)
        elif value == 0:
            _status_new = _status_old & ~(1 << _real_pin)
        self._i2c.writeto(self._addr,bytearray([_reg[0], _status_new]))

class k10_pin(object):
    def __init__(self, pin_num, mode=PinMode.IN, pull=None):
        self._num = pin_num
        if mode not in [PinMode.IN, PinMode.OUT, PinMode.PWM, PinMode.ANALOG, PinMode.OUT_DRAIN]:
            raise ValueError('mode must be IN, OUT, PWM, ANALOG or OUT_DRAIN')
        try:
            self.id = pins_remap_k10[pin_num]
            if self.id == -1:
                raise ValueError('P%d is not defined in K10' % pin_num)
        except IndexError:
            raise ValueError("Out of Pin range")
        if self.id >= 50:
            # IO扩展芯片
            if mode == PinMode.PWM:
                raise ValueError('PWM not supported on P%d' % pin_num)
            elif mode == PinMode.ANALOG:
                raise ValueError('ANALOG not supported on P%d' % pin_num)
            elif mode == PinMode.OUT_DRAIN:
                raise ValueError('OUT_DRAIN not supported on P%d' % pin_num)
            self._pin = extIO(self.id, mode=mode)
        else:
            # 原生esp32S3引脚
            if mode == PinMode.IN:
                self._pin = Pin(self.id, mode=Pin.IN, pull=Pin.PULL_UP)
            elif mode == PinMode.OUT:
                self._pin = Pin(self.id, mode=Pin.OUT, pull=Pin.PULL_UP)
            elif mode == PinMode.PWM:
                self.pwm = PWM(Pin(self.id), duty=0)
            elif mode == PinMode.ANALOG:
                self.adc = ADC(Pin(self.id))
                self.adc.atten(ADC.ATTN_11DB)
        self._mode = mode

    def irq(self, handler=None, trigger=Pin.IRQ_RISING):
        if self.id >= 50:
            raise ValueError('IRQ not supported on P%d' % self._num)
        if not self._mode == PinMode.IN:
            raise ValueError('P%d is not in IN mode' % self._num)
        return self._pin.irq(handler, trigger)
    
    def read_digital(self):
        if not self._mode == PinMode.IN:
            raise ValueError('P%d is not in IN mode' % self._num)
        return self._pin.value()
    
    def write_digital(self, value):
        if not self._mode == PinMode.OUT:
            raise ValueError('P%d is not in OUT mode' % self._num)
        self._pin.value(value)
    
    def read_analog(self):
        if self.id >= 50:
            raise ValueError('ANALOG not supported on P%d' % self._num)
        if not self._mode == PinMode.ANALOG:
            raise ValueError('P%d is not in ANALOG mode' % self._num)
        return self.adc.read()
    
    def write_analog(self, duty=0, freq=1000):
        if self.id >= 50:
            raise ValueError('PWM not supported on P%d' % self._num)
        if not self._mode == PinMode.PWM:
            raise ValueError('P%d is not in PWM mode' % self._num)
        self.pwm.duty(duty)
        self.pwm.freq(freq)

class pin(object):
    def __init__(self, pin_num, mode = PinMode.IN, pull=None):
        self.pin_num = pin_num
        self.mode = mode
        self.pull = pull
        self._pin = k10_pin(self.pin_num, self.mode, self.pull)

    def read_digital(self):
        self._pin = k10_pin(self.pin_num, PinMode.IN)
        return self._pin.read_digital()
    
    def write_digital(self, value):
        self._pin = k10_pin(self.pin_num, PinMode.OUT, Pin.PULL_UP)
        self._pin.write_digital(value)

    def read_analog(self):
        self._pin = k10_pin(self.pin_num, PinMode.ANALOG)
        return self._pin.read_analog()
    
    def write_analog(self, value=0, freq=5000):
        self._pin = k10_pin(self.pin_num, PinMode.PWM)
        self._pin.write_analog(duty=value, freq=freq)

    def irq(self, handler=None, trigger=Pin.IRQ_FALLING):
        self._pin = k10_pin(self.pin_num, PinMode.IN)
        self._pin.irq(handler, trigger)

class Button(object):
    def __init__(self, pin_num, reverse=False):
        '''
        :param pin_num: 按键引脚编号
        :param reverse: 是否反转按键逻辑电平
        '''
        self._reverse = reverse
        (self._press_level, self._release_level) = (0, 1) if not self._reverse else (1, 0)
        self._id = pins_remap_k10[pin_num]
        self._pin = None
        self._last_value = None
        #检查是否已经存在绑定到同一引脚的按钮
        if self._id in __dfrobot_buttons:
            #覆盖已有的按钮回调
            __dfrobot_buttons[self._id] = self
        else:
            #添加到按钮字典中
            __dfrobot_buttons[self._id] = self

        if self._id >= 50:
            self._pin = extIO(self._id, mode=PinMode.IN)
        else:
            self._pin = Pin(self._id, mode=Pin.IN, pull=Pin.PULL_UP)
        
        self.event_pressed = None
        self.event_released = None
        self._pressed_count = 0
        self._was_pressed = False

        #初始化按钮状态
        self._last_value = self._pin.value()
        #配置中断引脚
        if self._id >= 50:
            #IO扩展芯片
            pass
        else:
            #原生IO口
            self._pin.irq(trigger=Pin.IRQ_FALLING, handler = self._irq_handler)
    def _irq_handler(self, pin):
        _irq_falling = True if pin.value() == self._press_level else False
        if self._pin.value() == (self._press_level if _irq_falling else self._release_level):
            if _irq_falling:
                if self.event_pressed is not None:
                    schedule(self.event_pressed, self._pin)
                self._was_pressed = True
                if(self._pressed_count < 100):
                    self._pressed_count += 1
            else:
                if self.event_released is not None:
                    schedule(self.event_released, self._pin)
    def check_state(self):
        _current_value = self._pin.value()
        #按下事件
        if _current_value == self._press_level and self._last_value != self._press_level:
            if self.event_pressed is not None:
                try:
                    schedule(self.event_pressed, None)
                except:
                    pass
            self._was_pressed = True
            if(self._pressed_count < 100):
                self._pressed_count += 1
        #释放事件
        elif _current_value == self._release_level and self._last_value != self._release_level:
            if self.event_released is not None:
                try:
                    schedule(self.event_released, None)
                except:
                    pass
        self._last_value = _current_value

    def is_pressed(self):
        if self._pin.value() == self._press_level:
            return True
        else:
            return False
        
    def was_pressed(self):
        #返回按键是否按下过,并清除按键按下状态
        if self._was_pressed:
            self._was_pressed = False
            return True
        else:
            return False
        
    def get_presses(self):
        #返回按键按下次数，并清除按键按下次数
        _count = self._pressed_count
        self._pressed_count = 0
        return _count
    
    def value(self):
        #返回按键电平值
        return self._pin.value()

    def status(self):
        #返回按键状态,按下返回1，未按下返回0
        _val = self._pin.value()
        if _val == 0:
            return 1
        else:
            return 0
    
    def irq(self, *args, **kwargs):
        if self._id >= 50:
            pass
        else:
            self._pin.irq(*args, **kwargs)

'''待优化'''        
class button(object):
    a = 'a'
    b = 'b'
    def __init__(self, type = 'a'):
        self._pin = None
        if type == 'a':
            self._pin = 5
        elif type == 'b':
            self._pin = 11
        else:
            self._pin = type
        self.func_event_change = None
        self.func_event_released = None
        self.button = Button(self._pin)

    def func(self, _):
        self.func_event_change()

    def func_released(self, _):
        self.func_event_released()

    @property
    def event_pressed(self):
        return self.func_event_change
    
    @event_pressed.setter
    def event_pressed(self, new_event_change):
        if new_event_change != self.func_event_change:
            self.func_event_change = new_event_change
            self.button.event_pressed = self.func

    @property
    def event_released(self):
        return self.func_event_released
    
    @event_released.setter
    def event_released(self, new_event_released):
        if new_event_released != self.func_event_released:
            self.func_event_released = new_event_released
            self.button.event_released = self.func_released
    def status(self):
        return self.button.status()


class AHT20(object):
    ## Default I2C address of AHT20 sensor 
    AHT20_DEF_I2C_ADDR           = 0x38
    ## Init command
    CMD_INIT                     = 0xBE  
    ## The first parameter of init command: 0x08
    CMD_INIT_PARAMS_1ST          = 0x08  
    ## The second parameter of init command: 0x00
    CMD_INIT_PARAMS_2ND          = 0x00  
    ## Waiting time for init completion: 0.01s
    CMD_INIT_TIME                = 0.01    
    ## Trigger measurement command
    CMD_MEASUREMENT              = 0xAC  
    ## The first parameter of trigger measurement command: 0x33
    CMD_MEASUREMENT_PARAMS_1ST   = 0x33  
    ## The second parameter of trigger measurement command: 0x00
    CMD_MEASUREMENT_PARAMS_2ND   = 0x00  
    ## Measurement command completion time：0.08s
    CMD_MEASUREMENT_TIME         = 0.08   
    ## Return data length when the measurement command is without CRC check.
    CMD_MEASUREMENT_DATA_LEN     = 6     
    ## Return data length when the measurement command is with CRC check.
    CMD_MEASUREMENT_DATA_CRC_LEN = 7     
    ## Soft reset command
    CMD_SOFT_RESET               = 0xBA  
    ## Soft reset time: 0.02s
    CMD_SOFT_RESET_TIME          = 0.02   
    ## Get status word command
    CMD_STATUS                   = 0x71

    _humidity = 0.0
    _temperature = 0.0

    def __init__(self):
        self._addr = self.AHT20_DEF_I2C_ADDR
        self._value_buffer = bytearray(1)
        self._i2c = k10_i2c
        self.reset()
        #self.init()
    def init(self):
      #解决新版硬件异常问题
        if self._ready():
            print("true 0")
        else:
            print("false 0")
        self._write_command_args(self.CMD_INIT, self.CMD_INIT_PARAMS_1ST, self.CMD_INIT_PARAMS_2ND)
        time.sleep(self.CMD_INIT_TIME)
        if self._ready():
            print("true 1")
        else:
            print("false 1")

    def reset(self):
      '''!
        @brief   Sensor soft reset, restore the sensor to the initial status
        @return  NONE
      '''
      self._write_command(self.CMD_SOFT_RESET)
      time.sleep(self.CMD_SOFT_RESET_TIME)

    def _write_command(self, cmd):
      self._value_buffer[0] = cmd
      self._i2c.writeto(self._addr, self._value_buffer)
      time.sleep(self.CMD_SOFT_RESET_TIME)

    def _write_command_args(self, cmd, args1, args2):
      #
      l = bytearray(2)
      l[0] = args1
      l[1] = args2
      self._write_bytes(cmd, l)

    def _read_bytes(self, reg, size):
        rslt = self._i2c.readfrom_mem(self._addr, reg, size)
        return rslt

    
    def _write_bytes(self, reg, buf):
        self._i2c.writeto_mem(self._addr, reg, buf)
        return len(buf)
    def _check_crc8(self, crc8, data):
      # CRC initial value: 0xFF
      # CRC8 check polynomial: CRC[7: 0] = X8 + X5 + X4 + 1  -  0x1 0011 0001 - 0x131
      crc = 0xFF
      pos = 0
      size = len(data)
      #print(data)
      while pos < size:
        i = 8
        #crc &= 0xFF
        crc ^= data[pos]
        while i > 0:
          if crc & 0x80:
            crc <<= 1
            crc ^= 0x31
          else:
            crc <<= 1
          i -= 1
        pos += 1
      crc &= 0xFF
      #print(crc)
      if crc8 == crc:
        return True
      return False
    
    def _ready(self):
      status = self._get_status_data()
      if status & 0x08:
        return True
      return False
    
    def _get_status_data(self):
      status = self._read_data(self.CMD_STATUS, 1)
      if len(status):
        return status[0]
      else:
        return 0
      
    def _read_data(self, cmd, len):
      return self._read_bytes(cmd, len)
    
    def _start_measurement_ready(self, crc_en = False):
      '''!
        @brief   Start measurement and determine if it's completed.
        @param crc_en Whether to enable check during measurement
        @n     True  If the measurement is completed, call a related function such as get* to obtain the measured data.
        @n     False If the measurement failed, the obtained data is the data of last measurement or the initial value 0 if the related function such as get* is called at this time.
        @return  Whether the measurement is done
        @retval True  If the measurement is completed, call a related function such as get* to obtain the measured data.
        @retval False If the measurement failed, the obtained data is the data of last measurement or the initial value 0 if the related function such as get* is called at this time.
      '''
      recv_len = self.CMD_MEASUREMENT_DATA_LEN
      if self._ready() == False:
        print("Not cailibration.")
        return False
      if crc_en:
        recv_len = self.CMD_MEASUREMENT_DATA_CRC_LEN
      self._write_command_args(self.CMD_MEASUREMENT, self.CMD_MEASUREMENT_PARAMS_1ST, self.CMD_MEASUREMENT_PARAMS_2ND)
      time.sleep(self.CMD_MEASUREMENT_TIME)
      #l_data = self._read_data(0x00, recv_len)
      l_data = self._i2c.readfrom(self._addr, recv_len)
      #print(l_data)
      if l_data[0] & 0x80:
        print("AHT20 is busy!")
        return False
      if crc_en and self._check_crc8(l_data[6], l_data[:6]) == False:
        print("crc8 check failed.")
        return False
      temp = l_data[1]
      temp <<= 8
      temp |= l_data[2]
      temp <<= 4
      temp = temp | (l_data[3] >> 4)
      temp = (temp & 0xFFFFF) * 100.0
      self._humidity = temp / 0x100000
  
      temp = l_data[3] & 0x0F
      temp <<= 8
      temp |= l_data[4]
      temp <<= 8
      temp |= l_data[5]
      temp = (temp & 0xFFFFF) * 200.0
      self._temperature = temp / 0x100000 - 50
      return True
    
    def temperature(self):
        return round(self._temperature, 2)
    
    def humidity(self):
        return round(self._humidity, 2)
    
    def measure(self):
        self._start_measurement_ready(crc_en = True)

class aht20(object):
    def __init__(self):
        self._aht20 = AHT20()
        time.sleep(1.5)
        self._aht20.measure()

    def measure(self):
        try:
            self._aht20.measure()
        except Exception as e:
            pass
    def read(self):
        return self._aht20.temperature(), self._aht20.humidity()
    
    def read_temp(self):
        return self._aht20.temperature()
    
    def read_temp_f(self):
        return round(self._aht20.temperature() * 1.8 + 32, 2)

    def read_humi(self):
        return self._aht20.humidity()      

'''
k10的环境光传感器类
'''
class Light(object):
    LTR303_DATA_CH1_0    = 0x88
    LTR303ALS_CTRL       = 0x80
    LTR303ALS_GAIN_MODE  = 0x01
    LTR303ALS_MEAS_RATE  = 0x85
    LTR303ALS_INTEG_RATE = 0x03
    def __init__(self):
        self._i2c = k10_i2c
        self._addr = 0x29
        try:
            self._begin()
        except:
            print("Light not detected")
    def _begin(self):
        self._writeReg(self.LTR303ALS_CTRL, self.LTR303ALS_GAIN_MODE)
        self._writeReg(self.LTR303ALS_MEAS_RATE, self.LTR303ALS_INTEG_RATE)

    def _writeReg(self, reg, value):
        self._i2c.writeto_mem(self._addr, reg, value.to_bytes(1, 'little'))

    def _read_bytes(self, reg, size):
        rslt = self._i2c.readfrom_mem(self._addr, reg, size)
        return rslt
    def read(self):
        _als = 0
        buf = self._read_bytes(self.LTR303_DATA_CH1_0,4)
        _als_ch1 = (buf[1] << 8) | buf[0]
        _als_ch0 = (buf[3] << 8) | buf[2]
        if(_als_ch1 + _als_ch0) != 0:
            _ratio = _als_ch1/(_als_ch1 + _als_ch0)
        else:
            _ratio = 0
        if _ratio < 0.45:
            _als = (1.7743 * _als_ch0 + 1.1059 * _als_ch1)
        elif _ratio < 0.64 and _ratio >= 0.45:
            _als = (4.2785*_als_ch0 - 1.9548*_als_ch1)
        elif _ratio<0.85 and _ratio>=0.64:
            _als = (0.5926*_als_ch0 + 0.1185*_als_ch1)
        else:
            _als = 0
        return round(_als, 2)

'''
k10的麦克风类
'''
class Es7243e(object):
    ES7243E_ADDR1 = 0X15
    ES7243E_ADDR2 = 0X11
    def __init__(self, i2c):
        self.i2c = i2c
        self.devices = self.i2c.scan()
        if self.devices:
            if self.ES7243E_ADDR1 in self.devices:
                self.ctrl_state(self.ES7243E_ADDR1,False)
                time.sleep(0.1)
                self.config(self.ES7243E_ADDR1)
                time.sleep(0.1)
                self.ctrl_state(self.ES7243E_ADDR1,True)
            elif self.ES7243E_ADDR2 in self.devices:
                self.ctrl_state(self.ES7243E_ADDR2,False)
                time.sleep(0.1)
                self.config(self.ES7243E_ADDR2)
                time.sleep(0.1)
                self.ctrl_state(self.ES7243E_ADDR2,True)
            else:
                print("mic init error")
                pass
    def write_cmd(self,addr, reg, cmd):
        send_buf = bytearray(2)
        send_buf[0] = reg
        send_buf[1] = cmd
        self.i2c.writeto(addr, send_buf)
    
    def ctrl_state(self, addr, state):
        if state:
            self.write_cmd(addr, 0xF9, 0x00)
            self.write_cmd(addr, 0xF9, 0x00)
            self.write_cmd(addr, 0x04, 0x01)
            self.write_cmd(addr, 0x17, 0x01)
            self.write_cmd(addr, 0x20, 0x10)
            self.write_cmd(addr, 0x21, 0x10)
            self.write_cmd(addr, 0x00, 0x80)
            self.write_cmd(addr, 0x01, 0x3A)
            self.write_cmd(addr, 0x16, 0x3F)
            self.write_cmd(addr, 0x16, 0x00)
        else:
            self.write_cmd(addr, 0x04, 0x02)
            self.write_cmd(addr, 0x04, 0x01)
            self.write_cmd(addr, 0xF7, 0x30)
            self.write_cmd(addr, 0xF9, 0x01)
            self.write_cmd(addr, 0x16, 0xFF)
            self.write_cmd(addr, 0x17, 0x00)
            self.write_cmd(addr, 0x01, 0x38)
            self.write_cmd(addr, 0x20, 0x00)
            self.write_cmd(addr, 0x21, 0x00)
            self.write_cmd(addr, 0x00, 0x00)
            self.write_cmd(addr, 0x00, 0x1E)
            self.write_cmd(addr, 0x01, 0x30)
            self.write_cmd(addr, 0x01, 0x00)
    
    def config(self,addr):
        self.write_cmd(addr, 0x01, 0x3A)
        self.write_cmd(addr, 0x00, 0x80)
        self.write_cmd(addr, 0xF9, 0x00)
        self.write_cmd(addr, 0x04, 0x02)
        self.write_cmd(addr, 0x04, 0x01)
        self.write_cmd(addr, 0xF9, 0x01)
        self.write_cmd(addr, 0x00, 0x1E)
        self.write_cmd(addr, 0x01, 0x00)
        
        self.write_cmd(addr, 0x02, 0x00)
        self.write_cmd(addr, 0x03, 0x20)
        self.write_cmd(addr, 0x04, 0x03)
        self.write_cmd(addr, 0x0D, 0x00)
        self.write_cmd(addr, 0x05, 0x00)
        self.write_cmd(addr, 0x06, 0x03)
        self.write_cmd(addr, 0x07, 0x00)
        self.write_cmd(addr, 0x08, 0xFF)

        self.write_cmd(addr, 0x09, 0xCA)
        self.write_cmd(addr, 0x0A, 0x85)
        self.write_cmd(addr, 0x0B, 0x2C)
        self.write_cmd(addr, 0x0E, 0xff)
        self.write_cmd(addr, 0x0F, 0x80)
        self.write_cmd(addr, 0x14, 0x0C)
        self.write_cmd(addr, 0x15, 0x0C)
        self.write_cmd(addr, 0x17, 0x02)
        self.write_cmd(addr, 0x18, 0x26)
        self.write_cmd(addr, 0x19, 0x77)
        self.write_cmd(addr, 0x1A, 0xF4)
        self.write_cmd(addr, 0x1B, 0x66)
        self.write_cmd(addr, 0x1C, 0x44)
        self.write_cmd(addr, 0x1E, 0x00)
        self.write_cmd(addr, 0x1F, 0x0C)
        self.write_cmd(addr, 0x20, 0x1A)
        self.write_cmd(addr, 0x21, 0x1A)
        
        self.write_cmd(addr, 0x00, 0x80)
        self.write_cmd(addr, 0x01, 0x3A)
        self.write_cmd(addr, 0x16, 0x3F)
        self.write_cmd(addr, 0x16, 0x00)
        
class Mic(object):
    def __init__(self,bits=16,sample_rate=16000,channels=1):
        self.mic = Es7243e(k10_i2c)
        self.i2s = I2S(0,sck = 0, ws = 38, sd = 39,mode=I2S.RX, bits=bits, 
                       format=I2S.MONO if channels == 1 else I2S.STEREO, rate=sample_rate, ibuf=20000)
        self.i2s.deinit()
        self.bits = bits
        self.sample_rate = sample_rate
        self.channels = channels
        self.time = time
    def reinit(self,bits=16,sample_rate=16000,channels=1):
        self.i2s = I2S(0,sck = 0, ws = 38, sd = 39,mode=I2S.RX, bits=bits, 
                       format=I2S.MONO if channels == 1 else I2S.STEREO, rate=sample_rate, ibuf=20000)
    def deinit(self):
        print("mic deinit\n")
        if self.i2s:
            self.i2s.deinit()
            self.i2s = None


    def write_wav_header(self, file, num_samples):
        # 计算文件大小
        byte_rate = self.sample_rate * self.channels * self.bits // 8
        block_align = self.channels * self.bits // 8
        data_size = num_samples * block_align
        file_size = data_size + 36
        
        # 写入 WAV 文件头
        file.write(b'RIFF')
        file.write(file_size.to_bytes(4, 'little'))
        file.write(b'WAVE')
        file.write(b'fmt ')  # 子块ID
        file.write((16).to_bytes(4, 'little'))  # 子块大小
        file.write((1).to_bytes(2, 'little'))   # 音频格式（1是PCM）
        file.write(self.channels.to_bytes(2, 'little'))
        file.write(self.sample_rate.to_bytes(4, 'little'))
        file.write(byte_rate.to_bytes(4, 'little'))
        file.write(block_align.to_bytes(2, 'little'))
        file.write(self.bits.to_bytes(2, 'little'))
        file.write(b'data')
        file.write(data_size.to_bytes(4, 'little'))

    def recode_to_wav(self,path,time):
        self.reinit(bits = self.bits, sample_rate = self.sample_rate, channels=self.channels)
        #创建录音缓存区
        buffer_size = 1024
        audio_buf = bytearray(buffer_size)

        #打开WAV文件
        with open(path, 'wb') as wav_file:
            #暂时写入WAV文件头，后续更新数据大小
            self.write_wav_header(wav_file,num_samples=0)

            #开始录音
            num_samples = 0
            start_time = self.time.time()
            while self.time.time() - start_time < time:
                #从I2S中读取数据
                self.i2s.readinto(audio_buf)
                wav_file.write(audio_buf)
                num_samples += len(audio_buf) // (self.bits // 8)
            #更新文件头中的实际数据大小
            wav_file.seek(0)
            self.write_wav_header(wav_file, num_samples)
        self.i2s.deinit()
        print("Recording saved to:", path)
    def recode_sys(self, name="",time=10):
        full_path = "/" + name
        self.recode_to_wav(path=full_path, time=time)

    def recode_tf(self, name="",time=10):
        full_path = "/sd/" + name
        self.recode_to_wav(path=full_path, time=time)

'''
K10扬声器类
'''
class Speaker(object):
    def __init__(self):
        print("init speaker\n")
        self.i2s = I2S(1,sck = 0,ws=38, sd= 45, mode=I2S.TX, bits=32, format=I2S.MONO, rate=16000, ibuf=20000)
        self.i2s.deinit()
        self._i2c = k10_i2c
        print("init done\n")
    def __del__(self):
        print("Speaker deleted\n")
        if self.i2s:
            self.i2s.deinit()
            self.i2s = None
        try:
            self._i2c.writeto_mem(0x20,0x2A,bytearray([0x00]))
        except:
            pass
    def deinit(self):
        print("Speaker deinit\n")
        if self.i2s:
            self.i2s.deinit()
            self.i2s = None
        try:
            self._i2c.writeto_mem(0x20,0x2A,bytearray([0x00]))
        except:
            pass
        
    def reinit(self,bits=16,sample_rate=16000,channels=1):
        self.i2s = I2S(1,sck = 0, ws = 38, sd = 45,mode=I2S.TX, bits=bits, 
                       format=I2S.MONO if channels == 1 else I2S.STEREO, rate=sample_rate, ibuf=20000)

    def parse_wav_header(self, wav_file):
        # 解析 WAV 文件头部
        wav_file.seek(0)
        riff = wav_file.read(4)
        if riff != b'RIFF':
            raise ValueError("Not a valid WAV file")

        wav_file.seek(22)
        num_channels = int.from_bytes(wav_file.read(2), 'little')
        sample_rate = int.from_bytes(wav_file.read(4), 'little')

        wav_file.seek(34)
        bits_per_sample = int.from_bytes(wav_file.read(2), 'little')

        return sample_rate, bits_per_sample, num_channels
    
    def play_tone(self,note):
        pass
    def play_tone_music(self,tone_music):
        pass
    def play_sys_music(self,path):
        full_path = "/" + path
        self.play_music(full_path)

    def play_tf_music(self, path):
        full_path = "/sd/" + path
        self.play_music(full_path)
        
    def play_music(self,path):
        #使能功放(k10 box才有的功能)
        try:
            self._i2c.writeto_mem(0x20,0x2A,bytearray([0x01]))
        except:
            pass
        #打开WAV文件
        with open(path,"rb") as wav_file:
            sample_rate, bits_per_sample, num_channels = self.parse_wav_header(wav_file)
            self.reinit(bits=bits_per_sample,sample_rate=sample_rate,channels=num_channels)
            while True:
                audio_buf = wav_file.read(1024)
                if not audio_buf:
                    break
                self.i2s.write(audio_buf)
        self.i2s.deinit()
        #失能功放
        try:
            self._i2c.writeto_mem(0x20,0x2A,bytearray([0x00]))
        except:
            pass

    def stop_music(self):
        self.i2s.deinit()
        #失能功放
        try:
            self._i2c.writeto_mem(0x20,0x2A,bytearray([0x00]))
        except:
            pass

'''
K10的SD卡类
'''
class TF_card(object):
    def __init__(self):
        try:
            self.spi_bus = SPI.Bus(host = 1, mosi = 42, miso = 41, sck = 44)
            self.sd = SDCard(spi_bus = self.spi_bus, cs = 40, freq = 1000000)
            vfs.mount(self.sd, "/sd")
        except:
            print("SD card not detected")
    def __del__(self):
        if self.spi_bus:
            self.spi_bus = None
    def deinit(self):
        if self.spi_bus:
            self.spi_bus = None
'''
K10的摄像头类
'''
class Camera(object):
    def __init__(self):
        self.cam = camera
        self._i2c = k10_i2c
    def init(self):
        #复位摄像头
        temp = self._i2c.readfrom_mem(0x20, 0x06, 1)
        self._i2c.writeto(0x20,bytearray([0x06, (temp[0] & 0xFD)]))
        temp = self._i2c.readfrom_mem(0x20, 0x02, 1)
        self._i2c.writeto(0x20,bytearray([0x02, (temp[0] | 0x00)]))
        time.sleep(0.1)
        temp = self._i2c.readfrom_mem(0x20, 0x02, 1)
        self._i2c.writeto(0x20,bytearray([0x02, (temp[0] | 0x02)]))
        self.cam.init(0)
    def capture(self):
        return self.cam.capture()
    def save(self):
        pass

'''
K10的屏幕类
'''
class Screen(object):
    def __init__(self,dir=2):
        self.spi_bus = SPI.Bus(host=2,mosi=21,miso=-1,sck=12)
        self.display_bus = lcd_bus.SPIBus(spi_bus = self.spi_bus, dc = 13, cs = 14, freq = 40000000)
        '''
        self.display = ili9341.ILI9341(data_bus = self.display_bus, display_width = 240, display_height = 320,
                                       reset_state = ili9341.STATE_LOW, color_byte_order = ili9341.BYTE_ORDER_BGR,
                                       color_space = lv.COLOR_FORMAT.RGB565, rgb565_byte_swap=True)
        '''
        self.display = ili9341.ILI9341(data_bus = self.display_bus, display_width = 240, display_height = 320,
                                       reset_state = ili9341.STATE_LOW, color_byte_order = ili9341.BYTE_ORDER_BGR,
                                       color_space = lv.COLOR_FORMAT.RGB565, rgb565_byte_swap=True)
        self.linewidth = 1
        
    #初始化屏幕，设置方向为(0~3)
    def init(self,dir=2):
        #用来打开屏幕背光
        myi2c = I2C(0, scl=Pin(48), sda=Pin(47), freq=100000)
        temp = myi2c.readfrom_mem(0x20, 0x02, 1)
        myi2c.writeto(0x20,bytearray([0x02, (temp[0] | 0x01)]))
        temp = myi2c.readfrom_mem(0x20, 0x06, 1)
        myi2c.writeto(0x20,bytearray([0x06, (temp[0] & 0xFE)]))

        self.display.set_power(True)
        self.display.init(1)
        if dir == 0:
            self.display.set_rotation(lv.DISPLAY_ROTATION._0)
        elif dir == 1:
            self.display.set_rotation(lv.DISPLAY_ROTATION._90)
        elif dir == 2:
            self.display.set_rotation(lv.DISPLAY_ROTATION._180)
        elif dir == 3:
            self.display.set_rotation(lv.DISPLAY_ROTATION._270)
        else:
            self.display.set_rotation(lv.DISPLAY_ROTATION._180)

        #self.screen = lv.obj()
        self.screen = lv.screen_active()
        self.img = lv.image(self.screen)
        self.canvas = lv.canvas(self.screen)
        self.screen.set_scrollbar_mode(lv.SCROLLBAR_MODE.OFF)
        self.img.set_scrollbar_mode(lv.SCROLLBAR_MODE.OFF)
        self.canvas.set_scrollbar_mode(lv.SCROLLBAR_MODE.OFF)


        self.canvas.set_size(240,320)
        self.canvas.align(lv.ALIGN.CENTER, 0, 0)
        self.canvas_buf = bytearray(240*320*4)
        self.canvas.set_buffer(self.canvas_buf, 240, 320, lv.COLOR_FORMAT.ARGB8888)
        self.canvas.fill_bg(lv.color_white(), lv.OPA.TRANSP)
        self.layer = lv.layer_t()
        self.canvas.init_layer(self.layer)
        self.area = lv.area_t()
        self.clear_rect = lv.draw_rect_dsc_t()
        
        self.img_dsc = lv.image_dsc_t(
            dict(
                header = dict(cf =lv.COLOR_FORMAT.RGB565, w=480, h=320),
                data_size = 480*320*2,
                data = None
            )
        )
        #显示摄像头画面的timer
        self.camera_timer = None

    #显示指定颜色背景
    def show_bg(self,color=0xFFFFFF):
        self.screen.set_style_bg_color(lv.color_hex(color),0)

    #将缓存内容显示
    def show_draw(self):
        self.canvas.finish_layer(self.layer)
        self.canvas.invalidate()
        #lv.screen_load(self.screen)

    #清除全屏，颜色为指定颜色
    def clear(self,line=0,font=None,color=None):
        if font == None:
            #清除全屏，颜色为指定颜色
            if color == None:
                self.canvas.fill_bg(lv.color_white(), lv.OPA.TRANSP)
            else:
                self.canvas.fill_bg(lv.color_hex(color), lv.OPA.COVER)
        else:
            #清除第几行文字
            pass
        pass

    #显示文字xxx在第line行,字号font_size,颜色color
    def draw_text(self,text="",line=None, x=0,y=0,font_size=16,color=0x0000FF):
        #self.canvas.finish_layer(self.layer)
        #self.canvas.get_draw_buf().clear(self.area)
        self.desc = lv.draw_label_dsc_t()
        self.desc.init()
        self.desc.color = lv.color_hex(color)
        self.desc.text = text
        if font_size == 16:
            self.desc.font = lv.font_montserrat_16
        elif font_size == 14:
            self.desc.font = lv.font_montserrat_14
        elif font_size == 12:
            self.desc.font = lv.font_montserrat_12
        else:
            font_size = 16

        #按坐标显示
        if line == None:
            self.area.x1 = x
            self.area.y1 = y
        #按行显示 
        else:
            self.area.x1 = 0
            self.area.y1 = line * (font_size + 2)
        self.area.set_width(240-self.area.x1)
        self.area.set_height(font_size + 2)

        #self.layer.draw_buf.clear(self.area)
        #self.canvas.fill_bg(lv.color_white(), lv.OPA.TRANSP)
        #self.canvas.get_draw_buf().clear(self.area)  # 强制清除画布缓冲区
        #bytearray(self.canvas.get_buf())[:] = b'\x00' * len(self.canvas_buf)
        self.layer.draw_buf.clear(self.area)  # 清除图层缓冲区
        #self.canvas_buf[:] = b'\x00' * len(self.canvas_buf)
        lv.draw_label(self.layer, self.desc, self.area)
        #self.layer.draw_buf.clear(self.area)

    #画点
    def draw_point(self,x=0,y=0,color=0x0000FF):
        #self.canvas.set_px(x=x, y=y, color = lv.color_hex(color))
        self.draw_line(x0=x,y0=y,x1=(x+self.linewidth),y1=y,color = color)

    #设置线宽/边框宽
    def set_width(self,width=1):
        self.linewidth = width

    #画线
    def draw_line(self,x0=0,y0=0,x1=0,y1=0,color=0x000000):
        self.desc = lv.draw_line_dsc_t()
        self.desc.init()
        self.desc.p1.x = x0
        self.desc.p1.y = y0
        self.desc.p2.x = x1
        self.desc.p2.y = y1
        self.desc.color = lv.color_hex(color)
        self.desc.width = self.linewidth
        self.desc.opa = 255
        self.desc.blend_mode = lv.BLEND_MODE.NORMAL
        self.desc.round_start = 0
        self.desc.round_end = 0
        lv.draw_line(self.layer, self.desc)

    #画圆,圆心(x,y),r半径,边框颜色bcolor,填充颜色fcolor
    def draw_circle(self, x=0,y=0,r=0,bcolor=0x000000,fcolor=None):
        self.desc = lv.draw_rect_dsc_t()
        self.desc.init()
        self.desc.radius = lv.RADIUS_CIRCLE
        if fcolor == None:
            #设置矩形背景透明度为透明色
            self.desc.bg_opa = lv.OPA.TRANSP
        else:
            #设置矩形背景透明度为不透明
            self.desc.bg_opa = lv.OPA.COVER
            #设置矩形背景色
            self.desc.bg_color = lv.color_hex(fcolor)
        #设置矩形边框宽度
        self.desc.border_width = self.linewidth
        #设置矩形边框颜色
        self.desc.border_color = lv.color_hex(bcolor)
        area = lv.area_t()
        area.x1 = x-r
        area.y1 = y-r
        area.set_width(2*r)
        area.set_height(2*r)
        lv.draw_rect(self.layer, self.desc, area)
        pass
    
    #显示矩形顶点(x,y),宽w、高h,边框颜色bcolor,填充颜色fcolor
    def draw_rect(self, x = 0, y = 0, w = 0, h = 0, bcolor = 0x000000, fcolor = None):
        self.desc = lv.draw_rect_dsc_t()
        self.desc.init()
        
        if fcolor == None:
            #设置矩形背景透明度为透明色
            self.desc.bg_opa = lv.OPA.TRANSP
        else:
            #设置矩形背景透明度为不透明
            self.desc.bg_opa = lv.OPA.COVER
            #设置矩形背景色
            self.desc.bg_color = lv.color_hex(fcolor)
        #设置矩形边框宽度
        self.desc.border_width = self.linewidth
        #设置矩形边框颜色
        self.desc.border_color = lv.color_hex(bcolor)
        area = lv.area_t()
        area.x1 = x
        area.y1 = y
        area.set_width(w)
        area.set_height(h)
        lv.draw_rect(self.layer, self.desc, area)
        pass
    def show_camera_feed(self, buf):
        # 将摄像头数据填充到canvas缓冲区
        #self.canvas_buf[:] = buf[:len(self.canvas_buf)]  # 假设buf与canvas分辨率匹配
        #self.show_draw()
        img_dsc = lv.draw_image_dsc_t()
        img_dsc.init()
        img_dsc.src = buf
        area = lv.area_t()
        area.x1 = 0
        area.y1 = 0
        area.set_width(240)
        area.set_height(320)
        lv.draw_image(self.layer,self.desc, area)
        lv.screen_load(self.screen)

    def show_camera_img(self,buf):
        lv.draw_sw_rgb565_swap(buf,240*320*2)
        self.img_dsc.data = buf
        self.img.set_src(self.img_dsc)
        lv.refr_now(None)

    def show_camera(self,camera):
        self.camera_timer = lv.timer_create(lambda t: self.show_camera_img(camera.capture()), 50, None)

    def close_show_camera(self):
        if self.camera_timer:
            lv.timer_del(self.camera_timer)
            self.camera_timer = None


class Wifibase(object):
    def __init__(self):
        self.sta = network.WLAN(network.STA_IF)
        self.ap = network.WLAN(network.AP_IF)

    def connectWiFi(self, ssid, passwd, timeout=10):
        if self.sta.isconnected():
            self.sta.disconnect()
        self.sta.active(True)
        list = self.sta.scan()
        for i, wifi_info in enumerate(list):
            try:
                if wifi_info[0].decode() == ssid:
                    self.sta.connect(ssid, passwd)
                    wifi_dbm = wifi_info[3]
                    break
            except UnicodeError:
                self.sta.connect(ssid, passwd)
                wifi_dbm = '?'
                break
            if i == len(list) - 1:
                raise OSError("SSID invalid / failed to scan this wifi")
        start = time.time()
        print("Connection WiFi", end="")
        while (self.sta.ifconfig()[0] == '0.0.0.0'):
            if time.ticks_diff(time.time(), start) > timeout:
                print("")
                raise OSError("Timeout!,check your wifi password and keep your network unblocked")
            print(".", end="")
            time.sleep_ms(500)
        print("")
        print('WiFi(%s,%sdBm) Connection Successful, Config:%s' % (ssid, str(wifi_dbm), str(self.sta.ifconfig())))

    def disconnectWiFi(self):
        if self.sta.isconnected():
            self.sta.disconnect()
        self.sta.active(False)
        print('disconnect WiFi...')

    def enable_APWiFi(self, essid, password=b'',channel=10):
        self.ap.active(True)
        if password:
            authmode=4
        else:
            authmode=0
        self.ap.config(essid=essid,password=password,authmode=authmode, channel=channel)

    def disable_APWiFi(self):
        self.ap.active(False)
        print('disable AP WiFi...')

'''继承Wifibase'''
class WiFi(Wifibase):
    def __init__(self):
        super().__init__()

    def connect(self, ssid, psd, timeout=10000):
        self.connectWiFi(ssid, psd, int(timeout/1000))
    
    def status(self):
        return self.sta.isconnected()

    def info(self):
        return str(self.sta.ifconfig())

'''
k10的mqtt客户端类
'''
class MqttClient():
    def __init__(self):
        self.client = None
        self.server = None
        self.port = None
        self.client_id = None
        self.user = None
        self.passsword = None
        self.topic_msg_dict = {}
        self.topic_callback = {}
        self.tim_count = 0
        self._connected = False
        self.lock = False

    def connect(self, **kwargs):
        server = kwargs.get('server',"iot.mpython.cn" )
        port = kwargs.get('port',1883 )
        client_id = kwargs.get('client_id',"" )
        user = kwargs.get('user',"" )
        psd = kwargs.get('psd',None)
        password = kwargs.get('password',None)
        if(psd==None and password==None):
            psd = ""
        elif(password!=None):
            psd = password
        try:
            self.client = MQTT(client_id, server, port, user, psd, 0)
            self.client.connect()
            self.server = server
            self.port = port
            self.client_id = client_id
            self.user = user
            self.passsword = psd
            print('Connected to MQTT Broker "{}"'.format(self.server))
            self._connected = True
            self.client.set_callback(self.on_message)
            time.sleep(0.5)
            self.tim = Timer(15)
            self.tim.init(period=100, mode=Timer.PERIODIC, callback=self.mqtt_heartbeat)
            gc.collect()
        except Exception as e:
            print('Connected to MQTT Broker error:{}'.format(e))

    def connected(self):
        return self._connected

    def publish(self, topic, content):
        try:
            self.lock = True
            self.client.publish(str(topic),str(content).encode("utf-8"))
            self.lock = False
        except Exception as e:
            print('publish error:{}'.format(e))

    def message(self, topic):
        topic = str(topic)
        if(not topic in self.topic_msg_dict):
            # self.topic_msg_dict[topic] = None
            self.topic_callback[topic] = False 
            self.subscribe(topic, self.default_callbak)
            return self.topic_msg_dict[topic]
        else:
            return self.topic_msg_dict[topic]
        
    def received(self, topic, callback):
        self.subscribe(topic, callback)

    def subscribe(self, topic, callback):
        self.lock = True
        try:
            topic = str(topic)
            if(not topic in self.topic_msg_dict):
                global _callback
                _callback = callback
                self.topic_msg_dict[topic] = None
                self.topic_callback[topic] = True
                exec('global mqtt_topic_' + bytes.decode(ubinascii.hexlify(topic)),globals())
                exec('mqtt_topic_' + bytes.decode(ubinascii.hexlify(topic)) + ' = _callback',globals())
                self.client.subscribe(topic)
                time.sleep(0.1)
            elif(topic in self.topic_msg_dict and self.topic_callback[topic] == False):
                global _callback
                _callback = callback
                self.topic_callback[topic] = True
                exec('global mqtt_topic_' + bytes.decode(ubinascii.hexlify(topic)),globals())
                exec('mqtt_topic_' + bytes.decode(ubinascii.hexlify(topic)) + ' = _callback',globals())
                time.sleep(0.1)
            else:
                print('Already subscribed to the topic:{}'.format(topic))
            self.lock = False
        except Exception as e:
            print('MQTT subscribe error:'+str(e))

    def on_message(self, topic, msg):
        try:
            gc.collect()
            topic = topic.decode('utf-8', 'ignore')
            msg = msg.decode('utf-8', 'ignore')
            #print("Received '{payload}' from topic '{topic}'\n".format(payload = msg, topic = topic))
            if(topic in self.topic_msg_dict):
                self.topic_msg_dict[topic] = msg
                if(self.topic_callback[topic]):
                    exec('global mqtt_topic_' + bytes.decode(ubinascii.hexlify(topic)),globals())
                    eval('mqtt_topic_' + bytes.decode(ubinascii.hexlify(topic))+'()',globals())
        except Exception as e:
            print('MQTT on_message error:'+str(e))
    
    def default_callbak(self):
        pass
    
    def mqtt_check_msg(self):
        try:
            self.client.check_msg()
        except Exception as e:
            print('MQTT check msg error:'+str(e))

    def mqtt_heartbeat(self,_):
        self.tim_count += 1 
        if(not self.lock):
            self.mqtt_check_msg()
        if(self.tim_count==200):
            self.tim_count = 0
            try:
                self.client.ping() # 心跳消息
                self._connected = True
            except Exception as e:
                print('MQTT keepalive ping error:'+str(e))
                self._connected = False

'''
k10的内置定时器类
用来加载固件后就定时获取部分板载外设的状态
'''
class _k10_timer(object):
    def __init__(self, temp = None, acc = None):
        self._timer = Timer(18)
        self._timer.init(period=50, mode=Timer.PERIODIC, callback=self.timer_callback)
        self._temp = temp
        self._num = 0
    def timer_callback(self, _):
        #50ms读取一次IO扩展芯片的状态
        self._num += 1
        dfrobot_global_extio_timer_handler(_)
        #100ms执行另外一个任务
        if self._num % 2 == 0:
            pass
        #1s执行另外的任务
        if self._num % 20 == 0:
            if isinstance(self._temp, aht20):
                self._temp.measure()
            self._num = 0
temp_humi = aht20()
_k10_measure_timer =_k10_timer(temp = temp_humi)