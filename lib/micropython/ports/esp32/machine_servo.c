/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <stdio.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"

#include "py/nlr.h"
#include "py/runtime.h"
#include "modmachine.h"
#include "mphalport.h"
#include "soc/gpio_periph.h"

// Forward dec'l
const mp_obj_type_t machine_servo_type;
//extern const mp_obj_type_t machine_servo_type;   
int pin_remap_esp32[] = {1, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 48, 47};              

typedef struct _esp32_servo_pwm_obj_t {
    mp_obj_base_t base;
    int pin;
    uint8_t active;
    uint8_t channel;
    uint16_t min_us;
    uint16_t max_us;
    uint16_t actuation_range;
} esp32_servo_pwm_obj_t;

// Which channel has which GPIO pin assigned?
// (-1 if not assigned)
static int chan_gpio[LEDC_CHANNEL_MAX];

// Params for PW operation
// 5khz
#define PWFREQ (500)
// High speed mode
#define PWMODE (LEDC_LOW_SPEED_MODE)
// 10-bit resolution (compatible with esp8266 PWM)
#define PWRES (LEDC_TIMER_10_BIT)
// Timer 1
#define PWTIMER (LEDC_TIMER_1)

// Config of timer upon which we run all PWM'ed GPIO pins
static bool pwm_inited = false;
static ledc_timer_config_t timer_cfg = {
    .duty_resolution = PWRES,
    .freq_hz = PWFREQ,
    .speed_mode = PWMODE,
    .timer_num = PWTIMER
};

static void pwm_init(void) {

    // Initial condition: no channels assigned
    for (int x = 0; x < LEDC_CHANNEL_MAX; ++x) {
        chan_gpio[x] = -1;
    }
    for (int y = 0; y < 40; y++)
    {
        if((GPIO.func_out_sel_cfg[y].func_sel >= 79) && (GPIO.func_out_sel_cfg[y].func_sel <= 86 ))
        {
            // mp_warning(NULL, "func_sel:%d gpio_num: %d",GPIO.func_out_sel_cfg[y].func_sel, y );
            GPIO.func_out_sel_cfg[y].func_sel = 39;
            // GPIO.func_out_sel_cfg[y].oen_sel = 0;
            // GPIO.func_out_sel_cfg[y].oen_inv_sel = 1;
        }
    }
    // Init with default timer params
    ledc_timer_config(&timer_cfg);
}

static int set_freq(int newval) {
    int ores = timer_cfg.duty_resolution;
    int oval = timer_cfg.freq_hz;

    // Find the highest bit resolution for the requested frequency
    if (newval <= 0) {
        newval = 1;
    }
    unsigned int res = 0;
    for (unsigned int i = APB_CLK_FREQ / newval; i > 1; i >>= 1, ++res) {
    }
    if (res == 0) {
        res = 1;
    } else if (res > PWRES) {
        // Limit resolution to PWRES to match units of our duty
        res = PWRES;
    }

    // Configure the new resolution and frequency
    timer_cfg.duty_resolution = res;
    timer_cfg.freq_hz = newval;
    if (ledc_timer_config(&timer_cfg) != ESP_OK) {
        timer_cfg.duty_resolution = ores;
        timer_cfg.freq_hz = oval;
        return 0;
    }
    return 1;
}

/******************************************************************************/

// MicroPython bindings for PWM

static void esp32_servo_pwm_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    esp32_servo_pwm_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "PWM(%u", self->pin);
    if (self->active) {
        mp_printf(print, ", freq=%u, duty=%u", timer_cfg.freq_hz,
            ledc_get_duty(PWMODE, self->channel));
    }
    mp_printf(print, ")");
}

static void esp32_servo_pwm_init_helper(esp32_servo_pwm_obj_t *self,
        size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_freq, ARG_angle, ARG_min_us, ARG_max_us, ARG_actuation_range };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_freq, MP_ARG_INT, {.u_int = 50} },
        { MP_QSTR_angle, MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_min_us, MP_ARG_INT, {.u_int = 750} },
        { MP_QSTR_max_us, MP_ARG_INT, {.u_int = 2250} },
        { MP_QSTR_actuation_range, MP_ARG_INT, {.u_int = 180} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args,
        MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    self->min_us = args[ARG_min_us].u_int;
    self->max_us = args[ARG_max_us].u_int;
    self->actuation_range = args[ARG_actuation_range].u_int;

    int channel;
    int avail = -1;

    // Find a free PWM channel, also spot if our pin is
    //  already mentioned.
    for (channel = 0; channel < LEDC_CHANNEL_MAX; ++channel) {
        if (chan_gpio[channel] == self->pin) {
            break;
        }
        if ((avail == -1) && (chan_gpio[channel] == -1)) {
            avail = channel;
        }
    }
    if (channel >= LEDC_CHANNEL_MAX) {
        if (avail == -1) {
            mp_raise_ValueError(MP_ERROR_TEXT("out of PWM channels"));
        }
        channel = avail;
    }
    self->channel = channel;

    // New PWM assignment
    self->active = 1;
    if (chan_gpio[channel] == -1) {
        ledc_channel_config_t cfg = {
            .channel = channel,
            .duty = (1 << timer_cfg.duty_resolution) / 2,
            .gpio_num = self->pin,
            .intr_type = LEDC_INTR_DISABLE,
            .speed_mode = PWMODE,
            .timer_sel = PWTIMER,
        };
        if (ledc_channel_config(&cfg) != ESP_OK) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
                MP_ERROR_TEXT("PWM not supported on pin %d"), self->pin));
        }
        chan_gpio[channel] = self->pin;
    }

    // Maybe change PWM timer
    int tval = args[ARG_freq].u_int;
    if (tval != -1) {
        if (tval != timer_cfg.freq_hz) {
            if (!set_freq(tval)) {
                nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
                    MP_ERROR_TEXT("Bad frequency %d"), tval));
            }
        }
    }

    // Set duty cycle?
    int angle = args[ARG_angle].u_int;
    if (angle != -1) {
        if ((angle < 0) || (angle > self->actuation_range))
            mp_raise_ValueError(MP_ERROR_TEXT("Angle out of range"));
        int us_range = self->max_us - self->min_us;
        int us = self->min_us + (int)(angle * us_range / self->actuation_range);
        if ((us < self->min_us) || (us > self->max_us))
            mp_raise_ValueError(MP_ERROR_TEXT("Pulse width out of range"));
        int duty = (int)((us * 1023)/ 20000);

        duty &= ((1 << PWRES)-1);
        duty >>= PWRES - timer_cfg.duty_resolution;
        ledc_set_duty(PWMODE, channel, duty);
        ledc_update_duty(PWMODE, channel);
    }
}

static mp_obj_t esp32_servo_pwm_make_new(const mp_obj_type_t *type,
        size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);
    // gpio_num_t pin_id = machine_pin_get_id(args[0]);
    int pin = mp_obj_get_int(args[0]);
    int pin_id = pin_remap_esp32[pin];


    // create PWM object from the given pin
    esp32_servo_pwm_obj_t *self = m_new_obj(esp32_servo_pwm_obj_t);
    self->base.type = &machine_servo_type;
    self->pin = pin_id;
    self->active = 0;
    self->channel = -1;

    // start the PWM subsystem if it's not already running
    if (!pwm_inited) {
        pwm_init();
        pwm_inited = true;
    }

    // start the PWM running for this channel
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
    esp32_servo_pwm_init_helper(self, n_args - 1, args + 1, &kw_args);

    return MP_OBJ_FROM_PTR(self);
}

static mp_obj_t esp32_servo_pwm_init(size_t n_args,
        const mp_obj_t *args, mp_map_t *kw_args) {
    esp32_servo_pwm_init_helper(args[0], n_args - 1, args + 1, kw_args);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(esp32_servo_pwm_init_obj, 1, esp32_servo_pwm_init);

static mp_obj_t esp32_servo_pwm_deinit(mp_obj_t self_in) {
    esp32_servo_pwm_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int chan = self->channel;

    // Valid channel?
    if ((chan >= 0) && (chan < LEDC_CHANNEL_MAX)) {
        // Mark it unused, and tell the hardware to stop routing
        chan_gpio[chan] = -1;
        ledc_stop(PWMODE, chan, 0);
        self->active = 0;
        self->channel = -1;
        //gpio_matrix_out(self->pin, SIG_GPIO_OUT_IDX, false, false);
        esp_rom_gpio_connect_out_signal(self->pin, SIG_GPIO_OUT_IDX, false, false);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(esp32_servo_pwm_deinit_obj, esp32_servo_pwm_deinit);

static mp_obj_t esp32_servo_pwm_freq(size_t n_args, const mp_obj_t *args) {
    if (n_args == 1) {
        // get
        return MP_OBJ_NEW_SMALL_INT(timer_cfg.freq_hz);
    }

    // set
    int tval = mp_obj_get_int(args[1]);
    if (!set_freq(tval)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
            MP_ERROR_TEXT("Bad frequency %d"), tval));
    }
    return mp_const_none;
}

static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(esp32_servo_pwm_freq_obj, 1, 2, esp32_servo_pwm_freq);

static mp_obj_t esp32_servo_pwm_duty(size_t n_args, const mp_obj_t *args) {
    esp32_servo_pwm_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int duty;

    if (n_args == 1) {
        // get
        duty = ledc_get_duty(PWMODE, self->channel);
        duty <<= PWRES - timer_cfg.duty_resolution;
        return MP_OBJ_NEW_SMALL_INT(duty);
    }

    // set
    duty = mp_obj_get_int(args[1]);
    duty &= ((1 << PWRES)-1);
    duty >>= PWRES - timer_cfg.duty_resolution;
    ledc_set_duty(PWMODE, self->channel, duty);
    ledc_update_duty(PWMODE, self->channel);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(esp32_servo_pwm_duty_obj,
    1, 2, esp32_servo_pwm_duty);

static mp_obj_t esp32_servo_pwm_write_us(mp_obj_t self_in, mp_obj_t _us) {
    esp32_servo_pwm_obj_t *self = MP_OBJ_TO_PTR(self_in);

    int us = mp_obj_get_int(_us);
    if ((us < self->min_us) || (us > self->max_us))
        mp_raise_ValueError(MP_ERROR_TEXT("Pulse width out of range"));
    int duty = (int)((us * 1023)/ 20000);

    duty &= ((1 << PWRES)-1);
    duty >>= PWRES - timer_cfg.duty_resolution;
    ledc_set_duty(PWMODE, self->channel, duty);
    ledc_update_duty(PWMODE, self->channel);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(esp32_servo_pwm_write_us_obj, esp32_servo_pwm_write_us);

static mp_obj_t esp32_servo_pwm_write_angle(mp_obj_t self_in, mp_obj_t _angle) {
    esp32_servo_pwm_obj_t *self = MP_OBJ_TO_PTR(self_in);

    int angle = mp_obj_get_int(_angle);
    if ((angle < 0) || (angle > self->actuation_range))
        mp_raise_ValueError(MP_ERROR_TEXT("Angle out of range"));
    int us_range = self->max_us - self->min_us;
    int us = self->min_us + (int)(angle * us_range / self->actuation_range);
    if ((us < self->min_us) || (us > self->max_us))
        mp_raise_ValueError(MP_ERROR_TEXT("Pulse width out of range"));
    int duty = (int)((us * 1023)/ 20000);

    duty &= ((1 << PWRES)-1);
    duty >>= PWRES - timer_cfg.duty_resolution;
    ledc_set_duty(PWMODE, self->channel, duty);
    ledc_update_duty(PWMODE, self->channel);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(esp32_servo_pwm_write_angle_obj, esp32_servo_pwm_write_angle);

static const mp_rom_map_elem_t esp32_servo_pwm_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&esp32_servo_pwm_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&esp32_servo_pwm_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_freq), MP_ROM_PTR(&esp32_servo_pwm_freq_obj) },
    { MP_ROM_QSTR(MP_QSTR_duty), MP_ROM_PTR(&esp32_servo_pwm_duty_obj) },
    { MP_ROM_QSTR(MP_QSTR_write_us), MP_ROM_PTR(&esp32_servo_pwm_write_us_obj) },
    { MP_ROM_QSTR(MP_QSTR_write_angle), MP_ROM_PTR(&esp32_servo_pwm_write_angle_obj) },
};
/*
static MP_DEFINE_CONST_DICT(esp32_servo_pwm_locals_dict,
    esp32_servo_pwm_locals_dict_table);

const mp_obj_type_t machine_servo_type = {
    { &mp_type_type },
    .name = MP_QSTR_LOW_SPEED_SERVO,
    .print = esp32_servo_pwm_print,
    .make_new = esp32_servo_pwm_make_new,
    .locals_dict = (mp_obj_dict_t*)&esp32_servo_pwm_locals_dict,
};
*/
static MP_DEFINE_CONST_DICT(esp32_servo_pwm_locals_dict,
    esp32_servo_pwm_locals_dict_table);
MP_DEFINE_CONST_OBJ_TYPE(
    machine_servo_type,
    MP_QSTR_Servo,
    MP_TYPE_FLAG_NONE,
    make_new, esp32_servo_pwm_make_new,
    print, esp32_servo_pwm_print,
    locals_dict, &esp32_servo_pwm_locals_dict
    );

/*
static const mp_rom_map_elem_t machine_servo_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_servo) },
    { MP_ROM_QSTR(MP_QSTR_Servo), MP_ROM_PTR(&machine_servo_type) },
};

static MP_DEFINE_CONST_DICT(machine_servo_module_globals, machine_servo_module_globals_table);

const mp_obj_module_t mp_module_servo = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&machine_servo_module_globals,
};
*/

