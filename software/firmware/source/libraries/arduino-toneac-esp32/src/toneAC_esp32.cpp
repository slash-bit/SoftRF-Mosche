/* ---------------------------------------------------------------------------
Created by Tim Eckel - teckel@leethost.com
Copyright 2019 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html

See "toneAC.h" for purpose, syntax, version history, links, and more.
--------------------------------------------------------------------------- */

#include "toneAC_internal.h"

#if defined(ESP32)

#include <mutex>
#include <esp32-hal-cpu.h>
#include <driver/mcpwm.h>

//#define MCPWM0APIN 25
//#define MCPWM0BPIN 26
// the available pins are platform hardware dependent, need setup:
//#define MCPWM0APIN 14 - what we used on T-Beam
//#define MCPWM0BPIN 15

static hw_timer_t *_tAC_timer = NULL;
static void IRAM_ATTR onTimer();

#if 0
static std::once_flag _tAC_init;

void toneAC_init() {
  std::call_once(_tAC_init, [](){
    // Initialize MCPWM
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, toneAC_pinA);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, toneAC_pinB);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1;
    pwm_config.cmpr_a = 0.0;
    pwm_config.cmpr_b = 0.0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 0, 0);
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    // also specifically set the output pins to LOW state:
digitalWrite(toneAC_pinA, LOW);
digitalWrite(toneAC_pinB, LOW);
    //mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    //mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    // Initialize timer
    _tAC_timer = timerBegin(0, ESP.getCpuFreqMHz(), true);
    timerAttachInterrupt(_tAC_timer, &onTimer, true);
  });
}
#endif

// need to re-init every time since we reset the pins
void toneAC_init() {
    // Initialize MCPWM
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, toneAC_pinA);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, toneAC_pinB);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1;
    pwm_config.cmpr_a = 0.0;
    pwm_config.cmpr_b = 0.0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 0, 0);
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    // Initialize timer
    _tAC_timer = timerBegin(0, ESP.getCpuFreqMHz(), true);
    timerAttachInterrupt(_tAC_timer, &onTimer, true);
}

static bool playing = false;

void toneAC_playNote(unsigned long frequency, uint8_t volume)
{
  // need to re-init every time since we reset the pins
  toneAC_init();

  float duty = 100.0 / _tAC_volume[volume - 1];
  mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, frequency);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);

  playing = true;
}

void noToneAC()
{
  if (! playing)
      return;

  timerAlarmDisable(_tAC_timer);    // this CRASHES if timer was not initialized!
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

  // also specifically set the output pins to LOW state:
  gpio_reset_pin((gpio_num_t) toneAC_pinA);
  gpio_reset_pin((gpio_num_t) toneAC_pinB);
  pinMode(toneAC_pinA, OUTPUT);
  pinMode(toneAC_pinB, OUTPUT);
  digitalWrite(toneAC_pinA, LOW);
  digitalWrite(toneAC_pinB, LOW);

  playing = false;
}

void noToneAC_setTimer(unsigned long delay) {
  timerAlarmWrite(_tAC_timer, delay * 1000, false);
  timerRestart(_tAC_timer);
  timerAlarmEnable(_tAC_timer);
}

static void IRAM_ATTR onTimer() {
  noToneAC();
}

#endif
