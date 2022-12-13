/*
 * SoundHelper.h
 * Copyright (C) 2022 Moshe Braner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SOUNDHELPER_H
#define SOUNDHELPER_H

/* 3 different sounds for different alarm levels: */
#define ALARM_TONE_HZ_LOW        2700
#define ALARM_TONE_HZ_IMPORTANT  3000
#define ALARM_TONE_HZ_URGENT     3300
#define ALARM_BEEPS_LOW        1
#define ALARM_BEEPS_IMPORTANT  2
#define ALARM_BEEPS_URGENT     5
#define ALARM_TONE_MS_LOW       700
#define ALARM_TONE_MS_IMPORTANT 250
#define ALARM_TONE_MS_URGENT    140

#define BUZZ_PERIOD 224

void Sound_setup(void);
void Sound_Notify(int8_t);
void Sound_loop(void);
void Sound_fini(void);

void red_LED(bool);
void green_LED(bool);
void blue_LED(bool);
void toggle_green_LED(void);
void toggle_red_LED(void);
void LED_notify(void);

extern bool self_test_sound;

#endif /* SOUNDHELPER_H */
