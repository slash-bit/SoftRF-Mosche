/*
 * SoundHelper.h
 * Copyright (C) 2016-2021 Linar Yusupov
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

#define ALARM_TONE_HZ     1040
#define ALARM_TONE_MS     1000

/* 3 different sounds for different alarm levels: */
#define ALARM_TONE_HZ_LOW        2000
#define ALARM_TONE_HZ_IMPORTANT  2700
#define ALARM_TONE_HZ_URGENT     3300
#define ALARM_BEEPS_LOW        1
#define ALARM_BEEPS_IMPORTANT  2
#define ALARM_BEEPS_URGENT     5
#define ALARM_TONE_MS_LOW       700
#define ALARM_TONE_MS_IMPORTANT 250
#define ALARM_TONE_MS_URGENT    140

enum
{
	BUZZER_VOLUME_FULL,
	BUZZER_VOLUME_LOW,
	BUZZER_OFF,
	BUZZER_EXT
};

void Sound_setup(void);
void Sound_Notify(int8_t);
void Sound_loop(void);
void Sound_fini(void);

#endif /* SOUNDHELPER_H */
