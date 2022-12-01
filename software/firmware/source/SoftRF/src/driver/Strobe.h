/*
 * Strobe.h
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

#ifndef STROBEHELPER_H
#define STROBEHELPER_H

/* gap between flashes in a burst */
#define STROBE_MS_GAP             75

/* different strobe patterns for alarm vs. no-alarm: */

/* alarm pattern: triple flash every 750 ms, duty cycle 20% */
#define STROBE_FLASHES_ALARM       3
#define STROBE_MS_ALARM           50
#define STROBE_MS_PAUSE_ALARM  (750-STROBE_FLASHES_ALARM*(STROBE_MS_ALARM+STROBE_MS_GAP)+STROBE_MS_GAP)

/* no-alarm pattern: double flash every 2500 ms, duty cycle 4% */
#define STROBE_FLASHES_NOALARM     2
#define STROBE_MS_NOALARM         50
#define STROBE_MS_PAUSE_NOALARM (2500-STROBE_FLASHES_NOALARM*(STROBE_MS_NOALARM+STROBE_MS_GAP)+STROBE_MS_GAP)

/* even if not airborne, flash the strobe for the first xx seconds as a test */
#define STROBE_INITIAL_RUN        90

void Strobe_setup(void);
void Strobe_loop(void);
void Strobe_fini(void);

extern uint32_t StrobeSetupMarker;

#if defined(ESP32)
#define STROBEPIN SOC_GPIO_PIN_STROBE
#else
#define STROBEPIN SOC_UNUSED_PIN
#endif

#endif /* STROBEHELPER_H */
