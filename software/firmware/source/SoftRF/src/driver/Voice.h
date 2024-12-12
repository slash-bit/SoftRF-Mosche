/*
 * Voice.h
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

#ifndef VOICEHELPER_H
#define VOICEHELPER_H

#if !defined(EXCLUDE_VOICE)
#if defined(ESP32)

#define VOICEPIN  25   // DAC channel 1

#define VOICEMS 3000   // how long to wait before next voice notification

void Voice_setup(void);
void Voice_test(int);
bool Voice_Notify(container_t *, bool);
void Voice_loop(void);
void Voice_fini(void);

// these are in waves.cpp:
extern int num_wav_files;
void clear_waves(void);
int parse_wav_tar(void);
bool word2wav(const char *word);
int read_wav_byte(uint8_t &data);

#endif /* ESP32 */
#endif /* EXCLUDE_VOICE */
#endif /* VOICEHELPER_H */
