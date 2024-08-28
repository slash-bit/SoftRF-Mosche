#ifndef IGC_H
#define IGC_H

#if defined(USE_SD_CARD)

void FlightLog_setup();
void openFlightLog();
bool logFlightPosition();
void closeFlightLog();
void FlightLogComment(const char *data);
void MD5_test();

extern bool FlightLogOpen;
extern char FlightLogPath[];

#endif

#endif
