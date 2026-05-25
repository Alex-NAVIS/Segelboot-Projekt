#ifndef NMEA0183_MODULE_H
#define NMEA0183_MODULE_H

// Initialisiert die serielle RS-422 Schnittstelle für NMEA0183
void setup_nmea0183();

// Muss zyklisch in der loop() aufgerufen werden
void loop_nmea0183();

#endif
