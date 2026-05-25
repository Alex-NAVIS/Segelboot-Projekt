#ifndef NETWORK_PARSER_MODULE_H
#define NETWORK_PARSER_MODULE_H

// Initialisiert den UDP-Port für die LAN/WLAN-Einspeisung
void setup_network_parser();

// Muss zyklisch in der Hauptschleife (loop) aufgerufen werden
void loop_network_parser();

#endif