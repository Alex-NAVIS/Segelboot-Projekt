// ======================================================================
//  Modul: LightControl_Module.h
//  Aufgabe:
//     - Steuerung der vier Lichtrelais (A–D) für das NAVIS-System.
//     - Synchronisation mit Weboberfläche (index.html) über /setLight.
//     - Zustände werden in SensorData gespeichert, damit alle
//       Clients (Browser) denselben Status sehen.
// ======================================================================

#pragma once
#include <Arduino.h>

// ----------------------------------------------------------
// Initialisierung der GPIO-Pins & Standardzustände
// ----------------------------------------------------------
void setupLightControl();

// ----------------------------------------------------------
// Steuerung über Webserver (z. B. /setLight?device=toplicht&state=on)
// device = "toplicht" | "positionslicht" | "lichtds" | "lichtcockpit"
// state  = "on" | "off"
// ----------------------------------------------------------
void setLightState(const String& device, const String& state);

// ----------------------------------------------------------
// Synchronisiert reale GPIO-Zustände mit SensorData-Variablen
// Wird regelmäßig oder nach Änderung aufgerufen.
// ----------------------------------------------------------
void updateRelayStatus();

// ----------------------------------------------------------
// syncRelaysIfChanged()
// ----------------------------------------------------------
// Prüft, ob sich die Relay-Zustände in sensorData geändert haben.
// Schaltet nur die Relais, die eine Zustandsänderung haben.
// Dadurch werden unnötige Schaltvorgänge vermieden.
// ----------------------------------------------------------
void syncRelaysIfChanged();