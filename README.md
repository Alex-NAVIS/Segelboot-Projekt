NAVIS — Unser Boot-ESP32 Projekt

Gemeinsam entwickelt. Direkt auf dem Boot. Echt.

Dieses Projekt ist unsere eigene Umsetzung eines maritimen Navigationssystems auf einem ESP32. Es verarbeitet GPS, AIS, IMU (9 Achsen: Roll, Pitch, Heading), digitale Kompasswerte, Windsensoren und weitere Bordfunktionen. Alles ist modular, wartbar und auf maximale Stabilität an Bord optimiert.

🔹 Hauptfunktionen

GPS-Navigation & Positionsverfolgung (GPS.cpp/h)

AIS-Integration für andere Schiffe

IMU-Sensor (IMU.cpp/h) für Roll, Pitch, Heading

Magnetdeklination & Kompasskorrektur (Mag_Dec.cpp/h)

Echolot/Tiefenmesser (echolot.cpp/h)

Datenlogging auf SD-Karte (NAVIS_SD.cpp/h)

Lichtsteuerung (LightControl_Module.cpp/h)

Webserver mit HTML-Seiten (WebServer_Module.cpp/h, ESP32_webserver.ino)

Zentrale Konfigurationsverwaltung (Config.cpp/h, ConfigStorage.cpp/h)

Sensor-Datenverwaltung & Fusion (Sensor_Data.cpp/h)

🔹 Hardware

ESP32 (Boot-Controller)

GPS-Modul

9-Achsen IMU (Gyroskop + Beschleunigung + Magnetometer)

WLAN AP-Modus für Weboberfläche und Konfiguration

SD-Karten-Slot für Logging

Optionale Sensoren: Echolot, Lichtsteuerung, Windsensor

🔹 Architektur

Firmware (ESP32_webserver.ino) → initiiert alle Module

Module getrennt nach Funktion: IMU, GPS, Sensoren, SD-Logging, Webserver, Lichtsteuerung

Weboberfläche → HTML/CSS/JS über LittleFS, Zugriff über ESP32 AP

Konfigurationsspeicherung → persistent im Flash (LittleFS / ConfigStorage)

Modular & erweiterbar → neue Sensoren oder Funktionen einfach hinzufügen

🔹 Software/Toolchain

C++ / Arduino IDE

ESP32 Plattform

Nutzung von LittleFS für Webinterface und Konfigurationsdaten

Serial Output & Webserver für Debugging und Fernzugriff

🔹 Insider-Tipps / Hinweise

IMU Kalibrierung: Offset für Gyroskop + Magnetometer speichern; bei Metallnähe auf dem Boot kann Heading gestört werden

Webserver: Asynchroner Zugriff ermöglicht stabile Darstellung von Sensorwerten in Echtzeit

Datenlogging: SD-Karte ist optional, aber empfehlenswert für Post-Mission Analyse

Wraparound Handling: Heading 0°/360° korrekt handhaben, um Sprünge in Navigation/Anzeige zu vermeiden

🔹 Nächste Schritte

Verbesserung der Web-UI Visualisierung (Charts, Roll/Pitch Kompassanzeige)

Erweiterung von NMEA-Output für Autopilot-Anbindung

OTA Updates für ESP32 Firmware

Weitere Sensorfusion, z.B. Wind, Echolot & GPS kombiniert
