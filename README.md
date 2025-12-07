 Segelboot-Projekt Zusammenfassung

Das Segelboot-Projekt (NAVIS und ALEX) ist ein umfassendes Embedded-System für Segelboote, das auf einem ESP32-Mikrocontroller basiert. Das Projekt implementiert ein maritimes Navigationssystem mit Kartendarstellung, GPS-Tracking, AIS-Integration, digitalen Kompass und Windsensorverarbeitung. Die Codebase ist modular aufgebaut mit spezialisierten Komponenten für verschiedene Sensoren und Funktionen, einschließlich Datenlogging über SD-Karte und Web-Server-Integration zur Fernüberwachung.

### Hauptfunktionspunkte
- GPS-Navigation und Positionsverfolgung
- AIS-Datenintegration für Schiffserkennung
- Digitale Kompass- und Magnetdeklinationsfunktionalität
- Echolot/Tiefenmesser-Modul
- Web-Server für Remote-Zugriff und Konfiguration
- SD-Karten-Datenlogging und Datenspeicherung
- Lichtkontroll-Modul für Beleuchtungssysteme
- IMU-Sensor-Integration (Beschleunigungsmesser/Gyroskop)
- Konfigurationsverwaltung und Sensor-Datenverwaltung

### Technologie-Stack
- Programmiersprache: C++ (75,8%), C (24,2%)
- Hardware-Plattform: ESP32 Mikrocontroller
- Hauptentwicklung: Arduino-Umgebung (ESP32_webserver.ino)
- Sensoren: GPS, IMU, Magnetometer, Tiefenmesser
- Speicher: SD-Karte für Datenspeicherung
- Kommunikation: Web-Server-Module

### Lizenz
Das Projekt verwendet die Unlicense-Lizenz, was bedeutet, dass die Software frei in die öffentliche Domäne gehört. Jeder darf den Code kopieren, ändern, veröffentlichen, nutzen, kompilieren, verkaufen oder verteilen - sowohl in Quellcode- als auch kompilierter Form, ohne Einschränkungen und ohne Haftung durch die Autoren.

