⛵ NAVIS — Das modulare ESP32 Segelboot-Projektsystem
Gemeinsam entwickelt. Direkt auf dem Boot. Echt._ [1]
Dieses Projekt ist eine hochgradig optimierte, maritime Systemzentrale auf Basis des ESP32. Es vereint die Funktionen von Kartenplotter-Schnittstellen, GPS, AIS-Empfang, digitalem Kompass, künstlichem Horizont und Windsensorik in einer zentralen, stromspander Hardwarekomponente. [1, 2]
Das System ist von Seglern für Segler konzipiert: Vollständig modular aufgebaut, auf maximale Stabilität im harten Bordalltag getrimmt und plattformunabhängig per Webbrowser bedienbar. [1]
________________________________________
⚓ Die entscheidenden Vorteile an Bord
•	Kompakt & Stromsparend: Ersetzt mehrere teure Einzelinstrumente durch einen einzigen Mikrocontroller mit minimalem Stromverbrauch (ideal für Langfahrt unter Segel).
•	Plattformunabhängig (Responsive Web-UI): Kein spezielles Display nötig. Egal ob Smartphone, Tablet (am Steuerstand) oder Laptop (am Navigationstisch) – der Zugriff erfolgt flüssig über jeden Browser.
•	Datensicherheit durch Offline-Konzept: Das System arbeitet als autarker WLAN-Access-Point. Es ist keinerlei Internetverbindung auf See erforderlich.
•	Echtzeit-Optimierung: Kurssprünge (Wraparound bei 0°/360°) werden algorithmisch abgefangen, um absolut saubere Anzeigen und Steuerbefehle zu garantieren. [1, 2, 3]
________________________________________
🧠 Sensorik & Signalverarbeitung (Deep-Dive)
Die größte Herausforderung an Bord ist die Signalqualität bei Seegang, Krängung und Vibrationen. Das System verarbeitet die Rohdaten der Sensoren daher in mehreren Stufen (Sensor_Data.cpp/h / IMU.cpp/h):
9-Achsen-Sensorfusion & Künstlicher Horizont
•	Dämpfung von Schiffsbewegungen: Roll- und Pitch-Werte (Krängung & Trimm) neigen bei Wellengang zum "Rauschen". Durch den Einsatz von komplementären Filtern oder Kalman-Filtern werden hochfrequente Störungen (z. B. Schläge durch Wellen) herausgefiltert. Zurück bleibt die echte, geglättete Lage des Bootes im Wasser.
•	Neigungskompensierter Kompass: Ein normaler Magnetometer-Sensor wird ungenau, sobald das Boot krängt. Unser System nutzt die Beschleunigungs- und Gyroskopdaten der IMU, um den mathematischen Horizont zu berechnen. Der Kompasskurs (Heading) bleibt dadurch auch bei 30 Grad Schräglage absolut präzise.
•	Mathematisches Wraparound-Handling: Bei Kursänderungen direkt im Norden (Übergang von 359° auf 0°) berechnen Standard-Filter oft fehlerhafte Sprünge. Die mathematische Engine fängt diesen Quadrantenwechsel sauber ab, um Fehlfunktionen in der Navigation und angeschlossenen Autopiloten zu verhindern. [1]
GPS & Navigations-Vektoren
•	GPS.cpp/h: Neben den reinen Längen- und Breitengraden extrahiert das Modul die NMEA-Sätze zur Berechnung des wahren Vektors über Grund.
•	Magnetische Deklination (Mag_Dec.cpp/h): Da der geografische Nordpol nicht dem magnetischen Nordpol entspricht, rechnet dieses Modul den gemessenen Magnetkurs (Magnetic Heading) anhand hinterlegter Tabellen oder mathematischer Modelle in den wahren Kurs (True Heading) um.
________________________________________
🖥️ User-Schnittstellen & Kartenmaterial-Integration
Die Benutzeroberfläche trennt die reine Instrumentenanzeige von der taktischen Navigation auf der Karte. Das Frontend kommuniziert asynchron über WebSockets mit dem ESP32 Core. [1]
Das Live-Cockpit (Echtzeit-Instrumente)
•	WebSockets für Latenzfreiheit: Anstatt die Seite permanent neu zu laden, pusht der ESP32 die Sensordaten (Wind, Tiefe, Kurs, Krängung) mehrmals pro Sekunde als schlankes JSON-Paket an den Browser.
•	Responsive Visualisierung: HTML5-Canvas und CSS-Transformationen animieren die Anzeigen (z.B. die Kompassrose oder die Neigungsanzeige des künstlichen Horizonts) absolut flüssig und ohne Ruckeln – selbst auf älteren Tablets am Steuerstand. [1, 2]
Kartenplotter-Schnittstelle & Kartenmaterial
Das System ist im Frontend für die Darstellung von interaktiven Seekarten vorbereitet (z.B. mittels OpenLayers oder Leaflet). Es kombiniert offene und amtliche Kartendaten:
•	OpenSeaMap Integration: Die kostenfreie, weltweite Open-Source-Seekarte wird direkt als Kachel-Ebene (Tile Layer) eingebunden. Sie liefert Leuchtfeuer, Betonnung, Hafenpläne und Flachwassergebiete, die über das WLAN geladen oder für den Offline-Betrieb gecacht werden können.
•	BSH-Kartenmaterial (Bundesamt für Seeschifffahrt und Hydrographie): Für Reviere in Nord- und Ostsee unterstützt die Benutzeroberfläche die Einbindung amtlicher WMS/WMTS-Dienste oder das Rendering entsprechend konvertierter BSH-Kacheln. Dies garantiert maximale Präzision bei Tiefenlinien und offiziellen Schifffahrtswegen.
•	AIS-Overlay: Die über das AIS-Modul empfangenen Daten anderer Schiffe werden mathematisch auf die GPS-Koordinaten der Karte projiziert. Jedes Ziel wird als dynamisches Icon mit Vektor (Kurs und Geschwindigkeit) direkt über der OpenSeaMap/BSH-Karte dargestellt.
________________________________________
📁 Projektstruktur
•	ESP32_Segelboot_AP/: Der Hauptordner für die Firmware.
•	ESP32_Segelboot_AP/src/: Enthält die .cpp und .h Dateien für die logischen Module (GPS, IMU, AIS, Sensor_Data etc.).
•	ESP32_Segelboot_AP/data/: Beinhaltet die Webserver-Dateien (HTML, CSS, JS), die via LittleFS in den Flash-Speicher geladen werden.
•	README.md: Diese Dokumentationsdatei. [1, 2]
________________________________________
🔧 Installation & Setup
1. Quellcode herunterladen
Klone das Repository in dein lokales Arbeitsverzeichnis:
git clone https://github.com
2. Hardware-Setup
•	Controller: ESP32 Development Board (z. B. ESP32-WROOM-32E oder ESP32-S3).
•	GNSS: NMEA-fähiges GPS/Glonass-Modul (z. B. NEO-M8N) via serieller Schnittstelle (UART).
•	Sensorik: 9-Achsen IMU (z. B. BNO055 oder MPU9250) via I2C für Neigung und Kompass.
•	Peripherie: SD-Karten-Slot (SPI) für die Reiseaufzeichnung. [1]
3. Firmware & Webinterface flashen
•	Öffne das Projekt in deiner bevorzugten IDE (z.B. PlatformIO).
•	Nutze das Dateisystem-Tool der IDE, um den Ordner data/ mittels LittleFS auf den ESP32 zu schreiben. Dadurch werden das Webinterface und die Basis-Kartenkonfigurationen übertragen.
•	Flashe anschließend den Programmcode auf den ESP32.
________________________________________
💡 Wichtige Praxis-Hinweise für den Bordbetrieb
•	Magnetische Ablenkung (Deviationsausgleich): Boote enthalten viel Metall (Motor, Kielbolzen, Wantenspanner). Wird die IMU in der Nähe montiert, verfälscht dies das Heading. Führe nach dem Festeinbau unbedingt eine Kalibrierungsfahrt (Kreise fahren) durch, um die Offsets über das Webinterface persistent zu speichern.
•	Offline-Karten-Caching: Da auf offener See kein Mobilfunknetz verfügbar ist, nutzt das Webinterface den LocalStorage des Browsers oder Service Worker, um einmal geladene OpenSeaMap-Kacheln für den Offline-Einsatz auf dem Tablet zwischenzuspeichern. [1, 2]
________________________________________
🚀 Roadmap / Nächste Schritte
•	Ausbau des Leaflet/OpenLayers Frontends zur direkten Speicherung von Offline-Kartenpaketen (mbtiles).
•	Erweiterung des NMEA0183- / NMEA2000-Outputs zur direkten Kopplung mit externen Autopiloten.
•	Integration einer Over-The-Air (OTA) Updatefunktion, um Software-Updates kabellos am Liegeplatz einzuspielen. [1, 2]

