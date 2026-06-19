/* =========================================================================
 * NAVIS DATENQUELLE
 * =========================================================================
 * "PC"  = Lokaler Entwicklungsserver
 * "ESP" = ESP32 / LittleFS
 * "AUTO" = Automatische Erkennung
 * =========================================================================
 */
const NAVIS_DATENQUELLE = "AUTO";

function getWeatherMode() {
    if (NAVIS_DATENQUELLE === "PC") {
        return "PC";
    }
    if (NAVIS_DATENQUELLE === "ESP") {
        return "ESP";
    }
    // AUTO
    return (
        window.location.hostname === "localhost" ||
        window.location.hostname === "127.0.0.1"
    ) ? "PC" : "ESP";
}

/* ----------------------------------------------------------------------
⛵ NAVIS 72h Offline-Wettersystem - JavaScript-Steuerung (PC & Boot)
---------------------------------------------------------------------- */
let activeWeatherData = null; // Speichert das geladene 72h-Paket im RAM
let currentHourIdx = 0;       // Speichert global die aktuell eingestellte Slider-Stunde
let currentWindLayer = null;  // Referenz auf die aktuell animierten Windfäden
let currentCurrentLayer = null; // Referenz auf die aktuell animierten Strömungsfäden

/* =========================================================================
 * NETZWERK-KOMPONENTEN: INIT NAVIS WEATHER (REVIER-DATEIAUSWAHL & BOX-SCAN)
 * =========================================================================
 * ZWECK: Initialisiert das Dropdown-Auswahlmenü ('select') für die verfügbaren
 * Wetter- und Revierdateien. Unterscheidet dabei intelligent zwischen einer
 * lokalen Entwicklungsumgebung und dem echten Bordbetrieb auf See.
 * 
 * DIE ZWEI BETRIEBSMODI IM DETAIL:
 * 
 * 1. 🖥 LOKALER PC-SERVER-MODUS (Dynamischer Ordner-Scan)
 *    - Wird aktiv, wenn der Hostname 'localhost' oder '127.0.0.1' lautet.
 *    - Ruft das Verzeichnis 'Wetter/' ab. Der lokale Server (z. B. Python) liefert 
 *      daraufhin ein HTML-Inhaltsverzeichnis (Directory Listing) zurück.
 *    - Nutzt den 'DOMParser', um das HTML asynchron zu parsen und alle Hyperlinks 
 *      ('<a>') zu extrahieren. Filterspezifisch werden nur '.json'-Dateien erfasst.
 *    - Text-Formatierung: Bereinigt die Dateinamen von Präfixen ("wind_") und 
 *      Unterstrichen, um über eine Capitalize-Schleife saubere, lesbare Menü-Einträge 
 *      (z. B. "Ostsee Kiel") zu generieren.
 * 
 * 2. ⛵ BORD-BETRIEB / ESP32-MODUS (LittleFS-Index-Auslesung)
 *    - Greift im regulären Schiffsbetrieb über das WLAN-Bordnetz des ESP32-Mikrocontrollers.
 *    - Ruft den ESP32-Endpunkt '/api/weather/list' auf. Der ESP32 scannt das LittleFS-Verzeichnis '/Wetter'
 *      dynamisch und liefert die verfügbaren Wetterdateien als JSON-Array zurück.. Ein angehängter Zeitstempel 
 *      ('?t=Date.now()') erzwingt den Abruf direkt aus dem Flash-Speicher ohne Browser-Cache.
 *    - Iteriert durch das JSON-Array und bindet die Pfade ('f.file') und Reviernamen 
 *      ('f.label') direkt als Optionen in das Auswahlmenü ein.
 * 
 * FEHLERBEHANDLUNG (ROBUSTNESS):
 * Beide Zweige besitzen separate Catch-Blöcke, die bei fehlenden Berechtigungen, 
 * Netzwerkunterbrechungen oder einer fehlerhaften Index-Datei das Dropdown-Menü 
 * mit einer klaren visuellen Rückmeldung ("Keine JSONs" / "Fehler Scan") sichern.
 * ========================================================================= */
function initNavisWeather() {
    const select = document.getElementById('weather-file-select');
    if (!select) return;
    const mode = getWeatherMode();
	if (mode === "PC") {
		console.log("⛵ NAVIS: PC-Servermodus aktiv. Scanne Ordner /Wetter/ dynamisch...");
		// Wir rufen direkt den Ordner auf. Der Python-Server antwortet mit einer HTML-Dateiliste
		fetch('Wetter/')
            .then(res => res.text())
            .then(htmlText => {
                select.innerHTML = '<option value="">-- Revier --</option>';
                // Wir nutzen ein temporäres DOM-Element, um die Links aus dem HTML des Python-Servers zu extrahieren
                const parser = new DOMParser();
                const doc = parser.parseFromString(htmlText, 'text/html');
                const links = doc.querySelectorAll('a');
                let dateiGefunden = false;

                links.forEach(link => {
                    const href = link.getAttribute('href');
                    // Filtere nur echte Wetter-JSONs heraus und ignoriere die index.json am PC
                    if (href && href.endsWith('.json') && href !== 'index.json') {
                        dateiGefunden = true;
                        // Erzeuge ein schönes Label aus dem Dateinamen (z.B. wind_ostsee_kiel.json -> Ostsee Kiel)
                        let label = href
							.replace('.json', '')
							.replace(/^wind_/, '')
							.split('_')
							.map(word => word.charAt(0).toUpperCase() + word.slice(1))
							.join(' ');
                        // Worte im Label groß schreiben (Capitalize)
                        label = label.split('_').map(word => word.charAt(0).toUpperCase() + word.slice(1)).join(' ');

                        let opt = document.createElement('option');
                        opt.value = 'Wetter/' + href; // Relativer Pfad für den PC-Server
                        opt.text = label;
                        select.add(opt);
                    }
                });

                if (!dateiGefunden) {
                    select.innerHTML = '<option value="">Keine JSONs</option>';
                }
            })
            .catch(err => {
                console.error("Fehler beim automatischen PC-Ordner-Scan:", err);
                select.innerHTML = '<option value="">Fehler Scan</option>';
            });
    } else {
        // Normaler Boot-Betrieb über das echte ESP32-LittleFS auf See (Nutzt Ihre index.json)
        fetch('/api/weather/list?t=' + Date.now())
            .then(res => {
                if (!res.ok) throw new Error("Indexdatei auf ESP32 fehlt.");
                return res.json();
            })
            .then(files => {
                select.innerHTML = '<option value="">-- Revier --</option>';
                files.forEach(f => {
                    let opt = document.createElement('option');
                    opt.value = '/Wetter/' + f.file; // Absoluter Pfad für den ESP32
                    opt.text = f.label;
                    select.add(opt);
                });
            })
            .catch(err => console.warn("Wetter-Index konnte nicht vom ESP32 geladen werden:", err));
    }
}

/* =========================================================================
 * NETZWERK-KOMPONENTEN: LOAD NEW WEATHER DATA (WETTERDATEN-LOADER & SLIDER-SET)
 * =========================================================================
 * ZWECK: Lädt eine neue, strukturierte Wetter- und GRIB-Datendatei asynchron 
 * vom Server, initialisiert die Steuerungselemente der Zeitachse (Slider) und 
 * rendert sofort den ersten verfügbaren Wetter-Frame (Zeitschritt 0).
 * 
 * CORE-FUNKTIONEN IM DETAIL:
 * 1. Sperr- & Schutzlogik (UI-Schutz):
 *    - Wird keine gültige Datei-URL übergeben, löscht 'resetWeatherLayers()' alle Layer.
 *    - Deaktiviert den Zeitschieberegler ('slider.disabled = true') während des Ladevorgangs,
 *      um unvollständige Eingaben oder Abfrage-Konflikte zu unterbinden.
 * 2. Smarter Cache-Buster (Umgebungs-Erkennung):
 *    - Prüft über das Protokoll ('file:') oder den Hostnamen, ob die App lokal läuft (PC).
 *    - Fügt im echten Bordbetrieb (WLAN/HTTP) einen zeitbasierten Parameter ('?t=Date.now()') 
 *      an, um veraltete Cache-Versionen im Browser zu umgehen und frische Borddaten zu erzwingen.
 * 3. Dynamische GUI-Skalierung (Responsive Slider):
 *    - Schreibt die JSON-Antwort nach erfolgreichem Parsing in die globale Variable 'activeWeatherData'.
 *    - Passt die physische Länge des Sliders dynamisch an die aktuelle Höhe des Navigations-
 *      Panels ('containerHeight * 0.45') an, um Layout-Brüche im Cockpit zu verhindern.
 * 4. Error-Handling & Fallback:
 *    Ein Catch-Block fängt fehlerhafte HTTP-Statuscodes oder korrupte JSON-Dateien ab, 
 *    gibt eine detaillierte Fehlermeldung aus und setzt alle Wetter-Visualisierungen sicher zurück.
 * ========================================================================= */
function loadNewWeatherData(fileUrl) {
    const slider = document.getElementById('weather-slider');
    const infoBar = document.getElementById('weather-info-bar');

    if (!fileUrl) {
        resetWeatherLayers();
        return;
    }

    if (slider) slider.disabled = true;

    // Cache-Buster nur im echten Boot-Modus (HTTP/WLAN) nutzen
    const mode = getWeatherMode();
	const fetchUrl = fileUrl + (mode === "ESP" ? '?t=' + Date.now() : '');

    fetch(fetchUrl)
        .then(res => {
            if (!res.ok) throw new Error("Wetterdatei konnte nicht geladen werden.");
            return res.json();
        })
        .then(data => {
            activeWeatherData = data; // Im RAM sichern
            if (slider) {
                slider.disabled = false;
                slider.value = 0;
            }

            // Slider-Länge dynamisch an das Panel anpassen
            const panel = document.getElementById('navis-weather-panel');
            if (panel && slider) {
                const containerHeight = panel.clientHeight;
                slider.style.width = (containerHeight * 0.45) + 'px';
            }

            if (infoBar) infoBar.style.display = 'block';

            // Ersten Frame sofort rendern
            updateWeatherFrame(0);
        })
        .catch(err => {
            console.error("Fehler beim Laden der Wetterdatei:", err);
            resetWeatherLayers();
        });
}

/* =========================================================================
 * NAVIGATIONS-FUNKTION: UPDATE WEATHER FRAME (DYNAMISCHE STRÖMUNG- & WIND-ANIMATION)
 * =========================================================================
 * ZWECK: Aktualisiert die animierten Strömungs- und Windkarten-Layer (Partikelströme) 
 * auf der Karte basierend auf dem gewählten Zeitschritt (Stunden-Index) eines GRIB-Sliders.
 * Berechnet parallel die realen Vorhersage-Uhrzeiten und stößt das Cockpit-Update an.
 * 
 * CORE-FUNKTIONEN IM DETAIL:
 * 1. Zeit- und UI-Synchronisation:
 *    - Sichert den aktuellen 'hourIdx' global für die Positions-Mausabfrage.
 *    - Schreibt die relative Vorhersagezeit (z. B. "+3 h") in das Panel.
 *    - Berechnet aus dem Basiszeitstempel ('meta.created') und dem Slider-Versatz die 
 *      reale meteorologische Uhrzeit und formatiert sie landesspezifisch ('de-DE').
 * 2. Ressourcen- und Layer-Bereinigung:
 *    - Entfernt vor dem Neuzeichnen existierende Instanzen des Wind- und Strömungslayers 
 *      ('currentWindLayer' / 'currentCurrentLayer') restlos von der Leaflet-Karte.
 * 3. Farbskalen-Generator ('generateScale'):
 *    - Erzeugt über eine lineare RGB-Farbkanalinpolation (Gradientenberechnung) 
 *    - Generiert aus wenigen Anker-Farbpunkten eine hochauflösende 60-stufige Farbskala, 
 *      die über Bit-Shifting (`<< 16`, `<< 8`) in Hex-Farbcodes für die Partikeldarstellung umgerechnet wird.
 * 4. Partikel-Animation (Leaflet Velocity Layer):
 *    - Formatiert die GRIB-Rasterdaten (U/V-Komponenten) in das standardisierte JSON-Format.
 *    - Wind-Visualisierung: Nutzt ein energetisches Farbspektrum (Blau-Grün-Gelb-Rot) bis 18 m/s.
 *    - Strömungs-Visualisierung: Nutzt ein nachtsichtschonendes, maritimes Blauspektrum (Tiefblau 
 *      bis leuchtendes Cyan) mit höherer Linienstärke (3px) und längerer Lebensdauer (300 Frames).
 * 
 * COCKPIT-KOPPLUNG:
 * Simuliert am Ende einen Klick auf die aktuelle Kartenmitte ('map.getCenter()'), um die textuelle 
 * Infobar des Cockpits sofort an den neu geladenen Zeitschritt anzupassen.
 * ========================================================================= */
function updateWeatherFrame(hourIdx) {
    hourIdx = parseInt(hourIdx, 10);
    currentHourIdx = hourIdx; // Stunde global sichern für den Maus-Lauscher

    if (!activeWeatherData || !activeWeatherData.frames || !activeWeatherData.frames[hourIdx]) return;

    const meta = activeWeatherData.metadata;
    const frame = activeWeatherData.frames[hourIdx];

    // Zeitanzeige im Panel aktualisieren (+Xh)
    const timeLabel = document.getElementById('weather-time-label');
    if (timeLabel) timeLabel.innerText = `+${hourIdx} h`;

    // Reale Uhrzeit berechnen (Basis 'created' + Slider-Stunden)
    const realTimeLabel = document.getElementById('weather-real-time');
    if (realTimeLabel && frame.time) {
        const forecastDate = new Date(frame.time);
        const datum = forecastDate.toLocaleDateString('de-DE', { weekday: 'short', day: '2-digit', month: '2-digit' });
        const zeit = forecastDate.toLocaleTimeString('de-DE', { hour: '2-digit', minute: '2-digit' });
        realTimeLabel.innerHTML = `${datum}<br>${zeit}`;
    }

    // --------------------------------------------------
    // ALTE LAYER ENTFERNEN
    // --------------------------------------------------
    if (currentWindLayer) {
        map.removeLayer(currentWindLayer);
        currentWindLayer = null;
    }
    if (currentCurrentLayer) {
        map.removeLayer(currentCurrentLayer);
        currentCurrentLayer = null;
    }

    // Checkbox-Status auslesen
    const showWind = document.getElementById('show-wind-layer')?.checked;
    const showCurrent = document.getElementById('show-current-layer')?.checked;

    // Farbskalen-Generator Hilfsfunktion
    const generateScale = (ankerFarben) => {
        const hochaufloesendeSkala = [];
        for (let i = 0; i < 60; i++) {
            const pos = (i / 59) * (ankerFarben.length - 1);
            const idx = Math.floor(pos);
            const rst = pos - idx;
            const fA = ankerFarben[idx];
            const fB = ankerFarben[idx + 1] || fA;
            const r = Math.round(fA.r + (fB.r - fA.r) * rst);
            const g = Math.round(fA.g + (fB.g - fA.g) * rst);
            const b = Math.round(fA.b + (fB.b - fA.b) * rst);
            hochaufloesendeSkala.push("#" + ((1 << 24) + (r << 16) + (g << 8) + b).toString(16).slice(1));
        }
        return hochaufloesendeSkala;
    };

    // --------------------------------------------------
    // WIND LAYER RENDERN
    // --------------------------------------------------
    if (showWind && frame.wind_u && frame.wind_v) {
        const velocityDataWind = [
            {
                "header": {
                    "refTime": meta.created, "la1": meta.la1, "lo1": meta.lo1, "la2": meta.la2, "lo2": meta.lo2,
                    "dx": meta.dx, "dy": meta.dy, "nx": meta.nx, "ny": meta.ny,
                    "parameterCategory": 2, "parameterNumber": 2, "parameterNumberName": "U-component_of_wind"
                },
                "data": frame.wind_u
            },
            {
                "header": {
                    "refTime": meta.created, "la1": meta.la1, "lo1": meta.lo1, "la2": meta.la2, "lo2": meta.lo2,
                    "dx": meta.dx, "dy": meta.dy, "nx": meta.nx, "ny": meta.ny,
                    "parameterCategory": 2, "parameterNumber": 3, "parameterNumberName": "V-component_of_wind"
                },
                "data": frame.wind_v
            }
        ];

        const windAnker = [
            { r: 0, g: 85, b: 102 }, { r: 0, g: 229, b: 255 }, { r: 0, g: 255, b: 136 },
            { r: 255, g: 170, b: 0 }, { r: 255, g: 0, b: 85 }
        ];

        currentWindLayer = L.velocityLayer({
            displayValues: false,
            data: velocityDataWind,
            maxVelocity: 18, lineWidth: 2.5, frameRate: 35,
            velocityScale: 0.006, particleAge: 80, particleMultiplier: 0.005,
            colorScale: generateScale(windAnker)
        });
        currentWindLayer.addTo(map);
    }

    // --------------------------------------------------
    // STRÖMUNG LAYER RENDERN
    // --------------------------------------------------
    if (showCurrent && frame.current_u && frame.current_v) {
        const velocityDataCurrent = [
            {
                "header": {
                    "refTime": meta.created, "la1": meta.la1, "lo1": meta.lo1, "la2": meta.la2, "lo2": meta.lo2,
                    "dx": meta.dx, "dy": meta.dy, "nx": meta.nx, "ny": meta.ny,
                    "parameterCategory": 2, "parameterNumber": 2, "parameterNumberName": "U-component_of_current"
                },
                "data": frame.current_u
            },
            {
                "header": {
                    "refTime": meta.created, "la1": meta.la1, "lo1": meta.lo1, "la2": meta.la2, "lo2": meta.lo2,
                    "dx": meta.dx, "dy": meta.dy, "nx": meta.nx, "ny": meta.ny,
                    "parameterCategory": 2, "parameterNumber": 3, "parameterNumberName": "V-component_of_current"
                },
                "data": frame.current_v
            }
        ];

		const currentAnker = [
			{ r: 0,   g: 30,  b: 50  }, // 0 kn: Sehr dunkles Tiefblau (schmilzt mit der Karte)
			{ r: 0,   g: 65,  b: 110 }, // Schwache Strömung: Ruhiges Dunkelblau
			{ r: 0,   g: 130, b: 180 }, // Mittlere Strömung: Angenehmes Ocean-Blau
			{ r: 0,   g: 195, b: 215 }, // Starke Strömung: Klares Türkis
			{ r: 0,   g: 255, b: 255 }  // Maximum: Stechendes Cyan/Weißblau
		];


        currentCurrentLayer = L.velocityLayer({
            displayValues: false,
            data: velocityDataCurrent,
            maxVelocity: 6, 
            lineWidth: 3.0, frameRate: 20,
            velocityScale: 0.01, 
            particleAge: 300, particleMultiplier: 0.003,
            colorScale: generateScale(currentAnker)
        });
        currentCurrentLayer.addTo(map);
    }

    // Trigger ein fiktives Maus-Event auf der aktuellen Kartenmitte
    if (typeof map !== 'undefined') {
		updateInfoBarAtLatLng(map.getCenter());
	}
}

/* =========================================================================
 * NAVIGATIONS-MATHEMATIK: INTERPOLATE DIRECTION (SPHÄRISCHE WINKELGLÄTTUNG)
 * =========================================================================
 * ZWECK: Berechnet die bilineare Interpolation von Richtungs- und Winkelangaben 
 * (z. B. Wellen- oder Windrichtung) für Koordinaten, die zwischen den vier 
 * Eckpunkten einer GRIB-Rasterzelle liegen.
 * 
 * DAS MATHEMATISCHE SPROUNG-PROBLEM BEI WINKELN:
 * Eine einfache lineare Interpolation führt bei Winkeln nahe dem Nord-Sprung 
 * (z. B. zwischen 350° und 10°) zu fatalen Fehlern, da der mathematische Mittelwert 
 * (350+10)/2 = 180° (Süden) statt der korrekten 0° (Norden) betragen würde.
 * 
 * LÖSUNGSVERFAHREN (VEKTORIELLE ZERLEGUNG):
 * 1. Komponenten-Zerlegung: Transformiert die vier Eckwinkel (d00 bis d11) über 
 *    'Math.sin' und 'Math.cos' in ihre rechtwinkligen X- und Y-Vektorkomponenten 
 *    im Radiantbereich ('toRad').
 * 2. Getrennte Interpolation: Berechnet die gewichteten Werte für den Sinus- 
 *    ('sinInterp') und Kosinus-Anteil ('cosInterp') separat anhand der Abstände 
 *    'xFactor' und 'yFactor' (0.0 bis 1.0) zur Zellecke.
 * 3. Winkel-Rückgewinnung: Setzt die gemittelten Komponenten mittels 'Math.atan2' 
 *    wieder zu einem resultierenden Winkel zusammen und konvertiert diesen in Grad.
 * 4. Kompass-Normierung: Der Ausdruck '(dir + 360) % 360' bereinigt negative Gradwerte 
 *    und normiert das Endergebnis stabil in den nautischen Kompasskreis von 0.0° bis 359.9°.
 * ========================================================================= */
function interpolateDirection(d00, d10, d01, d11, xFactor, yFactor) {
    const toRad = d => d * Math.PI / 180;
    const sx00 = Math.sin(toRad(d00));
    const sx10 = Math.sin(toRad(d10));
    const sx01 = Math.sin(toRad(d01));
    const sx11 = Math.sin(toRad(d11));
    const cx00 = Math.cos(toRad(d00));
    const cx10 = Math.cos(toRad(d10));
    const cx01 = Math.cos(toRad(d01));
    const cx11 = Math.cos(toRad(d11));
    const sinInterp =
        (1 - yFactor) * ((1 - xFactor) * sx00 + xFactor * sx10) +
        yFactor * ((1 - xFactor) * sx01 + xFactor * sx11);
    const cosInterp =
        (1 - yFactor) * ((1 - xFactor) * cx00 + xFactor * cx10) +
        yFactor * ((1 - xFactor) * cx01 + xFactor * cx11);
    let dir =
        Math.atan2(sinInterp, cosInterp) *
        180 / Math.PI;
    dir = (dir + 360) % 360;
    return dir;
}

/* =========================================================================
 * NAVIGATIONS-FUNKTION: COCKPIT-AUSGABE (VEKTORRECHNUNG & DOM-UPDATE)
 * =========================================================================
 * ZWECK: Berechnet aus den interpolierten meteorologischen und hydrographischen 
 * Rasterwerten die finalen, seemännischen Maßeinheiten und schreibt diese 
 * direkt in die HTML-Anzeigeelemente des Bord-Cockpits.
 * 
 * MATHEMATISCHE VEKTOR- UND EINHEITEN-KONVERTIERUNG:
 * 1. Windgeschwindigkeit: Berechnet den Betrag des Windvektors (Satz des Pythagoras) 
 *    aus den U/V-Komponenten in m/s und rechnet diesen per Faktor 1.94384 in Knoten (kn) um.
 * 2. Windrichtung: Wandelt den mathematischen Vektorwinkel via 'Math.atan2' und 
 *    '270 - angleDeg' in das klassische nautische System um (Woher kommt der Wind? 0°-360°).
 * 3. Strömungs-Vektor: Bestimmt die Stromgeschwindigkeit ebenfalls als Vektorlänge 
 *    und berechnet die Stromrichtung (Wohin setzt der Strom? Modulo-normiert auf 0°-360°).
 * 
 * UI-RENDER-PROZESS (DOM-MANIPULATION):
 * Aktualisiert dedizierte HTML-Textknoten ('innerText') mit formatierten Strings:
 * - Windstärke & Richtung (z. B. "12 kn / 240°")
 * - Wellenparameter (Höhe in 'm' auf 1 Stelle, Richtung in '°', Periode in 's' auf 1 Stelle)
 * - Strömungsdaten (Geschwindigkeit in 'kn' auf 1 Stelle, Richtung in '°')
 * - Gezeiten (Wasserstandshöhe in 'm' auf 2 Stellen genau inklusive der Tendenztext-Injektion)
 * ========================================================================= */
function updateInfoBarAtLatLng(latlng) {
    if (!activeWeatherData || !activeWeatherData.frames) return;
    
    const meta = activeWeatherData.metadata;
    const frame = activeWeatherData.frames[currentHourIdx];
    
    // 1. Bounding-Box Prüfung
    if (latlng.lat > meta.la1 || latlng.lat < meta.la2 || latlng.lng < meta.lo1 || latlng.lng > meta.lo2) {
        document.getElementById('info-wind').innerText = "-- kn / --°";
        document.getElementById('info-wave-h').innerText = "-- m";
        document.getElementById('info-wave-d').innerText = "--°";
        document.getElementById('info-wave-p').innerText = "-- s";

        document.getElementById('current-speed').innerText = "-- kn";
        document.getElementById('current-direction').innerText = "--°";
        document.getElementById('tide-height').innerText = "-- m (--)";
        return;
    }
    
    // 2. Errechne die genaue Position im Gitter als Fließkommazahl (wichtig für die Glättung!)
    const latPos = (meta.la1 - latlng.lat) / meta.dy;
    const lonPos = (latlng.lng - meta.lo1) / meta.dx;
    
    // Die vier umliegenden Ganzzahl-Indizes im Raster bestimmen (Oben links, Omen rechts, etc.)
    const x0 = Math.floor(lonPos);
    const x1 = Math.min(x0 + 1, meta.nx - 1);
    const y0 = Math.floor(latPos);
    const y1 = Math.min(y0 + 1, meta.ny - 1);
    
    // Die Gewichtungsfaktoren für die nähe zum jeweiligen Punkt (0.0 bis 1.0)
    const xFactor = lonPos - x0;
    const yFactor = latPos - y0;
    
    // Hilfsfunktion: Berechnet den linearen 1D-Array-Index im JSON
    const getIdx = (x, y) => (y * meta.nx) + x;
    
    // Lokale Hilfsfunktion für die bilineare Wasserstand-Interpolation beliebiger Frames
    const getInterpolatedTideForFrame = (targetFrame) => {
        if (!targetFrame || !targetFrame.tide_height) return 0;
        const h00 = targetFrame.tide_height[getIdx(x0, y0)] || 0;
        const h10 = targetFrame.tide_height[getIdx(x1, y0)] || 0;
        const h01 = targetFrame.tide_height[getIdx(x0, y1)] || 0;
        const h11 = targetFrame.tide_height[getIdx(x1, y1)] || 0;
        return (1 - yFactor) * ((1 - xFactor) * h00 + xFactor * h10) + yFactor * ((1 - xFactor) * h01 + xFactor * h11);
    };
    
    // 3. Bilineare Interpolation für die U-Komponente des Windes
    const u00 = frame.wind_u?.[getIdx(x0, y0)] ?? 0;
    const u10 = frame.wind_u?.[getIdx(x1, y0)] ?? 0;
    const u01 = frame.wind_u?.[getIdx(x0, y1)] ?? 0;
    const u11 = frame.wind_u?.[getIdx(x1, y1)] ?? 0;
    const u_interpolated = (1 - yFactor) * ((1 - xFactor) * u00 + xFactor * u10) + yFactor * ((1 - xFactor) * u01 + xFactor * u11);
    
    // 4. Bilineare Interpolation für die V-Komponente des Windes
    const v00 = frame.wind_v?.[getIdx(x0, y0)] ?? 0;
    const v10 = frame.wind_v?.[getIdx(x1, y0)] ?? 0;
    const v01 = frame.wind_v?.[getIdx(x0, y1)] ?? 0;
    const v11 = frame.wind_v?.[getIdx(x1, y1)] ?? 0;
    const v_interpolated = (1 - yFactor) * ((1 - xFactor) * v00 + xFactor * v10) + yFactor * ((1 - xFactor) * v01 + xFactor * v11);
    
    // 5. Bilineare Interpolation für die Wellenhöhe
    const wh00 = (frame.wave_height ? frame.wave_height[getIdx(x0, y0)] : 0) || 0;
    const wh10 = (frame.wave_height ? frame.wave_height[getIdx(x1, y0)] : 0) || 0;
    const wh01 = (frame.wave_height ? frame.wave_height[getIdx(x0, y1)] : 0) || 0;
    const wh11 = (frame.wave_height ? frame.wave_height[getIdx(x1, y1)] : 0) || 0;
    const waveH_interpolated = (1 - yFactor) * ((1 - xFactor) * wh00 + xFactor * wh10) + yFactor * ((1 - xFactor) * wh01 + xFactor * wh11);
    
    // Bilineare Interpolation Wellenrichtung
    const wd00 = (frame.wave_direction ? frame.wave_direction[getIdx(x0, y0)] : 0) || 0;
    const wd10 = (frame.wave_direction ? frame.wave_direction[getIdx(x1, y0)] : 0) || 0;
    const wd01 = (frame.wave_direction ? frame.wave_direction[getIdx(x0, y1)] : 0) || 0;
    const wd11 = (frame.wave_direction ? frame.wave_direction[getIdx(x1, y1)] : 0) || 0;

    const waveD_interpolated = interpolateDirection(
        wd00,
        wd10,
        wd01,
        wd11,
        xFactor,
        yFactor
    );

    // Bilineare Interpolation Wellenperiode
    const wp00 = (frame.wave_period ? frame.wave_period[getIdx(x0, y0)] : 0) || 0;
    const wp10 = (frame.wave_period ? frame.wave_period[getIdx(x1, y0)] : 0) || 0;
    const wp01 = (frame.wave_period ? frame.wave_period[getIdx(x0, y1)] : 0) || 0;
    const wp11 = (frame.wave_period ? frame.wave_period[getIdx(x1, y1)] : 0) || 0;

    const waveP_interpolated =
        (1 - yFactor) * ((1 - xFactor) * wp00 + xFactor * wp10) +
        yFactor * ((1 - xFactor) * wp01 + xFactor * wp11);
        
    // 7. Ausgabe ins Cockpit-Display
    // --------------------------------------------------
    // Strömung interpolieren
    // --------------------------------------------------

    const cu00 = (frame.current_u ? frame.current_u[getIdx(x0, y0)] : 0) || 0;
    const cu10 = (frame.current_u ? frame.current_u[getIdx(x1, y0)] : 0) || 0;
    const cu01 = (frame.current_u ? frame.current_u[getIdx(x0, y1)] : 0) || 0;
    const cu11 = (frame.current_u ? frame.current_u[getIdx(x1, y1)] : 0) || 0;

    const currentU =
        (1 - yFactor) * ((1 - xFactor) * cu00 + xFactor * cu10) +
        yFactor * ((1 - xFactor) * cu01 + xFactor * cu11);

    const cv00 = (frame.current_v ? frame.current_v[getIdx(x0, y0)] : 0) || 0;
    const cv10 = (frame.current_v ? frame.current_v[getIdx(x1, y0)] : 0) || 0;
    const cv01 = (frame.current_v ? frame.current_v[getIdx(x0, y1)] : 0) || 0;
    const cv11 = (frame.current_v ? frame.current_v[getIdx(x1, y1)] : 0) || 0;

    const currentV =
        (1 - yFactor) * ((1 - xFactor) * cv00 + xFactor * cv10) +
        yFactor * ((1 - xFactor) * cv01 + xFactor * cv11);

	// --------------------------------------------------
	// Wasserstand interpolieren
	// --------------------------------------------------
	const th00 = (frame.tide_height ? frame.tide_height[getIdx(x0, y0)] : 0) || 0;
	const th10 = (frame.tide_height ? frame.tide_height[getIdx(x1, y0)] : 0) || 0;
	const th01 = (frame.tide_height ? frame.tide_height[getIdx(x0, y1)] : 0) || 0;
	const th11 = (frame.tide_height ? frame.tide_height[getIdx(x1, y1)] : 0) || 0;

	const tideInterpolated =
		(1 - yFactor) * ((1 - xFactor) * th00 + xFactor * th10) +
		yFactor * ((1 - xFactor) * th01 + xFactor * th11);

	// --------------------------------------------------
	// Wasserstand Tendenz-Berechnung (Davor / Danach abgleichen)
	// --------------------------------------------------
	let tendencyText = "gleichbleibend";

	// Lokale Ermittlung des Vergleichswerts
	let compareTide = tideInterpolated;
	if (currentHourIdx > 0) {
		const prevFrame = activeWeatherData.frames[currentHourIdx - 1];
		if (prevFrame && prevFrame.tide_height) {
			const p00 = prevFrame.tide_height[getIdx(x0, y0)] || 0;
			const p10 = prevFrame.tide_height[getIdx(x1, y0)] || 0;
			const p01 = prevFrame.tide_height[getIdx(x0, y1)] || 0;
			const p11 = prevFrame.tide_height[getIdx(x1, y1)] || 0;
			compareTide = (1 - yFactor) * ((1 - xFactor) * p00 + xFactor * p10) + yFactor * ((1 - xFactor) * p01 + xFactor * p11);
			if (tideInterpolated > compareTide) tendencyText = "auflaufend";
			if (tideInterpolated < compareTide) tendencyText = "ablaufend";
		}
	} else if (activeWeatherData.frames.length > 1) {
		const nextFrame = activeWeatherData.frames[currentHourIdx + 1];
		if (nextFrame && nextFrame.tide_height) {
			const n00 = nextFrame.tide_height[getIdx(x0, y0)] || 0;
			const n10 = nextFrame.tide_height[getIdx(x1, y0)] || 0;
			const n01 = nextFrame.tide_height[getIdx(x0, y1)] || 0;
			const n11 = nextFrame.tide_height[getIdx(x1, y1)] || 0;
			compareTide = (1 - yFactor) * ((1 - xFactor) * n00 + xFactor * n10) + yFactor * ((1 - xFactor) * n01 + xFactor * n11);
			if (compareTide > tideInterpolated) tendencyText = "auflaufend";
			if (compareTide < tideInterpolated) tendencyText = "ablaufend";
		}
	}

    // --------------------------------------------------
    // Wind
    // --------------------------------------------------
    const speedMS = Math.sqrt(u_interpolated * u_interpolated + v_interpolated * v_interpolated);
    const speedKnots = Math.round(speedMS * 1.94384);
    let angleRad = Math.atan2(v_interpolated, u_interpolated);
    let angleDeg = angleRad * (180 / Math.PI);
    let windDirectionDeg = ((Math.round(270 - angleDeg) % 360) + 360) % 360;
    if (windDirectionDeg === 0 && speedKnots > 0) {
        windDirectionDeg = 360;
    }

    // --------------------------------------------------
    // Strömung
    // --------------------------------------------------
    const currentKnots = Math.sqrt(currentU * currentU + currentV * currentV);
    let currentAngle = Math.atan2(currentU, currentV) * 180 / Math.PI;
    currentAngle = ((currentAngle % 360) + 360) % 360;

    // --------------------------------------------------
    // Ausgabe
    // --------------------------------------------------
    document.getElementById('info-wind').innerText = `${speedKnots} kn / ${windDirectionDeg}°`;
    document.getElementById('info-wave-h').innerText = waveH_interpolated.toFixed(1) + " m";
    document.getElementById('info-wave-d').innerText = Math.round(waveD_interpolated) + "°";
    document.getElementById('info-wave-p').innerText = waveP_interpolated.toFixed(1) + " s";
    document.getElementById('current-speed').innerText = `${currentKnots.toFixed(1)} kn`;
    document.getElementById('current-direction').innerText = `${Math.round(currentAngle)}°`;
    document.getElementById('tide-height').innerText = `${tideInterpolated.toFixed(2)} m (${tendencyText})`;
}

/* =========================================================================
 * UI-EVENT-HANDLER: DOM CONTENT LOADED (SYSTEM-INITIALISIERUNG & MAP EVENTS)
 * =========================================================================
 * ZWECK: Zentraler Einstiegspunkt der Anwendung nach dem vollständigen Laden 
 * des HTML-Dokuments. Initialisiert die Wetterdaten-Struktur, bindet dynamische 
 * Karten-Abfragen und koppelt die Steuerelemente der Layer-Umschaltung.
 * 
 * CORE-FUNKTIONEN IM DETAIL:
 * 1. Asynchroner Start (Verzögerter Datei-Scan):
 *    Triggert 'initNavisWeather()' über ein 'setTimeout' mit 500 Millisekunden 
 *    Verzögerung. Dies entlastet den Haupt-Thread beim Seitenaufbau und stellt sicher, 
 *    dass alle DOM-Auswahlelemente (Dropdowns) stabil gerendert und ansprechbar sind.
 * 2. Echtzeit-Positionsabfrage (Maus- & Touch-Lauscher):
 *    - Validiert die sichere Existenz des Leaflet-Kartenobjekts ('map'), um Fehler zu vermeiden.
 *    - 'mousemove' & 'click': Bindet Event-Listener an die Karte, die bei jeder 
 *      Cursor-Bewegung oder Touch-Eingabe die geografischen Koordinaten ('e.latlng') abgreifen 
 *      und sofort an die GRIB-Wetterinterpolations-Funktion 'updateInfoBarAtLatLng()' übergeben.
 * 3. Interaktive Layer-Steuerung (GRIB-Neuzeichnung):
 *    - Greift auf die Checkboxen für die Wind- ('cbWind') und Strömungsanzeige ('cbCurrent') zu.
 *    - Registriert 'change'-Event-Listener auf beiden Elementen. Sobald die Crew einen Layer 
 *      ein- oder ausblendet, wird 'updateWeatherFrame()' mit dem global gesicherten Zeitschritt 
 *      ('currentHourIdx') aufgerufen, um das visuelle Kartenbild im Cockpit sofort zu aktualisieren.
 * ========================================================================= */
document.addEventListener("DOMContentLoaded", () => {
    // Wetterdateien laden
    setTimeout(initNavisWeather, 500);
    // ----------------------------------
    // Leaflet Maus- und Touchsteuerung
    // ----------------------------------
    if (typeof map !== 'undefined') {

        map.on('mousemove', (e) => {
            updateInfoBarAtLatLng(e.latlng);
        });
        map.on('click', (e) => {
            updateInfoBarAtLatLng(e.latlng);
        });
    }

    // ----------------------------------
    // Layer-Umschaltung
    // ----------------------------------
    const cbWind = document.getElementById('show-wind-layer');
    const cbCurrent = document.getElementById('show-current-layer');

    if (cbWind) {
        cbWind.addEventListener('change', () => {
            updateWeatherFrame(currentHourIdx);
        });
    }

    if (cbCurrent) {
        cbCurrent.addEventListener('change', () => {
            updateWeatherFrame(currentHourIdx);
        });
    }
	
	const weatherImport = document.getElementById( "weather-import-file");
	if (weatherImport) {
		weatherImport.addEventListener("change", uploadWeatherFile);
	}
});

/* =========================================================================
 * NAVIGATIONS-FUNKTION: RESET WEATHER LAYERS (SYSTEM-RESET & CLEANUP)
 * =========================================================================
 * ZWECK: Setzt alle meteorologischen Kartenebenen, GRIB-Datenstrukturen im Arbeitsspeicher
 * und die zugehörigen Steuerungselemente der Benutzeroberfläche vollständig zurück.
 * Dient als Sicherheitsnetz bei Ladefehlern oder zum sauberen Deaktivieren der Wetteranzeige.
 * 
 * CORE-FUNKTIONEN IM DETAIL:
 * 1. Karten-Bereinigung (Layer-Cleanup):
 *    - Überprüft die Existenz des Leaflet-Kartenobjekts ('map').
 *    - Validiert über 'map.hasLayer()', ob die Partikel-Animationen für Wind und Strömung
 *      aktiv auf der Karte gezeichnet sind, und entfernt diese sicher, um Speicherlecks zu verhindern.
 * 2. Status- und RAM-Reset:
 *    - Überschreibt die globalen Layer-Instanzen mit 'null'.
 *    - Löscht das aktive GRIB-Datenobjekt ('activeWeatherData = null') aus dem RAM,
 *      um veraltete Datenstände im nachfolgenden Systemzyklus auszuschließen.
 * 3. UI-Komponenten-Rückstellung:
 *    - Slider-Sperre: Deaktiviert den Zeitschieberegler ('disabled = true') und nullt den Wert.
 *    - Text-Reset: Setzt das relative Stunden-Label im Control-Panel zurück auf "+0h".
 *    - Sichtbarkeits-Steuerung: Blendet die Cockpit-Informationsleiste ('weather-info-bar') 
 *      vollständig aus ('display = none').
 *    - Dropdown-Reset: Setzt das Revier-Auswahlmenü auf die neutrale Startposition zurück.
 * ========================================================================= */
function resetWeatherLayers() {
	if (typeof map !== 'undefined') {
		if (currentWindLayer && map.hasLayer(currentWindLayer)) {
			map.removeLayer(currentWindLayer);
		}
		if (currentCurrentLayer && map.hasLayer(currentCurrentLayer)) {
			map.removeLayer(currentCurrentLayer);
		}
	}
    currentWindLayer = null; currentCurrentLayer = null;
    activeWeatherData = null;
    
    const slider = document.getElementById('weather-slider');
    if (slider) { slider.disabled = true; slider.value = 0; }
    
    if (document.getElementById('weather-time-label')) document.getElementById('weather-time-label').innerText = "+0h";
    if (document.getElementById('weather-info-bar')) document.getElementById('weather-info-bar').style.display = 'none';
    if (document.getElementById('weather-file-select')) document.getElementById('weather-file-select').value = "";
}

/* =========================================================================
 * UI-EVENT-HANDLER: TOGGLE WEATHER PANEL (WETTER-PANEL MINIMIEREN/MAXIMIEREN)
 * =========================================================================
 * ZWECK: Schaltet die Sichtbarkeit des seitlichen Wetter-Bedienpanels 
 * ('navis-weather-panel') per Knopfdruck um. Erlaubt es dem Skipper, das Panel 
 * bei Bedarf einzuklappen, um mehr freie Kartenfläche (Sichtfeld) zu gewinnen.
 * 
 * FUNKTIONSWEISE UND CSS-KOPPLUNG:
 * 1. Status-Abfrage: Prüft mittels 'classList.contains("collapsed")', ob das 
 *    Panel aktuell im minimierten Zustand konfiguriert ist.
 * 2. Ausklapp-Vorgang (If-Zweig): 
 *    - Entfernt die CSS-Klasse 'collapsed'. Das Panel fährt (typischerweise über 
 *      CSS-Transitions gesteuert) weich in die volle Breite aus.
 *    - Ändert das Symbol des Buttons ('btn') auf die diagonal nach innen zeigenden 
 *      Pfeile ("⤡"), um visuell die Aktion zum erneuten Verkleinern zu signalisieren.
 * 3. Einklapp-Vorgang (Else-Zweig):
 *    - Fügt die CSS-Klasse 'collapsed' hinzu, wodurch das Panel auf der Oberfläche 
 *      ausgeblendet oder an den Bildschirmrand geschoben wird.
 *    - Ändert das Button-Symbol spiegelverkehrt auf die diagonal nach außen zeigenden 
 *      Expansions-Pfeile ("⤢").
 * ========================================================================= */
function toggleWeatherPanel() {
    const panel = document.getElementById("navis-weather-panel");
    const btn = document.getElementById("weather-pin-toggle");
    if (panel.classList.contains("collapsed")) {
        panel.classList.remove("collapsed");
        btn.innerText = "⤡";
    } else {
        panel.classList.add("collapsed");
        btn.innerText = "⤢";
    }
}

function openWeatherImportDialog() {
    const input = document.getElementById("weather-import-file");
    if (!input) {
        return;
    }
    input.value = ""; // wichtig: erlaubt Re-Upload derselben Datei
    input.click();
}

async function uploadWeatherFile(event) {
    const file = event.target.files[0];
    if (!file) return;
    try {
        const text = await file.text();
        const json = JSON.parse(text);
        // ------------------------------------------------
        // NAVIS Wetterdatei prüfen
        // ------------------------------------------------
        if (!json.metadata || json.metadata.format !== "NAVIS-WEATHER-V1") {
            alert("Keine gültige NAVIS Wetterdatei.");
            return;
        }
        // ------------------------------------------------
        // Upload vorbereiten
        // ------------------------------------------------
        const uploadFile = new File(
                [file],
                "Wetter/" + file.name,
                {
                    type: "application/json"
                }
            );
        const formData = new FormData();
        formData.append("file",uploadFile,uploadFile.name);
        // ------------------------------------------------
        // Upload
        // ------------------------------------------------
        const response = await fetch(
                "/upload",
                {
                    method: "POST",
                    body: formData
                }
            );
        const result = await response.text();
        if (!response.ok) {
            throw new Error(result || "Upload fehlgeschlagen");
        }
        alert("Wetterdatei erfolgreich importiert.");
        // Dropdown aktualisieren
        initNavisWeather();
    }
    catch(error) {
        console.error(error);
        alert("Import fehlgeschlagen:\n" + error.message);
    }
    event.target.value = "";
}