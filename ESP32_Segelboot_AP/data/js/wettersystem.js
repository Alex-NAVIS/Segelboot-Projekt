
/* ----------------------------------------------------------------------
⛵ NAVIS 72h Offline-Wettersystem - JavaScript-Steuerung (PC & Boot)
---------------------------------------------------------------------- */
let activeWeatherData = null; // Speichert das geladene 72h-Paket im RAM
let currentHourIdx = 0;       // Speichert global die aktuell eingestellte Slider-Stunde
let currentWindLayer = null;  // Referenz auf die aktuell animierten Windfäden
let currentCurrentLayer = null; // Referenz auf die aktuell animierten Strömungsfäden

/**
 * 1. Initialisierung beim Laden der Seite
 * Erkennt automatisch PC- oder Boot-Betrieb und scannt das Verzeichnis
 * vollautomatisch nach ALLEN verfügbaren JSON-Wetterdateien.
 */
function initNavisWeather() {
    const select = document.getElementById('weather-file-select');
    if (!select) return;

    // Erkennt, ob die Karte lokal auf dem PC-Server läuft
    const isPCServer = window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1';

    if (isPCServer) {
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
                        let label = href.replace('.json', '').replace('wind_', '').replace('_', ' ');
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
        fetch('/Wetter/index.json?t=' + Date.now())
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

/**
 * 2. Wetterdaten laden
 */
function loadNewWeatherData(fileUrl) {
    const slider = document.getElementById('weather-slider');
    const infoBar = document.getElementById('weather-info-bar');

    if (!fileUrl) {
        resetWeatherLayers();
        return;
    }

    if (slider) slider.disabled = true;

    // Cache-Buster nur im echten Boot-Modus (HTTP/WLAN) nutzen
    const isPC = window.location.protocol === 'file:' || window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1';
    const fetchUrl = fileUrl + (isPC ? '' : '?t=' + Date.now());

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
/**
 * 3. Slider-Bewegung verarbeiten & Wetter-Frames rendern (Wind & Strömung)
 */
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
/**
 * Kernfunktion: Berechnet aus einer Maus-Koordinate (Lat/Lng) den nächstgelegenen
 * Datenpunkt aus dem JSON-Raster und aktualisiert die Cockpit-Info-Bar.
 * Hochpräzise Info-Bar mit bilinearer Interpolation (Glättung zwischen den Rasterpunkten)
 */
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
    const tideInterpolated = getInterpolatedTideForFrame(frame);
	
    // --------------------------------------------------
    // Wasserstand Tendenz-Berechnung (Davor / Danach abgleichen)
    // --------------------------------------------------
    let tendencyText = "gleichbleibend";
    const tolerance = 0.005; // 0.5 cm Toleranz gegen mathematisches Rauschen

    if (currentHourIdx > 0) {
        // Normalfall und letzter Wert: Abgleich mit der Stunde davor
        const prevTide = getInterpolatedTideForFrame(activeWeatherData.frames[currentHourIdx - 1]);
        if (tideInterpolated - prevTide > tolerance) tendencyText = "auflaufend";
        else if (prevTide - tideInterpolated > tolerance) tendencyText = "ablaufend";
    } else if (activeWeatherData.frames.length > 1) {
        // Erster Wert (Index 0): Abgleich mit der Stunde danach (Stunde +1)
        const nextTide = getInterpolatedTideForFrame(activeWeatherData.frames[currentHourIdx + 1]);
        if (nextTide - tideInterpolated > tolerance) tendencyText = "auflaufend";
        else if (tideInterpolated - nextTide > tolerance) tendencyText = "ablaufend";
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



// --------------------------------------------------
// NAVIS Wetter-System Initialisierung
// --------------------------------------------------
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
});

/**
 * 4. Wettersystem komplett zurücksetzen
 */
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


function toggleWeatherPanel() {

    const panel =
        document.getElementById("navis-weather-panel");

    const btn =
        document.getElementById("weather-pin-toggle");

    if (panel.classList.contains("collapsed")) {

        panel.classList.remove("collapsed");

        btn.innerText = "⤡";

    } else {

        panel.classList.add("collapsed");

        btn.innerText = "⤢";

    }
}
