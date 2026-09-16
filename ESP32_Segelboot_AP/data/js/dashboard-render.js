/* ============================================================
   NAVIS RAYMARINE-STYLE CANVAS ENGINE (TEIL 1 VON 3)
   ============================================================ */

const canvas = document.getElementById('navisCanvas');
const ctx = canvas.getContext('2d');

// Autopilot Status-Texte analog zur index.html
const AP_MODE_TEXT = { 0: "OFF", 1: "Kompass", 2: "GPS", 3: "Wind" };

// ============================================================
// DÄMPFUNGS- & ANIMATIONS-EINSTELLUNGEN
// ============================================================
const ANIMATION_SETTINGS = {
    enabled: true,       // true = weich gleitend (60 FPS), false = sprunghaft direkt
    smoothFactor: 0.1,  // Dämpfung: 0.01 (sehr träge) bis 1.0 (direkt)
};
// Interne Zustände für die flüssige Bewegung (State)
let currentRenderState = {
    kompass: 7, roll: -2, pitch: -1,
    winddir_gemessen: 63, winddir_berechnet: 133,
    windspeed_gemessen: 5.3, windspeed_berechnet: 6.3,
    gps_speed: 6.6, vmg_cse: 8.04, gps_kurs: 286,
    missweisung: 2.2, Echolot: 31.1,
    autopilot_offset: 0.0
};
// Ziel-Zustände (Hier landen die echten Daten vom ESP32)
let targetState = { ...currentRenderState };
// Globale Zwischenspeicher für strukturierte Objekte (Verhindert undefined-Crashs)
let latestRawData = { 
    gps_speed: 6.6, vmg_cse: 8.04, gps_kurs: 286,
    gps_stunde: 12, gps_minute: 0, gps_sekunde: 0,
    gps_lat: 12.6145, gps_lon: -51.0704
}; 
let autopilotCache = { mode: 0, offset: 0.0, target_lat: null };
// Einheitliche Schriftgröße für das Hauptlayout
const GL_SIZE = 32;

// Datenstruktur für die hochauflösende Wind-Trendanalyse (1 Hz)
let twdHistory = [];
const MAX_HISTORY_POINTS = 300; // 5 Minuten im 1-Sekunden-Takt = 300 Punkte
let twdDeltaStats = { min: 0, max: 0, currentGlobal: 0, deviation: 0 };
// Letzter Zeitstempel für den 1-Sekunden-Takt
let lastWindLogTime = 0;
// Konstante für die maximale optische Breite des Dreiecks (z.B. max. 15 Grad Abweichung)
const MAX_WIND_DEVIATION_LIMIT = 15; 


// ============================================================
// MATHEMATISCHE ENGINE-FUNKTIONEN (LERP)
// ============================================================

function lerp(start, end, amt) {
    return start + (end - start) * amt;
}

function lerpAngle(start, end, amt) {
    let diff = end - start;
    while (diff < -180) diff += 360;
    while (diff > 180) diff -= 360;
    return (start + diff * amt + 360) % 360;
}

/**
 * Zentrale Render-Schleife (Zeichnet permanent flüssige Zwischenschritte mit 60 FPS)
 */
function animationLoop() {
    if (ANIMATION_SETTINGS.enabled) {
        const factor = ANIMATION_SETTINGS.smoothFactor; 
        currentRenderState.kompass = lerpAngle(currentRenderState.kompass, targetState.kompass, factor);
        currentRenderState.gps_kurs = lerpAngle(currentRenderState.gps_kurs, targetState.gps_kurs, factor);
        currentRenderState.winddir_gemessen = lerpAngle(currentRenderState.winddir_gemessen, targetState.winddir_gemessen, factor);
        currentRenderState.winddir_berechnet = lerpAngle(currentRenderState.winddir_berechnet, targetState.winddir_berechnet, factor);
        currentRenderState.roll = lerp(currentRenderState.roll, targetState.roll, factor);
        currentRenderState.pitch = lerp(currentRenderState.pitch, targetState.pitch, factor);
        currentRenderState.gps_speed = lerp(currentRenderState.gps_speed, targetState.gps_speed, factor);
        currentRenderState.vmg_cse = lerp(currentRenderState.vmg_cse, targetState.vmg_cse, factor);
        currentRenderState.windspeed_gemessen = lerp(currentRenderState.windspeed_gemessen, targetState.windspeed_gemessen, factor);
        currentRenderState.windspeed_berechnet = lerp(currentRenderState.windspeed_berechnet, targetState.windspeed_berechnet, factor);
        currentRenderState.Echolot = lerp(currentRenderState.Echolot, targetState.Echolot, factor);
        currentRenderState.autopilot_offset = lerp(currentRenderState.autopilot_offset, targetState.autopilot_offset, factor);
        currentRenderState.missweisung = lerp(currentRenderState.missweisung, targetState.missweisung, factor);
    } else {
        currentRenderState = { ...targetState };
    }
    // Übergibt die beruhigten Animationswerte an das eigentliche Grafik-Raster (In Teil 2 definiert)
    drawDashboardFrame(currentRenderState, latestRawData);
    // Schleife ununterbrochen am Laufen halten
    requestAnimationFrame(animationLoop);
}
/* ============================================================
   NAVIS RAYMARINE-STYLE CANVAS ENGINE 
   ============================================================ */
// Sichert den aktuellen Zustand der Kacheln permanent im Browser-Speicher
function saveDashboardLayout() {
    localStorage.setItem('navis_left_fields', JSON.stringify(leftDynamicFields));
    localStorage.setItem('navis_right_fields', JSON.stringify(rightDynamicFields));
    localStorage.setItem('navis_center_config', JSON.stringify(centerConfig)); // <-- NEU!
    console.log("⚓ Dashboard-Layout & Zentrum-Konfiguration gespeichert.");
}

// Konfiguration für die 4 linken, frei tauschbaren Felder
// Standard-Belegung für die linke Seite (falls kein Speicher vorhanden ist)
// Die originalen Kachel-Startwerte ganz vom Beginn des Chats
const DEFAULT_LEFT_FIELDS = [
    { yStart: 0,   yText: 20,  type: "BOAT SPEED" }, // Feld 1 (Ganz oben)
    { yStart: 135, yText: 155, type: "VMG CSE" },     // Feld 2
    { yStart: 275, yText: 295, type: "AWA" },         // Feld 3
    { yStart: 415, yText: 430, type: "TWA" }          // Feld 4
];

const DEFAULT_RIGHT_FIELDS = [
    { yStart: 0,   yText: 25,  type: "°M ETW" },          // Feld rechts 1 (oben)
    { yStart: 110, yText: 135, type: "SOG / COG" },       // Feld rechts 2 (Mitte)
    { yStart: 220, yText: 245, type: "SEA TEMP / DEPTH" } // Feld rechts 3 (unten)
];

// Die originalen Zentrums-Startwerte (Farben exakt aus deinem Quellcode)
const DEFAULT_CENTER_CONFIG = {
    showGpsKurs: true,   // COG Doppeldreieck
    showWegpunkt: true,  // GKS Sollkurs-Kreis
    showApKurs: true,    // Pinnen-Positionszeiger
    showAwa: true,       // Scheinbarer Wind
    showTwa: true,       // Wahrer Wind
    showAbdrift: true,   // Gezeiten/Abdrift-Pfeil
    // DIE ORIGINALEN FARBEN AUS DEINEM CODE:
    colorGpsKurs: "#ff9900",  // Originales Fahrten-Orange für COG
    colorWegpunkt: "#ff9900", // Originales Orange für den Wegpunkt (rgba(255,153,0,0.9))
    colorApKurs: "#ff9900",   // Originales Orange für den Pinnen-Ist-Winkel
    colorAwa: "#0055ff",      // Originales sattes Blau für den AWA-Pfeil
    colorTwa: "#1a75ff"       // Originales helleres Blau für den TWA-Pfeil
};

// Auslesen aus dem Speicher beim Skriptstart
let leftDynamicFields = JSON.parse(localStorage.getItem('navis_left_fields')) || DEFAULT_LEFT_FIELDS;
let rightDynamicFields = JSON.parse(localStorage.getItem('navis_right_fields')) || DEFAULT_RIGHT_FIELDS;
let centerConfig = JSON.parse(localStorage.getItem('navis_center_config')) || DEFAULT_CENTER_CONFIG;

// Alle auswählbaren Optionen für das Dropdown-Menü
const ALL_DASHBOARD_OPTIONS = [
    "BOAT SPEED", "VMG CSE", "AWA", "TWA", "TWD",
    "°M ETW", "SOG", "COG", "DEPTH", "SEA TEMP", 
    "SOG / COG", "SEA TEMP / DEPTH"
];

function updateWindTrendAndTactics(display) {
    const now = Date.now();
    
    // Berechne die aktuelle globale Windrichtung (TWD) nach Kompass
    let currentTWD = (Number(display.kompass) + Number(display.winddir_berechnet)) % 360;
    if (currentTWD < 0) currentTWD += 360;
    
    twdDeltaStats.currentGlobal = currentTWD;

    // KORREKTUR: Jetzt JEDE SEKUNDE (1000ms) einen Datenpunkt erfassen!
    if (now - lastWindLogTime >= 1000) {
        lastWindLogTime = now;
        
        twdHistory.push(currentTWD);
        
        // Puffer auf 300 Punkte (5 Min) begrenzen
        if (twdHistory.length > MAX_HISTORY_POINTS) {
            twdHistory.shift();
        }
        
        if (twdHistory.length > 0) {
            // Mathematisch korrekte Mittelung von Winkeln via Sinus/Cosinus
            let sumSin = 0, sumCos = 0;
            twdHistory.forEach(angle => {
                let rad = angle * Math.PI / 180;
                sumSin += Math.sin(rad);
                sumCos += Math.cos(rad);
            });
            
            let avgRad = Math.atan2(sumSin, sumCos);
            let avgDeg = (avgRad * 180 / Math.PI + 360) % 360;
            
            // Ermittle die maximale Abweichung (Delta) vom 5-Minuten-Mittelwert
            let maxDev = 0;
            twdHistory.forEach(angle => {
                let diff = Math.abs(angle - avgDeg);
                if (diff > 180) diff = 360 - diff;
                if (diff > maxDev) maxDev = diff;
            });
            
            // KORREKTUR: Begrenze das maximale Delta (z.B. auf 15°), um optische Fehler zu vermeiden!
            if (maxDev > MAX_WIND_DEVIATION_LIMIT) {
                maxDev = MAX_WIND_DEVIATION_LIMIT;
            }
            
            // Werte geglättet für den Zeichner bereitstellen
            twdDeltaStats.min = (avgDeg - maxDev + 360) % 360;
            twdDeltaStats.max = (avgDeg + maxDev) % 360;
            twdDeltaStats.deviation = maxDev; // Gedämpfter Sektor
        }
    }
}

// ============================================================
// MODUL 1: MATHEMATISCHE LIVE-BERECHNUNG FÜR WEGPUNKT
// ============================================================
function calculateWaypointData(raw, apTargetLat, apTargetLon) {
    if (apTargetLat !== null && apTargetLon !== null &&
        raw.gps_lat !== undefined && raw.gps_lon !== undefined) {
        const R_EARTH_NM = 3440.065; 
        const lat1 = (raw.gps_lat * Math.PI) / 180;
        const lon1 = (raw.gps_lon * Math.PI) / 180;
        const lat2 = (apTargetLat * Math.PI) / 180;
        const lon2 = (apTargetLon * Math.PI) / 180;
        const dLat = lat2 - lat1;
        const dLon = lon2 - lon1;
        const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        raw.wpt_dist = R_EARTH_NM * c;
        const yCoord = Math.sin(dLon) * Math.cos(lat2);
        const xCoord = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);
        let bearingDeg = (Math.atan2(yCoord, xCoord) * 180) / Math.PI;
        raw.wpt_brg = (bearingDeg + 360) % 360;
    }
}

// ============================================================
// MODUL 2: LINKE SPALTE ZEICHNEN
// ============================================================
function drawLeftColumn(display, raw) {
    leftDynamicFields.forEach(field => {
        let val = "--";
        let unit = "";
        const pairSize = 20;
        switch(field.type) {
            case "BOAT SPEED":
                val = formatValue(display.gps_speed); unit = "kn";
                drawDataBlock(field.type, val, unit, 25, field.yText, GL_SIZE);
                break;
            case "VMG CSE":
                val = formatValue(display.vmg_cse, 2); unit = "kn";
                drawDataBlock(field.type, val, unit, 25, field.yText, GL_SIZE);
                break;
            case "AWA":
                val = formatValue(display.winddir_gemessen, 0); unit = "°";
                drawWindTrendArrow(170, field.yStart + 40, display.winddir_gemessen); 
                drawDataBlock(field.type, val, unit, 25, field.yText, GL_SIZE);
                break;
            case "TWA":
                val = formatValue(display.winddir_berechnet, 0); unit = "°";
                drawWindTrendArrow(170, field.yStart + 40, display.winddir_berechnet); 
                drawDataBlock(field.type, val, unit, 25, field.yText, GL_SIZE);
                break;
            case "SOG":
                val = formatValue(display.gps_speed); unit = "kn";
                drawDataBlock(field.type, val, unit, 25, field.yText, GL_SIZE);
                break;
            case "COG":
                val = formatValue(display.gps_kurs, 0); unit = "°M";
                drawDataBlock(field.type, val, unit, 25, field.yText, GL_SIZE);
                break;
            case "DEPTH":
                val = formatValue(display.Echolot, 1); unit = "m";
                drawDataBlock(field.type, val, unit, 25, field.yText, GL_SIZE);
                break;
            case "SEA TEMP":
                val = "31.1"; unit = "°C";
                drawDataBlock(field.type, val, unit, 25, field.yText, GL_SIZE);
                break;
            case "TWD":
                val = formatValue(display.winddir_berechnet, 0); unit = "°M";
                drawDataBlock(field.type, val, unit, 25, field.yText, GL_SIZE);
                break;
            case "°M ETW":
                let tStr = "--:--:--";
                if (raw.gps_stunde !== undefined) {
                    tStr = `${String(raw.gps_stunde).padStart(2,'0')}:${String(raw.gps_minute).padStart(2,'0')}:${String(raw.gps_sekunde).padStart(2,'0')}`;
                }
                drawDataBlock(field.type, tStr, "", 25, field.yText, GL_SIZE);
                break;
            case "SOG / COG": 
                drawDataBlock("SOG", formatValue(display.gps_speed), "kn", 25, field.yText, pairSize);
                drawDataBlock("COG", formatValue(display.gps_kurs, 0), "°M", 130, field.yText, pairSize);
                break;
            case "SEA TEMP / DEPTH": 
                drawDataBlock("SEA TEMP", "31.1", "°C", 25, field.yText, pairSize); 
                drawDataBlock("DEPTH", formatValue(display.Echolot, 1), "m", 130, field.yText, pairSize);
                break;
        }
    });
    // Statische Felder unten links
    drawDataBlock("TWD", formatValue(display.winddir_berechnet, 0), "°M", 25, 510, 24);
    ctx.fillStyle = "#8a96a3"; ctx.font = "13px Arial"; ctx.textAlign = "left";
    ctx.fillText(`Var: ${formatValue(display.missweisung, 1)}°W`, 25, 565);
}

// ============================================================
// MODUL 3: ZENTRUM (RUNDINSTRUMENTE & SKALEN MIT FILTERN)
// ============================================================
function drawCenterInstruments(cx, cy, radius, display, raw) {
    let tempApObj = { mode: autopilotCache.mode, offset: display.autopilot_offset };
    
    // 1. Basis-Elemente (bleiben immer sichtbar)
	drawTopRudderArc(cx, cy, radius, tempApObj, centerConfig);
	drawCompassRose(cx, cy, radius, display.kompass, centerConfig);
    drawRollColorBackground(cx, cy);
    drawRollArcGauge(cx, cy, display.roll);
    drawPitchGauge(cx, cy, display.pitch);
    drawBoatIndicator(cx, cy, display.roll, display.pitch);
    
    // 2. Abdrift-Pfeil (Nur wenn in den Einstellungen aktiviert)
    if (centerConfig.showAbdrift) {
        drawTideDriftArrow(cx, cy, display, raw);
    }
    
    // 3. Wind-Pfeile filtern und Farben übergeben
    // Wir reichen das gesamte centerConfig-Objekt an deine drawWindArrows weiter
    if (centerConfig.showAwa || centerConfig.showTwa) {
        drawWindArrows(cx, cy, radius, display, centerConfig); 
    }
}

// ============================================================
// MODUL 4: RECHTE OBERE DYNAMISCHE SPALTE
// ============================================================
function drawRightColumn(display, raw) {
    rightDynamicFields.forEach(field => {
        const pairSize = 20;
        switch(field.type) {
            case "°M ETW":
                let timeStr = "--:--:--";
                if (raw.gps_stunde !== undefined) {
                    timeStr = `${String(raw.gps_stunde).padStart(2,'0')}:${String(raw.gps_minute).padStart(2,'0')}:${String(raw.gps_sekunde).padStart(2,'0')}`;
                }
                drawDataBlock("°M ETW", timeStr, "", 825, field.yText, GL_SIZE);
                break;
            case "SOG / COG": 
                drawDataBlock("SOG", formatValue(display.gps_speed), "kn", 825, field.yText, pairSize);
                drawDataBlock("COG", formatValue(display.gps_kurs, 0), "°M", 930, field.yText, pairSize);
                break;
            case "SEA TEMP / DEPTH": 
                drawDataBlock("SEA TEMP", "31.1", "°C", 825, field.yText, pairSize); 
                drawDataBlock("DEPTH", formatValue(display.Echolot, 1), "m", 930, field.yText, pairSize);
                break;
            case "BOAT SPEED":
                drawDataBlock("BOAT SPEED", formatValue(display.gps_speed), "kn", 825, field.yText, GL_SIZE);
                break;
            case "VMG CSE":
                drawDataBlock("VMG CSE", formatValue(display.vmg_cse, 2), "kn", 825, field.yText, GL_SIZE);
                break;
            case "SOG":
                drawDataBlock("SOG", formatValue(display.gps_speed), "kn", 825, field.yText, GL_SIZE);
                break;
            case "COG":
                drawDataBlock("COG", formatValue(display.gps_kurs, 0), "°M", 825, field.yText, GL_SIZE);
                break;
            case "SEA TEMP":
                drawDataBlock("SEA TEMP", "31.1", "°C", 825, field.yText, GL_SIZE);
                break;
            case "DEPTH":
                drawDataBlock("DEPTH", formatValue(display.Echolot, 1), "m", 825, field.yText, GL_SIZE);
                break;
            case "AWA":
                drawDataBlock("AWA", formatValue(display.winddir_gemessen, 0), "°", 825, field.yText, GL_SIZE);
                break;
            case "TWA":
                drawDataBlock("TWA", formatValue(display.winddir_berechnet, 0), "°", 825, field.yText, GL_SIZE);
                break;
            case "TWD":
                drawDataBlock("TWD", formatValue(display.winddir_berechnet, 0), "°M", 825, field.yText, GL_SIZE);
                break;
        }
    });
}

// ============================================================
// MODUL 5: RECHTS UNTEN (AUTOPILOT & COORD POSITIONS)
// ============================================================
function drawRightBottom(display, raw, apTargetLat, apTargetLon) {
    let apMode = autopilotCache ? AP_MODE_TEXT[autopilotCache.mode] : "OFF";
    let hasWpt = (apTargetLat !== null && raw.wpt_dist !== undefined);
    let apValueString = apMode;
    if (autopilotCache && autopilotCache.mode > 0) {
        let offsetSign = display.autopilot_offset >= 0 ? "+" : "";
        apValueString += ` (${offsetSign}${formatValue(display.autopilot_offset, 1)}°)`;
    }
    drawDataBlock("AUTOPILOT", apValueString, "", 825, 355, 20);
    if (hasWpt) {
        ctx.fillStyle = "#8a96a3"; ctx.font = "11px Arial"; ctx.textAlign = "left";
        ctx.fillText("NAV INFO:", 825, 412);
        ctx.fillStyle = "#00ff66"; ctx.font = "bold 13px monospace";
        ctx.fillText(`DIST: ${formatValue(raw.wpt_dist, 1)} NM`, 890, 412);
        ctx.fillText(`BRG: ${Math.round(raw.wpt_brg).toString().padStart(3,'0')}°M`, 890, 428);
    }
    // Aktuelle Bootsposition
    let curLatStr = Number.isFinite(Number(raw.gps_lat)) ? `${Number(raw.gps_lat) >= 0 ? "N" : "S"} ${Math.abs(Number(raw.gps_lat)).toFixed(4)}°` : "N --°--.---'";
    let curLonStr = Number.isFinite(Number(raw.gps_lon)) ? `${Number(raw.gps_lon) >= 0 ? "E" : "W"} ${Math.abs(Number(raw.gps_lon)).toFixed(4)}°` : "E --°--.---'";
    ctx.fillStyle = "#7f8c8d"; ctx.font = "bold 11px Arial"; ctx.textAlign = "left";
    ctx.fillText("BOAT POS", 825, 465);
    ctx.fillStyle = "#cbd5e1"; ctx.font = "14px monospace";
    ctx.fillText(curLatStr, 825, 482);
    ctx.fillText(curLonStr, 825, 497);
    // Wegpunkt-Zielkoordinaten
    let wptLatStr = (apTargetLat !== null && Number.isFinite(Number(apTargetLat))) ? `${Number(apTargetLat) >= 0 ? "N" : "S"} ${Math.abs(Number(apTargetLat)).toFixed(4)}°` : "N --°--.---'";
    let wptLonStr = (apTargetLon !== null && Number.isFinite(Number(apTargetLon))) ? `${Number(apTargetLon) >= 0 ? "E" : "W"} ${Math.abs(Number(apTargetLon)).toFixed(4)}°` : "E --°--.---'";
    ctx.fillStyle = "#7f8c8d"; ctx.font = "bold 11px Arial"; ctx.textAlign = "left";
    ctx.fillText("WPT POS", 825, 520);
    ctx.fillStyle = "#cbd5e1"; ctx.font = "14px monospace";
    ctx.fillText(wptLatStr, 825, 537);
    ctx.fillText(wptLonStr, 825, 552);
}

// ============================================================
// HAUPTFUNKTION: GENERIERT DAS GESAMTE DASHBOARD
// ============================================================
function drawDashboardFrame(display, raw) {
	updateWindTrendAndTactics(display);
    // 1. Hintergrund leeren & einfärben
    ctx.fillStyle = "#0c0e12";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    
    // 2. Gitterlinien / Trennlinien im originalen Dunkelgrau zeichnen
    ctx.strokeStyle = "#1f2530";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(220, 0); ctx.lineTo(220, canvas.height);
    ctx.moveTo(804, 0); ctx.lineTo(804, canvas.height);
    ctx.moveTo(0, 135); ctx.lineTo(220, 135);
    ctx.moveTo(0, 275); ctx.lineTo(220, 275);
    ctx.moveTo(0, 415); ctx.lineTo(220, 415);
    ctx.moveTo(804, 110); ctx.lineTo(1024, 110);
    ctx.moveTo(804, 220); ctx.lineTo(1024, 220);
    ctx.moveTo(804, 330); ctx.lineTo(1024, 330);
    ctx.moveTo(804, 440); ctx.lineTo(1024, 440);
    ctx.stroke();
    // Fixpunkte für die Mitte ermitteln
    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    const radius = 210;
    let apTargetLat = autopilotCache ? autopilotCache.target_lat : null;
    let apTargetLon = autopilotCache ? autopilotCache.target_lon : null;

    // 3. Aufruf der modularisierten Teilbereiche
    calculateWaypointData(raw, apTargetLat, apTargetLon);
    drawLeftColumn(display, raw);
    drawCenterInstruments(cx, cy, radius, display, raw);
    drawRightColumn(display, raw);
    drawRightBottom(display, raw, apTargetLat, apTargetLon);
}

// ============================================================
// TOUCH-OPTIMIERTES AUSWAHLMENÜ FÜR DAS BOOT
// ============================================================
let touchOverlay = document.getElementById('canvasTouchOverlay');
if (!touchOverlay) {
    // 1. Haupt-Container für das abgedunkelte Overlay im Hintergrund erstellen
    touchOverlay = document.createElement('div');
    touchOverlay.id = 'canvasTouchOverlay';
    touchOverlay.style.position = 'fixed';
    touchOverlay.style.top = '0';
    touchOverlay.style.left = '0';
    touchOverlay.style.width = '100vw';
    touchOverlay.style.height = '100vh';
    touchOverlay.style.background = 'rgba(0, 0, 0, 0.8)'; // Hintergrund abdunkeln
    touchOverlay.style.zIndex = '9999';
    touchOverlay.style.display = 'none';
    touchOverlay.style.justifyContent = 'center';
    touchOverlay.style.alignItems = 'center';

    // 2. Das zentrierte Menü-Fenster (Content-Box) erstellen
    let menuBox = document.createElement('div');
    menuBox.style.background = '#141923';
    menuBox.style.border = '2px solid #2a3547';
    menuBox.style.borderRadius = '12px';
    menuBox.style.padding = '25px';
    menuBox.style.width = '90%';
    menuBox.style.maxWidth = '600px';
    menuBox.style.boxShadow = '0 10px 30px rgba(0,0,0,0.5)';
    // Titel für das Menü
    let title = document.createElement('h3');
    title.id = 'touchMenuTitle';
    title.style.margin = '0 0 20px 0';
    title.style.color = '#ff9900'; // Raymarine Orange
    title.style.fontFamily = 'Arial, sans-serif';
    title.style.fontSize = '22px';
    title.style.textAlign = 'center';
    menuBox.appendChild(title);

    // 3. Das Touch-Grid für die großen Buttons erstellen
    let grid = document.createElement('div');
    grid.style.display = 'grid';
    grid.style.gridTemplateColumns = 'repeat(2, 1fr)'; // 2 Spalten nebeneinander
    grid.style.gap = '15px';                           // Großer Abstand zwischen Buttons
    grid.style.marginBottom = '20px';
    // Alle verfügbaren Anzeige-Optionen als riesige Buttons einbauen
    ALL_DASHBOARD_OPTIONS.forEach(opt => {
        let btn = document.createElement('button');
        btn.textContent = opt;
        btn.style.background = '#1e2530';
        btn.style.color = '#ffffff';
        btn.style.border = '1px solid #4e5a6b';
        btn.style.borderRadius = '8px';
        btn.style.padding = '20px 10px';              // Massig Touch-Fläche (Y-Achse)
        btn.style.fontSize = '18px';                   // Gut lesbare, große Schrift
        btn.style.fontWeight = 'bold';
        btn.style.fontFamily = 'Arial, sans-serif';
        btn.style.cursor = 'pointer';
        btn.style.transition = 'background 0.2s';
        // Feedback beim Drücken (Aktiv-Zustand per Touch)
        btn.addEventListener('touchstart', () => btn.style.background = '#ff9900');
        btn.addEventListener('touchend', () => btn.style.background = '#1e2530');
        // Klick/Touch-Event zur Auswahl
        btn.addEventListener('click', function() {
            if (touchOverlay.activeField) {
                touchOverlay.activeField.type = opt;
                saveDashboardLayout(); // Direkt im LocalStorage sichern
                touchOverlay.style.display = 'none';
            }
        });

        grid.appendChild(btn);
    });
    menuBox.appendChild(grid);

    // 4. Einen großen Schließen-Button unten hinzufügen
    let closeBtn = document.createElement('button');
    closeBtn.textContent = "Abbrechen";
    closeBtn.style.width = '100%';
    closeBtn.style.background = '#3a4454';
    closeBtn.style.color = '#cbd5e1';
    closeBtn.style.border = 'none';
    closeBtn.style.borderRadius = '8px';
    closeBtn.style.padding = '15px';
    closeBtn.style.fontSize = '16px';
    closeBtn.style.fontFamily = 'Arial, sans-serif';
    closeBtn.addEventListener('click', () => touchOverlay.style.display = 'none');
    menuBox.appendChild(closeBtn);
    touchOverlay.appendChild(menuBox);
    document.body.appendChild(touchOverlay);

    // Schließen, wenn man außerhalb der Box in den dunklen Bereich tippt
    touchOverlay.addEventListener('click', function(e) {
        if (e.target === touchOverlay) touchOverlay.style.display = 'none';
    });
}

// ============================================================
// RECHTS- UND LINKS-AUSWERTUNG AUF DEM CANVAS (ÖFFNET DAS OVERLAY)
// ============================================================
// ============================================================
// ERWEITERTE KLICK-AUSWERTUNG AUF DEM CANVAS (LINKS, RECHTS & MITTE)
// ============================================================
canvas.addEventListener('click', function(event) {
    const rect = canvas.getBoundingClientRect();
    const clickX = event.clientX - rect.left;
    const clickY = event.clientY - rect.top;
    let overlay = document.getElementById('canvasTouchOverlay');
    if (!overlay) return;

    // --- 1. AUSWERTUNG LINKE SPALTE ---
    if (clickX >= 0 && clickX <= 220 && clickY >= 0 && clickY < 490) {
        let hitField = leftDynamicFields.find((f, index) => {
            let nextY = leftDynamicFields[index + 1] ? leftDynamicFields[index + 1].yStart : 490;
            return clickY >= f.yStart && clickY < nextY;
        });
        if (hitField) {
            // Zeige das Kachel-Raster und verstecke das Zentrums-Menü (falls es existiert)
            showOverlayContainer('grid');
            document.getElementById('touchMenuTitle').textContent = `Feld links ändern (${hitField.type})`;
            overlay.activeField = hitField; 
            overlay.style.display = 'flex';  
            return;
        }
    }

    // --- 2. AUSWERTUNG RECHTE SPALTE ---
    if (clickX >= 804 && clickX <= canvas.width && clickY >= 0 && clickY < 330) {
        let hitField = rightDynamicFields.find((f, index) => {
            let nextY = rightDynamicFields[index + 1] ? rightDynamicFields[index + 1].yStart : 330;
            return clickY >= f.yStart && clickY < nextY;
        });
        if (hitField) {
            // Zeige das Kachel-Raster und verstecke das Zentrums-Menü (falls es existiert)
            showOverlayContainer('grid');
            document.getElementById('touchMenuTitle').textContent = `Feld rechts ändern (${hitField.type})`;
            overlay.activeField = hitField; 
            overlay.style.display = 'flex';  
            return;
        }
    }

    // --- 3. NEU: AUSWERTUNG MITTELBLOCK / ZENTRUM (KOMPASS) ---
    if (clickX > 220 && clickX < 804) {
        // Erzeuge das Zentrums-Menü, falls es noch nicht in der Box gebaut wurde
        initCenterSettingsMenu(overlay);
        
        // Zeige das Zentrums-Menü und verstecke das Kachel-Raster
        showOverlayContainer('center');
        overlay.style.display = 'flex';
        return;
    }
});

// ============================================================
// HILFSFUNKTIONEN FÜR DIE DYNAMISCHE MENÜ-WEICHE
// ============================================================
// Wechselt die Ansicht im Overlay fliegend hin und her
function showOverlayContainer(mode) {
    let gridContainer = document.getElementById('touchGridContainer');
    let centerContainer = document.getElementById('touchCenterContainer');
    let title = document.getElementById('touchMenuTitle');
    let closeBtn = document.getElementById('touchCloseBtn');

    if (mode === 'grid') {
        if (gridContainer) gridContainer.style.display = 'grid';
        if (centerContainer) centerContainer.style.display = 'none';
        if (title) title.style.display = 'block';
        if (closeBtn) closeBtn.textContent = "Abbrechen";
    } else if (mode === 'center') {
        if (gridContainer) gridContainer.style.display = 'none';
        if (centerContainer) centerContainer.style.display = 'flex';
        if (title) title.style.display = 'none'; // Zentrum nutzt eigenen Titel
        if (closeBtn) closeBtn.textContent = "Fertig / Schließen";
    }
}

// Baut das Einstellungsmenü für die Mitte einmalig in das bestehende Overlay ein
// Baut das Einstellungsmenü für die Mitte einmalig in das bestehende Overlay ein
function initCenterSettingsMenu(overlay) {
    if (document.getElementById('touchCenterContainer')) return; // Bereits gebaut!

    let menuBox = overlay.querySelector('div');
    let closeBtn = menuBox.lastChild; // Den Abbrechen-Button finden (wird als ID markiert)
    closeBtn.id = "touchCloseBtn";

    // Haupt-Container für die Zentrums-Einstellungen
    let centerContainer = document.createElement('div');
    centerContainer.id = 'touchCenterContainer';
    centerContainer.style.display = 'none';
    centerContainer.style.flexDirection = 'column';
    centerContainer.style.width = '100%';

    let cTitle = document.createElement('h3');
    cTitle.textContent = "⚙️ Zentrum Instrumenten-Filter";
    cTitle.style.margin = '0 0 20px 0';
    cTitle.style.color = '#ff9900';
    cTitle.style.fontFamily = 'Arial, sans-serif';
    cTitle.style.fontSize = '22px';
    cTitle.style.textAlign = 'center';
    centerContainer.appendChild(cTitle);

    let listContainer = document.createElement('div');
    listContainer.style.display = 'flex';
    listContainer.style.flexDirection = 'column';
    listContainer.style.gap = '12px';
    listContainer.style.marginBottom = '20px';

    const items = [
        { key: "showGpsKurs", label: "GPS Kurs (COG) anzeigen", hasColor: true, colorKey: "colorGpsKurs" },
        { key: "showWegpunkt", label: "Wegpunkt-Zeiger anzeigen", hasColor: true, colorKey: "colorWegpunkt" },
        { key: "showApKurs", label: "Autopilot Sollkurs-Dreieck anzeigen", hasColor: true, colorKey: "colorApKurs" },
        { key: "showAwa", label: "Scheinbarer Wind (AWA) Pfeil", hasColor: true, colorKey: "colorAwa" },
        { key: "showTwa", label: "Wahrer Wind (TWA) Pfeil", hasColor: true, colorKey: "colorTwa" },
        { key: "showAbdrift", label: "Abdrift (Tide/Drift) anzeigen", hasColor: false }
    ];

    items.forEach(item => {
        let row = document.createElement('div');
        row.style.display = 'flex';
        row.style.justifyContent = 'space-between';
        row.style.alignItems = 'center';
        row.style.background = '#1e2530';
        row.style.padding = '10px 15px';
        row.style.borderRadius = '8px';
        row.style.border = '1px solid #4e5a6b';

        let label = document.createElement('label');
        label.style.color = '#ffffff';
        label.style.fontFamily = 'Arial, sans-serif';
        label.style.fontSize = '16px';
        label.style.display = 'flex';
        label.style.alignItems = 'center';
        label.style.gap = '15px';
        label.style.cursor = 'pointer';
        label.style.flexGrow = '1';

        let checkbox = document.createElement('input');
        checkbox.type = 'checkbox';
        checkbox.id = 'cb_' + item.key; // <-- KORREKTUR: ID ZUGEWIESEN
        checkbox.style.transform = 'scale(1.5)';
        checkbox.checked = centerConfig[item.key];
        
        checkbox.addEventListener('change', () => {
            centerConfig[item.key] = checkbox.checked;
            saveDashboardLayout();
        });

        label.appendChild(checkbox);
        label.appendChild(document.createTextNode(item.label));
        row.appendChild(label);

        if (item.hasColor) {
            let colorInput = document.createElement('input');
            colorInput.type = 'color';
            colorInput.id = 'cp_' + item.colorKey; // <-- KORREKTUR: ID ZUGEWIESEN
            colorInput.value = centerConfig[item.colorKey];
            colorInput.style.width = '45px';
            colorInput.style.height = '32px';
            colorInput.style.border = 'none';
            colorInput.style.borderRadius = '4px';
            colorInput.style.cursor = 'pointer';

            colorInput.addEventListener('change', () => {
                centerConfig[item.colorKey] = colorInput.value;
                saveDashboardLayout();
            });
            row.appendChild(colorInput);
        }

        listContainer.appendChild(row);
    });

    centerContainer.appendChild(listContainer);

    let resetBtn = document.createElement('button');
    resetBtn.textContent = "🔄 Standardwerte wiederherstellen";
    resetBtn.style.width = '100%';
    resetBtn.style.background = '#2a3547'; // Dezentes Dunkelgrau/Blau
    resetBtn.style.color = '#ff9900';     // Raymarine Orange für den Text
    resetBtn.style.border = '1px solid #4e5a6b';
    resetBtn.style.borderRadius = '8px';
    resetBtn.style.padding = '12px';
    resetBtn.style.fontSize = '15px';
    resetBtn.style.fontWeight = 'bold';
    resetBtn.style.fontFamily = 'Arial, sans-serif';
    resetBtn.style.marginBottom = '12px';  // Abstand zum Schließen-Button
    resetBtn.style.cursor = 'pointer';

    resetBtn.addEventListener('click', () => {
        if (confirm("Möchtest du alle Zentrum-Einstellungen auf die Startwerte zurücksetzen?")) {
            // 1. centerConfig mit den originalen Standardwerten überschreiben
            centerConfig = Object.assign({}, DEFAULT_CENTER_CONFIG);
            
            // 2. Sofort im LocalStorage sichern
            saveDashboardLayout();
            
            // 3. Die HTML-Elemente im aktuell offenen Menü sofort live aktualisieren
            items.forEach(item => {
                let cb = document.getElementById('cb_' + item.key);
                if (cb) cb.checked = centerConfig[item.key];
                
                if (item.hasColor) {
                    let cp = document.getElementById('cp_' + item.colorKey);
                    if (cp) cp.value = centerConfig[item.colorKey];
                }
            });
            
            console.log("⚓ Zentrum-Filter auf Startwerte zurückgesetzt.");
        }
    });
    centerContainer.appendChild(resetBtn);
    
    // Sortierung im DOM: Füge das Zentrums-Menü (inkl. Reset-Button) vor dem Schließen-Button ein
    menuBox.insertBefore(centerContainer, closeBtn);

    // Dem Kachelraster (Grid) nachträglich eine ID verpassen, damit wir es steuern können
    let existingGrid = menuBox.querySelector('div');
    if (existingGrid && existingGrid !== centerContainer) {
        existingGrid.id = 'touchGridContainer';
    }
}



// ============================================================
// MODULARE GRAPHISCHE ZEICHENFUNKTIONEN
// ============================================================
/**
 * Zeichnet den oberen Ruderlagenbogen rein basierend auf den Hardware-Istwerten.
 * Die Pfeile leuchten permanent und direkt, solange der Motor laut ESP32 Strom erhält.
 */
function drawTopRudderArc(cx, cy, r, ap, config) {
    // Falls die Funktion ohne Menü aufgerufen wird (Sicherheits-Fallback), nutzen wir centerConfig
    if (!config) config = centerConfig;

    const arcRadius = r + 16; 
    const arcWidth = 14; 
    
    // Pure Istwerte aus dem ESP32-JSON extrahieren
    const mode = Number(ap.mode) || 0;
    const offset = Number(ap.offset) || 0;
    const pinneState = Number(ap.pinne) || 0; // 0=Stop, 1=Einfahren, 2=Ausfahren

    ctx.save();
    ctx.translate(cx, cy);

    // 1. Farbband Backbord (Rot)
    let gradLeft = ctx.createLinearGradient(-arcRadius, -arcRadius, 0, -arcRadius);
    gradLeft.addColorStop(0, "rgba(231, 76, 60, 0.6)");  
    gradLeft.addColorStop(1, "rgba(231, 76, 60, 0.05)"); 
    ctx.strokeStyle = gradLeft; ctx.lineWidth = arcWidth; ctx.beginPath();
    ctx.arc(0, 0, arcRadius, 210 * Math.PI / 180, 270 * Math.PI / 180); ctx.stroke();

    // 2. Farbband Steuerbord (Grün)
    let gradRight = ctx.createLinearGradient(0, -arcRadius, arcRadius, -arcRadius);
    gradRight.addColorStop(0, "rgba(38, 166, 91, 0.05)"); 
    gradRight.addColorStop(1, "rgba(38, 166, 91, 0.6)");  
    ctx.strokeStyle = gradRight; ctx.lineWidth = arcWidth; ctx.beginPath();
    ctx.arc(0, 0, arcRadius, 270 * Math.PI / 180, 330 * Math.PI / 180); ctx.stroke();

    // 3. Umlaufende Skalenstriche (360°)
    for (let deg = 0; deg < 360; deg += 10) {
        ctx.save();
        ctx.rotate(deg * Math.PI / 180);
        let isInArcZone = (deg >= 210 && deg <= 330);
        if (deg === 270) {
            ctx.fillStyle = "#ffffff"; ctx.fillRect(-1.5, -arcRadius - (arcWidth/2), 3, arcWidth); 
        } else if (deg % 30 === 0) {
            ctx.fillStyle = "#cbd5e1"; ctx.fillRect(-1, -arcRadius - (isInArcZone ? arcWidth/2 : 4), 2, isInArcZone ? arcWidth - 2 : 8);
        } else {
            ctx.fillStyle = "rgba(255,255,255,0.25)"; ctx.fillRect(-0.5, -arcRadius - (isInArcZone ? arcWidth/4 : 2), 1, isInArcZone ? arcWidth / 2 : 5);
        }
        ctx.restore();
    }

    // 4. Gehäuse-Außenkreis
    ctx.strokeStyle = "#4e5a6b"; ctx.lineWidth = 1.5; ctx.beginPath();
    ctx.arc(0, 0, arcRadius + (arcWidth / 2), 0, 2 * Math.PI); ctx.stroke();

    // 5. Status-Text des Autopiloten
    const arcY = -arcRadius - (arcWidth / 2);
    ctx.fillStyle = mode > 0 ? "#38bdf8" : "#8a96a3";
    ctx.font = "bold 13px Arial";
    ctx.textAlign = "center";
    ctx.textBaseline = "bottom";
    ctx.fillText(`AP: ${AP_MODE_TEXT[mode] || "OFF"}`, 0, arcY - 24);

    // ============================================================
    // REINE ISTWERT-ANZEIGE: MOTOR-AKTIVITÄT (OHNE BLINKEN)
    // ============================================================
    const arrowY = arcY - 14;
    const arrowSize = 6;

    // Linker Pfeil leuchtet permanent, solange pinneState exakt 1 ist (Einfahren)
    ctx.fillStyle = (pinneState === 1) ? "#ff9900" : "#2d3748";
    ctx.beginPath();
    ctx.moveTo(-35, arrowY);
    ctx.lineTo(-25, arrowY - arrowSize);
    ctx.lineTo(-25, arrowY + arrowSize);
    ctx.closePath();
    ctx.fill();

    // Rechter Pfeil leuchtet permanent, solange pinneState exakt 2 ist (Ausfahren)
    ctx.fillStyle = (pinneState === 2) ? "#ff9900" : "#2d3748";
    ctx.beginPath();
    ctx.moveTo(35, arrowY);
    ctx.lineTo(25, arrowY - arrowSize);
    ctx.lineTo(25, arrowY + arrowSize);
    ctx.closePath();
    ctx.fill();

    // ============================================================
    // DYNAMISCH FILTERBARE ANZEIGE: POSITION / WINKEL DER PINNE
    // ============================================================
    // Der gesamte Zeiger wird ausgeblendet, wenn im Touch-Overlay deaktiviert
    if (config.showApKurs) {
        ctx.save();
        // Zeigt den exakten, aktuellen physikalischen Ist-Winkel der Pinnenstellung
        ctx.rotate(offset * Math.PI / 180);

        const outerPush = 6;
        const arrowTipY = arcY - outerPush;

        // KORREKTUR: Nutzt jetzt dynamisch deine gewählte Wunschfarbe (config.colorApKurs) statt festem Orange!
        ctx.fillStyle = config.colorApKurs; 
        ctx.beginPath();
        ctx.moveTo(0, arrowTipY); 
        ctx.lineTo(-6, arrowTipY + arcWidth + 4); 
        ctx.lineTo(6, arrowTipY + arcWidth + 4);  
        ctx.closePath(); 
        ctx.fill();

        // Schwarzer Kernpunkt im Zeiger für besseren Kontrast
        ctx.fillStyle = "#0c0e12";
        ctx.beginPath();
        ctx.arc(0, arrowTipY + (arcWidth / 2) + 2, 2, 0, 2 * Math.PI);
        ctx.fill();

        ctx.restore(); 
    }

    ctx.restore(); 
}

// ============================================================
// MODULARE UNTERFUNKTIONEN FÜR DIE KOMPASSROSE
// ============================================================

/**
 * 1. Zeichnet die gestrichelten Layline-Steuerlinien (Upwind/Downwind) zum wahren Wind
 */
function drawCompassLaylines(r, heading) {
    // Falls noch keine Historie da ist, nutzen wir ein kleines Standarddelta von 4 Grad für die Optik
    let dev = twdDeltaStats.deviation || 4; 
    
    // Optimaler Am-Wind-Winkel deines Bootes (z.B. 45° zum wahren Wind)
    const targetTwa = 45; 

    // Berechnung der theoretischen, globalen Steuerkurse (Soll-Kurse nach Kompass)
    let headingStarboard = (twdDeltaStats.currentGlobal - targetTwa + 360) % 360;
    let headingPort = (twdDeltaStats.currentGlobal + targetTwa) % 360;

    // Radius-Punkte definieren
    const innerR = r - 15; // Basis des Dreiecks
    const outerR = r;      // Spitze an der Skala
    const symW = 6;        // Halbe Grundbreite des Dreieckskeils

    // --- 1. STEUERBORDBUG (Rote Layline, Dreieck & transparentes Kuchenstück) ---
    ctx.save();
    ctx.rotate(headingStarboard * Math.PI / 180);
    
    // NEU: Die transparente Gesamtfläche (Kuchenstück inklusive Schwankung) zum Mittelpunkt zeichnen
    ctx.fillStyle = "rgba(231, 76, 60, 0.06)"; // Sehr sanftes, transparentes Rot für die Fläche
    ctx.beginPath();
    ctx.moveTo(0, 0); // Start im Zentrum
    ctx.lineTo(-symW - (dev * 0.5), -innerR); // Zur linken Ecke der Basis
    ctx.lineTo(0, -outerR);                    // Zur äußeren Spitze an der Skala
    ctx.lineTo(symW + (dev * 0.5), -innerR);   // Zur rechten Ecke der Basis
    ctx.closePath();
    ctx.fill();

    // NEU: Feine transparente Begrenzungslinien von den äußeren Dreiecksecken zum Mittelpunkt
    ctx.strokeStyle = "rgba(231, 76, 60, 0.25)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, 0); ctx.lineTo(-symW - (dev * 0.5), -innerR);
    ctx.moveTo(0, 0); ctx.lineTo(symW + (dev * 0.5), -innerR);
    ctx.stroke();

    // GESTRICHELTE HAUPTLINIE (In der Mitte des Sektors)
    ctx.strokeStyle = "#e74c3c";
    ctx.lineWidth = 1.5;
    ctx.setLineDash([6, 4]); 
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, -innerR);
    ctx.stroke();
    
    // SCHMALES SCHWANKUNGS-DREIECK (An der Skalenspitze)
    ctx.setLineDash([]); 
    ctx.fillStyle = "rgba(231, 76, 60, 0.4)"; 
    ctx.strokeStyle = "#e74c3c";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, -outerR); 
    ctx.lineTo(-symW - (dev * 0.5), -innerR); 
    ctx.lineTo(symW + (dev * 0.5), -innerR);  
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();


    // --- 2. BACKBORDBUG (Grüne Layline, Dreieck & transparentes Kuchenstück) ---
    ctx.save();
    ctx.rotate(headingPort * Math.PI / 180);
    
    // NEU: Die transparente Gesamtfläche (Kuchenstück inklusive Schwankung) zum Mittelpunkt zeichnen
    ctx.fillStyle = "rgba(38, 166, 91, 0.06)"; // Sehr sanftes, transparentes Grün für die Fläche
    ctx.beginPath();
    ctx.moveTo(0, 0); // Start im Zentrum
    ctx.lineTo(-symW - (dev * 0.5), -innerR); // Zur linken Ecke der Basis
    ctx.lineTo(0, -outerR);                    // Zur äußeren Spitze an der Skala
    ctx.lineTo(symW + (dev * 0.5), -innerR);   // Zur rechten Ecke der Basis
    ctx.closePath();
    ctx.fill();

    // NEU: Feine transparente Begrenzungslinien von den äußeren Dreiecksecken zum Mittelpunkt
    ctx.strokeStyle = "rgba(38, 166, 91, 0.25)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, 0); ctx.lineTo(-symW - (dev * 0.5), -innerR);
    ctx.moveTo(0, 0); ctx.lineTo(symW + (dev * 0.5), -innerR);
    ctx.stroke();

    // GESTRICHELTE HAUPTLINIE (In der Mitte des Sektors)
    ctx.strokeStyle = "#26a65b";
    ctx.lineWidth = 1.5;
    ctx.setLineDash([6, 4]); 
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, -innerR);
    ctx.stroke();
    
    // SCHMALES SCHWANKUNGS-DREIECK (An der Skalenspitze)
    ctx.setLineDash([]); 
    ctx.fillStyle = "rgba(38, 166, 91, 0.4)"; 
    ctx.strokeStyle = "#26a65b";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, -outerR); 
    ctx.lineTo(-symW - (dev * 0.5), -innerR); 
    ctx.lineTo(symW + (dev * 0.5), -innerR);  
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();

    // Setzt den Linienstil global zurück, damit der Rest des Dashboards sauber bleibt
    ctx.setLineDash([]); 
}

/**
 * 2. Zeichnet das orange Wegpunkt-Zielvisier (WPT BRG)
 */
function drawCompassWaypointTarget(r, config) {
    if (!latestRawData || !autopilotCache || 
        autopilotCache.target_lat === null || autopilotCache.target_lon === null ||
        latestRawData.gps_lat === undefined || latestRawData.gps_lon === undefined) return;

    if (!config) config = centerConfig; // Sicherheits-Fallback

    const lat1 = (latestRawData.gps_lat * Math.PI) / 180;
    const lon1 = (latestRawData.gps_lon * Math.PI) / 180;
    const lat2 = (autopilotCache.target_lat * Math.PI) / 180;
    const lon2 = (autopilotCache.target_lon * Math.PI) / 180;

    const dLon = lon2 - lon1;
    const yCoord = Math.sin(dLon) * Math.cos(lat2);
    const xCoord = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);
    
    let bearingDeg = (Math.atan2(yCoord, xCoord) * 180) / Math.PI;
    bearingDeg = (bearingDeg + 360) % 360; 

    ctx.save();
    ctx.rotate(bearingDeg * Math.PI / 180); 
    
    const wptY = -r; 
    const wptRadius = 8; 
    
    ctx.beginPath();
    ctx.arc(0, wptY, wptRadius, 0, 2 * Math.PI); 
    
    // KORREKTUR: Nutzt jetzt dynamisch deine gewählte Wunschfarbe (config.colorWegpunkt) statt festem Orange!
    ctx.fillStyle = config.colorWegpunkt; 
    ctx.fill();
    
    ctx.strokeStyle = "#05070a";
    ctx.lineWidth = 2;
    ctx.stroke();
    
    // Fadenkreuz im Inneren des Wegpunkt-Kreises zeichnen
    ctx.strokeStyle = "#05070a";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(-wptRadius + 3, wptY); ctx.lineTo(wptRadius - 3, wptY);
    ctx.moveTo(0, wptY - wptRadius + 3); ctx.lineTo(0, wptY + wptRadius - 3);
    ctx.stroke();
    
    ctx.restore();
}

/**
 * 3. Zeichnet die eckige Doppel-Pfeil 8 für den GPS-Kurs (COG)
 */
function drawCompassGpsCourse(r, config) {
    if (!currentRenderState || currentRenderState.gps_kurs === undefined) return;
    if (!config) config = centerConfig; // Sicherheits-Fallback

    ctx.save();
    ctx.rotate(currentRenderState.gps_kurs * Math.PI / 180);
    
    const cogY = -r; 
    const symW = 6;  
    const symH = 9;  

    // KORREKTUR: Nutzt jetzt dynamisch deine gewählte Wunschfarbe statt festem Orange!
    ctx.fillStyle = config.colorGpsKurs; 
    ctx.strokeStyle = "#05070a"; // Die dunkle Konturlinie bleibt für den Kontrast erhalten
    ctx.lineWidth = 1.5; 

    ctx.beginPath();
    ctx.moveTo(0, cogY - 2);               
    ctx.lineTo(-symW, cogY - 2 - symH);    
    ctx.lineTo(symW, cogY - 2 - symH);     
    ctx.closePath();

    ctx.moveTo(0, cogY + 2);               
    ctx.lineTo(-symW, cogY + 2 + symH);    
    ctx.lineTo(symW, cogY + 2 + symH);     
    ctx.closePath();

    ctx.fill();
    ctx.stroke();
    ctx.restore();
}

/**
 * 4. Zeichnet die statischen Skalenstriche und Himmelsrichtungen (N, E, S, W)
 */
function drawCompassRoseTicks(r) {
    ctx.strokeStyle = "#4e5a6b"; 
    ctx.lineWidth = 3; 
    ctx.beginPath(); 
    ctx.arc(0, 0, r, 0, 2 * Math.PI); 
    ctx.stroke();

    ctx.fillStyle = "#ffffff"; 
    ctx.font = "bold 14px Arial"; 
    ctx.textAlign = "center"; 
    ctx.textBaseline = "middle";

    for (let deg = 0; deg < 360; deg += 10) {
        ctx.save(); 
        ctx.rotate(deg * Math.PI / 180);
        if (deg % 30 === 0) {
            let label = deg;
            if (deg === 0) label = "N"; 
            if (deg === 90) label = "E"; 
            if (deg === 180) label = "S"; 
            if (deg === 270) label = "W";
            ctx.fillText(label, 0, -r + 20); 
            ctx.fillRect(-2, -r, 4, 10);
        } else {
            ctx.fillStyle = "#8a96a3"; 
            ctx.fillRect(-1, -r, 2, 6);
        }
        ctx.restore();
    }
}

/**
 * 5. Zeichnet das feststehende digitale Doppel-Display (Kompass, SOG & STW)
 */
function drawCompassDigitalBoxes(x, y, r, heading) {
    const boxY = y - (r * 0.65);
    const topBoxWidth = 70;
    const lowerBoxWidth = 100; 
    const boxHeight = 28;

    // 1. OBERE BOX: Kompasskurs (HDG)
    ctx.fillStyle = "#1e2530"; 
    ctx.fillRect(x - (topBoxWidth / 2), boxY - 14, topBoxWidth, boxHeight);
    ctx.strokeStyle = "#00ff66"; 
    ctx.lineWidth = 2; 
    ctx.strokeRect(x - (topBoxWidth / 2), boxY - 14, topBoxWidth, boxHeight);

    ctx.fillStyle = "#ffffff"; 
    ctx.font = "bold 18px Arial"; 
    ctx.textAlign = "center"; 
    ctx.textBaseline = "middle";
    ctx.fillText(Math.round(heading).toString().padStart(3, '0'), x, boxY);

    // Vektorrechnung für STW & SOG
    let currentSOG = 0;
    let calculatedSTW = 0;
    
    if (currentRenderState && currentRenderState.gps_kurs !== undefined && currentRenderState.gps_speed !== undefined) {
        const cog = Number(currentRenderState.gps_kurs);
        currentSOG = Number(currentRenderState.gps_speed);
        const hdg = Number(heading);
        
        const angleDiffRad = ((cog - hdg) * Math.PI) / 180;
        calculatedSTW = Math.max(0.0, currentSOG * Math.cos(angleDiffRad));
    }

    // 2. UNTERE BOX: Geteiltes SOG/STW Feld
    const lowerBoxY = boxY + boxHeight + 4; 

    ctx.fillStyle = "#141923"; 
    ctx.fillRect(x - (lowerBoxWidth / 2), lowerBoxY - 14, lowerBoxWidth, boxHeight);
    ctx.strokeStyle = "#4e5a6b"; 
    ctx.lineWidth = 1.5; 
    ctx.strokeRect(x - (lowerBoxWidth / 2), lowerBoxY - 14, lowerBoxWidth, boxHeight);

    const leftX = x - 24;
    ctx.textAlign = "center";
    ctx.fillStyle = "#8a96a3"; ctx.font = "9px Arial"; 
    ctx.fillText("SOG", leftX, lowerBoxY - 7); 
    ctx.fillStyle = "#cbd5e1"; ctx.font = "bold 13px Arial"; 
    ctx.fillText(currentSOG.toFixed(1), leftX, lowerBoxY + 5); 

    ctx.strokeStyle = "rgba(78, 90, 107, 0.4)";
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x, lowerBoxY - 10); ctx.lineTo(x, lowerBoxY + 10); ctx.stroke();

    const rightX = x + 24;
    ctx.fillStyle = "#8a96a3"; ctx.font = "9px Arial"; 
    ctx.fillText("STW", rightX, lowerBoxY - 7); 
    ctx.fillStyle = "#cbd5e1"; ctx.font = "bold 13px Arial"; 
    ctx.fillText(calculatedSTW.toFixed(1), rightX, lowerBoxY + 5); 
}

// ============================================================
// DIE NEUE HAUPTFUNKTION: ULTRA-KOMPAKT & SAUBER
// ============================================================
function drawCompassRose(x, y, r, heading, config) {
    // Falls die Funktion ohne Menü aufgerufen wird (Sicherheits-Fallback), nutzen wir centerConfig
    if (!config) config = centerConfig;
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(-heading * Math.PI / 180); // Das gesamte System dreht sich gegen den Kurs

    // 1. Hintergrundkreis
    ctx.fillStyle = "#05070a"; 
    ctx.beginPath(); 
    ctx.arc(0, 0, r, 0, 2 * Math.PI); 
    ctx.fill();

    // 2. Die mitdrehenden Komponenten rendern (mit Konfigurations-Weiche)
    drawCompassLaylines(r, heading);
    
    // GKS Kurs-Punkt (Wegpunkt) filtern & config übergeben
    if (config.showWegpunkt) {
        drawCompassWaypointTarget(r, config);
    }
    
    // GPS Kurs (Doppeldreieck / 8er) filtern & config übergeben
    if (config.showGpsKurs) {
        drawCompassGpsCourse(r, config);
    }
    drawCompassRoseTicks(r);
    ctx.restore(); // Rotation JETZT aufheben, damit die Digitalanzeige fest oben verankert bleibt

    // 3. Feststehende Boxen darüberlegen
    drawCompassDigitalBoxes(x, y, r, heading);
}

function drawRollColorBackground(x, y) {
    const bgRadius = 110; const bandWidth = 10;
    ctx.save(); ctx.translate(x, y);

    let gradRed = ctx.createLinearGradient(-bgRadius, 0, 0, bgRadius);
    gradRed.addColorStop(0, "rgba(231, 76, 60, 0.5)"); gradRed.addColorStop(1, "rgba(45, 56, 70, 0.05)");
    ctx.strokeStyle = gradRed; ctx.lineWidth = bandWidth; ctx.beginPath();
    ctx.arc(0, 0, bgRadius, 30 * Math.PI / 180, 90 * Math.PI / 180); ctx.stroke();

    let gradGreen = ctx.createLinearGradient(0, bgRadius, bgRadius, 0);
    gradGreen.addColorStop(0, "rgba(45, 56, 70, 0.05)"); gradGreen.addColorStop(1, "rgba(38, 166, 91, 0.5)");
    ctx.strokeStyle = gradGreen; ctx.lineWidth = bandWidth; ctx.beginPath();
    ctx.arc(0, 0, bgRadius, 90 * Math.PI / 180, 150 * Math.PI / 180); ctx.stroke();

    ctx.strokeStyle = "#4e5a6b"; ctx.lineWidth = 1.5; ctx.beginPath();
    ctx.arc(0, 0, bgRadius + (bandWidth / 2), 15 * Math.PI / 180, 165 * Math.PI / 180); ctx.stroke();
    ctx.restore();
}

function drawRollArcGauge(x, y, currentRoll) {
    const arcRadius = 110; 
    ctx.save(); 
    ctx.translate(x, y);
    
    // 1. Skalenbogen zeichnen
    ctx.strokeStyle = "#2d3846"; ctx.lineWidth = 2; ctx.beginPath();
    ctx.arc(0, 0, arcRadius, 30 * Math.PI / 180, 150 * Math.PI / 180); ctx.stroke();

    // 2. Skalenstriche einzeichnen
    for (let angle = -60; angle <= 60; angle += 10) {
        ctx.save(); ctx.rotate(angle * Math.PI / 180);
        if (angle === 0) {
            ctx.fillStyle = "#ffffff"; ctx.fillRect(-1.5, arcRadius - 8, 3, 10);
        } else if (angle % 30 === 0) {
            ctx.fillStyle = "#7f8c8d"; ctx.fillRect(-1, arcRadius - 6, 2, 8);
        } else {
            ctx.fillStyle = "#4a5568"; ctx.fillRect(-0.5, arcRadius - 4, 1, 5);
        }
        ctx.restore();
    }

    // 3. Beweglicher Zeiger (Orange, rotierte Achse)
    ctx.save(); 
    ctx.rotate(currentRoll * Math.PI / 180);
    ctx.fillStyle = "#ff9900"; ctx.beginPath();
    ctx.moveTo(0, arcRadius - 2); ctx.lineTo(-6, arcRadius + 10); ctx.lineTo(6, arcRadius + 10);
    ctx.closePath(); ctx.fill(); 
    ctx.restore(); // KORREKTUR: Hebt die Drehung für den Zeiger sofort wieder auf!

    // 4. FESTSTEHENDE ZAHL: Genau in der Mitte unter der Skala
    ctx.fillStyle = "#8a96a3";
    ctx.font = "bold 13px Arial";
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    // Steht fest verankert bei y = arcRadius + 16 (dreht sich nicht mit)
    ctx.fillText(`${currentRoll >= 0 ? '+' : ''}${Math.round(currentRoll)}° Roll`, 0, arcRadius + 16);

    ctx.restore(); 
}

function drawPitchGauge(x, y, currentPitch) {
    const scaleHeight = 90; const offsetX = 67; const maxPitchDeg = 15;
    ctx.save(); ctx.translate(x, y);
    
    // 1. Vertikale Skalenlinie
    ctx.strokeStyle = "#2d3846"; ctx.lineWidth = 2; ctx.beginPath();
    ctx.moveTo(offsetX, -scaleHeight / 2); ctx.lineTo(offsetX, scaleHeight / 2); ctx.stroke();

    // 2. Skalenstriche
    for (let deg = -maxPitchDeg; deg <= maxPitchDeg; deg += 5) {
        const tickY = (deg / maxPitchDeg) * (-scaleHeight / 2);
        ctx.beginPath();
        if (deg === 0) {
            ctx.fillStyle = "#ffffff"; ctx.fillRect(offsetX, tickY - 1, 10, 2);
        } else if (deg % maxPitchDeg === 0) {
            ctx.fillStyle = "#cbd5e1"; ctx.fillRect(offsetX, tickY - 0.5, 7, 1);
        } else {
            ctx.fillStyle = "#4a5568"; ctx.fillRect(offsetX, tickY - 0.5, 4, 1);
        }
    }

    // 3. Beweglicher Zeiger (Orange, wandert auf und ab)
    let limitedPitch = Math.max(-maxPitchDeg, Math.min(maxPitchDeg, currentPitch));
    const pointerY = (limitedPitch / maxPitchDeg) * (-scaleHeight / 2);
    ctx.fillStyle = "#ff9900"; ctx.beginPath();
    ctx.moveTo(offsetX + 12, pointerY - 5); ctx.lineTo(offsetX + 12, pointerY + 5); ctx.lineTo(offsetX + 3, pointerY);
    ctx.closePath(); ctx.fill(); 

    // 4. FESTSTEHENDE ZAHL: Permanent ganz oben über der Skala verankert
    ctx.fillStyle = "#8a96a3";
    ctx.font = "bold 13px Arial";
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    // Steht fest genau über der Oberkante der Linie bei (-scaleHeight / 2) - 6 Pixel Puffer
    ctx.fillText(`P: ${currentPitch >= 0 ? '+' : ''}${Math.round(currentPitch)}°`, offsetX, (-scaleHeight / 2) - 6);

    ctx.restore();
}

function drawBoatIndicator(x, y, roll, pitch) {
    ctx.save(); 
    ctx.translate(x, y);
    // Das Boot bleibt fest verankert und starr aufrecht

    // Äußere Form der Boots-Silhouette im Raymarine-Design
    ctx.strokeStyle = "#40526e"; 
    ctx.lineWidth = 3; 
    ctx.beginPath();
    ctx.moveTo(-30, 60); 
    ctx.lineTo(30, 60);
    ctx.bezierCurveTo(35, 10, 25, -45, 0, -75); 
    ctx.bezierCurveTo(-25, -45, -35, 10, -30, 60);
    ctx.closePath(); 
    ctx.stroke();

    ctx.restore();
}

function drawWindArrows(x, y, r, display, config) {
    // Falls die Funktion ohne Menü aufgerufen wird (Sicherheits-Fallback), nutzen wir centerConfig
    if (!config) config = centerConfig;

    // --- 1. SCHEINBARER WIND (AWA) ---
    if (config.showAwa && display.winddir_gemessen !== undefined) {
        ctx.save(); 
        ctx.translate(x, y); 
        ctx.rotate(Number(display.winddir_gemessen) * Math.PI / 180);
        
        // KORREKTUR: Nutzt jetzt dynamisch deine gewählte Wunschfarbe (config.colorAwa) statt festem Blau!
        drawArrow(0, -r, config.colorAwa, "A", 66); 
        ctx.restore();
    }

    // --- 2. WAHRER WIND (TWA) ---
    if (config.showTwa && display.winddir_berechnet !== undefined) {
        ctx.save(); 
        ctx.translate(x, y); 
        ctx.rotate(Number(display.winddir_berechnet) * Math.PI / 180);
        
        // KORREKTUR: Nutzt jetzt dynamisch deine gewählte Wunschfarbe (config.colorTwa) statt festem Hellblau!
        drawArrow(0, -r + 15, config.colorTwa, "T", 44); 
        ctx.restore();
    }
}

/**
 * Modifizierte Pfeil-Zeichenfunktion mit variablem Längen-Parameter
 */
function drawArrow(targetX, targetY, color, label, arrowLength = 44) {
    ctx.fillStyle = color; 
    ctx.beginPath();
    // Startet an der Basis (oben) und zieht den Pfad um 'arrowLength' nach unten zur Spitze
    ctx.moveTo(targetX, targetY + arrowLength); // Die scharfe Spitze zeigt nach innen
    ctx.lineTo(targetX - 14, targetY);         // Linke Ecke der Basis am Außenrand
    ctx.lineTo(targetX + 14, targetY);         // Rechte Ecke der Basis am Außenrand
    ctx.closePath(); 
    ctx.fill();
    
    // Text-Label ("A" oder "T") im oberen Drittel des Pfeils platzieren
    ctx.fillStyle = "#ffffff"; 
    ctx.font = "bold 14px Arial"; 
    ctx.textAlign = "center"; 
    ctx.textBaseline = "middle";
    ctx.fillText(label, targetX, targetY + 14);
}

function drawWindTrendArrow(x, y, angle) {
    ctx.save(); ctx.translate(x, y); ctx.rotate((angle * Math.PI / 180) + Math.PI);
    let normAngle = ((angle % 360) + 360) % 360;
    let isBackbord = (normAngle > 180 && normAngle < 360);
    let gradient = ctx.createLinearGradient(0, 15, 0, -29);
    
    if (isBackbord) {
        gradient.addColorStop(0, "#4a1510"); gradient.addColorStop(0.4, "#962d22"); gradient.addColorStop(1, "#ff3b30");
    } else {
        gradient.addColorStop(0, "#0f3d21"); 
        gradient.addColorStop(0.4, "#1e6b38"); 
        gradient.addColorStop(1, "#2ecc71");
    }
    
    ctx.fillStyle = gradient; 
    ctx.beginPath();
    ctx.moveTo(0, -29); 
    ctx.lineTo(-14, 15); 
    ctx.lineTo(0, 7); 
    ctx.lineTo(14, 15);
    ctx.closePath(); 
    ctx.fill();
    
    ctx.strokeStyle = "rgba(255,255,255,0.2)"; 
    ctx.lineWidth = 1; 
    ctx.stroke();
    
    ctx.fillStyle = "#ffffff"; 
    ctx.beginPath(); 
    ctx.arc(0, 0, 3, 0, 2 * Math.PI); 
    ctx.fill();
    ctx.restore();
}

/**
 * Zeichnet den Abdriftpfeil basierend auf einer einheitlichen, gedämpften Datenbasis (display).
 * Dadurch schwingt der Pfeil flüssig mit den simulierten 10-Hz-Wellen mit!
 */
/**
 * Zeichnet den Abdriftpfeil voll zentriert im Achsenmittelpunkt.
 * Der Zahlenwert wird in abgedunkeltem Weiß direkt darübergelegt.
 */
/**
 * Sprungfreie Version: Berechnet die Pfeilrichtung direkt aus der Winkeldifferenz
 * zwischen GPS-Kurs (COG) und Kompass (HDG). Keine 180°-Sprünge mehr!
 */
function drawTideDriftArrow(cx, cy, display, raw) {
    const hdg = Number(display.kompass);      // Wo zeigt der Bug hin
    const cog = Number(display.gps_kurs);     // Wo bewegt sich das Boot hin (GPS COG)
    const sog = Number(display.gps_speed);    // Geschwindigkeit über Grund (SOG)
    const stw = Number(display.gps_speed);    // Fallback Fahrt durchs Wasser

    if (isNaN(hdg) || isNaN(cog) || isNaN(sog)) return;

    // ============================================================
    // SPRUNGFREIE WINKELBERECHNUNG
    // ============================================================
    // Wir berechnen die direkte, relative Abdrift: Wie weit drückt der Strom
    // den GPS-Kurs (COG) im Vergleich zum Kompasskurs (HDG) zur Seite?
    let driftAngleRelToBoat = (cog - hdg + 360) % 360;

    // Falls das Boot exakt gegenan fährt und der Strom von vorne drückt,
    // sorgt diese Zeile dafür, dass der Pfeil stabil nach hinten zeigt, statt zu springen:
    if (sog < stw && Math.abs(((cog - hdg + 180) % 360) - 180) > 90) {
        // Das Boot wird effektiv abgebremst/rückwärts versetzt
        driftAngleRelToBoat = (driftAngleRelToBoat + 180) % 360;
    }

    // Mathematische Berechnung der Stromstärke (Intensität)
    const hdgRad = (hdg * Math.PI) / 180;
    const cogRad = (cog * Math.PI) / 180;
    const bX = stw * Math.sin(hdgRad);
    const bY = stw * Math.cos(hdgRad);
    const gX = sog * Math.sin(cogRad);
    const gY = sog * Math.cos(cogRad);
    let driftSpeed = Math.sqrt((gX - bX) ** 2 + (gY - bY) ** 2);

    // ============================================================
    // GRAFIK: DURCHGEHENDER PFEIL MIT MITTIGER DREHACHSE
    // ============================================================
    ctx.save();
    ctx.translate(cx, cy); 

    // Umrechnung für Canvas (0° oben bei 12 Uhr am Bug)
    const renderAngleRad = (driftAngleRelToBoat * Math.PI) / 180 - Math.PI / 2;
    ctx.rotate(renderAngleRad);

    // Kompakte Pfeil-Geometrie
    const arrowLength = 55;     
    const headLength = 22;      
    const shaftWidth = 5;       
    const headWidth = 14;       

    const halfL = arrowLength / 2; 
    const startX = -halfL;         
    const tipX = halfL;            
    const headStartX = tipX - headLength; 

    // Farbverlauf von transparent (Mitte) zu sattem Blau (Spitze)
    let gradient = ctx.createLinearGradient(startX, 0, tipX, 0);
    gradient.addColorStop(0, "rgba(56, 189, 248, 0.2)");   
    gradient.addColorStop(0.5, "rgba(29, 88, 204, 0.75)"); 
    gradient.addColorStop(1, "#0044ff");                   

    ctx.fillStyle = gradient;
    ctx.strokeStyle = "#4da3ff"; 
    ctx.lineWidth = 1.5;
    ctx.lineJoin = "round";

    // Pfeil zeichnen
    ctx.beginPath();
    ctx.moveTo(startX, -shaftWidth);      
    ctx.lineTo(headStartX, -shaftWidth);  
    ctx.lineTo(headStartX, -headWidth);   
    ctx.lineTo(tipX, 0);                  
    ctx.lineTo(headStartX, headWidth);    
    ctx.lineTo(headStartX, shaftWidth);   
    ctx.lineTo(startX, shaftWidth);       
    ctx.closePath();
    
    ctx.fill();
    ctx.stroke();
    ctx.restore();

    // ============================================================
    // TEXT: DER KONTRASTREICHE ZAHLENWERT EXAKT MITTIG DARÜBER
    // ============================================================
    ctx.save();
    ctx.translate(cx, cy); 

    ctx.font = "bold 18px Arial";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";

    ctx.strokeStyle = "#0c0e12"; 
    ctx.lineWidth = 5;
    ctx.strokeText(driftSpeed.toFixed(1), 0, 0);

    ctx.fillStyle = "#cbd5e1"; 
    ctx.fillText(driftSpeed.toFixed(1), 0, 0);

    ctx.restore();
}

// ============================================================
// DATA FORMAT & OVERLAY BLOCKS
// ============================================================

function drawDataBlock(title, value, unit, x, y, size) {
    ctx.save();
    ctx.textBaseline = "alphabetic";
    ctx.textAlign = "left";
    ctx.fillStyle = "#7f8c8d";
    ctx.font = "bold 12px Arial";
    ctx.fillText(title, x, y);
    
    let fullDisplayString = value + (unit ? " " + unit : "");
    const valueY = y + 12 + 6 + size;
    ctx.fillStyle = "#ffffff";
    
    // KORREKTUR: Mit Backticks umrandet, damit der String dynamisch evaluiert wird
    ctx.font = `bold ${size}px monospace`;
    ctx.fillText(fullDisplayString, x, valueY);
    ctx.restore();
}

function formatValue(val, digits = 1) {
    const num = Number(val);
    return Number.isFinite(num) ? num.toFixed(digits) : "--";
}

function getNumber(value, fallback = 0) {
    const num = Number(value);
    return Number.isFinite(num) ? num : fallback;
}

function openCenterSettingsMenu() {
    let overlay = document.getElementById('canvasTouchOverlay');
    if (!overlay) return;

    // Content-Box leeren, um sie für das Zentrums-Menü neu aufzubauen
    let menuBox = overlay.querySelector('div');
    menuBox.innerHTML = ""; 

    // Titel
    let title = document.createElement('h3');
    title.textContent = "⚙️ Zentrum Instrumenten-Filter";
    title.style.margin = '0 0 20px 0';
    title.style.color = '#ff9900';
    title.style.fontFamily = 'Arial, sans-serif';
    title.style.fontSize = '22px';
    title.style.textAlign = 'center';
    menuBox.appendChild(title);

    // Grid für die Optionen
    let listContainer = document.createElement('div');
    listContainer.style.display = 'flex';
    listContainer.style.flexDirection = 'column';
    listContainer.style.gap = '15px';
    listContainer.style.marginBottom = '25px';

    // Definition der Schalter (Key, Label, hatFarbe, FarbKey)
    const items = [
        { key: "showGpsKurs", label: "GPS Kurs (COG) anzeigen", hasColor: true, colorKey: "colorGpsKurs" },
        { key: "showWegpunkt", label: "Wegpunkt-Zeiger anzeigen", hasColor: true, colorKey: "colorWegpunkt" },
        { key: "showApKurs", label: "Autopilot Sollkurs-Dreieck anzeigen", hasColor: true, colorKey: "colorApKurs" }, // <-- NEU!
        { key: "showAwa", label: "Scheinbarer Wind (AWA) Pfeil", hasColor: true, colorKey: "colorAwa" },
        { key: "showTwa", label: "Wahrer Wind (TWA) Pfeil", hasColor: true, colorKey: "colorTwa" },
        { key: "showAbdrift", label: "Abdrift (Tide/Drift) anzeigen", hasColor: false }
    ];

    items.forEach(item => {
        let row = document.createElement('div');
        row.style.display = 'flex';
        row.style.justifyContent = 'space-between';
        row.style.alignItems = 'center';
        row.style.background = '#1e2530';
        row.style.padding = '12px 15px';
        row.style.borderRadius = '8px';
        row.style.border = '1px solid #4e5a6b';

        // Label & Checkbox (Große Touch-Fläche)
        let label = document.createElement('label');
        label.style.color = '#ffffff';
        label.style.fontFamily = 'Arial, sans-serif';
        label.style.fontSize = '16px';
        label.style.display = 'flex';
        label.style.alignItems = 'center';
        label.style.gap = '15px';
        label.style.cursor = 'pointer';
        label.style.flexGrow = '1';

        let checkbox = document.createElement('input');
        checkbox.type = 'checkbox';
        checkbox.style.transform = 'scale(1.6)'; // Schön groß für Touch-Bedienung
        checkbox.checked = centerConfig[item.key];
        
        checkbox.addEventListener('change', () => {
            centerConfig[item.key] = checkbox.checked;
            saveDashboardLayout();
        });

        label.appendChild(checkbox);
        label.appendChild(document.createTextNode(" " + item.label));
        row.appendChild(label);

        // Farbwähler (nur wenn das Element eine wählbare Farbe hat)
        if (item.hasColor) {
            let colorInput = document.createElement('input');
            colorInput.type = 'color';
			colorInput.id = 'cp_' + item.colorKey;
            colorInput.value = centerConfig[item.colorKey];
            colorInput.style.width = '45px';
            colorInput.style.height = '35px';
            colorInput.style.border = 'none';
            colorInput.style.borderRadius = '4px';
            colorInput.style.cursor = 'pointer';
            colorInput.style.background = 'none';

            colorInput.addEventListener('change', () => {
                centerConfig[item.colorKey] = colorInput.value;
                saveDashboardLayout();
            });
            row.appendChild(colorInput);
        }

        listContainer.appendChild(row);
    });
    menuBox.appendChild(listContainer);

    // Fertig-Button zum Schließen
    let closeBtn = document.createElement('button');
    closeBtn.textContent = "Fertig / Schließen";
    closeBtn.style.width = '100%';
    closeBtn.style.background = '#ff9900';
    closeBtn.style.color = '#000000';
    closeBtn.style.border = 'none';
    closeBtn.style.borderRadius = '8px';
    closeBtn.style.padding = '15px';
    closeBtn.style.fontSize = '18px';
    closeBtn.style.fontWeight = 'bold';
    closeBtn.addEventListener('click', () => {
        overlay.style.display = 'none';
        // Nach dem Schließen bauen wir das Menü wieder auf die Standard-Kachel-Auswahl zurück
        rebuildStandardKachelMenu(overlay);
    });
    menuBox.appendChild(closeBtn);

    overlay.style.display = 'flex';
}

// Hilfsfunktion: Öffnet das Kachel-Menü für Links/Rechts
function openKachelMenu(titleText, hitField) {
    let overlay = document.getElementById('canvasTouchOverlay');
    if (!overlay) return;
    document.getElementById('touchMenuTitle').textContent = `${titleText} (${hitField.type})`;
    overlay.activeField = hitField;
    overlay.style.display = 'flex';
}

// Hilfsfunktion: Baut die Box wieder zurück für die normalen Kacheln, falls geschlossen
function rebuildStandardKachelMenu(overlay) {
    // Da wir das Menü-HTML vorhin mit innerHTML geleert haben, bauen wir es hier einfach 
    // beim nächsten Klick auf eine Kachel dynamisch neu auf. Ein Page-Reload ist nicht nötig!
    // Wir zwingen das Skript einfach dazu, das Element beim nächsten Mal komplett frisch zu erstellen:
    overlay.remove();
}

// ============================================================
// TELEMETRIE-SYSTEM
// ============================================================

window.addEventListener("navisTelemetryUpdate", (ev) => {
    if (ev.detail) {
        // 1. Hole die Daten direkt so ab, wie dein WS-Client sie schickt
        const rawData = ev.detail.raw || {};
        const dispData = ev.detail.display || {};
        
        // 2. Aktualisiere die globalen Rohdaten-Caches deiner Anzeige
        latestRawData = { ...latestRawData, ...rawData };
        
        // 3. Autopilot-Objekt auslesen (Exakt so, wie dein ESP32 es schickt)
        if (rawData.autopilot !== undefined) {
            autopilotCache = {
                mode: getNumber(rawData.autopilot.mode, autopilotCache.mode),
                offset: getNumber(rawData.autopilot.offset, autopilotCache.offset),
                modus_counter: getNumber(rawData.autopilot.modus_counter, autopilotCache.modus_counter),
                pinne: getNumber(rawData.autopilot.pinne, autopilotCache.pinne),
                target_lat: rawData.autopilot.target_lat !== undefined ? rawData.autopilot.target_lat : autopilotCache.target_lat,
                target_lon: rawData.autopilot.target_lon !== undefined ? rawData.autopilot.target_lon : autopilotCache.target_lon
            };
        }

        // 4. Ziel-Zustände für die LERP-Animation deiner Anzeige setzen
        // Wir nutzen dispData für die bereits im WS-Client gefilterten Werte (Kompass, Roll, Pitch)
        targetState.kompass = getNumber(dispData.kompass, targetState.kompass);
        targetState.roll    = getNumber(dispData.roll, targetState.roll);
        targetState.pitch   = getNumber(dispData.pitch, targetState.pitch);
        
        // Die restlichen GPS- und Windwerte nehmen wir aus den Rohdaten und animieren sie im Dashboard flüssig per LERP
        targetState.gps_kurs           = getNumber(rawData.gps_kurs, targetState.gps_kurs);
        targetState.gps_speed          = getNumber(rawData.gps_speed, targetState.gps_speed);
        targetState.winddir_gemessen   = getNumber(rawData.winddir_gemessen, targetState.winddir_gemessen);
        targetState.winddir_berechnet  = getNumber(rawData.winddir_berechnet, targetState.winddir_berechnet);
        targetState.windspeed_gemessen = getNumber(rawData.windspeed_gemessen, targetState.windspeed_gemessen);
        targetState.windspeed_berechnet= getNumber(rawData.windspeed_berechnet, targetState.windspeed_berechnet);
        targetState.Echolot            = getNumber(rawData.Echolot, targetState.Echolot);
        targetState.missweisung        = getNumber(rawData.missweisung, targetState.missweisung);
        
        // Das Autopilot-Offset für die Ruderlagen-Animation setzen
        if (autopilotCache) {
            targetState.autopilot_offset = getNumber(autopilotCache.offset, targetState.autopilot_offset);
        }
    }
});

// Startet die flüssige 60 FPS Render-Schleife der Anzeige
animationLoop();

