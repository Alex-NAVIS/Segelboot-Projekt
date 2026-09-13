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
   NAVIS RAYMARINE-STYLE CANVAS ENGINE (TEIL 2 VON 3)
   ============================================================ */

/**
 * Baut das visuelle Raster und die gesamte Koordinatenverteilung auf
 */
function drawDashboardFrame(display, raw) {
    // 1. Hintergrund leeren & einfärben
    ctx.fillStyle = "#0c0e12";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    
    // 2. Gitterlinien / Trennlinien im originalen Dunkelgrau zeichnen
    ctx.strokeStyle = "#1f2530";
    ctx.lineWidth = 1;
    ctx.beginPath();
    // Vertikale Rastertrennungen
    ctx.moveTo(220, 0); ctx.lineTo(220, canvas.height);
    ctx.moveTo(804, 0); ctx.lineTo(804, canvas.height);
    // Horizontale Teiler links
    ctx.moveTo(0, 135); ctx.lineTo(220, 135);
    ctx.moveTo(0, 275); ctx.lineTo(220, 275);
    ctx.moveTo(0, 415); ctx.lineTo(220, 415);
    // Horizontale Teiler rechts (Layout gestrafft: 4 Trennlinien statt 5)
    ctx.moveTo(804, 110); ctx.lineTo(1024, 110);
    ctx.moveTo(804, 220); ctx.lineTo(1024, 220);
    ctx.moveTo(804, 330); ctx.lineTo(1024, 330);
    ctx.moveTo(804, 440); ctx.lineTo(1024, 440);
    ctx.stroke();

    // Zentrumskoordinaten für das Hauptinstrument im Mittelblock
    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    const radius = 210;

    // ============================================================
    // MATHEMATISCHE LIVE-BERECHNUNG FÜR WEGPUNKT (DIST & BRG)
    // ============================================================
    let apTargetLat = autopilotCache ? autopilotCache.target_lat : null;
    let apTargetLon = autopilotCache ? autopilotCache.target_lon : null;

    if (apTargetLat !== null && apTargetLon !== null &&
        raw.gps_lat !== undefined && raw.gps_lon !== undefined) {
        
        const R_EARTH_NM = 3440.065; 
        const lat1 = (raw.gps_lat * Math.PI) / 180;
        const lon1 = (raw.gps_lon * Math.PI) / 180;
        const lat2 = (apTargetLat * Math.PI) / 180;
        const lon2 = (apTargetLon * Math.PI) / 180;

        const dLat = lat2 - lat1;
        const dLon = lon2 - lon1;

        const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                  Math.cos(lat1) * Math.cos(lat2) *
                  Math.sin(dLon / 2) * Math.sin(dLon / 2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        
        raw.wpt_dist = R_EARTH_NM * c;

        const yCoord = Math.sin(dLon) * Math.cos(lat2);
        const xCoord = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);
        
        let bearingDeg = (Math.atan2(yCoord, xCoord) * 180) / Math.PI;
        raw.wpt_brg = (bearingDeg + 360) % 360;
    }

    // ============================================================
    // LINKS: GESCHWINDIGKEITEN & WIND
    // ============================================================
    drawDataBlock("BOAT SPEED", formatValue(display.gps_speed), "kn", 25, 20, GL_SIZE);
    drawDataBlock("VMG CSE", formatValue(display.vmg_cse, 2), "kn", 25, 155, GL_SIZE);
    
    drawDataBlock("AWA", formatValue(display.winddir_gemessen, 0), "°", 25, 295, GL_SIZE);
    drawWindTrendArrow(170, 335, display.winddir_gemessen); 

    drawDataBlock("TWA", formatValue(display.winddir_berechnet, 0), "°", 25, 430, GL_SIZE);
    drawWindTrendArrow(170, 470, display.winddir_berechnet); 

    drawDataBlock("TWD", formatValue(display.winddir_berechnet, 0), "°M", 25, 510, 24);

    ctx.fillStyle = "#8a96a3"; ctx.font = "13px Arial";
    ctx.textAlign = "left";
    ctx.fillText(`Var: ${formatValue(display.missweisung, 1)}°W`, 25, 565);

    // ============================================================
    // ZENTRUM: SKALEN & INSTRUMENTE
    // ============================================================
    let tempApObj = { mode: autopilotCache.mode, offset: display.autopilot_offset };
    drawTopRudderArc(cx, cy, radius, tempApObj);
    drawCompassRose(cx, cy, radius, display.kompass);
    drawRollColorBackground(cx, cy);
    drawRollArcGauge(cx, cy, display.roll);
    drawPitchGauge(cx, cy, display.pitch);
    drawBoatIndicator(cx, cy, display.roll, display.pitch);
    drawTideDriftArrow(cx, cy, display, raw);
    drawWindArrows(cx, cy, radius, display);

    // ============================================================
    // RECHTS: TIME & PRIMARY GPS (Nach oben gerückt)
    // ============================================================
    let timeStr = "--:--:--";
    if (raw.gps_stunde !== undefined) {
        timeStr = `${String(raw.gps_stunde).padStart(2,'0')}:${String(raw.gps_minute).padStart(2,'0')}:${String(raw.gps_sekunde).padStart(2,'0')}`;
    }
    drawDataBlock("°M ETW", timeStr, "", 825, 25, GL_SIZE);

    const pairSize = 20;
    drawDataBlock("SOG", formatValue(display.gps_speed), "kn", 825, 135, pairSize);
    drawDataBlock("COG", formatValue(display.gps_kurs, 0), "°M", 930, 135, pairSize);

    drawDataBlock("SEA TEMP", "31.1", "°C", 825, 245, pairSize); // Liegt jetzt eins höher
    drawDataBlock("DEPTH", formatValue(display.Echolot, 1), "m", 930, 245, pairSize);

    // ============================================================
    // RECHTS UNTEN: INTEGRATION AUTOPILOT & WAYPOINT DATA
    // ============================================================
    let apMode = autopilotCache ? AP_MODE_TEXT[autopilotCache.mode] : "OFF";
    let hasWpt = (apTargetLat !== null && raw.wpt_dist !== undefined);
    
    // Wir bauen eine dynamische Statuszeile: "Kompass (+1.5°)" oder bei aktivem GPS "GPS 4.2NM 045°"
    let apValueString = apMode;
    if (autopilotCache && autopilotCache.mode > 0) {
        let offsetSign = display.autopilot_offset >= 0 ? "+" : "";
        apValueString += ` (${offsetSign}${formatValue(display.autopilot_offset, 1)}°)`;
    }
    
    // Hauptfeld für Autopilot-Modus zeichnen (y=355)
    drawDataBlock("AUTOPILOT", apValueString, "", 825, 355, 20);
    
    // Wenn Navigationsziele aktiv sind, blenden wir die Live-Daten kompakt direkt darunter ein
    if (hasWpt) {
        ctx.fillStyle = "#8a96a3"; ctx.font = "11px Arial"; ctx.textAlign = "left";
        ctx.fillText("NAV INFO:", 825, 412);
        ctx.fillStyle = "#00ff66"; ctx.font = "bold 13px monospace";
        ctx.fillText(`DIST: ${formatValue(raw.wpt_dist, 1)} NM`, 890, 412);
        ctx.fillText(`BRG: ${Math.round(raw.wpt_brg).toString().padStart(3,'0')}°M`, 890, 428);
    }

    // ============================================================
    // RECHTS UNTEN: GENERIERTE FREIFLÄCHE FÜR AKTUELLE POS & WPT POS
    // ============================================================
    // 1. Aktuelle Bootsposition (Aktuelle GPS Position)
    let curLatStr = "N --°--.---'"; let curLonStr = "E --°--.---'";
    if (Number.isFinite(Number(raw.gps_lat))) {
        const lat = Number(raw.gps_lat); curLatStr = `${lat >= 0 ? "N" : "S"} ${Math.abs(lat).toFixed(4)}°`;
    }
    if (Number.isFinite(Number(raw.gps_lon))) {
        const lon = Number(raw.gps_lon); curLonStr = `${lon >= 0 ? "E" : "W"} ${Math.abs(lon).toFixed(4)}°`;
    }

    ctx.fillStyle = "#7f8c8d"; ctx.font = "bold 11px Arial"; ctx.textAlign = "left";
    ctx.fillText("BOAT POS", 825, 465);
    ctx.fillStyle = "#cbd5e1"; ctx.font = "14px monospace";
    ctx.fillText(curLatStr, 825, 482);
    ctx.fillText(curLonStr, 825, 497);

    // 2. Wegpunkt-Zielkoordinaten (WPT POS) direkt darunter gruppiert
    let wptLatStr = "N --°--.---'"; let wptLonStr = "E --°--.---'";
    if (apTargetLat !== null && Number.isFinite(Number(apTargetLat))) {
        const lat = Number(apTargetLat); wptLatStr = `${lat >= 0 ? "N" : "S"} ${Math.abs(lat).toFixed(4)}°`;
    }
    if (apTargetLon !== null && Number.isFinite(Number(apTargetLon))) {
        const lon = Number(apTargetLon); wptLonStr = `${lon >= 0 ? "E" : "W"} ${Math.abs(lon).toFixed(4)}°`;
    }

    ctx.fillStyle = "#7f8c8d"; ctx.font = "bold 11px Arial"; ctx.textAlign = "left";
    ctx.fillText("WPT POS", 825, 520);
    ctx.fillStyle = "#cbd5e1"; ctx.font = "14px monospace";
    ctx.fillText(wptLatStr, 825, 537);
    ctx.fillText(wptLonStr, 825, 552);
}

// ============================================================
// MODULARE GRAPHISCHE ZEICHENFUNKTIONEN
// ============================================================

function drawTopRudderArc(x, y, r, autopilot) {
    const arcRadius = r + 16; 
    const arcWidth = 14; 
    const ap = autopilot || { mode: 0, offset: 0 };
    const mode = Number(ap.mode) || 0;
    const offset = Number(ap.offset) || 0;

    ctx.save();
    ctx.translate(x, y);

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

    // 3. Umlaufende Striche (360°)
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

    // 5. Zeiger & Kurztext (Mitdrehend)
    if (mode !== 0) {
        let shortLabel = ""; let modeColor = "#ffffff";
        if (mode === 1) { shortLabel = "C"; modeColor = "#00ff66"; }
        if (mode === 2) { shortLabel = "T"; modeColor = "#3498db"; }
        if (mode === 3) { shortLabel = "W"; modeColor = "#f1c40f"; }
		
        ctx.save();
        ctx.rotate(offset * Math.PI / 180);
        const outerPush = 12;
        const arrowTipY = -arcRadius - (arcWidth / 2) - outerPush;

        ctx.fillStyle = "#ff9900"; ctx.beginPath();
        ctx.moveTo(0, arrowTipY); ctx.lineTo(-7, arrowTipY + arcWidth + 6); ctx.lineTo(7, arrowTipY + arcWidth + 6);  
        ctx.closePath(); ctx.fill();

        let offsetSign = offset > 0 ? "+" : "";
        let shortStatusText = `${shortLabel} ${offsetSign}${offset.toFixed(1)}°`;

        ctx.fillStyle = modeColor; ctx.font = "bold 13px Arial"; ctx.textAlign = "center"; ctx.textBaseline = "bottom";
        ctx.fillText(shortStatusText, 0, arrowTipY - 8);
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
    if (!currentRenderState || currentRenderState.winddir_berechnet === undefined) return;

    // TWD holen und aktuellen relativen Windwinkel (TWA) bestimmen
    const twd = ((currentRenderState.winddir_berechnet % 360) + 360) % 360;
    const twa = ((currentRenderState.winddir_berechnet - heading + 360) % 360);
    const isDownwind = (twa > 90 && twa < 270);

    // Bestimmung des optimalen wahren Windwinkels (Target TWA) je nach Kurs
    const targetTWA = isDownwind ? 140 : 45;

    // Berechnung der beiden optimalen Steuerkurse auf der Kompassrose
    const laylineSB = (twd + targetTWA) % 360;
    const laylineBB = (twd - targetTWA + 360) % 360;

    // Interne Hilfsfunktion zur Linienzeichnung
    function drawDashedLayline(angleDeg, color) {
        ctx.save();
        ctx.rotate(angleDeg * Math.PI / 180);
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 4]); // 6px Linie, 4px Pause
        
        ctx.beginPath();
        ctx.moveTo(0, -r * 0.45); 
        ctx.lineTo(0, -r * 0.98);
        ctx.stroke();
        ctx.restore();
    }

    // Steuerbord-Layline (Grün) / Backbord-Layline (Rot)
    drawDashedLayline(laylineSB, "rgba(46, 204, 113, 0.65)"); 
    drawDashedLayline(laylineBB, "rgba(231, 76, 60, 0.65)");  
}

/**
 * 2. Zeichnet das orange Wegpunkt-Zielvisier (WPT BRG)
 */
function drawCompassWaypointTarget(r) {
    if (!latestRawData || !autopilotCache || 
        autopilotCache.target_lat === null || autopilotCache.target_lon === null ||
        latestRawData.gps_lat === undefined || latestRawData.gps_lon === undefined) return;

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
    ctx.fillStyle = "rgba(255, 153, 0, 0.9)"; 
    ctx.fill();
    
    ctx.strokeStyle = "#05070a";
    ctx.lineWidth = 2;
    ctx.stroke();
    
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
function drawCompassGpsCourse(r) {
    if (!currentRenderState || currentRenderState.gps_kurs === undefined) return;

    ctx.save();
    ctx.rotate(currentRenderState.gps_kurs * Math.PI / 180);
    
    const cogY = -r; 
    const symW = 6;  
    const symH = 9;  

    ctx.fillStyle = "#ff9900"; 
    ctx.strokeStyle = "#05070a";
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
function drawCompassRose(x, y, r, heading) {
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(-heading * Math.PI / 180); // Das gesamte System dreht sich gegen den Kurs

    // 1. Hintergrundkreis
    ctx.fillStyle = "#05070a"; 
    ctx.beginPath(); 
    ctx.arc(0, 0, r, 0, 2 * Math.PI); 
    ctx.fill();

    // 2. Die mitdrehenden Komponenten rendern
    drawCompassLaylines(r, heading);
    drawCompassWaypointTarget(r);
    drawCompassGpsCourse(r);
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

function drawWindArrows(x, y, r, display) {
    if (display.winddir_gemessen !== undefined) {
        ctx.save(); 
        ctx.translate(x, y); 
        ctx.rotate(Number(display.winddir_gemessen) * Math.PI / 180);
        
        // KORREKTUR: Pfeillänge von 44 auf 66 Pixel (+50%) erhöht.
        // Die Spitze zieht sich dadurch weit nach unten Richtung Mittelpunkt!
        drawArrow(0, -r, "#0055ff", "A", 66); 
        ctx.restore();
    }
    if (display.winddir_berechnet !== undefined) {
        ctx.save(); 
        ctx.translate(x, y); 
        ctx.rotate(Number(display.winddir_berechnet) * Math.PI / 180);
        
        // Der wahre Wind (T) behält seine Standard-Länge von 44 Pixeln
        drawArrow(0, -r + 15, "#1a75ff", "T", 44); 
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

/**
 * Zeichnet den oberen Ruderlagenbogen rein basierend auf den Hardware-Istwerten.
 * Die Pfeile leuchten permanent und direkt, solange der Motor laut ESP32 Strom erhält.
 */
function drawTopRudderArc(cx, cy, r, ap) {
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
    // REINE ISTWERT-ANZEIGE: POSITION / WINKEL DER PINNE
    // ============================================================
    ctx.save();
    // Zeigt den exakten, aktuellen physikalischen Ist-Winkel der Pinnenstellung
    ctx.rotate(offset * Math.PI / 180);

    const outerPush = 6;
    const arrowTipY = arcY - outerPush;

    // Positionszeiger in Fahrten-Orange
    ctx.fillStyle = "#ff9900"; 
    ctx.beginPath();
    ctx.moveTo(0, arrowTipY); 
    ctx.lineTo(-6, arrowTipY + arcWidth + 4); 
    ctx.lineTo(6, arrowTipY + arcWidth + 4);  
    ctx.closePath(); 
    ctx.fill();

    // Schwarzer Kernpunkt im Zeiger
    ctx.fillStyle = "#0c0e12";
    ctx.beginPath();
    ctx.arc(0, arrowTipY + (arcWidth / 2) + 2, 2, 0, 2 * Math.PI);
    ctx.fill();

    ctx.restore(); 

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

