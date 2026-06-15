/**
 * A*-Routenplaner für das ESP32-Segelboot-Projekt
 */

// --- 1. DATEN-Schnittstellen (Für Test lokal, später ESP32-API) ---

async function fetchPolarData() {
    // Test-Auslesen der myboot.json
    try {
        const response = await fetch('polar/myboot.json');
        return await response.json();
    } catch (e) {
        console.error("Fehler beim Laden der Polardaten, nutze Fallback", e);
        // Fallback-Struktur falls Datei offline
        return { "twa":, "tws":, "speed": [[0,0,0,0,0],[0,4,5,5,4],[0,5,7,7,5],[0,6,8,8,6],[0,6,8,8,6]] };
    }
}

async function checkCollision(lat, lon) {
    // Test-Zugriff auf Kachelordner tiles_nav/
    // Verwandelt Koordinate in Kachelname (Beispielhaft je nach deinem Gitter)
    const tileX = Math.floor(lon);
    const tileY = Math.floor(lat);
    try {
        const response = await fetch(`tiles_nav/${tileX}_${tileY}.json`);
        const tileData = await response.json();
        return tileData.isLand; // true wenn Land
    } catch (e) {
        // Wenn Kachel nicht existiert oder Fehler: OpenSea-Annahme oder konservativ Land
        return false; 
    }
}

function getWeatherData(hoursAhead, lat, lon) {
    // Zugriff auf das globale wettersystem.js (Stunden 0-71)
    // Falls wettersystem.js geladen ist, existiert ein globales Objekt (z.B. `wetterDaten`)
    if (typeof wetterDaten !== 'undefined' && wetterDaten[hoursAhead]) {
        return wetterDaten[hoursAhead]; // Gibt { windDir, windSpeed, waveHeight, currentDir, currentSpeed }
    }
    // Fallback Dummy-Werte
    return { windDir: 240, windSpeed: 12, waveHeight: 0.5, currentDir: 0, currentSpeed: 0 };
}

// --- 2. HELFER-FUNKTIONEN (Schnittwinkel & Geschwindigkeiten) ---

function getBoatSpeed(polar, twa, tws) {
    // Findet die passende Geschwindigkeit im Polardiagramm via Index-Suche
    const twaDeg = Math.abs(twa) > 180 ? 360 - Math.abs(twa) : Math.abs(twa);
    
    let tIndex = polar.tws.findIndex(s => s >= tws);
    let aIndex = polar.twa.findIndex(a => a >= twaDeg);
    
    if (tIndex === -1) tIndex = polar.tws.length - 1;
    if (aIndex === -1) aIndex = polar.twa.length - 1;
    
    return polar.speed[tIndex][aIndex] || 0.1; // Vermeide 0 wegen Division durch 0
}

function calculateDistance(lat1, lon1, lat2, lon2) {
    // Haversine-Formel für echte Seemeilen (NM)
    const R = 3440.065; // Erdradius in Seemeilen
    const dLat = (lat2 - lat1) * Math.PI / 180;
    const dLon = (lon2 - lon1) * Math.PI / 180;
    const a = Math.sin(dLat/2) * Math.sin(dLat/2) +
              Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) * 
              Math.sin(dLon/2) * Math.sin(dLon/2);
    return 2 * R * Math.asin(Math.sqrt(a));
}

function calculateBearing(lat1, lon1, lat2, lon2) {
    // Berechnet den Kompasskurs von Punkt A nach Punkt B
    const dLon = (lon2 - lon1) * Math.PI / 180;
    const lat1Rad = lat1 * Math.PI / 180;
    const lat2Rad = lat2 * Math.PI / 180;
    const y = Math.sin(dLon) * Math.cos(lat2Rad);
    const x = Math.cos(lat1Rad) * Math.sin(lat2Rad) - Math.sin(lat1Rad) * Math.cos(lat2Rad) * Math.cos(dLon);
    return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
}

// --- 3. A* ROUTING ALGORITHMUS ---

export async function planRoute(start, ziel, optionen = { schnellste: true, wellenVermeiden: false }) {
    const polar = await fetchPolarData();
    
    let openSet = [];
    let closedSet = new Set();
    
    const startNode = {
        lat: start.lat,
        lon: start.lon,
        g: 0, // Zeit in Stunden vom Start bis hierhin
        f: calculateDistance(start.lat, start.lon, ziel.lat, ziel.lon) / 5, // Schätzung (5 Knoten Schnitt)
        parent: null,
        hoursAhead: 0
    };
    
    openSet.push(startNode);
    
    // Auflösung des Suchgitters (z.B. 0.05 Grad Schritte (~3 Seemeilen))
    const stepSize = 0.05; 
    const directions = [
        {dLat: stepSize, dLon: 0}, {dLat: -stepSize, dLon: 0},
        {dLat: 0, dLon: stepSize}, {dLat: 0, dLon: -stepSize},
        {dLat: stepSize, dLon: stepSize}, {dLat: -stepSize, dLon: -stepSize},
        {dLat: stepSize, dLon: -stepSize}, {dLat: -stepSize, dLon: stepSize}
    ];

    while (openSet.length > 0) {
        // Knoten mit niedrigstem f-Wert holen
        openSet.sort((a, b) => a.f - b.f);
        let current = openSet.shift();
        
        // Ziel erreicht? (Innerhalb der Schrittweite)
        if (calculateDistance(current.lat, current.lon, ziel.lat, ziel.lon) < (stepSize * 60)) {
            return reconstructPath(current);
        }
        
        const posKey = `${current.lat.toFixed(3)},${current.lon.toFixed(3)}`;
        closedSet.add(posKey);
        
        // Wetter für aktuelle Fahrtzeit holen (begrenzt auf max 71 Stunden im Array)
        const currentHoursAhead = Math.min(Math.floor(current.g), 71);
        const weather = getWeatherData(currentHoursAhead, current.lat, current.lon);
        
        for (let dir of directions) {
            let neighborLat = current.lat + dir.dLat;
            let neighborLon = current.lon + dir.dLon;
            
            const neighborKey = `${neighborLat.toFixed(3)},${neighborLon.toFixed(3)}`;
            if (closedSet.has(neighborKey)) continue;
            
            // 1. Kollisionsprüfung (Land/Küste)
            if (await checkCollision(neighborLat, neighborLon)) continue;
            
            // 2. Nautische Berechnungen
            const dist = calculateDistance(current.lat, current.lon, neighborLat, neighborLon);
            const heading = calculateBearing(current.lat, current.lon, neighborLat, neighborLon);
            
            // TWA (True Wind Angle) berechnen
            let twa = (heading - weather.windDir + 360) % 360;
            if (twa > 180) twa = 360 - twa; 
            
            // Im direkten Gegenwindbereich (TWA < 45) kann nicht gesegelt werden (Kreuzen nötig)
            let boatSpeed = getBoatSpeed(polar, twa, weather.windSpeed);
            if (twa < 40) boatSpeed = boatSpeed * 0.4; // Strafzeit für Kreuzen (VMG sinkt)
            
            // Strömung einrechnen (vereinfachter Vektor)
            const currentEffect = Math.cos((heading - weather.currentDir) * Math.PI / 180) * weather.currentSpeed;
            const effectiveSpeed = Math.max(0.5, boatSpeed + currentEffect); // Mindestfortschritt sichern
            
            // Fahrzeit für dieses Segment berechnen
            let travelTime = dist / effectiveSpeed;
            
            // 3. Kostenfunktion (Optionen aus navissystem.js einbeziehen)
            let cost = travelTime;
            if (optionen.wellenVermeiden && weather.waveHeight > 1.0) {
                cost += weather.waveHeight * 2.0; // Straftopf für hohe Wellen
            }
            
            let tentative_g = current.g + cost;
            
            let neighborNode = openSet.find(n => `${n.lat.toFixed(3)},${n.lon.toFixed(3)}` === neighborKey);
            
            if (!neighborNode) {
                neighborNode = {
                    lat: neighborLat,
                    lon: neighborLon,
                    g: tentative_g,
                    f: tentative_g + (calculateDistance(neighborLat, neighborLon, ziel.lat, ziel.lon) / 5),
                    parent: current,
                    hoursAhead: currentHoursAhead
                };
                openSet.push(neighborNode);
            } else if (tentative_g < neighborNode.g) {
                neighborNode.g = tentative_g;
                neighborNode.f = tentative_g + (calculateDistance(neighborLat, neighborLon, ziel.lat, ziel.lon) / 5);
                neighborNode.parent = current;
            }
        }
    }
    return []; // Keine Route gefunden
}

function reconstructPath(node) {
    let path = [];
    let curr = node;
    while (curr !== null) {
        path.push({ lat: curr.lat, lon: curr.lon, timeElapsed: curr.g });
        curr = curr.parent;
    }
    return path.reverse();
}
