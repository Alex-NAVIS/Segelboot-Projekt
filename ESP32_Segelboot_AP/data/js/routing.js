/**
 * A*-Routenplaner für das ESP32-Segelboot-Projekt
 * STUFE 2.9: Fertig für Kartentest, inklusive RAM-Kachel-Optimierung, openMap-Schutz & Cache-Fix
 */

const tileCache = new Map(); 
const vmgCache = new Map();  

// --- 1. PRIORITY QUEUE (BINARY MIN-HEAP) ---
class MinHeap {
    constructor() { this.data = []; }
    insert(node) {
        this.data.push(node);
        this.upHeap(this.data.length - 1);
    }
    extractMin() {
        if (this.data.length === 0) return null;
        const min = this.data[0];
        const end = this.data.pop();
        if (this.data.length > 0) {
            this.data[0] = end;
            this.downHeap(0);
        }
        return min;
    }
    size() { return this.data.length; }
    upHeap(i) {
        while (i > 0) {
            const p = Math.floor((i - 1) / 2);
            if (this.data[i].f >= this.data[p].f) break;
            const tmp = this.data[i]; this.data[i] = this.data[p]; this.data[p] = tmp;
            i = p;
        }
    }
    downHeap(i) {
        const len = this.data.length;
        while (2 * i + 1 < len) {
            let left = 2 * i + 1, right = left + 1, best = left;
            if (right < len && this.data[right].f < this.data[left].f) best = right;
            if (this.data[i].f <= this.data[best].f) break;
            const tmp = this.data[i]; this.data[i] = this.data[best]; this.data[best] = tmp;
            i = best;
        }
    }
}

// --- 2. DATEN-SCHNITTSTELLEN UND CACHING ---
async function fetchPolarData() {
    try {
        const response = await fetch('polar/myboot.json');
        const boot = await response.json();
        
        // 1. Windgeschwindigkeiten (Spalten) sichern und numerisch sortieren
        const tws = boot.windSpeeds.map(Number).sort((a, b) => a - b);
        
        // 2. Windwinkel (Zeilen) extrahieren und numerisch sortieren
        const twa = Object.keys(boot.polar)
            .map(Number)
            .sort((a, b) => a - b);

        // 3. Matrix absolut fehlerfrei aufbauen
        const speed = twa.map(angle => {
            // Finde den exakten Original-Key aus der JSON, der numerisch matcht
            const originalAngleKey = Object.keys(boot.polar).find(k => Number(k) === angle);
            const angleData = boot.polar[originalAngleKey] || {};
            
            return tws.map(ws => {
                // Finde den exakten Windgeschwindigkeits-Key in den Unterdaten
                const originalWsKey = Object.keys(angleData).find(k => Number(k) === ws);
                return Number(angleData[originalWsKey]) || 0.05; // 0.05 als minimaler Vortrieb gegen Division durch 0
            });
        });

        console.log("Polardaten erfolgreich geladen und normiert:", { tws, twa, matrixSize: `${speed.length}x${speed[0].length}` });

        return { tws, twa, speed };
    } catch (e) {
        console.warn("Polardaten-Parser-Fehler, lade sicheres Ostsee-Fallback", e);
        return {
            "tws":,
            "twa":,
            "speed": [
                [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05],
                [1.2, 2.1, 3.1, 4.0, 4.4, 4.6, 4.7, 4.8],
                [2.8, 4.2, 5.3, 5.9, 6.2, 6.4, 6.5, 6.6],
                [3.5, 5.1, 6.1, 6.6, 6.9, 7.1, 7.2, 7.4],
                [3.9, 5.5, 6.4, 6.9, 7.2, 7.5, 7.7, 8.0],
                [4.1, 5.7, 6.6, 7.1, 7.5, 7.8, 8.1, 8.5],
                [4.2, 5.9, 6.8, 7.3, 7.7, 8.1, 8.5, 9.1],
                [3.9, 5.6, 6.6, 7.1, 7.5, 7.9, 8.3, 9.0],
                [3.1, 4.7, 5.7, 6.3, 6.8, 7.2, 7.7, 8.5],
                [2.4, 3.8, 4.8, 5.5, 6.0, 6.4, 6.8, 7.6]
            ]
        };
    }
}

async function checkCollision(lat, lon) {
    const tileX = Math.floor(lon);
    const tileY = Math.floor(lat);
    const cacheKey = `${tileX}_${tileY}`;

    if (tileCache.has(cacheKey)) {
        return tileCache.get(cacheKey);
    }

    try {
        const response = await fetch(`tiles_nav/${tileX}_${tileY}.json`);
        const tileData = await response.json();
        tileCache.set(cacheKey, tileData.isLand);
        return tileData.isLand;
    } catch (e) {
        // KRITIK-FIX 1: Auch fehlende Dateien als "false" (Wasser) im Cache sichern, um Dauer-Fetchs zu killen
        tileCache.set(cacheKey, false);
        return false; 
    }
}

function getWeatherData(hoursAhead, lat, lon) {
    if (typeof wetterDaten !== 'undefined' && wetterDaten[hoursAhead]) {
        return wetterDaten[hoursAhead]; 
    }
    // Standard-Wind aus West (270 Grad), Strömung setzt nach Nord-Ost
    return { windDir: 270, windSpeed: 14, waveHeight: 0.6, currentDir: 45, currentSpeed: 1.0 };
}

// --- 3. NAUTISCHE MATHEMATIK ---
function getBoatSpeedInterpolated(polar, twa, tws) {
        console.log("POLAR INPUT:", polar);

    const sArr = polar.tws;
    const aArr = polar.twa;

    console.log("sArr =", sArr);
    console.log("aArr =", aArr);
	const twaDeg = Math.abs(twa) > 180 ? 360 - Math.abs(twa) : Math.abs(twa);
    const sArr = polar.tws;
    const aArr = polar.twa;
    
    let s0 = sArr.findIndex(s => s >= tws) - 1;
    if (s0 < 0) s0 = 0; if (s0 >= sArr.length - 1) s0 = sArr.length - 2;
    let s1 = s0 + 1;

    let a0 = aArr.findIndex(a => a >= twaDeg) - 1;
    if (a0 < 0) a0 = 0; if (a0 >= aArr.length - 1) a0 = aArr.length - 2;
    let a1 = a0 + 1;

    const q11 = polar.speed[a0][s0], q21 = polar.speed[a1][s0];
    const q12 = polar.speed[a0][s1], q22 = polar.speed[a1][s1];

    const t_fac = (tws - sArr[s0]) / (sArr[s1] - sArr[s0]);
    const a_fac = (twaDeg - aArr[a0]) / (aArr[a1] - aArr[a0]);

    const r1 = (1 - t_fac) * q11 + t_fac * q12;
    const r2 = (1 - t_fac) * q21 + t_fac * q22;
    
    return Math.max(0.05, (1 - a_fac) * r1 + a_fac * r2);
}

function getCachedOptimalUpwindTWA(polar, tws) {
    const roundedTWS = Math.round(tws);
    if (vmgCache.has(roundedTWS)) {
        return vmgCache.get(roundedTWS);
    }

    let maxVMG = -1;
    let bestTWA = 45;
    
    for (let angle = 35; angle <= 60; angle += 1) {
        const speed = getBoatSpeedInterpolated(polar, angle, tws);
        const vmg = speed * Math.cos(angle * Math.PI / 180);
        if (vmg > maxVMG) {
            maxVMG = vmg;
            bestTWA = angle;
        }
    }
    vmgCache.set(roundedTWS, bestTWA);
    return bestTWA;
}

function calculateDistance(lat1, lon1, lat2, lon2) {
    const R = 3440.065; // NM
    const dLat = (lat2 - lat1) * Math.PI / 180;
    const dLon = (lon2 - lon1) * Math.PI / 180;
    const a = Math.sin(dLat/2) * Math.sin(dLat/2) +
              Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) * 
              Math.sin(dLon/2) * Math.sin(dLon/2);
    return 2 * R * Math.asin(Math.sqrt(a));
}

function calculateBearing(lat1, lon1, lat2, lon2) {
    const dLon = (lon2 - lon1) * Math.PI / 180;
    const lat1Rad = lat1 * Math.PI / 180;
    const lat2Rad = lat2 * Math.PI / 180;
    const y = Math.sin(dLon) * Math.cos(lat2Rad);
    const x = Math.cos(lat1Rad) * Math.sin(lat2Rad) - Math.sin(lat1Rad) * Math.cos(lat2Rad) * Math.cos(dLon);
    return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
}

// --- 4. A* CORE ENGINE ---
export async function planRoute(start, ziel, optionen = { schnellste: true, wellenVermeiden: false }) {
    const polar = await fetchPolarData();
	console.log("=== POLAR TEST ===");
console.log(polar);

console.log("polar.tws:", polar?.tws);
console.log("polar.twa:", polar?.twa);
console.log("polar.speed:", polar?.speed);
    console.log("POLAR GELADEN:", polar);
console.log("polar.tws =", polar.tws);
console.log("polar.twa =", polar.twa);
    const openHeap = new MinHeap();
    const openMap = new Map(); // Speicher- und Verlaufs-Kontrolle
    const closedSet = new Set();
    
    const goalRadiusNM = 0.5; // KRITIK-FIX 1: Sauberes Ankommen
    const stepSize = 0.01; 
    
    // Geographische Anpassung der Schrittweite (Längengrade verjüngen sich zum Pol)
    const cosLat = Math.cos(start.lat * Math.PI / 180);
    const stepSizeLon = stepSize / Math.max(0.1, cosLat); // KRITIK-FIX 3: Minimiert geografische Verzerrung
    
    const rHalfLat = stepSize * 0.4142;
    const rHalfLon = stepSizeLon * 0.4142;

    const directions = [
        {dLat: stepSize, dLon: 0}, {dLat: -stepSize, dLon: 0}, {dLat: 0, dLon: stepSizeLon}, {dLat: 0, dLon: -stepSizeLon},
        {dLat: stepSize, dLon: stepSizeLon}, {dLat: -stepSize, dLon: -stepSizeLon}, {dLat: stepSize, dLon: -stepSizeLon}, {dLat: -stepSize, dLon: stepSizeLon},
        // 16 Richtungen ausbalanciert (Tippfehler rHalfHalfLon korrigiert)
        {dLat: stepSize, dLon: rHalfLon}, {dLat: stepSize, dLon: -rHalfLon}, {dLat: -stepSize, dLon: rHalfLon}, {dLat: -stepSize, dLon: -rHalfLon},
        {dLat: rHalfLat, dLon: stepSizeLon}, {dLat: rHalfLat, dLon: -stepSizeLon}, {dLat: -rHalfLat, dLon: stepSizeLon}, {dLat: -rHalfLat, dLon: -stepSizeLon}
    ];
    
    const startNode = {
        lat: start.lat, lon: start.lon,
        g: 0, f: calculateDistance(start.lat, start.lon, ziel.lat, ziel.lon) / 5,
        parent: null
    };
    
    openHeap.insert(startNode);
    openMap.set(`${start.lat.toFixed(4)},${start.lon.toFixed(4)}`, startNode.g);

    while (openHeap.size() > 0) {
        let current = openHeap.extractMin();
        const currentKey = `${current.lat.toFixed(4)},${current.lon.toFixed(4)}`;
        
        if (closedSet.has(currentKey)) continue;
        closedSet.add(currentKey);
        
        if (calculateDistance(current.lat, current.lon, ziel.lat, ziel.lon) <= goalRadiusNM) {
            return reconstructPath(current);
        }
        
        const currentHoursAhead = Math.min(Math.floor(current.g), 71);
        const weather = getWeatherData(currentHoursAhead, current.lat, current.lon);
        const optimalUpwindTWA = getCachedOptimalUpwindTWA(polar, weather.windSpeed);
        
        for (let dir of directions) {
            let neighborLat = current.lat + dir.dLat;
            let neighborLon = current.lon + dir.dLon;
            const neighborKey = `${neighborLat.toFixed(4)},${neighborLon.toFixed(4)}`;
            
            if (closedSet.has(neighborKey)) continue;
            if (await checkCollision(neighborLat, neighborLon)) continue;
            
            const dist = calculateDistance(current.lat, current.lon, neighborLat, neighborLon);
            const heading = calculateBearing(current.lat, current.lon, neighborLat, neighborLon);
            
            let twa = (heading - weather.windDir + 360) % 360;
            if (twa > 180) twa = 360 - twa; 
            
            let boatSpeed = getBoatSpeedInterpolated(polar, twa, weather.windSpeed);
            
            if (twa < optimalUpwindTWA) {
                // Physikalischer Strömungsabriss im toten Winkel (Ungenauigkeit bestrafen)
                const penalty = Math.pow(twa / optimalUpwindTWA, 3); 
                boatSpeed = boatSpeed * penalty;
            }
            
            // Strömungsvektor-Kompensation (Vorbereitung COG/SOG)
            const headingRad = heading * Math.PI / 180;
            const currentRad = weather.currentDir * Math.PI / 180;
            const boatX = boatSpeed * Math.sin(headingRad);
            const boatY = boatSpeed * Math.cos(headingRad);
            const streamX = weather.currentSpeed * Math.sin(currentRad);
            const streamY = weather.currentSpeed * Math.cos(currentRad);
            
            const speedOverGround = Math.sqrt((boatX + streamX) ** 2 + (boatY + streamY) ** 2);
            const effectiveSpeed = Math.max(0.1, speedOverGround);
            
            let travelTime = dist / effectiveSpeed;
            let cost = travelTime;
            
            if (optionen.wellenVermeiden && weather.waveHeight > 0.8) {
                cost += travelTime * (weather.waveHeight * 0.4); 
            }
            let tentative_g = current.g + cost;
            
            // KRITIK-FIX 2: Nutze openMap zur echten Prüfung existierender g-Kosten
            const existingG = openMap.get(neighborKey);
            
            if (existingG === undefined || tentative_g < existingG) {
                const h = calculateDistance(neighborLat, neighborLon, ziel.lat, ziel.lon) / Math.max(3, boatSpeed);
                const newNode = {
                    lat: neighborLat, lon: neighborLon,
                    g: tentative_g, f: tentative_g + h,
                    parent: current
                };
                
                openHeap.insert(newNode);
                openMap.set(neighborKey, tentative_g);
            }
        }
    }
    return []; 
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
window.planRoute = planRoute;
// Macht die Modul-Funktion für das globale Navissystem (navissystem.js) sichtbar
if (typeof window !== 'undefined') {
    window.planRoute = planRoute;
}