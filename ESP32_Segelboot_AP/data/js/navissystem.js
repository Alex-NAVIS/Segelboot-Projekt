/**
 * NAVIS V2.0 - High-Performance 4D Segler-Engine (Background Worker)
 */

// Pako für Dekompression laden
importScripts('https://cloudflare.com');

// Autonome Caches und Globale Konfiguration
const tileCache = new Map();
const vmgCache = new Map();
let WORKER_BASE_URL = "";

// =========================================================================
// HELPER CLASSES & MATHEMATICS (Kern-Logik)
// =========================================================================
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

function calculateDistance(lat1, lon1, lat2, lon2) {
    const R = 3440.065; 
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

function getMinSailableTWA(polar, tws) {
    if (!polar || !polar.tws || !polar.twa) return 45.0;
    const sArr = polar.tws;
    const aArr = polar.twa;
    
    let s_idx = sArr.findIndex(s => s >= tws);
    if (s_idx === -1) s_idx = sArr.length - 1;

    for (let i = 0; i < aArr.length; i++) {
        const angle = aArr[i];
        const speed = polar.speed[i][s_idx];
        // Nutze deine echten Nullen: Sobald Fahrt > 0.5 Knoten, ist es fahrbar
        if (speed > 0.5 && angle > 0) {
            return parseFloat(angle); 
        }
    }
    return 45.0; 
}

function getBoatSpeedInterpolated(polar, twa, tws) {
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
    if (vmgCache.has(roundedTWS)) return vmgCache.get(roundedTWS);
    let maxVMG = -1; let bestTWA = 45;
    for (let angle = 35; angle <= 60; angle += 1) {
        const speed = getBoatSpeedInterpolated(polar, angle, tws);
        const vmg = speed * Math.cos(angle * Math.PI / 180);
        if (vmg > maxVMG) { maxVMG = vmg; bestTWA = angle; }
    }
    vmgCache.set(roundedTWS, bestTWA);
    return bestTWA;
}

// =========================================================================
// NETWORK FETCHERS & DATA PARSERS (Nutzen absolute URLs)
// =========================================================================
async function fetchPolarData() {
    // ... Implementierung mit fetch(WORKER_BASE_URL + 'polar/myboot.json') ...
}

async function getTileType(lat, lon) {
    // ... Implementierung mit fetch(`${WORKER_BASE_URL}tiles_nav/${cacheKey}.nav.gz`) ...
}

function getWeatherData(hoursAhead, lat, lon, activeWeatherData, weatherFrameStart) {
    // ... 4D Wetter-Interpolation & Tide-Fallback ...
}

function reconstructPath(current) {
    // ... Pfad-Rekonstruktion ...
}

// =========================================================================
// ASYNCHRONER A* ROUTENPLANER KERN
// =========================================================================
async function executeRouteCalculation(start, ziel, activeWeatherData, weatherFrame, options) {
  // Labor-Schaltzentrale für die Fehlersuche (Standard: Nur Wind aktiv!)
    const cfg = {
        wind: optionen.windAktiv !== undefined ? optionen.windAktiv : true,
        welle: optionen.welleAktiv !== undefined ? optionen.welleAktiv : true,
        stroemung: optionen.stroemungAktiv !== undefined ? optionen.stroemungAktiv : true,
        kursMalus: optionen.kursMalusAktiv !== undefined ? optionen.kursMalusAktiv : true,
        xteKorridor: optionen.xteAktiv !== undefined ? optionen.xteAktiv : true
    };

    const polar = await fetchPolarData();
    const openHeap = new MinHeap();
    const openMap = new Map(); 
    const closedSet = new Set();
    const goalRadiusNM = 0.25; 
    const stepSize = 0.01; 
    
    const cosLat = Math.cos(start.lat * Math.PI / 180);
    const stepSizeLon = stepSize / Math.max(0.1, cosLat); 
    
    // 64 High-Res Richtungen generieren (Nautisch eingenordet)
    const NUM_DIRECTIONS = 32;
    const directions = [];
    for (let i = 0; i < NUM_DIRECTIONS; i++) {
        const angleDeg = (i * 360) / NUM_DIRECTIONS;
        const angleRad = angleDeg * Math.PI / 180;
        
        // KORREKTUR: dLat ist Y (Nord/Süd) -> Cosinus. dLon ist X (Ost/West) -> Sinus.
        directions.push({
            dLat: stepSize * Math.cos(angleRad),
            dLon: stepSizeLon * Math.sin(angleRad),
            angleDeg: angleDeg
        });
    }
    
    const startNode = {
        lat: start.lat, lon: start.lon,
        g: 0, 
        cost_g: 0, 
        f: calculateDistance(start.lat, start.lon, ziel.lat, ziel.lon) / 4.5,
        parent: null
    };
    
    // Präziser Key verhindert Gitter-Verzerrungen und Überlagerungen
    const startKey4D = `${start.lat.toFixed(2)},${start.lon.toFixed(2)},${NAVIS_ROUTE.weatherFrame}`;
    openHeap.insert(startNode);
    openMap.set(startKey4D, startNode.cost_g);

    while (openHeap.size() > 0) {
		let current = openHeap.extractMin();

		// const currentHoursAhead = Math.min(NAVIS_ROUTE.weatherFrame + Math.floor(current.g), 71);
		//const currentHoursAhead = Math.min(NAVIS_ROUTE.weatherFrame + Math.round(current.g), 71);
		const currentHoursAhead = Math.min(NAVIS_ROUTE.weatherFrame + Math.floor(current.g), 71);
		
        
        const currentKey4D = `${current.lat.toFixed(2)},${current.lon.toFixed(2)},${currentHoursAhead}`;
        
        if (closedSet.has(currentKey4D)) continue;
        closedSet.add(currentKey4D);
		openMap.delete(currentKey4D);
        
        if (calculateDistance(current.lat, current.lon, ziel.lat, ziel.lon) <= goalRadiusNM) {
            return reconstructPath(current);
        }
        
        let incomingBearing = null;
        if (current.parent) {
            incomingBearing = calculateBearing(current.parent.lat, current.parent.lon, current.lat, current.lon);
        }
        const weather = getWeatherData(currentHoursAhead, current.lat, current.lon);
        
        // Erkennt nun absolut fehlerfrei deine echten Nullen bis 45° aus der JSON-Datei!
        const dynamicMinTWA = Math.max(44.0, getMinSailableTWA(polar, weather.windSpeed));
        const optimalUpwindTWA = Math.max(45.0, getCachedOptimalUpwindTWA(polar, weather.windSpeed));
        
        for (let dir of directions) {
            let neighborLat = current.lat + dir.dLat;
            let neighborLon = current.lon + dir.dLon;
            
            const tileValue = await getTileType(neighborLat, neighborLon);
            if (tileValue === 2) continue; 
            
            const dist = calculateDistance(current.lat, current.lon, neighborLat, neighborLon);
            
            // Nutzt den exakten, vordefinierten Kompasskurs des Rasters (eliminiert Rundungsfehler)
            const heading = dir.angleDeg;
            
            // =========================================================================
            // HARD-GUARD: DER TOTEN WINKEL AUS DEINEN POLARDATEN SPERREN
            // =========================================================================
            let twaDeg = 0;
            if (cfg.wind) {
                let angleDiff = Math.abs(weather.windDir - heading);
                twaDeg = angleDiff > 180 ? 360 - angleDiff : angleDiff;
                
                // Wenn der geplante Kurs im toten Winkel liegt -> Sofort überspringen!
                if (twaDeg < dynamicMinTWA) {
                    continue; 
                }
            }
            
            let boatSpeed = 4.5; 
            if (cfg.wind) {
                boatSpeed = getBoatSpeedInterpolated(polar, twaDeg, weather.windSpeed);
                if (twaDeg < optimalUpwindTWA) {
                    const penalty = Math.pow(twaDeg / optimalUpwindTWA, 3); 
                    boatSpeed = boatSpeed * penalty;
                }
            }
            
            // KORREKTUR: Die Geschwindigkeits-Vektoren sind nun exakt mit den Richtungs-Schritten synchronisiert!
            // dLat nutzt Cosinus -> Also nutzt auch boatY (Nord/Süd) den Cosinus.
            // dLon nutzt Sinus -> Also nutzt auch boatX (Ost/West) den Sinus.
            let boatX = boatSpeed * Math.sin(heading * Math.PI / 180);
            let boatY = boatSpeed * Math.cos(heading * Math.PI / 180);
            let streamX = 0, streamY = 0;
            
            if (cfg.stroemung) {
                const currentRad = weather.currentDir * Math.PI / 180;
                streamX = weather.currentSpeed * Math.sin(currentRad);
                streamY = weather.currentSpeed * Math.cos(currentRad);
            }
            let speedOverGround = Math.sqrt((boatX + streamX) ** 2 + (boatY + streamY) ** 2);
            
            if (cfg.welle) {
                let waveEncounterAngle = Math.abs(heading - weather.waveDir);
				if (waveEncounterAngle > 180)
					waveEncounterAngle = 360 - waveEncounterAngle;
				
                const waveCos = Math.cos(waveEncounterAngle * Math.PI / 180);
                let waveFactor = 1.0;
                if (waveCos > 0) waveFactor = Math.max(0.4, 1.0 - (waveCos * weather.waveHeight * 0.4));
                else waveFactor = Math.min(1.5, 1.0 + (Math.abs(waveCos) * weather.waveHeight * 0.15));
                speedOverGround = speedOverGround * waveFactor;
            }
            
            const effectiveSpeed = Math.max(0.1, speedOverGround);
			let travelTime = dist / effectiveSpeed;
			const targetBearing = calculateBearing(current.lat, current.lon, ziel.lat, ziel.lon);
			let targetAngle = Math.abs(heading - targetBearing);

			if (targetAngle > 180)
				targetAngle = 360 - targetAngle;
			const vmgToTarget = boatSpeed * Math.cos(targetAngle * Math.PI / 180);
			// Kurs bringt uns nicht zum Ziel
			if (vmgToTarget < 0) {
				continue;
			}

			let cost = travelTime;

			// schlechte Zielannäherung bestrafen
			cost += (boatSpeed - vmgToTarget) * 0.25;

			// Bei Flaute stärker Richtung Ziel zwingen
			if (weather.windSpeed < 6) {
				cost += (targetAngle / 180) * 0.15;
			}
			
			const oldDistance = calculateDistance(current.lat, current.lon, ziel.lat, ziel.lon);
			const newDistance = calculateDistance(neighborLat, neighborLon, ziel.lat, ziel.lon);
			const progressNM = oldDistance - newDistance;
			
			const profile = optionen.profile || "fastest";
			// Standard-Heuristikgewicht
			let heuristicWeight = 1.0;
			// =====================================================
			// GLOBALE STRÖMUNGS-BEWERTUNG
			// Gilt für ALLE Routing-Profile
			// =====================================================
			let currentAngle =
				Math.abs(((weather.currentDir - heading + 540) % 360) - 180);

			// Rückenstrom = 180°
			// Gegenstrom = 0°
			// Querströmung = 90°
			const currentAlignment = Math.cos(currentAngle * Math.PI / 180);

			// -1 = voller Gegenstrom
			//  0 = Querströmung
			// +1 = voller Rückenstrom

			// =====================================================
			// GLOBALE STRÖMUNGS-BEWERTUNG
			// =====================================================
			if (cfg.stroemung) {
				if (currentAlignment < 0) {
					// Gegenstrom deutlich vermeiden
					cost += Math.abs(currentAlignment) * weather.currentSpeed * 1.50;
				} else {
					// Rückenstrom stärker nutzen
					cost -= currentAlignment * weather.currentSpeed * 1.00;
				}
			}

			// ------------------------------------
			// ⚡ Schnellste Route
			// ------------------------------------
			if (profile === "fastest") {
				heuristicWeight = 2.0;
				// Nur bei Flaute
	
			}

			// ------------------------------------
			// ⚓ Küstenroute
			// ------------------------------------
			else if (profile === "coastal") {
				heuristicWeight = 1.5;
				if (tileValue === 1) {
					// Küstenzellen massiv bevorzugen
					cost *= 0.45;
				} else {
					// Offene See deutlich verteuern
					cost *= 1.50;
				}
				// Hohe See zusätzlich vermeiden
				cost += weather.waveHeight * 0.75;
				// Starker Wind küstennah bevorzugt umgehen
				if (weather.windSpeed > 20) {
					cost += (weather.windSpeed - 20) * 0.20;
				}
			}

			// ------------------------------------
			// 🛡 Sicherste Route
			// ------------------------------------
			else if (profile === "safest") {

				heuristicWeight = 1.8;

				cost += weather.waveHeight * 3.0;
				cost += weather.windSpeed * 0.40;

				if (weather.waveHeight > 2.5) {
					cost += (weather.waveHeight - 2.5) * 4.0;
				}

				if (weather.windSpeed > 25) {
					cost += (weather.windSpeed - 25) * 0.8;
				}
			}

			// ------------------------------------
			// 🛋 Komfort-Route
			// ------------------------------------
			else if (profile === "comfort") {
				heuristicWeight = 1.8;
				cost += weather.waveHeight * 2.5;
				cost += weather.windSpeed * 0.20;
				let waveEncounterAngle = Math.abs(heading - weather.waveDir);
				if (waveEncounterAngle > 180) {
					waveEncounterAngle = 360 - waveEncounterAngle;
				}

				// Harte Bugsee
				if (waveEncounterAngle < 45) {
					cost += weather.waveHeight * 6.0;
				}

				// Schräge See
				else if (waveEncounterAngle < 90) {
					cost += weather.waveHeight * 3.0;
				}

				// Achterliche See
				else if (waveEncounterAngle > 150) {
					cost -= weather.waveHeight * 1.0;
				}

				// Krängung vermeiden
				if (twaDeg > 60 && twaDeg < 120) {
					cost += weather.windSpeed * 0.30;
				}

				// Starkwind deutlich meiden
				if (weather.windSpeed > 20) {
					cost += (weather.windSpeed - 20) * 0.50;
				}

				// Hohe Wellen exponentiell bestrafen
				if (weather.waveHeight > 2.0) {
					cost += Math.pow(weather.waveHeight - 2.0, 2 ) * 3.0;
				}
			}

			// ------------------------------------
			// 💨 Wind vermeiden
			// ------------------------------------
			else if (profile === "avoid_wind") {

				heuristicWeight = 1.6;

				if (weather.windSpeed > 15) {

					cost +=
						Math.pow(
							(weather.windSpeed - 15) / 5,
							2
						);
				}
			}

			// ------------------------------------
			// 🌊 Wellen vermeiden
			// ------------------------------------
			else if (profile === "avoid_waves") {

				heuristicWeight = 1.8;

				cost +=
					Math.pow(weather.waveHeight, 2) * 3.0;
			}

			// ------------------------------------
			// 🔋 Energieeffizient
			// ------------------------------------
			else if (profile === "eco") {

				heuristicWeight = 1.5;

				// Kreuzschläge vermeiden
				if (twaDeg < 60) {

					cost +=
						Math.pow(
							(60 - twaDeg) / 10,
							2
						);
				}

				// Zu tiefe Kurse vermeiden
				if (twaDeg > 150) {

					cost +=
						Math.pow(
							(twaDeg - 150) / 10,
							2
						);
				}

				cost += weather.waveHeight * 0.75;
			}

			// ------------------------------------
			// Kurswechsel-Malus
			// ------------------------------------
			if (cfg.kursMalus && incomingBearing !== null) {
				let headingChange =	Math.abs(heading - incomingBearing);
				if (headingChange > 180)
					headingChange = 360 - headingChange;
				if (headingChange > 4.0) {
					cost += Math.pow(headingChange / 30, 2) * 1.50;
				}
			}

            // Cross-Track-Error (XTE Korridor schließt unkontrollierte Bögen aus)
            if (cfg.xteKorridor) {
                const distStartToTarget = calculateDistance(start.lat, start.lon, ziel.lat, ziel.lon);
                if (distStartToTarget > 0.3) {
                    const distStartToNeighbor = calculateDistance(start.lat, start.lon, neighborLat, neighborLon);
                    const bearingStartToTarget = calculateBearing(start.lat, start.lon, ziel.lat, ziel.lon);
                    const bearingStartToNeighbor = calculateBearing(start.lat, start.lon, neighborLat, neighborLon);
                    let angleDiff = Math.abs(bearingStartToNeighbor - bearingStartToTarget);
                    if (angleDiff > 180) angleDiff = 360 - angleDiff;
                    
                    const crossTrackErrorNM = distStartToNeighbor * Math.sin(angleDiff * Math.PI / 180);
                    const maxAllowedCrosstrackNM = Math.max(2.0, distStartToTarget * 0.15);
					
                    if (Math.abs(crossTrackErrorNM) > maxAllowedCrosstrackNM) {
                        const xteExcess = Math.abs(crossTrackErrorNM) - maxAllowedCrosstrackNM;
                        cost += travelTime * Math.pow(xteExcess, 2) * 4.0; 
                    }
                }
            }

            let tentative_g = current.g + travelTime;      
            let tentative_cost_g = current.cost_g + cost; 
            
			//const neighborHoursAhead = Math.min(NAVIS_ROUTE.weatherFrame +	Math.floor(tentative_g), 71);
			//const neighborHoursAhead = Math.min(NAVIS_ROUTE.weatherFrame +	Math.round(tentative_g), 71);
			const neighborHoursAhead = Math.min(NAVIS_ROUTE.weatherFrame + Math.floor(tentative_g), 71);
			
			const neighborKey4D = `${neighborLat.toFixed(2)},${neighborLon.toFixed(2)},${neighborHoursAhead}`;
	
            if (closedSet.has(neighborKey4D)) continue;
            
            const existingCostG = openMap.get(neighborKey4D);
            
            if (existingCostG === undefined || tentative_cost_g < existingCostG) {
                // AUSBALANCIERTE HEURISTIK: Berechnet die verbleibende Luftlinie zum Ziel stabil geteilt durch 4.0 Knoten.
                // Das zieht den Suchbaum sauber nach vorn, ohne Am-Wind-Kurse künstlich abzustrafen.
                const h = calculateDistance(neighborLat, neighborLon, ziel.lat, ziel.lon) / 6.0;
                const etaUTC = new Date(
					NAVIS_ROUTE.departureTime.getTime()
					+ tentative_g * 3600 * 1000
				);
                const newNode = {
					lat: neighborLat,
					lon: neighborLon,
					g: tentative_g,
					cost_g: tentative_cost_g,

					// Profilgewichtete Heuristik
					f: tentative_cost_g + (h * heuristicWeight),

					// ETA
					etaHours: tentative_g,
					etaUTC: etaUTC.toISOString(),
					forecastHour: neighborHoursAhead,

					// Navigation
					heading: heading,
					distance: dist,

					// Boot
					boatSpeed: boatSpeed,
					sog: speedOverGround,

					// Wind
					windSpeed: weather.windSpeed,
					windDir: weather.windDir,
					twa: twaDeg,

					// Strömung
					currentSpeed: weather.currentSpeed,
					currentDir: weather.currentDir,
					currentAngle:
						Math.abs(((weather.currentDir - heading + 540) % 360) - 180),

					// Wellen
					waveHeight: weather.waveHeight,
					waveDir: weather.waveDir,
					waveAngle:
						Math.abs(((weather.waveDir - heading + 540) % 360) - 180),

					parent: current
				};
                openHeap.insert(newNode);
                openMap.set(neighborKey4D, tentative_cost_g);
            }
        }
    }
    return []; 
}

// =========================================================================
// THREAD COMMUNICATIONS RECEIVER
// =========================================================================
self.onmessage = async function (e) {
    const { start, ziel, activeWeatherData, weatherFrame, options, baseUrl } = e.data;
    WORKER_BASE_URL = baseUrl;
    try {
        const calculatedRoute = await executeRouteCalculation(start, ziel, activeWeatherData, weatherFrame, options);
        self.postMessage({ success: true, route: calculatedRoute });
    } catch (err) {
        self.postMessage({ success: false, error: err.message });
    }
};
