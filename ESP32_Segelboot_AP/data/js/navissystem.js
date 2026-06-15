console.log("NAVIS VERSION TEST 12345");
/* ----------------------------------------------------------------------
🧭 NAVIS V2.0 Kontext-Routing Engine
---------------------------------------------------------------------- */
const NAVIS_ROUTE = {
    start: null,
    ziel: null,
    waypoints: [],
    departureTime: null,
    profile: "fast",
    strategy: "eta",
    weatherFrame: 0
};

// --------------------------------------------------
// NAVIS Darstellung (Leaflet)
// --------------------------------------------------
const NAVIS_VISUAL = {
    startMarker: null,
    zielMarker: null,
    waypointMarkers: [],
    routeLine: null 
};

function updateNavisPolyline() {
    console.log("=== 🌐 updateNavisPolyline gestartet ===");
    
    if (NAVIS_VISUAL.routeLine) {
        map.removeLayer(NAVIS_VISUAL.routeLine);
        NAVIS_VISUAL.routeLine = null;
    }

    if (!NAVIS_ROUTE.start || !NAVIS_ROUTE.ziel) {
        console.log("ℹ️ Keine Linie: Start oder Ziel fehlt.");
        return;
    }

    const pathCoordinates = [];
    
    // Start hinzufügen
    pathCoordinates.push([NAVIS_ROUTE.start.lat, NAVIS_ROUTE.start.lon]);

    // Wegpunkte hinzufügen
    if (NAVIS_ROUTE.waypoints && NAVIS_ROUTE.waypoints.length > 0) {
        NAVIS_ROUTE.waypoints.forEach(wp => {
            pathCoordinates.push([wp.lat, wp.lon]);
        });
    }

    // Ziel hinzufügen (Hier stand vorher NAIVS statt NAVIS!)
    pathCoordinates.push([NAVIS_ROUTE.ziel.lat, NAVIS_ROUTE.ziel.lon]);
    
    console.log("📐 Versuche Linie zu zeichnen mit Koordinaten:", pathCoordinates);

    try {
        NAVIS_VISUAL.routeLine = L.polyline(pathCoordinates, {
            color: '#00b8d4', 
            weight: 5, 
            opacity: 0.9, 
            dashArray: '8, 12'
        }).addTo(map);
        console.log("✅ Polyline erfolgreich zur Karte hinzugefügt!");
    } catch (error) {
        console.error("🔴 Leaflet-Fehler beim Zeichnen:", error);
    }
}

function createNavisEmojiIcon(emoji) {
    return L.divIcon({
        className: 'navis-custom-emoji-icon',
        html: `
            <div style="
                font-size: 28px;
                line-height: 1;
                text-shadow: 0 2px 4px rgba(0,0,0,0.8);
                display:flex;
                align-items:center;
                justify-content:center;
            ">
                ${emoji}
            </div>
        `,
        iconSize: [32, 32],
        iconAnchor: [16, 16],
        popupAnchor: [0, -16]
    });
}

function createNavisNode(latlng) {
    return {
        lat: latlng.lat,
        lon: latlng.lng,
        tileX: Math.floor(latlng.lng / TILE_SPAN),
        tileY: Math.floor(latlng.lat / TILE_SPAN)
    };
}

const TILE_SPAN = 0.256; 

// Initialisiere die Abfahrtszeit im Panel auf die aktuelle Uhrzeit
document.addEventListener("DOMContentLoaded", () => {
    const now = new Date();
    const localISO = new Date(now.getTime() - now.getTimezoneOffset() * 60000).toISOString().slice(0, 16);
    const timeInput = document.getElementById("routing-departure-time");
    if (timeInput) timeInput.value = localISO;
});

// Leaflet Rechtsklick-Menü im NAVIS Design
if (typeof map !== 'undefined') {
    map.on('contextmenu', function (e) {
        const menuContent = document.createElement('div');
        menuContent.style.display = 'flex';
        menuContent.style.flexDirection = 'column';
        menuContent.style.gap = '6px';
        menuContent.style.minWidth = '190px';
        menuContent.style.padding = '4px';
        menuContent.style.fontFamily = '"Segoe UI",Roboto,Arial,sans-serif';
        menuContent.style.fontSize = '12px';

        function styleNavisMenuButton(btn) {
            btn.style.width = '100%';
            btn.style.padding = '7px 10px';
            btn.style.background = '#003d52';
            btn.style.color = '#ffffff';
            btn.style.border = '1px solid #007a99';
            btn.style.borderRadius = '4px';
            btn.style.cursor = 'pointer';
            btn.style.fontSize = '12px';
            btn.style.textAlign = 'left';
            btn.style.transition = 'all 0.2s ease';
            btn.onmouseenter = () => {
                btn.style.background = '#00b8d4';
                btn.style.color = '#001a26';
            };
            btn.onmouseleave = () => {
                btn.style.background = '#003d52';
                btn.style.color = '#ffffff';
            };
        }

        // 📍 Startpunkt setzen
        const btnStart = document.createElement('button');
        btnStart.innerText = '📍 Startpunkt setzen';
        styleNavisMenuButton(btnStart);
        btnStart.onclick = () => {
            setNavisPoint('start', e.latlng);
            map.closePopup();
        };

        // 🏁 Zielpunkt setzen
        const btnZiel = document.createElement('button');
        btnZiel.innerText = '🏁 Zielpunkt setzen';
        styleNavisMenuButton(btnZiel);
        btnZiel.onclick = () => {
            setNavisPoint('ziel', e.latlng);
            map.closePopup();
        };

        // 🧭 Wegpunkt hinzufügen
        const btnWp = document.createElement('button');
        btnWp.innerText = '🧭 Wegpunkt hinzufügen';
        styleNavisMenuButton(btnWp);
        btnWp.onclick = () => {
            addNavisWaypoint(e.latlng);
            map.closePopup();
        };

        // ❌ Abbrechen
        const btnCancel = document.createElement('button');
        btnCancel.innerText = '❌ Abbrechen';
        styleNavisMenuButton(btnCancel);
        btnCancel.style.background = '#5c2424';
        btnCancel.style.border = '1px solid #ff5555';
        btnCancel.onmouseenter = () => {
            btnCancel.style.background = '#aa3333';
            btnCancel.style.color = '#ffffff';
        };

        btnCancel.onmouseleave = () => {
            btnCancel.style.background = '#5c2424';
            btnCancel.style.color = '#ffffff';
        };

        btnCancel.onclick = () => {
            map.closePopup();
        };

        menuContent.appendChild(btnStart);
        menuContent.appendChild(btnZiel);
        menuContent.appendChild(btnWp);
        menuContent.appendChild(btnCancel);

        L.popup({
            closeButton: false,
            autoClose: true,
            className: 'navis-context-popup'
        })
            .setLatLng(e.latlng)
            .setContent(menuContent)
            .openOn(map);
    });
}

function styleNavisMenuButton(btn){
    btn.style.width = '100%';
    btn.style.padding = '7px 10px';
    btn.style.background = '#003d52';
    btn.style.color = '#ffffff';
    btn.style.border = '1px solid #007a99';
    btn.style.borderRadius = '4px';
    btn.style.cursor = 'pointer';
    btn.style.fontSize = '12px';
    btn.style.textAlign = 'left';
    btn.style.transition = 'all 0.2s ease';
    btn.onmouseenter = () => {
        btn.style.background = '#00b8d4';
        btn.style.color = '#001a26';
    };
    btn.onmouseleave = () => {
        btn.style.background = '#003d52';
        btn.style.color = '#ffffff';
    };
}

// Start- oder Zielpunkt visuell setzen
function setNavisPoint(type, latlng) {
    const point = createNavisNode(latlng), isStart = type === 'start', key = isStart ? 'start' : 'ziel', markerKey = isStart ? 'startMarker' : 'zielMarker', emoji = isStart ? "📍" : "🏁", label = isStart ? "Startpunkt" : "Zielpunkt", displayId = `route-${key}-display`;
    NAVIS_ROUTE[key] = point;
    if (NAVIS_VISUAL[markerKey]) map.removeLayer(NAVIS_VISUAL[markerKey]);

    NAVIS_VISUAL[markerKey] = L.marker([point.lat, point.lon], { draggable: true, icon: createNavisEmojiIcon(emoji) }).addTo(map);
    NAVIS_VISUAL[markerKey].bindPopup(`<b>${emoji} ${label}</b><br>Ziehen zum Verschieben.`);
    
    // Dieses Event feuert durchgehend WÄHREND des Ziehens
    NAVIS_VISUAL[markerKey].on('drag', function(event) {
        const pos = event.target.getLatLng();
        NAVIS_ROUTE[key] = createNavisNode(pos);
        document.getElementById(displayId).innerText = `Lat: ${pos.lat.toFixed(4)} | Lon: ${pos.lng.toFixed(4)}`;
        updateNavisPolyline(); 
    });

    // Dieses Event feuert beim LOSLASSEN
    NAVIS_VISUAL[markerKey].on('dragend', function(event) {
        const pos = event.target.getLatLng();
        console.log(`${emoji} ${label.split('punkt')[0]} verschoben -> Tile t_${NAVIS_ROUTE[key].tileX}_${NAVIS_ROUTE[key].tileY}`);
        updateNavisPolyline();
    });

    document.getElementById(displayId).innerText = `Lat: ${point.lat.toFixed(4)} | Lon: ${point.lon.toFixed(4)}`;
    updateNavisPolyline();
}

// Zwischen-Wegpunkte dynamisch hinzufügen
function addNavisWaypoint(latlng) {
    const wpIndex = NAVIS_ROUTE.waypoints.length + 1, routeNode = createNavisNode(latlng);
    NAVIS_ROUTE.waypoints.push(routeNode);

    const wpMarker = L.marker([routeNode.lat, routeNode.lon], { draggable: true, icon: createNavisEmojiIcon("🧭") }).addTo(map);
    wpMarker.bindPopup(`🧭 Wegpunkt #${wpIndex}`);
    NAVIS_VISUAL.waypointMarkers.push(wpMarker);

    // Dieses Event feuert durchgehend WÄHREND des Ziehens
    wpMarker.on('drag', function(event) {
        const pos = event.target.getLatLng(), markerIndex = NAVIS_VISUAL.waypointMarkers.indexOf(wpMarker);
        if (markerIndex > -1) {
            NAVIS_ROUTE.waypoints[markerIndex] = createNavisNode(pos);
            updateNavisPolyline(); 
        }
    });

    // Dieses Event feuert beim LOSLASSEN
    wpMarker.on('dragend', function(event) {
        const markerIndex = NAVIS_VISUAL.waypointMarkers.indexOf(wpMarker);
        if (markerIndex > -1) {
            console.log(`🧭 WP ${markerIndex + 1} verschoben -> Tile t_${NAVIS_ROUTE.waypoints[markerIndex].tileX}_${NAVIS_ROUTE.waypoints[markerIndex].tileY}`);
            updateNavisPolyline();
        }
    });

    wpMarker.on('contextmenu', function(event) {
        L.DomEvent.stopPropagation(event);
        
        // Marker-Index dynamisch ermitteln (nicht den alten statischen wpIndex nutzen!)
        const currentIndex = NAVIS_VISUAL.waypointMarkers.indexOf(wpMarker);
        
        if (confirm(`Wegpunkt #${currentIndex + 1} entfernen?`)) {
            if (currentIndex > -1) { 
                NAVIS_VISUAL.waypointMarkers.splice(currentIndex, 1); 
                NAVIS_ROUTE.waypoints.splice(currentIndex, 1); 
            }
            map.removeLayer(wpMarker); 
            
            reindexNavisWaypoints(); // 👈 HIER NEU: Alle verbleibenden Marker neu nummerieren
            updateWaypointDisplay();
            updateNavisPolyline(); 
        }
    });
    updateWaypointDisplay();
    updateNavisPolyline();
}

// HIER NEU: Hilfsfunktion zur dynamischen Nummerierung aller Wegpunkt-Popups
function reindexNavisWaypoints() {
    NAVIS_VISUAL.waypointMarkers.forEach((marker, index) => {
        const newNumber = index + 1;
        
        // Popup-Text des Markers zur Laufzeit anpassen
        marker.setPopupContent(`🧭 Wegpunkt #${newNumber}`);
        
        // Falls das Bestätigungs-Popup beim Löschen aufgerufen wird, 
        // müssen wir sicherstellen, dass das Kontextmenü die neuen Indizes kennt.
        // Das passiert automatisch, da wir im 'contextmenu' oben .indexOf() nutzen!
    });
    console.log(`🔄 Wegpunkte renummeriert. Anzahl verbleibend: ${NAVIS_VISUAL.waypointMarkers.length}`);
}

// Aktualisiert die Anzahl im linken Panel
function updateWaypointDisplay() {
    const display = document.getElementById("route-waypoints-display");
    if (display) {
        display.innerText = `${NAVIS_ROUTE.waypoints.length} Wegpunkt(e) hinzugefügt`;
    }
}

console.log("NAVIS: Vor triggerNavisRouting");

// Routenberechnung ausführen
window.triggerNavisRouting = function () {
    // 1. Validierung der Pflichtfelder
    if (!NAVIS_ROUTE.start || !NAVIS_ROUTE.ziel) {
        alert("Bitte zuerst Start- und Zielpunkt per Rechtsklick markieren!");
        return;
    }

    // 2. Variablen-Initialisierung
    const startPos = NAVIS_ROUTE.start;
    const zielPos = NAVIS_ROUTE.ziel;
    const departureTime = new Date(document.getElementById("routing-departure-time").value);
    const profile = document.querySelector('input[name="routingProfile"]:checked').value;
    const strategy = document.getElementById("routing-time-strategy").value;
  
    let weatherFrameIdx = 0;

    // 3. Wetter-Frame-Index berechnen
    if (activeWeatherData && activeWeatherData.metadata && activeWeatherData.metadata.created) {
        const timeDiffHours = (departureTime - new Date(activeWeatherData.metadata.created)) / (1000 * 60 * 60);
        weatherFrameIdx = Math.max(0, Math.min(71, Math.floor(timeDiffHours)));
    }

    // 4. Globale Routen-Objekte aktualisieren
    NAVIS_ROUTE.departureTime = departureTime;
    NAVIS_ROUTE.profile = profile;
    NAVIS_ROUTE.strategy = strategy;
    NAVIS_ROUTE.weatherFrame = weatherFrameIdx;

    // 5. Konsolen-Logging für Debugging
    console.log("=== 🚀 NAVIS GRID ROUTER ENGAGED ===");
    console.log(`Gewähltes Profil: ${profile}`);
    console.log(`Kurswechsel-Strafe (Smooth): Aktiviert`);
    console.log(`Anzahl Zwischen-Wegpunkte: ${NAVIS_ROUTE.waypoints.length}`);
    console.log(`Start-Tile: t_${startPos.tileX}_${startPos.tileY}`);
    
    NAVIS_ROUTE.waypoints.forEach((wp, i) => {
        console.log(`WP #${i + 1}-Tile: t_${wp.tileX}_${wp.tileY}`);
    });
    
    console.log(`Ziel-Tile: t_${zielPos.tileX}_${zielPos.tileY}`);
    console.log("🚀 Starte A*-Routing");

    // 6. Asynchrone Routenplanung (A*) ausführen
    (async () => {
        try {
            // KEIN dynamischer Import mehr! Wir nutzen die global bereitstehende Funktion:
            const route = await planRoute(
                {
                    lat: NAVIS_ROUTE.start.lat,
                    lon: NAVIS_ROUTE.start.lon
                },
                {
                    lat: NAVIS_ROUTE.ziel.lat,
                    lon: NAVIS_ROUTE.ziel.lon
                },
                {
                    schnellste: profile === "fast",
                    wellenVermeiden: profile === "comfort"
                }
            );
            
            console.log("=== 🏁 NAVIS ENGINE: ROUTE BERECHNET ===");
            console.log("Anzahl berechneter Wegpunkte:", route.length);
            if (route.length > 0) {
                console.log(`Gesamtzeit der Route: ${route[route.length - 1].timeElapsed.toFixed(2)} Stunden.`);
            }
            
            // Übergabe an deine Zeichenfunktion
            drawCalculatedRoute(route);
            
        } catch (err) {
            console.error("Routingfehler im A*-Kern:", err);
            alert("Routing fehlgeschlagen: " + err.message);
        }
    })();
};

console.log("NAVIS: Nach triggerNavisRouting");
console.log(window.triggerNavisRouting);


// Funktion zum Umschalten zwischen Groß (ausgeklappt) und Klein (eingeklappt)
function toggleRoutingPanel() {
    const panel = document.getElementById("navis-routing-panel");
    const btn = document.getElementById("routing-pin-toggle");
    
    if (panel.classList.contains("collapsed")) {
        panel.classList.remove("collapsed");
        btn.innerText = "⤡"; // Icon ändert sich zu "Verkleinern"
    } else {
        panel.classList.add("collapsed");
        btn.innerText = "⤢"; // Icon ändert sich zu "Vergrößern"
    }
}

// Optionale Komfort-Erweiterung: Wenn der User einen Start- oder Zielpunkt setzt,
// klappt das Panel automatisch auf, damit er sieht, was passiert.
const originalSetNavisPoint = setNavisPoint;
setNavisPoint = function(type, latlng) {
    originalSetNavisPoint(type, latlng);
    const panel = document.getElementById("navis-routing-panel");
    if (panel.classList.contains("collapsed")) {
        toggleRoutingPanel();
    }
};

/**
 * A*-Routenplaner für das ESP32-Segelboot-Projekt
 * STUFE 2.9: Fertig für Kartentest, inklusive RAM-Kachel-Optimierung, openMap-Schutz & Cache-Fix
 */
// =========================================================================
// INTEGRATION: NAVIS A* SEGEL-ROUTER KERN (SYNTAX-FIXED)
// =========================================================================

const tileCache = new Map();
const vmgCache = new Map();

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

async function fetchPolarData() {
    try {
        const response = await fetch('polar/myboot.json');
        const boot = await response.json();
        
        const tws = boot.windSpeeds.map(Number).sort((a, b) => a - b);
        const twa = Object.keys(boot.polar).map(Number).sort((a, b) => a - b);

        const speed = twa.map(angle => {
            const originalAngleKey = Object.keys(boot.polar).find(k => Number(k) === angle);
            const angleData = boot.polar[originalAngleKey] || {};
            return tws.map(ws => {
                const originalWsKey = Object.keys(angleData).find(k => Number(k) === ws);
                return Number(angleData[originalWsKey]) || 0.05;
            });
        });

        console.log("Polardaten erfolgreich geladen und normiert:", { tws, twa, matrixSize: `${speed.length}x${speed[0].length}` });
        return { tws, twa, speed };
    } catch (e) {
        console.warn("Polardaten-Parser-Fehler, lade sicheres Ostsee-Fallback", e);
        // SYNTAX-FIX: Werte im Fallback wieder korrekt eingetragen
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
    if (tileCache.has(cacheKey)) return tileCache.get(cacheKey);
    try {
        const response = await fetch(`tiles_nav/${tileX}_${tileY}.json`);
        const tileData = await response.json();
        tileCache.set(cacheKey, tileData.isLand);
        return tileData.isLand;
    } catch (e) {
        tileCache.set(cacheKey, false);
        return false; 
    }
}

function getWeatherData(hoursAhead, lat, lon) {
    if (typeof wetterDaten !== 'undefined' && wetterDaten[hoursAhead]) {
        return wetterDaten[hoursAhead]; 
    }
    return { windDir: 270, windSpeed: 14, waveHeight: 0.6, currentDir: 45, currentSpeed: 1.0 };
}

// NAUTISCHE MATHEMATIK FIX: Doppelte Variablen-Deklaration entfernt
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

// --- 4. A* CORE ENGINE ---
async function planRoute(start, ziel, optionen = { schnellste: true, wellenVermeiden: false }) {
    const polar = await fetchPolarData();
    const openHeap = new MinHeap();
    const openMap = new Map(); // Speicher- und Verlaufs-Kontrolle
    const closedSet = new Set();
    const goalRadiusNM = 0.5; // Robustes Ankommen am Ziel
    const stepSize = 0.01; 
    
    // Geographische Anpassung der Schrittweite (Längengrade verjüngen sich zum Pol)
    const cosLat = Math.cos(start.lat * Math.PI / 180);
    const stepSizeLon = stepSize / Math.max(0.1, cosLat); // Minimiert geografische Verzerrung
    
    const rHalfLat = stepSize * 0.4142;
    const rHalfLon = stepSizeLon * 0.4142;

    // KORREKTUR: Tippfehler 'rHalfHalfLon' wurde zu 'rHalfLon' bereinigt
    const directions = [
        {dLat: stepSize, dLon: 0}, {dLat: -stepSize, dLon: 0}, {dLat: 0, dLon: stepSizeLon}, {dLat: 0, dLon: -stepSizeLon},
        {dLat: stepSize, dLon: stepSizeLon}, {dLat: -stepSize, dLon: -stepSizeLon}, {dLat: stepSize, dLon: -stepSizeLon}, {dLat: -stepSize, dLon: stepSizeLon},
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
                const penalty = Math.pow(twa / optimalUpwindTWA, 3); 
                boatSpeed = boatSpeed * penalty;
            }
            
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

// Funktion zum Umschalten zwischen Groß (ausgeklappt) und Klein (eingeklappt)
function toggleRoutingPanel() {
    const panel = document.getElementById("navis-routing-panel");
    const btn = document.getElementById("routing-pin-toggle");
    if (!panel || !btn) return;
    if (panel.classList.contains("collapsed")) {
        panel.classList.remove("collapsed");
        btn.innerText = "⤡"; 
    } else {
        panel.classList.add("collapsed");
        btn.innerText = "⤢"; 
    }
}
window.toggleRoutingPanel = toggleRoutingPanel;

// Optionale Komfort-Erweiterung: Wenn der User einen Start- oder Zielpunkt setzt,
// klappt das Panel automatisch auf.
const originalSetNavisPoint = setNavisPoint;
setNavisPoint = function(type, latlng) {
    originalSetNavisPoint(type, latlng);
    const panel = document.getElementById("navis-routing-panel");
    if (panel && panel.classList.contains("collapsed")) {
        toggleRoutingPanel();
    }
};
