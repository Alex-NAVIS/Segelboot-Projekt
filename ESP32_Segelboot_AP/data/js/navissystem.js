/* ----------------------------------------------------------------------
🧭 NAVIS V2.0 Kontext-Routing Engine
---------------------------------------------------------------------- */
const NAVIS_ROUTE = {
    start: null,
    ziel: null,
    waypoints: [],
    departureTime: null,
    profile: "fastest",
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
    
    if (NAVIS_VISUAL.routeLine) {
        map.removeLayer(NAVIS_VISUAL.routeLine);
        NAVIS_VISUAL.routeLine = null;
    }

    if (!NAVIS_ROUTE.start || !NAVIS_ROUTE.ziel) {
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

    try {
        NAVIS_VISUAL.routeLine = L.polyline(pathCoordinates, {
            color: '#00b8d4', 
            weight: 5, 
            opacity: 0.9, 
            dashArray: '8, 12'
        }).addTo(map);
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
    const timeInput = document.getElementById("routing-departure-time");
    if (!timeInput) return;
    // UTC direkt ins Eingabefeld
    timeInput.value = new Date().toISOString().slice(0, 16);
    updateDepartureTimeDisplay();
    timeInput.addEventListener("change", updateDepartureTimeDisplay);
});

function updateDepartureTimeDisplay() {
    const input = document.getElementById("routing-departure-time");
    const display = document.getElementById("routing-localtime-display");
    if (!input || !display || !input.value) return;
    // UTC-Wert aus Eingabefeld lesen
    const utcDate = new Date(input.value + ":00Z");
    display.innerHTML = `🕒 Lokale Systemzeit: ${utcDate.toLocaleString()}`;
}

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
        document.getElementById(displayId).innerText = `Lat: ${pos.lat.toFixed(2)} | Lon: ${pos.lng.toFixed(2)}`;
        updateNavisPolyline(); 
    });

    // Dieses Event feuert beim LOSLASSEN
    NAVIS_VISUAL[markerKey].on('dragend', function(event) {
        const pos = event.target.getLatLng();
        updateNavisPolyline();
    });

    document.getElementById(displayId).innerText = `Lat: ${point.lat.toFixed(2)} | Lon: ${point.lon.toFixed(2)}`;
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
}

// Aktualisiert die Anzahl im linken Panel
function updateWaypointDisplay() {
    const display = document.getElementById("route-waypoints-display");
    if (display) {
        display.innerText = `${NAVIS_ROUTE.waypoints.length} Wegpunkt(e) hinzugefügt`;
    }
}

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
    const departureTime =
    new Date(document.getElementById("routing-departure-time").value + ":00Z");
    const profile = document.querySelector('input[name="routingProfile"]:checked').value;
    const strategy = document.getElementById("routing-time-strategy").value;
  
    let weatherFrameIdx = 0;

    // 3. Wetter-Frame-Index berechnen
    if (activeWeatherData && activeWeatherData.metadata && activeWeatherData.metadata.created) {
        const timeDiffHours = (departureTime - new Date(activeWeatherData.metadata.created)) / (1000 * 60 * 60);
        weatherFrameIdx = Math.max(0, Math.min( activeWeatherData.frames.length - 1, Math.floor(timeDiffHours)));
    }

    // 4. Globale Routen-Objekte aktualisieren
    NAVIS_ROUTE.departureTime = departureTime;
    NAVIS_ROUTE.profile = profile;
    NAVIS_ROUTE.strategy = strategy;
    NAVIS_ROUTE.weatherFrame = weatherFrameIdx;

	NAVIS_ROUTE.waypoints.forEach((wp, i) => {console.log(`WP #${i + 1}-Tile: t_${wp.tileX}_${wp.tileY}`);});
	console.log(`Ziel-Tile: t_${zielPos.tileX}_${zielPos.tileY}`);
	switch(profile) {
		case "fastest":
			console.log("⚡ Schnellste Route");
			break;
		case "comfort":
			console.log("🛋 Komfortabelste Route");
			break;
		case "safest":
			console.log("🛡 Sicherste Route");
			break;
		case "eco":
			console.log("🔋 Energieeffizienteste Route");
			break;
		case "avoid_wind":
			console.log("💨 Starkwind vermeiden");
			break;
		case "avoid_waves":
			console.log("🌊 Wellen vermeiden");
			break;
		case "coastal":
			console.log("⚓ Küstennah fahren");
			break;
	}
    console.log("🚀 Starte A*-Routing");

    // 6. Asynchrone Routenplanung (A*) ausführen
    (async () => {
        try {
            // KEIN dynamischer Import mehr! Wir nutzen die global bereitstehende Funktion:
            const route = await planRoute(
				{ lat: NAVIS_ROUTE.start.lat, lon: NAVIS_ROUTE.start.lon },
				{ lat: NAVIS_ROUTE.ziel.lat, lon: NAVIS_ROUTE.ziel.lon },
				{
					profile: profile,
					windAktiv: true,
					welleAktiv: true,
					stroemungAktiv: true,
					kursMalusAktiv: true,  // 🌟 AKTIVIEREN: Zwingt das Boot auf gerade Linien
					xteAktiv: true         // 🌟 AKTIVIEREN: Verhindert das unendliche Ausbrechen nach außen
				}
			);
            
            // Übergabe an deine Zeichenfunktion
            drawCalculatedRoute(route);
            
        } catch (err) {
            console.error("Routingfehler im A*-Kern:", err);
            alert("Routing fehlgeschlagen: " + err.message);
        }
    })();
};

console.log(window.triggerNavisRouting);

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
            "tws": [4, 6, 8, 10, 12, 14, 16, 20],
            "twa": [0, 30, 45, 60, 75, 90, 110, 130, 150, 180],
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

async function getTileType(lat, lon) {
    const TILE_SPAN_LOCAL = 0.256; 
    const tileX = Math.floor(lon / TILE_SPAN_LOCAL);
    const tileY = Math.floor(lat / TILE_SPAN_LOCAL);
    const cacheKey = `t_${tileX}_${tileY}`;

    let tileData = null;

    // RAM-Cache Abfrage
    if (tileCache.has(cacheKey)) {
        tileData = tileCache.get(cacheKey);
    } else {
        try {
            const response = await fetch(`tiles_nav/${cacheKey}.nav.gz`);
            if (!response.ok) {
                tileCache.set(cacheKey, 0); // 404 = Kachel existiert nicht im Python-Skript = Offene See
                return 0;
            }
            
            const arrayBuffer = await response.arrayBuffer();
            const decompressed = pako.ungzip(new Uint8Array(arrayBuffer));
            const dataView = new DataView(decompressed.buffer);
            
            const gridSizeX = dataView.getUint16(4, false);
            const minLon = dataView.getFloat32(8, false);
            const minLat = dataView.getFloat32(12, false);
            const maxLon = dataView.getFloat32(16, false);
            const maxLat = dataView.getFloat32(20, false);
            const matrixBytes = decompressed.subarray(28);
            
            tileData = {
                gridSize: gridSizeX,
                minLon, minLat, maxLon, maxLat,
                matrix: matrixBytes
            };
            tileCache.set(cacheKey, tileData);
            
        } catch (e) {
            console.error(`Fehler beim Entpacken von Kachel ${cacheKey}:`, e);
            tileCache.set(cacheKey, 0); // Fallback bei Defekt: Offene See
            return 0;
        }
    }

    // Wenn als reine offene See gecacht
    if (tileData === 0) return 0;

    const size = tileData.gridSize;
    const pctX = (lon - tileData.minLon) / (tileData.maxLon - tileData.minLon);
    const pctY = (lat - tileData.minLat) / (tileData.maxLat - tileData.minLat);
    
    const pixelX = Math.floor(pctX * size);
    const pixelY = (size - 1) - Math.floor(pctY * size); // Rasterio-Y-Spiegelung
    
    const clampedX = Math.max(0, Math.min(size - 1, pixelX));
    const clampedY = Math.max(0, Math.min(size - 1, pixelY));
    
    const arrayIndex = (clampedY * size) + clampedX;
    
    // Liefert garantiert: 0 = See, 1 = Küste, 2 = Land
    return tileData.matrix[arrayIndex];
}

function getWeatherData(hoursAhead, lat, lon) {
    // 1. Raum-Komponente: Kachel-ID berechnen (Gitter-Maßstab TILE_SPAN = 0.256°)
    const TILE_SPAN_LOCAL = 0.256; 
    const tileX = Math.floor(lon / TILE_SPAN_LOCAL);
    const tileY = Math.floor(lat / TILE_SPAN_LOCAL);
    const tileKey = `t_${tileX}_${tileY}`;

	// ======================================================
	// ECHTE NAVIS-WETTERDATEN AUS activeWeatherData
	// ======================================================
	if (
		typeof activeWeatherData !== 'undefined' &&
		activeWeatherData &&
		activeWeatherData.frames &&
		activeWeatherData.frames[hoursAhead]
	) {
		const frame = activeWeatherData.frames[hoursAhead];
		const meta = activeWeatherData.metadata;
		const latPos = (meta.la1 - lat) / meta.dy;
		const lonPos = (lon - meta.lo1) / meta.dx;
		if (lonPos < 0 || lonPos > meta.nx - 1 || latPos < 0 || latPos > meta.ny - 1) {
			return {
				windDir: 240,
				windSpeed: 12,
				waveHeight: 0.5,
				waveDir: 240,
				currentDir: 90,
				currentSpeed: 0.2
			};
		}

		const x0 = Math.floor(lonPos);
		const y0 = Math.floor(latPos);
		const x1 = Math.min(x0 + 1, meta.nx - 1);
		const y1 = Math.min(y0 + 1, meta.ny - 1);
		const xFactor = lonPos - x0;
		const yFactor = latPos - y0;
		const getIdx = (x, y) => (y * meta.nx) + x;
		const bilinear = (array) => {
			if (!array) return 0;
			const v00 = array[getIdx(x0, y0)] ?? 0;
			const v10 = array[getIdx(x1, y0)] ?? 0;
			const v01 = array[getIdx(x0, y1)] ?? 0;
			const v11 = array[getIdx(x1, y1)] ?? 0;
			return (
				(1 - yFactor) *
					((1 - xFactor) * v00 + xFactor * v10) +
				yFactor *
					((1 - xFactor) * v01 + xFactor * v11)
			);
		};
		const windU = bilinear(frame.wind_u);
		const windV = bilinear(frame.wind_v);
		const currentU = bilinear(frame.current_u);
		const currentV = bilinear(frame.current_v);
		const windSpeed = Math.sqrt(windU * windU + windV * windV) * 1.94384;
		let windDir = ((270 - (Math.atan2(windV, windU) * 180 / Math.PI)) % 360 + 360) % 360;
		const currentSpeed = Math.sqrt(currentU * currentU + currentV * currentV);
		let currentDir = (Math.atan2(currentU, currentV) * 180 / Math.PI);
		currentDir = ((currentDir % 360) + 360) % 360;

		const waveDir =	bilinear(frame.wave_direction) ?? windDir;

		return {
			windDir,
			windSpeed,
			waveHeight: bilinear(frame.wave_height),
			waveDir,
			currentDir,
			currentSpeed
		};
	}

    // 3. ECHTES 4D-GEZEITEN- & WETTER-FALLBACK (Für lokalen Test im Browser)
    // Wind- und Wellenfronten (Wandernd im Raum und in der Zeit)
    const wavePhase = (tileX * 0.5) + (tileY * 0.3) - (hoursAhead * 0.1);
    const windSpeedCalc = 12 + Math.sin(wavePhase) * 6; // Wind pendelt zw. 6 und 18 Knoten
    const windDirBase = 240 + (hoursAhead * 0.5) + (tileX * 2) % 40;
    const waveHeightCalc = 0.2 + (windSpeedCalc * 0.06);

    // --- ECHTE 4D-GEZEITEN-LOGIK (EBBE & FLUT SIMULATION) ---
    // Ein voller Gezeitenzyklus (M2-Tide) dauert ca. 12.42 Stunden.
    const TIDE_PERIOD = 12.42;
    
    // Die Phase der Tide verschiebt sich geografisch (Flutwelle läuft zeitversetzt durchs Revier)
    // Wir simulieren hier eine Flutwelle, die von Westen (niedriges tileX) nach Osten wandert.
    const geoPhaseShift = tileX * 0.6 + tileY * 0.2;
    const currentTidePhase = ((hoursAhead + geoPhaseShift) % TIDE_PERIOD) / TIDE_PERIOD * 2 * Math.PI;

    // Basis-Strömungsrichtung im Revier (z.B. Elbe/Fahrwasser-Achse Ost-West: 90° / 270°)
    const mainChannelAxis = 90; 
    
    // Sinus-Welle steuert das Ebbe- und Flutstrom-Verhalten (-1.0 bis +1.0)
    const tideFactor = Math.sin(currentTidePhase);
    
    let currentDirCalc = mainChannelAxis;
    if (tideFactor < 0) {
        // Wenn der Faktor negativ ist, kentert der Strom und läuft in die Gegenrichtung (Ebbe)
        currentDirCalc = (mainChannelAxis + 180) % 360;
    }

    // Strömungsgeschwindigkeit pulsiert mit dem Tidenstrom (maximal im Gezeitenstrom, Nullpunkt beim Stillwasser)
    // An engen Stellen (simuliert durch tileY-Modul) wird der Strom physisch beschleunigt
    const maxCurrentInRegion = 1.2 + (tileY % 2) * 0.8; // Bis zu 2.0 Knoten Strom!
    const currentSpeedCalc = Math.abs(tideFactor) * maxCurrentInRegion;

    return {
		windDir: (windDirBase + 360) % 360,
		windSpeed: Math.max(2, windSpeedCalc),
		waveHeight: Math.max(0.1, waveHeightCalc),
		// Testweise identisch zum Wind
		// später echte Wellenrichtung einsetzen
		waveDir: (windDirBase + 360) % 360,
		currentDir: currentDirCalc,
		currentSpeed: currentSpeedCalc
	};
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

// =========================================================================
// HIGH-PERFORMANCE 4D SEGLER-ENGINE (AXIS-ALIGNED & FIXED)
// =========================================================================

/**
 * Ermittelt dynamisch den minimalen Am-Wind-Winkel aus den Polardaten.
 * Steht ganz oben, damit planRoute fehlerfrei darauf zugreifen kann!
 */
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
window.getMinSailableTWA = getMinSailableTWA;

function formatNavisETA(node) {
    if (!node.etaUTC) {
        return {
            travel: "--",
            utc: "--",
            local: "--"
        };
    }
    const etaDate = new Date(node.etaUTC);
    return {
        travel: `${(node.etaHours ?? 0).toFixed(2)} h`,
        utc:
            etaDate.getUTCDate().toString().padStart(2, '0') + "." +
            (etaDate.getUTCMonth() + 1).toString().padStart(2, '0') + "." +
            etaDate.getUTCFullYear() + " " +
            etaDate.getUTCHours().toString().padStart(2, '0') + ":" +
            etaDate.getUTCMinutes().toString().padStart(2, '0'),
        local: etaDate.toLocaleString()
    };
}

// --- 4. A* CORE ENGINE ---
async function planRoute(start, ziel, optionen = {}) {
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

// --- 5. INTERFACE PANELS CONTROL ---
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

function reconstructPath(node) {
    let path = [];
    let curr = node;
    while (curr !== null) {
		path.push({
			lat: curr.lat,
			lon: curr.lon,
			eta: curr.g || 0,
			timeElapsed: curr.g || 0,
			etaHours: curr.etaHours || 0,
			etaUTC: curr.etaUTC || null,
			forecastHour: curr.forecastHour || 0,
			heading: curr.heading || 0,
			boatSpeed: curr.boatSpeed || 0,
			sog: curr.sog || 0,
			windSpeed: curr.windSpeed || 0,
			windDir: curr.windDir || 0,
			twa: curr.twa || 0,
			currentSpeed: curr.currentSpeed || 0,
			currentDir: curr.currentDir || 0,
			currentAngle: curr.currentAngle || 0,
			waveHeight: curr.waveHeight || 0,
			waveDir: curr.waveDir || 0,
			waveAngle: curr.waveAngle || 0
		});
        curr = curr.parent;
    }
    return path.reverse();
}

window.planRoute = planRoute;


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