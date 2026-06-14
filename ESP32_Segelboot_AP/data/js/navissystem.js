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

// Routenberechnung ausführen
function triggerNavisRouting() {
    if (!NAVIS_ROUTE.start || !NAVIS_ROUTE.ziel) { alert("Bitte zuerst Start- und Zielpunkt per Rechtsklick markieren!"); return; }
    const startPos = NAVIS_ROUTE.start, zielPos = NAVIS_ROUTE.ziel, departureTime = new Date(document.getElementById("routing-departure-time").value), profile = document.querySelector('input[name="routingProfile"]:checked').value, strategy = document.getElementById("routing-time-strategy").value;
    let weatherFrameIdx = 0;

    if (activeWeatherData && activeWeatherData.metadata && activeWeatherData.metadata.created) {
        weatherFrameIdx = Math.max(0, Math.min(71, Math.floor((departureTime - new Date(activeWeatherData.metadata.created)) / (1000 * 60 * 60))));
    }

    NAVIS_ROUTE.departureTime = departureTime; NAVIS_ROUTE.profile = profile; NAVIS_ROUTE.strategy = strategy; NAVIS_ROUTE.weatherFrame = weatherFrameIdx;

    console.log("=== 🚀 NAVIS GRID ROUTER ENGAGED ===");
    console.log(`Gewähltes Profil: ${profile}`); console.log(`Kurswechsel-Strafe (Smooth): Aktiviert`);
    console.log(`Anzahl Zwischen-Wegpunkte: ${NAVIS_ROUTE.waypoints.length}`);
    console.log(`Start-Tile: t_${startPos.tileX}_${startPos.tileY}`);
    NAVIS_ROUTE.waypoints.forEach((wp, i) => console.log(`WP #${i + 1}-Tile: t_${wp.tileX}_${wp.tileY}`));
    console.log(`Ziel-Tile: t_${zielPos.tileX}_${zielPos.tileY}`);

    alert(`Taktik-Routing initialisiert!\n\nProfil: ${profile}\nWegpunkte: ${NAVIS_ROUTE.waypoints.length}\nKurswechsel-Vermeidung aktiv.\n\nDer A*-Segment-Pfadfinder startet.`);
}

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