/* =========================================================
   NAVIS ROUTE
========================================================= */

const modes = [
    "bearing",
    "route",
    "marker"
];

let currentModeIndex = 0;
let currentMode = modes[currentModeIndex];

let routeMode = false;

let routePoints = [];
let routeMarkers = [];
let routeLine = null;

let waypointMarker = null;
let waypointLine = null;

// Marker-Typen
const markerTypes = [
  { icon: "⚓", name: "Ankerplatz", color: "#ffd700" },
  { icon: "🏝", name: "Bucht", color: "#00ffaa" },
  { icon: "⛽", name: "Tankstelle", color: "#ff8800" },
  { icon: "⚠", name: "Gefahr", color: "#ff0000" },
  { icon: "🪨", name: "Untiefe", color: "#aa5500" },
  { icon: "🐟", name: "Angelplatz", color: "#00ccff" },
  { icon: "🛠", name: "Reparatur", color: "#cccccc" },
  { icon: "📷", name: "Foto", color: "#ff66ff" },
  { icon: "📝", name: "Eigene Notiz", color: "#ffffff" }
];

map.on('click', e => {

  // ================================
  // 📝 MARKER MODUS (nur hier Popup!)
  // ================================
  if (currentMode === "marker") {

    let html = `<div style="font-family:Arial;font-size:14px;min-width:220px;">`;
    html += `<b>Marker Typ wählen</b><br><br>`;

    markerTypes.forEach((m, i) => {
      html += `
        <label style="display:block;margin:4px 0;cursor:pointer;">
          <input type="radio" name="markerType" value="${i}">
          ${m.icon} ${m.name}
        </label>
      `;
    });

    html += `
      <br>
      <b>Name / Beschreibung:</b><br>
      <input id="markerTextInput" type="text" style="width:95%;padding:4px;margin-top:4px;">
      <br><br>
      <button id="markerOkBtn">OK</button>
    </div>`;

    const container = document.createElement("div");
    container.innerHTML = html;

    L.popup()
      .setLatLng(e.latlng)
      .setContent(container)
      .openOn(map);

    setTimeout(() => {
      const radios = document.querySelectorAll('input[name="markerType"]');
      const textInput = document.getElementById("markerTextInput");

      radios.forEach(r => {
        r.addEventListener("change", () => {
          textInput.value = markerTypes[r.value].name;
        });
      });

      document.getElementById("markerOkBtn").onclick = () => {
        const selected = document.querySelector('input[name="markerType"]:checked');
        if (!selected) return;

        const type = markerTypes[selected.value];
        const customName = textInput.value || type.name;

        const marker = L.marker(e.latlng, {
          icon: createDiamondIcon(type.color, 24, type.icon),
          draggable: true
        }).addTo(map);

        marker.bindPopup(`
          <b>${type.icon} ${customName}</b><br>
          ${e.latlng.lat.toFixed(5)},
          ${e.latlng.lng.toFixed(5)}
        `);

        marker.on('contextmenu', () => {
          if (confirm("Marker löschen?")) {
            map.removeLayer(marker);
          }
        });

        map.closePopup();
      };
    }, 50);

    return; // ❗ wichtig: verhindert Wegpunktlogik
  }

  // ================================
  // 🧭 PEILMODUS
  // ================================
  if (currentMode === "bearing") {

    if (waypointMarker) {
      try { map.removeLayer(waypointMarker); } catch(e){}
      if (waypointLine) {
        try { map.removeLayer(waypointLine); } catch(e){}
        waypointLine = null;
      }
      waypointMarker = null;
    }

    const boatLat = state.gps_lat;
    const boatLon = state.gps_lon;

    const { bearing, distance } = bearingAndDistance(
      boatLat, boatLon,
      e.latlng.lat, e.latlng.lng
    );

    waypointMarker = L.marker(e.latlng, {
      icon: createDiamondIcon('red')
    }).addTo(map)
      .bindPopup(`Peilziel<br>${e.latlng.lat.toFixed(5)}, ${e.latlng.lng.toFixed(5)}<br>${bearing.toFixed(1)}° / ${(distance / 1852).toFixed(2)} sm`)
      .openPopup();

    waypointLine = L.polyline([[boatLat, boatLon], [e.latlng.lat, e.latlng.lng]], {
      color: 'red',
      weight: 2,
      opacity: 0.7,
      dashArray: '5,5'
    }).addTo(map);

    return;
  }

  // ================================
  // 🧭 ROUTE MODUS
  // ================================
  if (currentMode === "route") {

    const latlng = e.latlng;
    routePoints.push(latlng);

    const m = L.marker(latlng, {
      draggable: true,
      icon: createDiamondIcon('blue')
    }).addTo(map);

    routeMarkers.push(m);

    updateMarkerTooltip(m, routeMarkers.length - 1);
    updateRouteDisplay();

    m.on('drag', () => {
      const i = routeMarkers.indexOf(m);
      if (i === -1) return;

      routePoints[i] = m.getLatLng();
      updateRouteDisplay();
      updateMarkerTooltip(m, i);
    });

    m.on('contextmenu', function(evt) {
      if (evt.originalEvent) {
			evt.originalEvent.preventDefault();
		}
      const i = routeMarkers.indexOf(m);
      if (i !== -1) removeWaypoint(i);
    });

    return;
  }
});

/* ------------------------------
   Marker-Tooltip (WP#, Kurs/Distanz)
   ------------------------------ */
function updateMarkerTooltip(marker, idx) {
  const p = routePoints[idx];
  const prev = idx === 0 ? { lat: state.gps_lat, lng: state.gps_lon } : routePoints[idx - 1];
  const { bearing, distance } = bearingAndDistance(prev.lat, prev.lng, p.lat, p.lng);
  marker.bindTooltip(`WP${idx+1}\n${bearing.toFixed(1)}° / ${(distance/1852).toFixed(2)} sm`, { permanent: true, offset: [0, -15] });
}

/* ------------------------------
   Route aktualisieren
   ------------------------------ */
function updateRouteDisplay() {
  // Linie entfernen, falls vorhanden
  if (routeLine) { 
    try { map.removeLayer(routeLine); } catch(e){} 
    routeLine = null; 
  }

  // Route zeichnen (Boot als Startpunkt)
  if (routePoints.length > 0) {
    const latlngs = [[state.gps_lat, state.gps_lon], ...routePoints.map(p => [p.lat, p.lng])];
    routeLine = L.polyline(latlngs, { color: '#00aaff', weight: 2, opacity: 0.9 }).addTo(map);
  }

  // Tooltips / Nummerierung aktualisieren
  routeMarkers.forEach((m, i) => updateMarkerTooltip(m, i));

  // Distanz berechnen (Startpunkt = Boot)
  if (routeMode && routePoints.length > 0) {
    let totalDistance = 0;
    let prev = { lat: state.gps_lat, lng: state.gps_lon };
    routePoints.forEach(p => {
      const { distance } = bearingAndDistance(prev.lat, prev.lng, p.lat, p.lng);
      totalDistance += distance;
      prev = p;
    });
    waypointInfo.innerHTML = `<b>Gesamtdistanz Route:</b> ${(totalDistance/1852).toFixed(2)} sm • ${routePoints.length} WP`;
  } else {
    waypointInfo.innerHTML = '';
  }
  updateWaypointList();
}

/* ------------------------------
   Wegpunkt entfernen (zentral)
   ------------------------------ */
function removeWaypoint(idx) {
  if (idx < 0 || idx >= routePoints.length) return;

  // remove marker from map & arrays
  try{ map.removeLayer(routeMarkers[idx]); }catch(e){}
  routeMarkers.splice(idx, 1);
  routePoints.splice(idx, 1);

  // rebuild tooltips & route
  updateRouteDisplay();
  updateWaypointList();
}

function updateWaypointList() {
  const tbody = document.querySelector("#waypointTable tbody");
  if (!tbody) return;
  tbody.innerHTML = "";

  routePoints.forEach((p, i) => {
    const tr = document.createElement("tr");

    const prev = i === 0 ? { lat: state.gps_lat, lng: state.gps_lon } : routePoints[i - 1];
    const fromShip = bearingAndDistance(state.gps_lat, state.gps_lon, p.lat, p.lng);
    const fromPrev = bearingAndDistance(prev.lat, prev.lng, p.lat, p.lng);

    const cells = [
      i + 1,
      p.lat.toFixed(5),
      p.lng.toFixed(5),
      fromShip.bearing.toFixed(1) + "°",
      (fromShip.distance / 1852).toFixed(2),
      i === 0 ? "—" : fromPrev.bearing.toFixed(1) + "°",
      i === 0 ? "—" : (fromPrev.distance / 1852).toFixed(2)
    ];

    cells.forEach(val => {
      const td = document.createElement("td");
      td.textContent = val;
      tr.appendChild(td);
    });

    const tdBtn = document.createElement("td");
    const btn = document.createElement("button");
    btn.textContent = "Autopilot";
    btn.onclick = () => sendAutopilotWP(p.lat, p.lng);
    tdBtn.appendChild(btn);
    tr.appendChild(tdBtn);

    tbody.appendChild(tr);
  });
}

function sendAutopilotWP(lat, lon) {
  updateRouteDisplay();
  if(demoMode) {
    console.log(`[Demo] Autopilot WP → Lat: ${lat}, Lon: ${lon}`);
    return;
  }
  // ESP erwartet z.B. GET: /autopilot?lat=...&lon=...
  const url = `/autopilot?lat=${lat.toFixed(6)}&lon=${lon.toFixed(6)}`;
  fetch(url)
    .then(res => {
      if(!res.ok) throw new Error("ESP Antwort fehlerhaft");
      return res.text();
    })
    .then(txt => console.log(`ESP-Antwort: ${txt}`))
    .catch(err => console.error("Fehler beim Senden an ESP:", err));
}

// Icon-Funktion für offline Marker (Raute)
function createDiamondIcon(color = 'red', size = 16, iconText = "") {

  const content = iconText
    ? `<div style="
        width:${size}px;
        height:${size}px;
        display:flex;
        align-items:center;
        justify-content:center;
        font-size:${size}px;
      ">${iconText}</div>`
    : `<div style="
        width:${size}px;
        height:${size}px;
        background:${color};
        transform: rotate(45deg);
        border:2px solid white;"></div>`;

  return L.divIcon({
    className: 'diamond-marker',
    html: content,
    iconSize: [size, size],
    iconAnchor: [size/2, size/2]
  });
}

/* ------------------------------
   Waypoint / Peilungsanzeige
   ------------------------------ */
function updateWaypointInfo() {
    if (!routeMode && waypointMarker) {
        const { bearing, distance } = bearingAndDistance(
            state.gps_lat,
            state.gps_lon,
            waypointMarker.getLatLng().lat,
            waypointMarker.getLatLng().lng
        );

        waypointInfo.innerHTML =
            `<b>Peilung zu Ziel:</b><br>` +
            `Kurs: ${bearing.toFixed(1)}°<br>` +
            `Entfernung: ${(distance / 1852).toFixed(2)} sm`;
    }
}





