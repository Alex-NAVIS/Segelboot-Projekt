
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

// --- Wegpunktliste ein-/ausblenden am unteren Kartenrand ---
let listVisible = false;
const list = document.getElementById('waypointListContainer');

document.addEventListener('mousemove', e => {
  const triggerHeight = window.innerHeight - 30; // 30px vom unteren Rand
  if (e.clientY >= triggerHeight && !listVisible) {
    list.classList.add('visible');
    listVisible = true;
  } else if (e.clientY < triggerHeight - 200 && listVisible) {
    list.classList.remove('visible');
    listVisible = false;
  }
});

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