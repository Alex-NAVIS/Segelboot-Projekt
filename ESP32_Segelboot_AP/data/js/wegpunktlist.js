function updateWaypointList() {
	const container = document.getElementById("waypoint-panel-content");
	if (!container) return;
	container.innerHTML = "";

	// Variable zum Aufsummieren der Gesamtstrecke ab dem Schiff
	let accumulatedDistance = 0;

	const panel = document.getElementById("navis-waypoint-panel");
	if (panel) {
		// 1. Wenn genau ein Wegpunkt existiert und das Panel zu ist -> Öffnen
		if (routePoints.length === 1 && panel.classList.contains("collapsed")) {
			toggleWaypointPanel();
		}
		// 2. Wenn KEIN Wegpunkt mehr existiert und das Panel offen ist -> Schließen
		else if (routePoints.length === 0 && !panel.classList.contains("collapsed")) {
			toggleWaypointPanel();
		}
	}

	routePoints.forEach((p, i) => {
		const prev = (i === 0)
			? { lat: state.gps_lat, lng: state.gps_lon }
			: routePoints[i - 1];
		const fromShip = bearingAndDistance(
			state.gps_lat,
			state.gps_lon,
			p.lat,
			p.lng
		);
		const fromPrev = bearingAndDistance(
			prev.lat,
			prev.lng,
			p.lat,
			p.lng
		);

		// Wenn es der erste WP ist, nehmen wir die Distanz vom Schiff.
		// Danach addieren wir die Teilstrecken von WP zu WP auf.
		if (i === 0) {
			accumulatedDistance = fromShip.distance;
		} else {
			accumulatedDistance += fromPrev.distance;
		}

		const card = document.createElement("div");
		card.className = "waypoint-card";
		card.innerHTML = `
			<div class="wp-header">
				<div class="wp-title">
					📍 Wegpunkt ${i + 1}
				</div>
				<div class="wp-actions">
					<button class="wp-action-btn wp-center-btn" title="Karte auf Wegpunkt zentrieren">
						🔍
					</button>
					<button class="wp-action-btn wp-auto-btn" title="An Autopilot senden">
						🧭
					</button>
					<button class="wp-action-btn wp-delete-btn" title="Wegpunkt löschen">
						🗑️
					</button>
				</div>
			</div>

			<div class="waypoint-divider"></div>
			<div class="waypoint-section">
				<div class="waypoint-section-title">
					Position
				</div>
				<div class="waypoint-row">
					<span class="waypoint-label">Breite</span>
					<span class="waypoint-value">${p.lat.toFixed(5)}</span>
				</div>
				<div class="waypoint-row">
					<span class="waypoint-label">Länge</span>
					<span class="waypoint-value">${p.lng.toFixed(5)}</span>
				</div>
			</div>

			<div class="waypoint-divider"></div>

			<div class="waypoint-section">
				<div class="waypoint-section-title">
					Navigation
				</div>
				<div class="waypoint-row">
					<span class="waypoint-label">Kurs Schiff</span>
					<span class="waypoint-value">${fromShip.bearing.toFixed(1)}°</span>
				</div>
				<div class="waypoint-row">
					<span class="waypoint-label">Entfernung</span>
					<span class="waypoint-value">${(fromShip.distance / 1852).toFixed(2)} sm</span>
				</div>
			</div>

			<div class="waypoint-divider"></div>
			<div class="waypoint-section">
				<div class="waypoint-section-title">
					Strecke
				</div>
				<div class="waypoint-row">
					<span class="waypoint-label">${i == 0 ? "Kurs Schiff→WP" : "Kurs WP→WP"}</span>
					<span class="waypoint-value">
						${i == 0 ? fromShip.bearing.toFixed(1) + "°" : fromPrev.bearing.toFixed(1) + "°"}
					</span>
				</div>
				<div class="waypoint-row">
					<span class="waypoint-label">Teilstrecke</span>
					<span class="waypoint-value">
						${i == 0 ? (fromShip.distance / 1852).toFixed(2) + " sm" : (fromPrev.distance / 1852).toFixed(2) + " sm"}
					</span>
				</div>
				<div class="waypoint-row">
					<span class="waypoint-label">Gesamtstrecke bis WP ${i + 1}</span>
					<span class="waypoint-value" style="font-weight: bold;">
						${(accumulatedDistance / 1852).toFixed(2)} sm
					</span>
				</div>
			</div>
		`;
		
		// Event-Handler für das Zentrieren der Karte
		card.querySelector(".wp-center-btn").onclick = () => {
			if (typeof map !== "undefined") {
				map.setView([p.lat, p.lng], map.getZoom(), { animate: true });
			} else {
				console.error("Karten-Objekt 'map' wurde nicht gefunden.");
			}
		};

		// Event-Handler für das Senden an den Autopiloten
		card.querySelector(".wp-auto-btn").onclick = () => {
			sendAutopilotWP(p.lat, p.lng);
		};

		// Event-Handler für das Löschen des Wegpunkts über deine bestehende Funktion
		card.querySelector(".wp-delete-btn").onclick = () => {
			removeWaypoint(i);
		};

		container.appendChild(card);
	});
}



/* =========================================================================
 * UI-EVENT-HANDLER: TOGGLE WAYPOINT PANEL
 * ========================================================================= */
function toggleWaypointPanel() {
	const panel = document.getElementById("navis-waypoint-panel");
	const btn = document.getElementById("waypoint-pin-toggle");
	if (panel.classList.contains("collapsed")) {
		panel.classList.remove("collapsed");
		btn.innerText = "⤡";
	} else {
		panel.classList.add("collapsed");
		btn.innerText = "⤢";
	}
	setTimeout(layoutCollapsedPanels, 50);
}

function sendAutopilotWP(lat, lon) {
	updateRouteDisplay();
	if (demoMode) {
		console.log(`[Demo] Autopilot WP → Lat: ${lat}, Lon: ${lon}`);
		return;
	}
	const url = `/autopilot?lat=${lat.toFixed(6)}&lon=${lon.toFixed(6)}`;
	fetch(url)
		.then(res => {
			if (!res.ok) throw new Error("ESP Antwort fehlerhaft");
			return res.text();
		})
		.then(txt => console.log(`ESP-Antwort: ${txt}`))
		.catch(err => console.error("Fehler beim Senden an ESP:", err));
}
