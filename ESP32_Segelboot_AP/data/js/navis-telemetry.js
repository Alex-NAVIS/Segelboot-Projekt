/* ============================================================
   NAVIS TELEMETRY & WEBSOCKET CLIENT (UNIVERSAL)
   ============================================================ */
   
// ------------------------------------------------------------
// Filterstärken
// Größer = reagiert schneller
// Kleiner = stärker geglättet
// ------------------------------------------------------------
const FILTER = {
    roll: 0.20,
    pitch: 0.20,
    kompass: 0.08
};

// ============================================================
// Zentraler Datenspeicher
// ============================================================

// Originaldaten vom ESP (niemals verändern)
window.navisTelemetry = {};
// letzter bekannter Systemstatus
window.navisSystemStatus = {
    ok:true,
    hasErrors:false,
    hasWarnings:false,
    issues:[]
};

// Geglättete Daten ausschließlich für die Anzeige
window.navisTelemetryDisplay = {
    roll: 0,
    pitch: 0,
    kompass: 0
};

// Anzeige wurde bereits initialisiert?
let displayInitialized = false;

// ------------------------------------------------------------
// Tiefpassfilter
// ------------------------------------------------------------
function lowpass(oldValue, newValue, alpha = 0.15) {
    return oldValue + alpha * (newValue - oldValue);
}

// ------------------------------------------------------------
// Winkeldifferenz (0° / 360° korrekt behandeln)
// ------------------------------------------------------------
function angleDiff(a, b) {
    return ((b - a + 540) % 360) - 180;
}

// Globaler Socket für Steuerbefehle aus der index.html
window.navisTelemetrySocket = null; 

let reconnectTimer = null;
let reconnectDelay = 1000;
const MAX_RECONNECT_DELAY = 5000;

// LED-Statusanzeige aktualisieren
function updateWsLed(color, glow) {
    const wsLed = document.getElementById("wsLed");
    if (!wsLed) return;
    wsLed.style.background = color;
    wsLed.style.boxShadow = `0 0 6px ${glow}`;
}

function ledDisconnected() { updateWsLed("#ff3333", "rgba(255,0,0,0.8)"); }
function ledConnecting()   { updateWsLed("#ffcc00", "rgba(255,204,0,0.8)"); }
function ledConnected()    { updateWsLed("#00ff66", "rgba(0,255,102,0.9)"); }

// NAVIS Hardware Status Bewertung
const isOnline = value => Number(value) === 1 || Number(value) === 2;

function checkHardwareStatus(data) {
    const issues = [];

    // Masteinheit & IMU
    if (!isOnline(data.mast_online)) issues.push({ level: "error", text: "Masteinheit OFFLINE" });
    if (!isOnline(data.imu_online)) issues.push({ level: "error", text: "IMU Sensor Fehler" });

    // GPS
    const gps = Number(data.gps_online);
    if (gps === 1) issues.push({ level: "warning", text: "GPS sucht Satelliten" });
    else if (gps !== 2) issues.push({ level: "error", text: "GPS Sensor Fehler" });

    // SD Karte
    const sd = Number(data.sd_online);
    if (sd === 1) issues.push({ level: "warning", text: "SD Karte voll" });
    else if (sd !== 2) issues.push({ level: "error", text: "SD Karte fehlt" });

    // Status-Objekt zusammenbauen
    const status = {
        ok: issues.length === 0,
        hasErrors: issues.some(i => i.level === "error"),
        hasWarnings: issues.some(i => i.level === "warning"),
        issues: issues
    };

    window.navisSystemStatus = status;

	window.dispatchEvent(
		new CustomEvent("navisSystemStatus", { 
			detail: status 
		})
	);
}

// NEU: Globale Funktion zum sicheren Senden von Befehlen an den ESP32
window.sendNavisCommand = function(commandObject) {
    if (window.navisTelemetrySocket && window.navisTelemetrySocket.readyState === WebSocket.OPEN) {
        window.navisTelemetrySocket.send(JSON.stringify(commandObject));
        return true;
    }
    console.error("WS: Befehl konnte nicht gesendet werden. Keine Verbindung.");
    return false;
};

function connectWS() {
    if (window.navisTelemetrySocket && window.navisTelemetrySocket.readyState === WebSocket.OPEN) return;

    ledConnecting();
    console.log("WS: Verbindungsversuch …");

    window.navisTelemetrySocket = new WebSocket(`ws://${window.location.host}/ws`);

    window.navisTelemetrySocket.onopen = () => {
        console.log("WS verbunden");
        ledConnected();
        reconnectDelay = 1000;
    };

    window.navisTelemetrySocket.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
			// ------------------------------------------------------------
			// Originaldaten speichern
			// ------------------------------------------------------------
			Object.assign(window.navisTelemetry, data);


			// ------------------------------------------------------------
			// Anzeige weichzeichnen
			// ------------------------------------------------------------
			const d = window.navisTelemetry;
			const f = window.navisTelemetryDisplay;

			// Hardwarestatus prüfen
			checkHardwareStatus(d);

			// Zuerst alle Rohdaten übernehmen
			Object.assign(f, d);

			// Roll / Pitch / Kompass nur einmal initialisieren
			if (!displayInitialized) {

				f.roll = Number(d.roll) || 0;
				f.pitch = Number(d.pitch) || 0;
				f.kompass = Number(d.kompass) || 0;

				displayInitialized = true;
			}
			else {

				if (isFinite(d.roll))
					f.roll = lowpass(f.roll, Number(d.roll), FILTER.roll);

				if (isFinite(d.pitch))
					f.pitch = lowpass(f.pitch, Number(d.pitch), FILTER.pitch);

				if (isFinite(d.kompass)) {

					f.kompass += angleDiff(f.kompass, Number(d.kompass)) * FILTER.kompass;

					if (f.kompass < 0)
						f.kompass += 360;

					if (f.kompass >= 360)
						f.kompass -= 360;
				}
			}


			// ------------------------------------------------------------
			// Event feuern
			// ------------------------------------------------------------
			const updateEvent = new CustomEvent("navisTelemetryUpdate", {
				detail: {
					raw: window.navisTelemetry,
					display: window.navisTelemetryDisplay
				}
			});

			window.dispatchEvent(updateEvent);

        } catch (e) {
            console.warn("WS Datenfehler", e);
        }
    };

    window.navisTelemetrySocket.onerror = () => {
        window.navisTelemetrySocket.close();
    };

    window.navisTelemetrySocket.onclose = () => {
        console.warn("WS getrennt");
        ledDisconnected();
        window.navisTelemetrySocket = null;

        if (!reconnectTimer) {
            reconnectTimer = setTimeout(() => {
                reconnectTimer = null;
                reconnectDelay = Math.min(reconnectDelay * 1.5, MAX_RECONNECT_DELAY);
                connectWS();
            }, reconnectDelay);
        }
    };
}

// Verbindung starten
connectWS();
