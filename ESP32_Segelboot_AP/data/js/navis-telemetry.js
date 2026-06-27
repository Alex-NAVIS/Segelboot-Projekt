/* ============================================================
   NAVIS TELEMETRY & WEBSOCKET CLIENT (UNIVERSAL)
   ============================================================ */

// Zentraler Datenspeicher für ALLE Seiten
window.navisTelemetry = {};

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
            
            // Daten im globalen Speicher zusammenführen
            Object.assign(window.navisTelemetry, data);

            // NEU: Event feuern, damit UI-Komponenten (Kompass, Karte) reagieren können
            const updateEvent = new CustomEvent("navisTelemetryUpdate", { detail: data });
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
