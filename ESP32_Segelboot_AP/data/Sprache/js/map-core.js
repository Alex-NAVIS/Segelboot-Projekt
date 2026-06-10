/* =========================================================
   NAVIS MAP CORE
========================================================= */

const DATA_URL = '/data.json';
const POLL_INTERVAL_MS = 1000;

window.demoMode = false;
window.followMode = false;
window.distanceSM = 0;

/* =========================================================
   GLOBAL STATE
========================================================= */

window.state = {
    gps_lat: 53.39675,
    gps_lon: 10.42105,
    gps_speed: 0,
    gps_kurs: 315,
    kompass: 310,
    winddir_berechnet: 120,
    windspeed_berechnet: 0
};

window.target = JSON.parse(JSON.stringify(window.state));

/* =========================================================
   MAP
========================================================= */

window.map = L.map('map', {
    zoomControl: true,
    attributionControl: false
}).setView(
    [window.state.gps_lat, window.state.gps_lon],
    10
);

const TILE_URL = 'tiles/osm/{z}/{x}/{y}.png';

window.layerOSM = new L.TileLayer.Throttled(
    TILE_URL,
    {
        minZoom: 1,
        maxZoom: 16,
        maxNativeZoom: 16,
        throttleDelay: 50
    }
);

window.layerOSM.addTo(window.map);

/* =========================================================
   UI ELEMENTE
========================================================= */

window.legendEl = document.getElementById('legend');
window.waypointInfo = document.getElementById('waypointInfo');

/* =========================================================
   GEODESY
========================================================= */

window.destinationPoint = function(lat, lon, bearing, dist) {

    const R = 6371000;

    const δ = dist / R;
    const θ = bearing * Math.PI / 180;

    const φ1 = lat * Math.PI / 180;
    const λ1 = lon * Math.PI / 180;

    const φ2 = Math.asin(
        Math.sin(φ1) * Math.cos(δ) +
        Math.cos(φ1) * Math.sin(δ) * Math.cos(θ)
    );

    const λ2 = λ1 + Math.atan2(
        Math.sin(θ) * Math.sin(δ) * Math.cos(φ1),
        Math.cos(δ) - Math.sin(φ1) * Math.sin(φ2)
    );

    return [
        φ2 * 180 / Math.PI,
        λ2 * 180 / Math.PI
    ];
};

window.bearingAndDistance = function(lat1, lon1, lat2, lon2) {

    const R = 6371000;

    const φ1 = lat1 * Math.PI / 180;
    const φ2 = lat2 * Math.PI / 180;

    const Δλ = (lon2 - lon1) * Math.PI / 180;

    const y = Math.sin(Δλ) * Math.cos(φ2);

    const x =
        Math.cos(φ1) * Math.sin(φ2) -
        Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);

    const brng =
        (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;

    const d = R * Math.acos(
        Math.sin(φ1) * Math.sin(φ2) +
        Math.cos(φ1) * Math.cos(φ2) * Math.cos(Δλ)
    );

    return {
        bearing: brng,
        distance: d
    };
};

/* =========================================================
   TRACK
========================================================= */

window.trackPoints = [];
window.liveTrackLine = null;

/* =========================================================
   LIVE DATA
========================================================= */

window.ws = null;

window.connectWS = function() {

    window.ws = new WebSocket(`ws://${location.hostname}/ws`);

    window.ws.onopen = () => {
        console.log("WebSocket verbunden");
    };

    window.ws.onmessage = (event) => {

        try {

            const j = JSON.parse(event.data);

            if ('gps_lat' in j)
                window.target.gps_lat = Number(j.gps_lat);

            if ('gps_lon' in j)
                window.target.gps_lon = Number(j.gps_lon);

            if ('gps_speed' in j)
                window.target.gps_speed = Number(j.gps_speed);

            if ('gps_kurs' in j)
                window.target.gps_kurs = Number(j.gps_kurs);

            if ('kompass' in j)
                window.target.kompass = Number(j.kompass);

            if ('winddir_berechnet' in j)
                window.target.winddir_berechnet =
                    Number(j.winddir_berechnet);

            if ('windspeed_berechnet' in j)
                window.target.windspeed_berechnet =
                    Number(j.windspeed_berechnet);

            if ("sm" in j) {
                window.distanceSM = Number(j.sm);

                if (window.updateLogDisplay) {
                    window.updateLogDisplay();
                }
            }

        } catch(e) {

            console.warn("WS Fehler:", e);
        }
    };

    window.ws.onerror = (err) => {

        console.warn("WebSocket Fehler:", err);
    };

    window.ws.onclose = () => {

        console.warn("WebSocket getrennt");

        setTimeout(window.connectWS, 3000);
    };
};

window.connectWS();

/* =========================================================
   TRACK HISTORY
========================================================= */

window.loadTrackHistory = async function() {

    try {

        const res = await fetch("/track", {
            cache: "no-store"
        });

        if (!res.ok) {
            throw new Error("Track konnte nicht geladen werden");
        }

        const data = await res.json();

        window.trackPoints.length = 0;

        data.forEach(p => {

            if (
                typeof p.lat !== "number" ||
                typeof p.lon !== "number"
            ) return;

            window.trackPoints.push([
                p.lat,
                p.lon
            ]);
        });

        if (window.liveTrackLine) {

            try {
                window.map.removeLayer(window.liveTrackLine);
            } catch(e){}
        }

        window.liveTrackLine = L.polyline(
            window.trackPoints,
            {
                color: "#00ffff",
                weight: 2,
                opacity: 0.8
            }
        ).addTo(window.map);

    } catch(err) {

        console.warn("Track Fehler:", err);
    }
};

/* =========================================================
   ANIMATION LOOP
========================================================= */

let lastTime = performance.now();

function integrate(now) {

    const dt = (now - lastTime) / 1000;

    lastTime = now;

    const sm = 1 - Math.exp(-2 * dt);

    for (const k in window.state) {

        if (typeof window.target[k] === 'number') {

            window.state[k] +=
                (window.target[k] - window.state[k]) * sm;
        }
    }

    if (window.updateBoatGeometry) {
        window.updateBoatGeometry();
    }

    requestAnimationFrame(integrate);
}

requestAnimationFrame(integrate);

/* =========================================================
   UI BUTTONS
========================================================= */

const centerToggle =
    document.getElementById('centerToggle');

centerToggle.onclick = () => {

    window.followMode = !window.followMode;

    centerToggle.style.background =
        window.followMode
            ? 'rgba(0,100,0,0.6)'
            : 'rgba(0,0,0,0.4)';
};

window.map.on('movestart', () => {

    if (window.followMode) {

        window.followMode = false;

        centerToggle.style.background =
            'rgba(0,0,0,0.4)';
    }
});

/* =========================================================
   NIGHT MODE
========================================================= */

window.isNightMode = false;

const nightToggle =
    document.getElementById('nightToggle');

if (nightToggle) {

    nightToggle.onclick = () => {

        window.isNightMode = !window.isNightMode;

        document.body.classList.toggle(
            'night-mode',
            window.isNightMode
        );

        nightToggle.innerHTML =
            window.isNightMode
                ? '☀ Tagmodus'
                : '🌙 Nachtmodus';
    };
}

/* =========================================================
   START
========================================================= */

window.loadTrackHistory();

setInterval(
    window.loadTrackHistory,
    300050
);