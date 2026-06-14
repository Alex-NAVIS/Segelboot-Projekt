/* =========================================================
   NAVIS WEATHER
========================================================= */

let windLayerDemo = null;

const WEATHER_API_KEY = "052994578ac0357e51c1a900f2212731";

/* =========================================================
   WEATHER TILE LAYER
========================================================= */

const rainLayer = L.tileLayer(
    `https://tile.openweathermap.org/map/precipitation_new/{z}/{x}/{y}.png?appid=${WEATHER_API_KEY}`,
    {
        opacity: 0.7,
        attribution: '© OpenWeatherMap'
    }
);

const cloudsLayer = L.tileLayer(
    `https://tile.openweathermap.org/map/clouds_new/{z}/{x}/{y}.png?appid=${WEATHER_API_KEY}`,
    {
        opacity: 0.5,
        attribution: '© OpenWeatherMap'
    }
);

const pressureLayer = L.tileLayer(
    `https://tile.openweathermap.org/map/pressure_new/{z}/{x}/{y}.png?appid=${WEATHER_API_KEY}`,
    {
        opacity: 0.6,
        attribution: '© OpenWeatherMap'
    }
);

const tempLayer = L.tileLayer(
    `https://tile.openweathermap.org/map/temp_new/{z}/{x}/{y}.png?appid=${WEATHER_API_KEY}`,
    {
        opacity: 0.6,
        attribution: '© OpenWeatherMap'
    }
);

/* =========================================================
   LAYER REGISTRY
========================================================= */

const layers = {
    wind: null,
    rain: rainLayer,
    clouds: cloudsLayer,
    pressure: pressureLayer,
    temp: tempLayer
};

/* =========================================================
   WIND DATA INDEX
========================================================= */

let windFiles = [];
let currentWindIndex = 0;
let windAnimationTimer = null;

/* =========================================================
   WIND LAYER LOADING
========================================================= */

async function loadWindData(filename) {

    return new Promise((resolve) => {

        if (!filename) {
            resolve(false);
            return;
        }

        // Alten Layer entfernen
        if (windLayerDemo) {
            try {
                map.removeLayer(windLayerDemo);
            } catch (e) {}

            windLayerDemo = null;
            layers.wind = null;
        }

        fetch("Wetter/" + filename + "?t=" + Date.now())
            .then(res => {

                if (!res.ok) {
                    throw new Error("Winddatei nicht lesbar");
                }

                return res.json();
            })
            .then(data => {

                windLayerDemo = L.velocityLayer({

                    data: data,

                    displayValues: true,

                    displayOptions: {
                        velocityType: "Wind",
                        position: "bottomleft",
                        emptyString: "keine Daten",
                        angleConvention: "bearingCW",
                        speedUnit: "kt"
                    },

                    maxVelocity: 25,
                    velocityScale: 0.008,

                    particleAge: 120,
                    lineWidth: 2,

                    particleMultiplier: 1 / 300,
                    frameRate: 25,

                    colorScale: [
                        "#ffffff",
                        "#99ccff",
                        "#66ffcc",
                        "#ffff66",
                        "#ffcc33",
                        "#ff6600",
                        "#ff0000"
                    ]
                });

                layers.wind = windLayerDemo;

                resolve(true);
            })
            .catch(err => {

                console.warn("Fehler beim Laden:", filename, err);

                resolve(false);
            });
    });
}

/* =========================================================
   WIND INDEX LADEN
========================================================= */

async function loadWindIndex() {

    try {

        const res = await fetch("Wetter/index.json?t=" + Date.now());

        if (!res.ok) {
            throw new Error("index.json nicht gefunden");
        }

        const data = await res.json();

        if (!Array.isArray(data)) {
            console.warn("Windindex ungültig");
            return [];
        }

        windFiles = data;

        return data;

    } catch (err) {

        console.warn("Keine Wind-Index Datei", err);

        windFiles = [];

        return [];
    }
}

/* =========================================================
   ERSTE WINDDATEI LADEN
========================================================= */

async function initWindLayer() {

    const files = await loadWindIndex();

    if (!files.length) {
        console.warn("Keine Winddateien vorhanden");
        return;
    }

    currentWindIndex = 0;

    const ok = await loadWindData(files[currentWindIndex]);

    if (ok && layers.wind) {
        layers.wind.addTo(map);
    }
}

/* =========================================================
   NÄCHSTE WINDDATEI
========================================================= */

async function nextWindFrame() {

    if (!windFiles.length) return;

    currentWindIndex++;

    if (currentWindIndex >= windFiles.length) {
        currentWindIndex = 0;
    }

    const wasVisible = windLayerDemo && map.hasLayer(windLayerDemo);

    await loadWindData(windFiles[currentWindIndex]);

    if (wasVisible && layers.wind) {
        layers.wind.addTo(map);
    }
}

/* =========================================================
   VORHERIGE WINDDATEI
========================================================= */

async function previousWindFrame() {

    if (!windFiles.length) return;

    currentWindIndex--;

    if (currentWindIndex < 0) {
        currentWindIndex = windFiles.length - 1;
    }

    const wasVisible = windLayerDemo && map.hasLayer(windLayerDemo);

    await loadWindData(windFiles[currentWindIndex]);

    if (wasVisible && layers.wind) {
        layers.wind.addTo(map);
    }
}

/* =========================================================
   WIND ANIMATION START
========================================================= */

function startWindAnimation(intervalMs = 4000) {

    stopWindAnimation();

    windAnimationTimer = setInterval(() => {
        nextWindFrame();
    }, intervalMs);

    console.log("Windanimation gestartet");
}

/* =========================================================
   WIND ANIMATION STOP
========================================================= */

function stopWindAnimation() {

    if (windAnimationTimer) {
        clearInterval(windAnimationTimer);
        windAnimationTimer = null;
    }

    console.log("Windanimation gestoppt");
}

/* =========================================================
   BESTIMMTE WINDDATEI LADEN
========================================================= */

async function setWindFrame(index) {

    if (!windFiles.length) return;

    if (index < 0 || index >= windFiles.length) return;

    currentWindIndex = index;

    const wasVisible = windLayerDemo && map.hasLayer(windLayerDemo);

    await loadWindData(windFiles[currentWindIndex]);

    if (wasVisible && layers.wind) {
        layers.wind.addTo(map);
    }
}

/* =========================================================
   AKTUELLEN WINDDATEI-NAMEN HOLEN
========================================================= */

function getCurrentWindFilename() {

    if (!windFiles.length) return null;

    return windFiles[currentWindIndex];
}

/* =========================================================
   LAYER EIN/AUSBLENDEN
========================================================= */

function toggleLayer(layerName, isVisible) {

    const layer = layers[layerName];

    if (!layer) {
        console.warn("Layer nicht gefunden:", layerName);
        return;
    }

    if (isVisible) {

        if (!map.hasLayer(layer)) {
            layer.addTo(map);
        }

    } else {

        if (map.hasLayer(layer)) {
            map.removeLayer(layer);
        }
    }
}

/* =========================================================
   ALLE WETTERLAYER AUS
========================================================= */

function removeAllWeatherLayers() {

    Object.values(layers).forEach(layer => {

        if (!layer) return;

        try {

            if (map.hasLayer(layer)) {
                map.removeLayer(layer);
            }

        } catch (e) {}
    });
}

/* =========================================================
   NUR EINEN LAYER AKTIVIEREN
========================================================= */

function showExclusiveWeatherLayer(layerName) {

    removeAllWeatherLayers();

    const layer = layers[layerName];

    if (!layer) return;

    layer.addTo(map);
}

/* =========================================================
   LAYER SICHTBAR?
========================================================= */

function isLayerVisible(layerName) {

    const layer = layers[layerName];

    if (!layer) return false;

    return map.hasLayer(layer);
}

/* =========================================================
   OPACITY ÄNDERN
========================================================= */

function setLayerOpacity(layerName, opacity) {

    const layer = layers[layerName];

    if (!layer) return;

    if (typeof layer.setOpacity === "function") {
        layer.setOpacity(opacity);
    }
}

/* =========================================================
   WIND NEU LADEN
========================================================= */

async function reloadWindLayer() {

    if (!windFiles.length) {
        await loadWindIndex();
    }

    if (!windFiles.length) return;

    const visible = windLayerDemo && map.hasLayer(windLayerDemo);

    await loadWindData(windFiles[currentWindIndex]);

    if (visible && layers.wind) {
        layers.wind.addTo(map);
    }
}

/* =========================================================
   AUTOMATISCH INITIALISIEREN
========================================================= */

document.addEventListener("DOMContentLoaded", async () => {

    await initWindLayer();

    console.log("NAVIS Weather initialisiert");
});