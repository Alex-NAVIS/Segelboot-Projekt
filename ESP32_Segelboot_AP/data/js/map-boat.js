/* =========================================================
   NAVIS BOAT
========================================================= */

let boatTriangle = L.polygon([], {
    color: '#ff0000',
    weight: 2,
    fillColor: '#ff0000',
    fillOpacity: 0.9
}).addTo(map);

const headingLineCompass = L.polyline([], {
    color:'#ff3333',
    weight:2
}).addTo(map);

const headingLineGPS = L.polyline([], {
    color:'#d6b600',
    weight:2
}).addTo(map);

const headingLineWind = L.polyline([], {
    color:'#00cc66',
    weight:2
}).addTo(map);

/* =========================================================
   BOOTSGEOMETRIE
========================================================= */

function updateBoatGeometry() {

    const pos = [state.gps_lat, state.gps_lon];

    const pixelSize = 14;

    const angle = state.kompass * Math.PI / 180;

    function offsetPoint(lat, lon, dx, dy) {

        const p = map.latLngToLayerPoint([lat, lon]);

        return map.layerPointToLatLng(
            L.point(p.x + dx, p.y + dy)
        );
    }

    const tip = offsetPoint(
        pos[0],
        pos[1],
        Math.sin(angle) * pixelSize,
        -Math.cos(angle) * pixelSize
    );

    const leftAngle = angle + Math.PI * 0.75;
    const rightAngle = angle - Math.PI * 0.75;

    const left = offsetPoint(
        pos[0],
        pos[1],
        Math.sin(leftAngle) * pixelSize * 0.6,
        -Math.cos(leftAngle) * pixelSize * 0.6
    );

    const right = offsetPoint(
        pos[0],
        pos[1],
        Math.sin(rightAngle) * pixelSize * 0.6,
        -Math.cos(rightAngle) * pixelSize * 0.6
    );

    boatTriangle.setLatLngs([
        tip,
        right,
        left
    ]);

    if (followMode) {
        map.setView(pos, map.getZoom(), {
            animate: true
        });
    }
}

/* =========================================================
   KURSLINIEN
========================================================= */

function updateHeadingLines() {

    const pos = [state.gps_lat, state.gps_lon];

    const compassEnd = destinationPoint(
        state.gps_lat,
        state.gps_lon,
        state.kompass,
        40000
    );

    const gpsEnd = destinationPoint(
        state.gps_lat,
        state.gps_lon,
        state.gps_kurs,
        40000
    );

    const windEnd = destinationPoint(
        state.gps_lat,
        state.gps_lon,
        state.winddir_berechnet,
        20000
    );

    headingLineCompass.setLatLngs([pos, compassEnd]);
    headingLineGPS.setLatLngs([pos, gpsEnd]);
    headingLineWind.setLatLngs([pos, windEnd]);
}

/* =========================================================
   LEGENDE
========================================================= */

function updateLegend() {

    legendEl.innerHTML =
        `🔴 Kompasskurs: ${state.kompass.toFixed(1)}°<br>` +
        `🟡 GPS-Kurs: ${state.gps_kurs.toFixed(1)}°<br>` +
        `🟢 Wind: ${state.winddir_berechnet.toFixed(1)}° / ` +
        `${state.windspeed_berechnet.toFixed(1)} kn`;
}

/* =========================================================
   UPDATE TIMER
========================================================= */

setInterval(updateHeadingLines, 100);
setInterval(updateLegend, 1000);