// ======================================================================
// NAVIS Karten-Grundaufbau
// ======================================================================
function initKarte(initialLat, initialLon) {
    const map = L.map('map', {
        zoomControl: true,
        attributionControl: false
    }).setView([initialLat, initialLon], 10);

    const TILE_URL = 'tiles/osm/{z}/{x}/{y}.png';

    // --- INITIALISIERUNG DER LOKALEN BROWSER-DATENBANK (IndexedDB) ---
    const DB_NAME = 'NavisTileCache';
    const DB_VERSION = 1;

    function getLocalDB() {
        return new Promise((resolve) => {
            const request = indexedDB.open(DB_NAME, DB_VERSION);

            request.onupgradeneeded = (e) => {
                const db = e.target.result;
                if (!db.objectStoreNames.contains('tiles')) {
                    db.createObjectStore('tiles', { keyPath: 'url' });
                }
            };

            request.onsuccess = (e) => resolve(e.target.result);
            request.onerror = () => resolve(null);
        });
    }

    // --- ERWEITERUNG DES LAYERS: ERST CACHE PRÜFEN, DANN ESP ANFRAGEN ---
    const OfflineFirst = L.TileLayer.Throttled.extend({
        createTile: function(coords, done) {
            // Nutzt die Request-Limitierung aus dem vorherigen Update, um Abstürze zu verhindern
            const tile = L.TileLayer.Throttled.prototype.createTile.call(this, coords, done);
            const tileUrl = this.getTileUrl(coords);

            // 1. Zuerst die lokale Datenbank auf dem Gerät (Tablet/PC) abfragen
            getLocalDB().then(db => {
                if (!db) {
                    tile.src = tileUrl;
                    return;
                }

                const transaction = db.transaction('tiles', 'readonly');
                const store = transaction.objectStore('tiles');
                const request = store.get(tileUrl);

                request.onsuccess = (e) => {
                    if (e.target.result) {
                        // Kachel existiert lokal auf dem Endgerät -> aus DB laden
                        tile.src = e.target.result.blobUrl;
                    } else {
                        // Kachel fehlt -> vom ESP32 laden und lokal speichern
                        fetch(tileUrl)
                            .then(res => res.blob())
                            .then(blob => {
                                const reader = new FileReader();
                                reader.onloadend = () => {
                                    const base64data = reader.result;
                                    const writeTx = db.transaction('tiles', 'readwrite');
                                    writeTx.objectStore('tiles').put({
                                        url: tileUrl,
                                        blobUrl: base64data
                                    });
                                };
                                reader.readAsDataURL(blob);
                            })
                            .catch(() => {
                                // Fehler beim Laden vom ESP -> dunkles Land-Tile bleibt sichtbar
                            });
                    }
                };
            });

            return tile;
        }
    });

    // --- LADE-PROFIL AKTIVIEREN ---
    const layerOSM = new OfflineFirst(
        TILE_URL,
        {
            minZoom: 1,
            maxZoom: 16,
            maxNativeZoom: 16,
            attribution: '© OpenStreetMap contributors (offline/cached)',
            throttleDelay: 100,
            maxParallelRequests: 2
        }
    );

    layerOSM.addTo(map);

    layerOSM.on('tileerror', () => {
        if (!map.hasLayer(layerOSM)) {
            map.addLayer(layerOSM);
        }
    });

    return {
        map,
        layerOSM
    };
}
