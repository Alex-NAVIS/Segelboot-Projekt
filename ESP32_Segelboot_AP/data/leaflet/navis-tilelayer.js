/**
 * L.TileLayer.Throttled - Cache-First Version (2026)
 */
L.TileLayer.Throttled = L.TileLayer.extend({

    initialize: function (url, options) {
        L.TileLayer.prototype.initialize.call(this, url, options);
        this._queue = [];
        this._timer = null;
        this._delay = options.throttleDelay || 100;
        this._fallbackSrc = this._createLandTile(options.landColor || '#1c1f26');
    },

    createTile: function (coords, done) {
        const tile = document.createElement('img');
        tile.className = 'leaflet-tile';
        tile.alt = '';
        tile.setAttribute('role', 'presentation');

        const tileUrl = this.getTileUrl(coords);

        // Standard-Handler
        tile.onload = () => done(null, tile);
        tile.onerror = () => {
            tile.src = this._fallbackSrc;
            done(null, tile);
        };

        /**
         * CACHE-CHECK:
         * Wir setzen die URL. Wenn der Browser sie im Cache hat, 
         * wird 'complete' fast augenblicklich auf true gesetzt.
         */
        tile.src = tileUrl;

        // Kleiner Timeout (0ms) schiebt die Prüfung ans Ende des aktuellen Execution-Stacks
        // Das gibt dem Browser Zeit, den Cache-Status zu prüfen.
        setTimeout(() => {
            if (tile.complete && tile.naturalWidth !== 0) {
                // Bild ist im Cache und geladen -> Nichts weiter tun, onload feuert.
                return;
            } else {
                // Bild ist NICHT im Cache -> URL kurz entfernen, Fallback setzen und queue
                tile.src = this._fallbackSrc;
                this._queue.push({
                    tile: tile,
                    url: tileUrl
                });
                this._startQueue();
            }
        }, 0);

        return tile;
    },

    _startQueue: function () {
        if (this._timer) return;

        this._timer = setInterval(() => {
            if (this._queue.length === 0) {
                this._stopTimer();
                return;
            }

            // Neueste Kacheln zuerst (LIFO)
            const entry = this._queue.pop(); 

            if (entry && entry.tile) {
                // Nur laden, wenn die Kachel noch Teil der Karte ist (nicht weggezoomt)
                if (document.body.contains(entry.tile)) {
                    entry.tile.src = entry.url;
                }
            }
        }, this._delay);
    },

    _stopTimer: function () {
        if (this._timer) {
            clearInterval(this._timer);
            this._timer = null;
        }
    },

    _createLandTile: function (color) {
        const c = document.createElement('canvas');
        c.width = 256; c.height = 256;
        const ctx = c.getContext('2d');
        ctx.fillStyle = color;
        ctx.fillRect(0, 0, 256, 256);
        return c.toDataURL('image/png');
    },

    onRemove: function (map) {
        this._queue = [];
        this._stopTimer();
        L.TileLayer.prototype.onRemove.call(this, map);
    }
});
