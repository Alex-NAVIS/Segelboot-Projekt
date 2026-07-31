/**
 * L.TileLayer.Throttled - Echtzeit-kontrollierte Version (2026)
 * Schützt den ESP32 vor Socket-Überlastung durch strikte Limitierung paralleler Requests.
 */
L.TileLayer.Throttled = L.TileLayer.extend({

    initialize: function (url, options) {
        L.TileLayer.prototype.initialize.call(this, url, options);
        this._queue = [];
        this._activeRequests = 0;
        // Maximal 2-3 gleichzeitige Requests erlauben (ESP32-S3 verträgt nicht mehr)
        this._maxParallelRequests = options.maxParallelRequests || 2; 
        this._fallbackSrc = this._createLandTile(options.landColor || '#1c1f26');
    },

    createTile: function (coords, done) {
        const tile = document.createElement('img');
        tile.className = 'leaflet-tile';
        tile.alt = '';
        tile.setAttribute('role', 'presentation');

        const tileUrl = this.getTileUrl(coords);

        // Standardmäßig sofort das leere Land-Tile anzeigen (verhindert weißes Blitzen)
        tile.src = this._fallbackSrc;

        // Kachel in die Warteschlange einreihen
        this._queue.push({
            tile: tile,
            url: tileUrl,
            done: done
        });

        // Warteschlange triggern
        setTimeout(() => this._processQueue(), 0);

        return tile;
    },

    _processQueue: function () {
        // Wenn das Limit aktiver Requests erreicht ist oder die Queue leer ist -> Stopp
        if (this._activeRequests >= this._maxParallelRequests || this._queue.length === 0) {
            return;
        }

        // LIFO: Die neueste Kachel (wo der Nutzer gerade hinschaut) zuerst laden
        const entry = this._queue.pop();

        if (!entry) return;

        // Prüfen, ob die Kachel überhaupt noch auf der Karte existiert (wichtig beim Zoomen!)
        if (!document.body.contains(entry.tile)) {
            entry.done(null, entry.tile); // Feuern, damit Leaflet Ressourcen freigibt
            setTimeout(() => this._processQueue(), 0);
            return;
        }

        // Request wird jetzt aktiv
        this._activeRequests++;

        // Hilfsfunktion zum Aufräumen nach Erfolg/Fehler
        const next = () => {
            this._activeRequests--;
            this._processQueue(); // Nächsten Request aus der Queue starten
        };

        entry.tile.onload = () => {
            entry.done(null, entry.tile);
            next();
        };

        entry.tile.onerror = () => {
            entry.tile.src = this._fallbackSrc;
            entry.done(null, entry.tile);
            next();
        };

        // Erst HIER wird der tatsächliche HTTP-Request an den ESP32 abgesetzt
        entry.tile.src = entry.url;

        // Falls noch Plätze frei sind, direkt den nächsten Request anstoßen
        if (this._activeRequests < this._maxParallelRequests) {
            setTimeout(() => this._processQueue(), 0);
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
        this._activeRequests = 0;
        L.TileLayer.prototype.onRemove.call(this, map);
    }
});
