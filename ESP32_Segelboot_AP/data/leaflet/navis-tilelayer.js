L.TileLayer.Throttled = L.TileLayer.extend({

    initialize: function (url, options) {
        L.TileLayer.prototype.initialize.call(this, url, options);
        this._queue = [];
        this._timer = null;
        this._delay = options.throttleDelay || 250;
    },

    createTile: function (coords, done) {
        const tile = document.createElement('img');
        tile.alt = '';
        tile.setAttribute('role', 'presentation');

        tile.onload = () => done(null, tile);
        tile.onerror = () => done(null, tile);

        this._queue.push({
            tile: tile,
            url: this.getTileUrl(coords)
        });

        this._startQueue();
        return tile;
    },

    _startQueue: function () {
        if (this._timer) return;

        this._timer = setInterval(() => {
            if (this._queue.length === 0) {
                clearInterval(this._timer);
                this._timer = null;
                return;
            }
            const entry = this._queue.shift();
            entry.tile.src = entry.url;
        }, this._delay);
    }
});
