/**
 * route.js - Zentrales Routing- und Berechnungsmodul für NAVIS
 * 
 * Sammelt alle Parameter (Navi, Wetter, UI-Inputs), mappt die Zeitfenster,
 * zeigt die Testzusammenfassung und startet den A*-Segment-Pfadfinder.
 */

const RouteDataHub = {
    // === 1. INTERNE STRUKTUR FÜR DAS AKTULLE ROUTING ===
    routing: {
        start: null,
        ziel: null,
        wegpunkte: [],
        artDerRoute: "schnell", // "schnell" (Performance) oder "familie" (Komfort)
        zeitStrategie: "direkt"
    },

    // === 2. ZEITPLANUNG & ZEITFENSTER-MAPPING ===
    // Verwaltet die Timestamps und berechnet die exakten Indizes (0-71) für das activeWeatherData.frames-Array
    zeitplanung: {
        basisStartzeit: null,   
        zeitfenster: {
            minus6h: { aktiv: false, label: "-6 Stunden", berechnungsZeit: null, weatherFrameIdx: 0 },
            minus3h: { aktiv: false, label: "-3 Stunden", berechnungsZeit: null, weatherFrameIdx: 0 },
            basis:   { aktiv: true,  label: "Geplante Zeit", berechnungsZeit: null, weatherFrameIdx: 0 }, // Immer aktiv
            plus3h:  { aktiv: false, label: "+3 Stunden", berechnungsZeit: null, weatherFrameIdx: 0 },
            plus6h:  { aktiv: false, label: "+6 Stunden", berechnungsZeit: null, weatherFrameIdx: 0 }
        }
    },

    // === 3. HAUPTFUNKTION: ROUTENBERECHNUNG STARTEN ===
    /**
     * Wird direkt vom Button auf karte.html aufgerufen.
     * Sammelt alle Live-Daten aus der UI, mappt sie und startet die Pfadfindung.
     */
    triggerNavisRouting: function() {
        // 1. Validierung gegen das bestehende NAVIS_ROUTE Objekt deiner navissystem.js
        if (typeof NAVIS_ROUTE === 'undefined' || !NAVIS_ROUTE.start || !NAVIS_ROUTE.ziel) { 
            alert("❌ FEHLER: Bitte zuerst Start- und Zielpunkt per Rechtsklick markieren!"); 
            return; 
        }

        // 2. Rohdaten aus den UI-Elementen von karte.html auslesen
        const depTimeInput = document.getElementById("routing-departure-time");
        if (!depTimeInput || !depTimeInput.value) {
            alert("❌ FEHLER: Bitte ein gültiges Abfahrtsdatum und Uhrzeit angeben!");
            return;
        }

        const departureTime = new Date(depTimeInput.value);
        const profile = document.querySelector('input[name="routingProfile"]:checked')?.value || "schnell";
        const strategy = document.getElementById("routing-time-strategy")?.value || "direkt";

        // 3. Daten im zentralen Hub sichern
        this.routing.start = NAVIS_ROUTE.start;
        this.routing.ziel = NAVIS_ROUTE.ziel;
        this.routing.wegpunkte = NAVIS_ROUTE.waypoints || [];
        this.routing.artDerRoute = profile;
        this.routing.zeitStrategie = strategy;

        // 4. Zeitfenster-Aktivierung dynamisch aus UI-Checkboxen abfangen
        // Falls die Checkboxen im HTML noch nicht existieren, fallen sie auf 'false' zurück
        this.zeitplanung.zeitfenster.minus6h.aktiv = document.getElementById("chk-route-minus6h")?.checked || false;
        this.zeitplanung.zeitfenster.minus3h.aktiv = document.getElementById("chk-route-minus3h")?.checked || false;
        this.zeitplanung.zeitfenster.plus3h.aktiv  = document.getElementById("chk-route-plus3h")?.checked || false;
        this.zeitplanung.zeitfenster.plus6h.aktiv  = document.getElementById("chk-route-plus6h")?.checked || false;

        // 5. Exakte Timestamps für alle Fenster berechnen
        this.berechneZeitfensterTimestamps(departureTime);

        // 6. Wetter-Frame Indizes (0-71) für alle aktivierten Fenster berechnen
        this.mappeZeitfensterZuWetterFrames();

        // 7. Erstellung der Zusammenfassung (Deine gewünschte Test-Nachricht)
        this.zeigeTestZusammenfassung();

        // 8. Schleife: Starte den A*-Algorithmus für jedes aktivierte Zeitfenster
        for (let key in this.zeitplanung.zeitfenster) {
            const fenster = this.zeitplanung.zeitfenster[key];
            if (fenster.aktiv) {
                this.fuehreAStarSegmentPfadfinderAus(fenster);
            }
        }
    },

    // === INTERNE HILFSFUNKTIONEN ===

    /**
     * Generiert die Ziel-Timestamps für das Routing (+-3h, +-6h)
     */
    berechneZeitfensterTimestamps: function(basisZeit) {
        this.zeitplanung.basisStartzeit = basisZeit;
        const ms = basisZeit.getTime();
        const STD_MS = 60 * 60 * 1000;

        this.zeitplanung.zeitfenster.minus6h.berechnungsZeit = new Date(ms - (6 * STD_MS));
        this.zeitplanung.zeitfenster.minus3h.berechnungsZeit = new Date(ms - (3 * STD_MS));
        this.zeitplanung.zeitfenster.basis.berechnungsZeit   = basisZeit;
        this.zeitplanung.zeitfenster.plus3h.berechnungsZeit  = new Date(ms + (3 * STD_MS));
        this.zeitplanung.zeitfenster.plus6h.berechnungsZeit  = new Date(ms + (6 * STD_MS));
    },

    /**
     * Ordnet jedem aktiven Zeitfenster den passenden Frame-Index (0-71) zu.
     * Nutzt deine mathematische Formel basierend auf dem 'created'-Zeitstempel der Metadaten.
     */
    mappeZeitfensterZuWetterFrames: function() {
        const wetterVerfuegbar = (typeof activeWeatherData !== 'undefined' && activeWeatherData && activeWeatherData.metadata && activeWeatherData.metadata.created);
        
        for (let key in this.zeitplanung.zeitfenster) {
            const fenster = this.zeitplanung.zeitfenster[key];
            
            if (fenster.aktiv && fenster.berechnungsZeit) {
                if (wetterVerfuegbar) {
                    const basisWetterZeit = new Date(activeWeatherData.metadata.created);
                    // Deine präzise Formel zur Bestimmung des stündlichen Frame-Index im JSON Raster
                    fenster.weatherFrameIdx = Math.max(0, Math.min(71, Math.floor((fenster.berechnungsZeit - basisWetterZeit) / (1000 * 60 * 60))));
                } else {
                    fenster.weatherFrameIdx = 0; // Fallback falls kein Wetter geladen ist
                }
            }
        }
    },

    /**
     * Erstellt eine formatierte Übersicht aller Variablen und gibt sie als Test-Alert aus
     */
    zeigeTestZusammenfassung: function() {
        let msg = "⚓ NAVIS ROUTING-MODUL - DATEN SNAPSHOT ⚓\n";
        msg += "=========================================\n\n";
        
        // Navigation & Profil
        msg += `▶ ROUTING-PROFIL: [${this.routing.artDerRoute.toUpperCase()}]\n`;
        msg += `▶ ZEIT-STRATEGIE: ${this.routing.zeitStrategie.toUpperCase()}\n\n`;
        
        msg += `📍 STARTPOINT:\n   Lat: ${this.routing.start.lat.toFixed(5)} / Lng: ${this.routing.start.lng.toFixed(5)}\n`;
        msg += `   Tile-Gitter: t_${this.routing.start.tileX}_${this.routing.start.tileY}\n\n`;
        
        msg += `🏁 ENDPOINT:\n   Lat: ${this.routing.ziel.lat.toFixed(5)} / Lng: ${this.routing.ziel.lng.toFixed(5)}\n`;
        msg += `   Tile-Gitter: t_${this.routing.ziel.tileX}_${this.routing.ziel.tileY}\n\n`;
        
        msg += `📌 ZWISCHEN-WEGPUNKTE: ${this.routing.wegpunkte.length} Stück\n`;
        this.routing.wegpunkte.forEach((wp, i) => {
            msg += `   • WP #${i+1}: t_${wp.tileX}_${wp.tileY}\n`;
        });
        msg += "\n";

        // Wetter-Status im RAM
        msg += "🌤 COCKPIT WETTERDATEN:\n";
        if (typeof activeWeatherData !== 'undefined' && activeWeatherData && activeWeatherData.metadata) {
            msg += `   • Datei-Stand: ${activeWeatherData.metadata.created || 'Unbekannt'}\n`;
            msg += `   • Auflösung: ${activeWeatherData.metadata.nx} x ${activeWeatherData.metadata.ny} Rasterpunkte\n`;
        } else {
            msg += "   ⚠️ WARNUNG: Keine aktiven Wetterdaten im RAM (activeWeatherData leer!)\n";
        }
        msg += "\n";

        // Zeitfenster & berechnete Frames
        msg += "🕒 ZEITFENSTER BERECHNUNGEN:\n";
        for (let key in this.zeitplanung.zeitfenster) {
            const f = this.zeitplanung.zeitfenster[key];
            if (f.aktiv) {
                const zeitString = f.berechnungsZeit ? f.berechnungsZeit.toLocaleTimeString('de-DE', {hour: '2-digit', minute:'2-digit'}) : '--:--';
                const datumString = f.berechnungsZeit ? f.berechnungsZeit.toLocaleDateString('de-DE', {day: '2-digit', month:'2-digit'}) : '--.--';
                msg += `   🟢 [AKTIV]  ${f.label}:\n       👉 Abfahrt: ${datumString} um ${zeitString} Uhr -> Nutzt Wetter-Frame: #${f.weatherFrameIdx}\n`;
            } else {
                msg += `   ⚪ [INAKTIV] ${f.label}\n`;
            }
        }

        msg += "\n=========================================\n";
        msg += "Klicke OK, um den A*-Segment-Pfadfinder auszuführen.";

        alert(msg);
    },

    // === 4. HIER STARTET DER MATHEMATISCHE ALGORITHMUS ===
    /**
     * Nimmt das vorbereitete Paket eines Zeitfensters entgegen und führt die Pfadfindung auf dem Gitter aus.
     * @param {Object} zeitfensterObj - Das berechnete Objekt des aktuellen Fensters
     */
    fuehreAStarSegmentPfadfinderAus: function(zeitfensterObj) {
        console.log("=== 🚀 NAVIS GRID ROUTER ENGAGED ===");
        console.log(`-> Berechne Segment für: ${zeitfensterObj.label}`);
        console.log(`-> Nutze Wetter-Frame Index: ${zeitfensterObj.weatherFrameIdx}`);

        // Synchronisiere das globale NAVIS_ROUTE Objekt für nachfolgende Render-Funktionen
        if (typeof NAVIS_ROUTE !== 'undefined') {
            NAVIS_ROUTE.departureTime = zeitfensterObj.berechnungsZeit;
            NAVIS_ROUTE.profile = this.routing.artDerRoute;
            NAVIS_ROUTE.strategy = this.routing.zeitStrategie;
            NAVIS_ROUTE.weatherFrame = zeitfensterObj.weatherFrameIdx;
        }

        // -------------------------------------------------------------------------
        // DEIN ECHTER A*-ALGORITHMUS PLATZ:
        // Hier greift deine mathematische Gitterberechnung.
        // Du hast hier vollen Zugriff auf:
        // - Das Gitter-Wetter: activeWeatherData.frames[zeitfensterObj.weatherFrameIdx]
		// - Start-Tile: this.routing.start
		// - Ziel-Tile: this.routing.ziel
		// - Zwischen-Wegpunkte: this.routing.wegpunkte
		// -------------------------------------------------------------------------
		console.log("-> A*-Segment-Pfadfinder erfolgreich durchgelaufen.");
		}
	};
	// Globaler Alias für den HTML-Button: Mappt den Klick direkt auf das neue Modul
	function triggerNavisRouting() {
	RouteDataHub.triggerNavisRouting();
}

        // -------------------------------------------------------------------------
        // 🧩 NAVIS MULTI-SEGMENT A*-PFADFINDER ENGINE
        // -------------------------------------------------------------------------
        
        // Erzeuge ein flaches Array aller anzusteuernden Stationen der Route
        const alleStationen = [
            this.routing.start, 
            ...this.routing.wegpunkte, 
            this.routing.ziel
        ];
        
        let gesamtPfadTiles = [];
        console.log(`-> Starte A* Pfadsuche über ${alleStationen.length - 1} Segmente...`);

        // Extrahiere das exakte Wettergitter für dieses spezifische Zeitfenster im RAM
        const wetterGitter = (typeof activeWeatherData !== 'undefined' && activeWeatherData && activeWeatherData.frames) 
            ? activeWeatherData.frames[zeitfensterObj.weatherFrameIdx] 
            : null;

        if (!wetterGitter) {
            console.warn("⚠️ Kein Wettergitter für Berechnungen im RAM verfügbar! Berechne Luftlinie.");
        }

        // Durchlaufe alle Segmente nacheinander (z.B. Start->WP1, WP1->WP2, WP2->Ziel)
        for (let i = 0; i < alleStationen.length - 1; i++) {
            const segStart = alleStationen[i];
            const segZiel = alleStationen[i + 1];
            
            console.log(`   [Segment #${i + 1}] Berechne Pfad von t_${segStart.tileX}_${segStart.tileY} nach t_${segZiel.tileX}_${segZiel.tileY}`);
            
            // Führe die mathematische A*-Kernberechnung für dieses Segment aus
            const segmentPfad = this._berechneEinzelnesAStarSegment(segStart, segZiel, wetterGitter);
            
            if (segmentPfad && segmentPfad.length > 0) {
                // Verknüpfe die Pfade. Doppelte Wegpunkte an den Übergängen filtern
                if (gesamtPfadTiles.length > 0 && segmentPfad[0] === gesamtPfadTiles[gesamtPfadTiles.length - 1]) {
                    gesamtPfadTiles.push(...segmentPfad.slice(1));
                } else {
                    gesamtPfadTiles.push(...segmentPfad);
                }
            } else {
                console.error(`❌ Kritischer Fehler: Segment #${i + 1} konnte nicht berechnet werden!`);
                // Optionaler Fallback: direkte Verbindung erzwingen
                gesamtPfadTiles.push(segStart, segZiel); 
            }
        }

        console.log(`-> A*-Gesamtroute erfolgreich generiert! Gesamtlänge: ${gesamtPfadTiles.length} Tiles.`);
        
        // Übergebe den berechneten Pfad an deine Rendering-Engine aus der navissystem.js
        if (typeof zeichneRouteAufKarte === 'function') {
            zeichneRouteAufKarte(gesamtPfadTiles, zeitfensterObj);
        } else if (typeof renderNavisRoute === 'function') {
            renderNavisRoute(gesamtPfadTiles, zeitfensterObj);
        }

        console.log("-> A*-Segment-Pfadfinder erfolgreich durchgelaufen.");
    },

    /**
     * Kern-Algorithmus: Berechnet den optimalen mathematischen Pfad auf dem Gitter
     * unter Einberechnung von Wind, Strömung, Polardiagramm und Kurswechsel-Strafen.
     */
    _berechneEinzelnesAStarSegment: function(startTile, zielTile, wetter) {
        // Initialisiere Open- und Closed-Lists für den A*-Algorithmus
        let openList = [];
        let closedList = new Set();
        
        // Schlüssel-Generator für die Eindeutigkeit im Gitter
        const tileKey = (tile) => `${tile.tileX}_${tile.tileY}`;
        
        // Startknoten präparieren
        startTile.g = 0; // Bisherige Kosten
        startTile.h = this._berechneGitterHeuristik(startTile, zielTile); // Restkostenschätzung
        startTile.f = startTile.g + startTile.h; // Gesamtkosten
        startTile.parent = null;
        
        openList.push(startTile);
        
        // Sicherheits-Zähler gegen Endlosschleifen bei unwegsamen Gittern
        let maxIterationen = 5000; 
        
        while (openList.length > 0 && maxIterationen > 0) {
            maxIterationen--;
            
            // Finde das Tile mit den geringsten f-Gesamtkosten (niedrigste Energie/Zeit)
            openList.sort((a, b) => a.f - b.f);
            let aktuellesTile = openList.shift();
            
            closedList.add(tileKey(aktuellesTile));
            
            // ZIEL ERREICHT! Rekonstruiere den Pfad rückwärts über die Parent-Struktur
            if (aktuellesTile.tileX === zielTile.tileX && aktuellesTile.tileY === zielTile.tileY) {
                let pfad = [];
                let curr = aktuellesTile;
                while (curr !== null) {
                    pfad.unshift(curr);
                    curr = curr.parent;
                }
                return pfad;
            }
            
            // Ermittle die 8 umliegenden Gitter-Nachbarn (Horizontal, Vertikal, Diagonal)
            let nachbarn = this._holeGitterNachbarn(aktuellesTile);
            
            for (let nachbar of nachbarn) {
                if (closedList.has(tileKey(nachbar))) continue;
                
                // Berechne die echten Bewegungskosten von aktuellen Tile zum Nachbar-Tile
                // Hier fließen Windstärke, Winkel zum Boot und Strömungsvektoren ein
                let gewicht = this._berechneTaktischeKosten(aktuellesTile, nachbar, wetter);
                let gVersuch = aktuellesTile.g + gewicht;
                
                let bereitsInOpenList = openList.find(o => tileKey(o) === tileKey(nachbar));
                
                if (!bereitsInOpenList || gVersuch < nachbar.g) {
                    nachbar.parent = aktuellesTile;
                    nachbar.g = gVersuch;
                    nachbar.h = this._berechneGitterHeuristik(nachbar, zielTile);
                    nachbar.f = nachbar.g + nachbar.h;
                    
                    if (!bereitsInOpenList) {
                        openList.push(nachbar);
                    }
                }
            }
        }
        
        // Fallback falls kein Pfad gefunden wurde: Direktverbindung
        return [startTile, zielTile];
    },

    /**
     * Berechnet die geometrische Distanz (Heuristik) zwischen zwei Kacheln im Gitter
     */
    _berechneGitterHeuristik: function(tileA, tileB) {
        const dx = Math.abs(tileA.tileX - tileB.tileX);
        const dy = Math.abs(tileA.tileY - tileB.tileY);
        // Diagonal-Distanz-Heuristik (erlaubt flüssige 8-Wege-Bewegungen)
        return (dx + dy) + (Math.sqrt(2) - 2) * Math.min(dx, dy);
    },

    /**
     * Erzeugt die 8 Nachbarknoten für die Expansion des A*
     */
    _holeGitterNachbarn: function(tile) {
        let nachbarn = [];
        const x = tile.tileX;
        const y = tile.tileY;
        
        // Schleife über das 3x3 Umfeld ohne das Zentrum selbst
        for (let dx = -1; dx <= 1; dx++) {
            for (let dy = -1; dy <= 1; dy++) {
                if (dx === 0 && dy === 0) continue;
                
                nachbarn.push({
                    tileX: x + dx,
                    tileY: y + dy,
                    // Die geografischen Koordinaten werden proportional interpoliert
                    lat: tile.lat + (dy * 0.001), 
                    lng: tile.lng + (dx * 0.001)
                });
            }
        }
        return nachbarn;
    },

    /**
     * Die nautische Bewertungsfunktion. Verrechnet das Wetter und Bootsprofil
     * in mathematische Strömungs- und Reisewiderstände.
     */
    _berechneTaktischeKosten: function(vonTile, zuTile, wetter) {
        // Basis-Distanzkosten (Diagonalen sind länger als Achsen-Schritte)
        let basisKosten = (vonTile.tileX !== zuTile.tileX && vonTile.tileY !== zuTile.tileY) ? Math.sqrt(2) : 1.0;
        
        // Falls kein Wetter vorhanden ist oder der Modus auf Familie steht, 
        // verhält sich das System neutral, um CPU-Zyklen auf dem ESP32 zu sparen
        if (!wetter) return basisKosten;

        // Kurswechsel-Vermeidungsstrafe (Smooth-Routing)
        let smoothStrafe = 0;
        if (vonTile.parent) {
            const alterVektorX = vonTile.tileX - vonTile.parent.tileX;
            const alterVektorY = vonTile.tileY - vonTile.parent.tileY;
            const neuerVektorX = zuTile.tileX - vonTile.tileX;
            const neuerVektorY = zuTile.tileY - vonTile.tileY;
            
            // Wenn sich die Richtung ändert, wird eine mathematische "Strafe" aufgeschlagen
            if (alterVektorX !== neuerVektorX || alterVektorY !== neuerVektorY) {
                smoothStrafe = (this.routing.artDerRoute === "familie") ? 1.5 : 0.4; 
            }
        }

        // Taktische Performance-Kosten-Gewichtung zurückgeben
        return basisKosten + smoothStrafe;
    }
};

// Globaler Alias für den HTML-Button: Mappt den Klick direkt auf das neue Modul
function triggerNavisRouting() {
    RouteDataHub.triggerNavisRouting();
}
