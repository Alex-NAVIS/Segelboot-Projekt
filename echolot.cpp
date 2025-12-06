#include "echolot.h"
#include "Config.h"
#include "Sensor_Data.h"
#include <math.h>

// ----------------------
// Parameter
// ----------------------
#define NUM_SAMPLES 5           // Anzahl Messungen für kurzfristigen Mittelwert / Ausreißerfilter
#define MAX_DIST_CM 500         // max. Distanz in cm (~5m)
#define MAX_OUTLIER_DELTA 1.0   // max. Differenz zum kurzfristigen Mittelwert (Meter)
#define RING_SIZE 20            // 20 Sekunden Ringspeicher für Minimum

// ----------------------
// Filter- und Ringspeicher
// ----------------------
static double dist_samples[NUM_SAMPLES] = {0};
static uint8_t sample_index = 0;

static double ring[RING_SIZE] = {0};
static uint8_t ring_index = 0;

// ----------------------
// Schallgeschwindigkeit Wasser berechnen
// ----------------------
void berechne_schallgeschwindigkeit() {
    Schallgeschwindigkeit_Wasser = 1449.2 
                                  + 4.6 * Wassertemperatur 
                                  - 0.055 * Wassertemperatur * Wassertemperatur 
                                  + 0.00029 * Wassertemperatur * Wassertemperatur * Wassertemperatur 
                                  + (1.34 - 0.01 * Wassertemperatur) * (Salzgehalt - 35);
}

// ----------------------
// Ringspeicher Update
// ----------------------
static void update_ring(double value) {
    ring[ring_index++] = value;
    if(ring_index >= RING_SIZE) ring_index = 0;
}

// ----------------------
// Minimum aus Ringspeicher
// ----------------------
static double get_ring_min() {
    double min_val = 1e6;
    for(uint8_t i=0;i<RING_SIZE;i++) {
        if(ring[i] > 0 && ring[i] < min_val) min_val = ring[i];
    }
    return (min_val < 1e6) ? min_val : -1;
}

// ----------------------
// Setup Echolot
// ----------------------
void setup_echolot() {
    pinMode(PIN_ECHOLOTT_TRIGGER, OUTPUT);
    pinMode(PIN_ECHOLOTT_ECHO, INPUT);

    berechne_schallgeschwindigkeit();
}

// ----------------------
// Einzelmessung
// ----------------------
static double messung() {
    digitalWrite(PIN_ECHOLOTT_TRIGGER, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ECHOLOTT_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ECHOLOTT_TRIGGER, LOW);

    long duration = pulseIn(PIN_ECHOLOTT_ECHO, HIGH, 30000); // Timeout 30ms
    double dist = duration * (Schallgeschwindigkeit_Wasser / 1000000.0) / 2.0; // Meter

    if(dist <= 0 || dist > MAX_DIST_CM / 100.0) dist = -1;
    return dist;
}

// ----------------------
// Sensor auslesen
// ----------------------
void echolot_lesen() {
    double new_dist = messung();

    // Kurzfristiger Mittelwert zur Ausreißerfilterung
    double sum = 0;
    uint8_t count = 0;
    for(uint8_t i=0;i<NUM_SAMPLES;i++) {
        if(dist_samples[i] > 0) {
            sum += dist_samples[i];
            count++;
        }
    }
    double mean = (count>0) ? sum / count : new_dist;

    // Ausreißer nur akzeptieren, wenn Differenz zum Mittelwert <= MAX_OUTLIER_DELTA
    if(new_dist > 0 && fabs(new_dist - mean) <= MAX_OUTLIER_DELTA) {
        dist_samples[sample_index++] = new_dist;
        if(sample_index >= NUM_SAMPLES) sample_index = 0;
    }

    // Mittelwert über die kurzfristigen Werte für Stabilisierung
    sum = 0; count = 0;
    for(uint8_t i=0;i<NUM_SAMPLES;i++) {
        if(dist_samples[i] > 0) {
            sum += dist_samples[i];
            count++;
        }
    }
    double avg_dist = (count>0) ? sum / count : -1;

    if(avg_dist > 0) {
        // Roll-Korrektur (±35° möglich)
        double roll_rad = sensorData.roll * DEG_TO_RAD;
        double dist_vertical = avg_dist * cos(roll_rad);

        // Ringspeicher aktualisieren
        update_ring(dist_vertical);

        // Minimum der letzten 20 Sekunden
        double min_dist = get_ring_min();

        // Wassertiefe unter Kiel/Schwert
        if(min_dist > 0)
            sensorData.Echolot = min_dist + Tiefgang_Boot - Wassertiefe_Einbau;
    }
}
