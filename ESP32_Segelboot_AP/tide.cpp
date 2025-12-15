#include "tide.h"
#include <math.h>
#include <algorithm>
#include <sys/time.h>

// -------------------- Globals --------------------
time_t tide_next_high_time = 0;
time_t tide_next_low_time = 0;
double tide_next_high_height = 0.0;
double tide_next_low_height = 0.0;
double tide_height_now = 0.0;
int tide_quality = 0;

// -------------------- Structures --------------------
struct Constituent {
  String name;
  double amp;  // meters
  double pha;  // degrees
};

struct Station {
  double lat;
  double lon;
  std::map<String, Constituent> cons;
};

// -------------------- Harmonic frequencies (deg/hour) --------------------
struct FreqEntry {
  const char *name;
  double degph;
};
static const FreqEntry freqTable[] = {
  { "SA", 0.0410686 }, { "SSA", 0.0821373 }, { "MM", 0.5443747 }, { "MSF", 1.0158958 }, { "MF", 1.0980331 }, { "Q1", 13.3986609 }, { "O1", 13.9430356 }, { "P1", 14.9589314 }, { "K1", 15.0410686 }, { "J1", 15.5854433 }, { "OO1", 16.1391017 }, { "2Q1", 12.8542862 }, { "2N2", 27.8953548 }, { "N2", 28.4397295 }, { "NU2", 28.5125831 }, { "M2", 28.9841042 }, { "L2", 29.5284789 }, { "T2", 29.9589333 }, { "S2", 30.0 }, { "R2", 30.0410667 }, { "K2", 30.0821373 }, { "EP2", 30.5443747 }, { "MU2", 27.9682084 }, { "LAMBDA2", 29.4556253 }, { "M3", 43.4761563 }, { "MK3", 44.0251729 }, { "MN4", 57.4238337 }, { "M4", 57.9682084 }, { "MS4", 58.9841042 }, { "N4", 57.9682084 }, { "M6", 86.9523127 }, { "S6", 90.0 }, { "M8", 115.9364168 }, { NULL, 0.0 }
};

// -------------------- Math helpers --------------------
static inline double deg2rad(double d) {
  return d * M_PI / 180.0;
}
static inline double rad2deg(double r) {
  return r * 180.0 / M_PI;
}

static double haversine_m(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = deg2rad(lat2 - lat1);
  double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  return R * 2.0 * atan2(sqrt(a), sqrt(1 - a));
}

// -------------------- Tile helpers --------------------
static void tile_indices(double lat, double lon, int &ix, int &iy) {
  // identisch zur Python-Logik:
  // ix = floor((lon + 180) / TILE_SIZE)
  // iy = floor((lat + 90)  / TILE_SIZE)

  ix = (int)floor(lon / TIDE_TILE_SIZE_DEG);
  iy = (int)floor(lat / TIDE_TILE_SIZE_DEG);
}


static String tile_filename_for(int ix, int iy) {
  char buf[64];
  snprintf(buf, sizeof(buf), "%s/%d_%d.csv", TIDE_TILE_FOLDER, ix, iy);
  return String(buf);
}

// -------------------- CSV parsing --------------------
static bool parse_tile(const String &path, std::vector<Station> &out) {
  if (!SD.exists(path)) return false;
  File f = SD.open(path);
  if (!f) return false;
  f.readStringUntil('\n');  // header

  while (f.available()) {
    String l = f.readStringUntil('\n');
    l.trim();
    if (!l.length()) continue;
    int i1 = l.indexOf(','), i2 = l.indexOf(',', i1 + 1), i3 = l.indexOf(',', i2 + 1), i4 = l.indexOf(',', i3 + 1);
    if (i4 < 0) continue;
    double lat = l.substring(0, i1).toDouble();
    double lon = l.substring(i1 + 1, i2).toDouble();
    String con = l.substring(i2 + 1, i3);
    double amp = l.substring(i3 + 1, i4).toDouble();
    double pha = l.substring(i4 + 1).toDouble();

    Station *st = nullptr;
    for (auto &s : out)
      if (fabs(s.lat - lat) < 1e-6 && fabs(s.lon - lon) < 1e-6) {
        st = &s;
        break;
      }
    if (!st) {
      Station ns;
      ns.lat = lat;
      ns.lon = lon;
      out.push_back(ns);
      st = &out.back();
    }
    st->cons[con] = { con, amp, pha };
  }
  f.close();
  return true;
}

static void load_9_tiles(double lat, double lon, std::vector<Station> &out) {
  out.clear();
  int ix, iy;
  tile_indices(lat, lon, ix, iy);
  for (int dy = -1; dy <= 1; dy++)
    for (int dx = -1; dx <= 1; dx++) {
      std::vector<Station> tmp;
      if (parse_tile(tile_filename_for(ix + dx, iy + dy), tmp))
        out.insert(out.end(), tmp.begin(), tmp.end());
    }
}

// -------------------- Interpolation --------------------
struct Phasor {
  double r = 0, i = 0, w = 0;
};

static void interpolate(const std::vector<Station> &all, const std::vector<int> &idx, double lat, double lon, std::map<String, Constituent> &out) {
  out.clear();
  std::vector<String> names;
  for (int i : idx)
    for (auto &kv : all[i].cons) {
      if (std::find(names.begin(), names.end(), kv.first) == names.end()) names.push_back(kv.first);
    }
  for (auto &name : names) {
    Phasor p;
    for (int i : idx) {
      auto it = all[i].cons.find(name);
      if (it == all[i].cons.end()) continue;
      double d = haversine_m(lat, lon, all[i].lat, all[i].lon) + 1.0;
      double w = 1.0 / (d * d);
      double ph = deg2rad(it->second.pha);
      p.r += it->second.amp * cos(ph) * w;
      p.i += it->second.amp * sin(ph) * w;
      p.w += w;
    }
    if (p.w > 0) {
      double r = p.r / p.w, i = p.i / p.w;
      out[name] = { name, sqrt(r * r + i * i), rad2deg(atan2(i, r)) };
    }
  }
}

// -------------------- Tide synthesis --------------------
static double tide_at(const std::map<String, Constituent> &c, time_t t) {
  double h = 0;
  double tt = (double)t;
  for (auto &kv : c) {
    double degph = 0;
    for (int i = 0; freqTable[i].name; i++)
      if (kv.first.equalsIgnoreCase(freqTable[i].name)) {
        degph = freqTable[i].degph;
        break;
      }
    if (!degph) continue;
    double w = degph * (M_PI / 180.0) / 3600.0;
    h += kv.second.amp * cos(w * tt + deg2rad(kv.second.pha));
  }
  return h;
}

static void refine(time_t tc, double h0, double h1, double h2, int step, time_t &tr, double &hr) {
  double a = (h0 + h2 - 2 * h1) / 2.0;
  if (fabs(a) < 1e-9) {
    tr = tc;
    hr = h1;
    return;
  }
  double b = (h2 - h0) / 2.0;
  double x = -b / (2 * a);
  if (x < -1) x = -1;
  if (x > 1) x = 1;
  tr = tc + (time_t)(x * step);
  hr = h1 - (b * b) / (4 * a);
}

static void find_extrema(const std::map<String, Constituent> &c, time_t now, time_t &ht, double &hh, time_t &lt, double &lh) {
  ht = lt = 0;
  int step = TIDE_SEARCH_STEP_SECONDS;
  int win = TIDE_SEARCH_WINDOW_HOURS * 3600;
  double h0 = tide_at(c, now), h1 = tide_at(c, now + step);
  for (int dt = step; dt < win; dt += step) {
    double h2 = tide_at(c, now + dt + step);
    if (!ht && h0 < h1 && h1 > h2) refine(now + dt, h0, h1, h2, step, ht, hh);
    if (!lt && h0 > h1 && h1 < h2) refine(now + dt, h0, h1, h2, step, lt, lh);
    if (ht && lt) return;
    h0 = h1;
    h1 = h2;
  }
}

// -------------------- Public API --------------------
void tide_map_init() {
  
}

bool tide_query(double lat, double lon, time_t now) {
  std::vector<Station> all;
  load_9_tiles(lat, lon, all);
  if (all.empty()) return false;

  // nearest stations
  std::vector<std::pair<double, int>> d;
  for (int i = 0; i < (int)all.size(); i++) d.push_back({ haversine_m(lat, lon, all[i].lat, all[i].lon), i });
  std::sort(d.begin(), d.end());
  int n = min(TIDE_MAX_NEAREST_STATIONS, (int)d.size());
  std::vector<int> idx;
  for (int i = 0; i < n; i++) idx.push_back(d[i].second);

  std::map<String, Constituent> cons;
  interpolate(all, idx, lat, lon, cons);
  if (cons.empty()) return false;

  tide_height_now = tide_at(cons, now);
  find_extrema(cons, now, tide_next_high_time, tide_next_high_height, tide_next_low_time, tide_next_low_height);

  double maxD = 50 * 1852.0;
  double q = 100 * (1.0 - d[0].first / maxD);
  if (q < 0) q = 0;
  if (q > 100) q = 100;
  tide_quality = (int)q;
  return true;
}