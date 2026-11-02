#!/usr/bin/env python3
"""
GoPro GPS Map Player
====================
Side-by-side video + map playback for GoPro footage with embedded GPS (GPMF).

Features
  * Reads GPS straight from the GoPro MP4 (GPS5 on HERO5-HERO11, GPS9 on HERO11+).
    No ffmpeg / exiftool needed.
  * Seek bar: drag or click anywhere; the map position follows the video time.
  * Click on the GPS track in the map to jump the video to that spot.
  * Import one or more KMZ/KML design files as toggleable map overlays
    (points, lines, polygons, KML styles, ExtendedData attributes, ground-overlay images).
  * GPS time-offset nudge, playback speed, follow-vehicle toggle, dark/street/satellite basemaps.
  * Drag & drop videos or KMZ/KML files onto the window.

Install:   pip install PySide6
Run:       python gopro_map_player.py [video.MP4] [design.kmz ...]
Debug:     python gopro_map_player.py --dump-gps video.MP4

Shortcuts: Space = play/pause, Left/Right = -/+5 s, Shift+Left/Right = -/+1 s,
           Ctrl+O = open video, Ctrl+I = import KMZ/KML
"""

import base64
import bisect
import json
import math
import mimetypes
import os
import posixpath
import re
import struct
import sys
import zipfile
import xml.etree.ElementTree as ET

# ============================================================================
# 1. GoPro GPS extraction (MP4 sample tables + GPMF parser)
# ============================================================================

# GPMF type char -> struct code (big endian)
_GPMF_TYPES = {
    'b': 'b', 'B': 'B', 's': 'h', 'S': 'H', 'l': 'i', 'L': 'I',
    'f': 'f', 'd': 'd', 'j': 'q', 'J': 'Q', 'c': 'c', 'F': '4s',
}


def _klv(buf):
    """Iterate GPMF key-length-value entries."""
    i, n = 0, len(buf)
    while i + 8 <= n:
        key = buf[i:i + 4]
        if key == b'\0\0\0\0':
            break
        typ, size = buf[i + 4], buf[i + 5]
        rep = struct.unpack_from('>H', buf, i + 6)[0]
        length = size * rep
        yield key, typ, size, rep, buf[i + 8:i + 8 + length]
        i += 8 + ((length + 3) & ~3)          # payload is 32-bit aligned


def _values(typ, size, rep, data):
    code = _GPMF_TYPES.get(chr(typ))
    if not code or code in ('c', '4s'):
        return []
    es = struct.calcsize('>' + code)
    count = (size * rep) // es
    return list(struct.unpack('>' + code * count, data[:es * count]))


def _parse_strm(buf, t0, dur, out):
    scal, fix, typestr, gps = [1.0], None, None, None
    for key, typ, size, rep, data in _klv(buf):
        if key == b'SCAL':
            scal = _values(typ, size, rep, data) or [1.0]
        elif key == b'GPSF':
            v = _values(typ, size, rep, data)
            fix = v[0] if v else None
        elif key == b'TYPE':
            typestr = data.split(b'\0')[0].decode('ascii', 'ignore')
        elif key in (b'GPS5', b'GPS9'):
            gps = (key, typ, size, rep, data)
    if not gps:
        return
    key, typ, size, rep, data = gps
    if rep == 0:
        return

    if chr(typ) == '?':                       # complex type (GPS9) described by TYPE
        if not typestr:
            return
        try:
            fmt = '>' + ''.join(_GPMF_TYPES[c] for c in typestr)
        except KeyError:
            return
        if struct.calcsize(fmt) != size:
            return
        rows = [struct.unpack_from(fmt, data, k * size) for k in range(rep)]
    else:
        vals = _values(typ, size, rep, data)
        per = len(vals) // rep
        rows = [vals[k * per:(k + 1) * per] for k in range(rep)]

    def sc(j):
        s = scal[j] if j < len(scal) else scal[-1]
        return s or 1

    for k, row in enumerate(rows):
        if len(row) < 4:
            continue
        lat, lon, alt, spd = (row[j] / sc(j) for j in range(4))
        f = row[8] if key == b'GPS9' and len(row) > 8 else fix
        if f is not None and f < 2:           # 0 = no lock, 2 = 2D, 3 = 3D
            continue
        if (abs(lat) < 1e-6 and abs(lon) < 1e-6) or abs(lat) > 90 or abs(lon) > 180:
            continue
        out[key].append((t0 + dur * k / rep, lat, lon, alt, spd))


def _walk_gpmf(buf, t0, dur, out):
    for key, typ, size, rep, data in _klv(buf):
        if typ == 0:                          # nested container
            if key == b'STRM':
                _parse_strm(data, t0, dur, out)
            else:
                _walk_gpmf(data, t0, dur, out)


def _boxes(f, start, end):
    pos = start
    while pos + 8 <= end:
        f.seek(pos)
        hdr = f.read(8)
        if len(hdr) < 8:
            return
        size, typ = struct.unpack('>I4s', hdr)
        hl = 8
        if size == 1:
            size = struct.unpack('>Q', f.read(8))[0]
            hl = 16
        elif size == 0:
            size = end - pos
        if size < hl:
            return
        yield typ, pos + hl, pos + size
        pos += size


def _child(f, start, end, typ):
    for t, s, e in _boxes(f, start, end):
        if t == typ:
            return s, e
    return None


def _read(f, s, e):
    f.seek(s)
    return f.read(e - s)


def read_gopro_gps(path):
    """Return list of (t_seconds, lat, lon, alt_m, speed_mps) from a GoPro MP4, or []."""
    with open(path, 'rb') as f:
        f.seek(0, 2)
        moov = _child(f, 0, f.tell(), b'moov')
        if not moov:
            return []
        for typ, s, e in _boxes(f, *moov):
            if typ != b'trak':
                continue
            mdia = _child(f, s, e, b'mdia')
            if not mdia:
                continue
            mdhd = _child(f, *mdia, b'mdhd')
            minf = _child(f, *mdia, b'minf')
            stbl = _child(f, *minf, b'stbl') if minf else None
            if not (mdhd and stbl):
                continue
            tables = {t: _read(f, a, b) for t, a, b in _boxes(f, *stbl)}
            if tables.get(b'stsd', b'')[12:16] != b'gpmd':
                continue

            md = _read(f, *mdhd)
            timescale = struct.unpack_from('>I', md, 20 if md[0] == 1 else 12)[0] or 1

            # sample sizes
            stsz = tables[b'stsz']
            fixed, count = struct.unpack_from('>II', stsz, 4)
            sizes = [fixed] * count if fixed else list(struct.unpack_from('>%dI' % count, stsz, 12))

            # chunk offsets
            if b'co64' in tables:
                c = tables[b'co64']
                n = struct.unpack_from('>I', c, 4)[0]
                chunks = struct.unpack_from('>%dQ' % n, c, 8)
            else:
                c = tables[b'stco']
                n = struct.unpack_from('>I', c, 4)[0]
                chunks = struct.unpack_from('>%dI' % n, c, 8)

            # samples-per-chunk -> sample offsets
            stsc = tables[b'stsc']
            n = struct.unpack_from('>I', stsc, 4)[0]
            entries = [struct.unpack_from('>III', stsc, 8 + 12 * k) for k in range(n)]
            offsets, si = [], 0
            for ci, coff in enumerate(chunks, 1):
                spc = 0
                for first, per, _ in entries:
                    if first <= ci:
                        spc = per
                    else:
                        break
                off = coff
                for _ in range(spc):
                    if si >= len(sizes):
                        break
                    offsets.append(off)
                    off += sizes[si]
                    si += 1

            # sample times
            stts = tables[b'stts']
            n = struct.unpack_from('>I', stts, 4)[0]
            times, durs, t = [], [], 0
            for k in range(n):
                cnt, delta = struct.unpack_from('>II', stts, 8 + 8 * k)
                for _ in range(cnt):
                    times.append(t / timescale)
                    durs.append(delta / timescale)
                    t += delta

            out = {b'GPS5': [], b'GPS9': []}
            for off, sz, t0, d in zip(offsets, sizes, times, durs):
                f.seek(off)
                _walk_gpmf(f.read(sz), t0, d, out)
            return out[b'GPS9'] or out[b'GPS5']
    return []


def _dist_m(lat1, lon1, lat2, lon2):
    x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    y = math.radians(lat2 - lat1)
    return 6371000.0 * math.hypot(x, y)


def _bearing(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


class GpsTrack:
    def __init__(self, pts):
        pts = sorted(pts, key=lambda p: p[0])
        clean = []
        for p in pts:                         # drop duplicate timestamps
            if not clean or p[0] > clean[-1][0]:
                clean.append(p)
        self.t = [p[0] for p in clean]
        self.lat = [p[1] for p in clean]
        self.lon = [p[2] for p in clean]
        self.alt = [p[3] for p in clean]
        self.speed = [p[4] for p in clean]
        n = len(clean)
        # heading: bearing to the next point at least 3 m away (stable when stopped)
        self.heading = [0.0] * n
        j, last = 0, 0.0
        for i in range(n):
            j = max(j, i + 1)
            while j < n and _dist_m(self.lat[i], self.lon[i], self.lat[j], self.lon[j]) < 3:
                j += 1
            if j < n:
                last = _bearing(self.lat[i], self.lon[i], self.lat[j], self.lon[j])
            self.heading[i] = last
        span = self.t[-1] - self.t[0] if n > 1 else 0
        self.rate_hz = (n - 1) / span if span > 0 else 0

    def __len__(self):
        return len(self.t)

    def interp(self, t):
        """Return (lat, lon, speed_mps, heading_deg, index) at time t (seconds)."""
        ts = self.t
        if t <= ts[0]:
            return self.lat[0], self.lon[0], self.speed[0], self.heading[0], 0
        if t >= ts[-1]:
            i = len(ts) - 1
            return self.lat[i], self.lon[i], self.speed[i], self.heading[i], i
        i = bisect.bisect_right(ts, t) - 1
        f = (t - ts[i]) / (ts[i + 1] - ts[i])
        lerp = lambda a: a[i] + f * (a[i + 1] - a[i])
        return lerp(self.lat), lerp(self.lon), lerp(self.speed), self.heading[i], i

    def nearest(self, lat, lon):
        k = math.cos(math.radians(lat))
        best, bi = float('inf'), 0
        for i, (a, b) in enumerate(zip(self.lat, self.lon)):
            d = (a - lat) ** 2 + ((b - lon) * k) ** 2
            if d < best:
                best, bi = d, i
        return bi


# ============================================================================
# 2. KMZ / KML -> GeoJSON
# ============================================================================

FILL_OPACITY_CAP = 0.5   # keep design polygons see-through so the drive track stays visible
_GEOM_TAGS = {'Point', 'LineString', 'LinearRing', 'Polygon', 'MultiGeometry',
              'Track', 'MultiTrack', 'Model'}


def _parse_xml(data):
    try:
        root = ET.fromstring(data)
    except ET.ParseError:
        # common in exported files: undeclared prefixes such as gx: -> strip all prefixes
        data = re.sub(rb'\sxmlns(:[\w.-]+)?\s*=\s*"[^"]*"', b'', data)
        data = re.sub(rb'(</?)[A-Za-z_][\w.-]*:', rb'\1', data)
        root = ET.fromstring(data)
    for el in root.iter():
        if isinstance(el.tag, str):
            el.tag = el.tag.rsplit('}', 1)[-1]
    return root


def _coords(text):
    out = []
    for tok in (text or '').split():
        p = tok.split(',')
        if len(p) >= 2:
            try:
                out.append([float(p[0]), float(p[1])])
            except ValueError:
                pass
    return out


def _kml_color(s):
    s = (s or '').strip().lstrip('#')
    if len(s) != 8:
        return None, None
    a, b, g, r = s[0:2], s[2:4], s[4:6], s[6:8]
    try:
        return '#' + r + g + b, int(a, 16) / 255.0
    except ValueError:
        return None, None


def _parse_style(st):
    d = {}
    ls = st.find('LineStyle')
    if ls is not None:
        c, a = _kml_color(ls.findtext('color'))
        if c:
            d['stroke'], d['strokeOpacity'] = c, a
        try:
            d['strokeWidth'] = max(float(ls.findtext('width')), 0.5)
        except (TypeError, ValueError):
            pass
    ps = st.find('PolyStyle')
    if ps is not None:
        c, a = _kml_color(ps.findtext('color'))
        if c:
            d['fill'], d['fillOpacity'] = c, min(a, FILL_OPACITY_CAP)
        if (ps.findtext('fill') or '').strip() == '0':
            d['doFill'] = False
        if (ps.findtext('outline') or '').strip() == '0':
            d['outline'] = False
    ic = st.find('IconStyle')
    if ic is not None:
        c, _ = _kml_color(ic.findtext('color'))
        if c:
            d['iconColor'] = c
    return d


def _geom(el):
    tag = el.tag
    if tag == 'Point':
        c = _coords(el.findtext('coordinates'))
        return {'type': 'Point', 'coordinates': c[0]} if c else None
    if tag in ('LineString', 'LinearRing'):
        c = _coords(el.findtext('coordinates'))
        return {'type': 'LineString', 'coordinates': c} if len(c) >= 2 else None
    if tag == 'Polygon':
        outer = _coords(el.findtext('outerBoundaryIs/LinearRing/coordinates'))
        if len(outer) < 3:
            return None
        rings = [outer]
        for ib in el.findall('innerBoundaryIs'):
            for lr in ib.findall('LinearRing'):
                r = _coords(lr.findtext('coordinates'))
                if len(r) >= 3:
                    rings.append(r)
        return {'type': 'Polygon', 'coordinates': rings}
    if tag == 'Track':
        c = []
        for co in el.findall('coord'):
            p = (co.text or '').split()
            if len(p) >= 2:
                c.append([float(p[0]), float(p[1])])
        return {'type': 'LineString', 'coordinates': c} if len(c) >= 2 else None
    if tag == 'Model':
        loc = el.find('Location')
        if loc is not None:
            try:
                return {'type': 'Point', 'coordinates': [float(loc.findtext('longitude')),
                                                          float(loc.findtext('latitude'))]}
            except (TypeError, ValueError):
                return None
    if tag in ('MultiGeometry', 'MultiTrack'):
        geoms = [g for g in (_geom(ch) for ch in el if ch.tag in _GEOM_TAGS) if g]
        return {'type': 'GeometryCollection', 'geometries': geoms} if geoms else None
    return None


def load_kml(path):
    """Return (geojson FeatureCollection, list of ground-overlay image dicts)."""
    if zipfile.is_zipfile(path):
        z = zipfile.ZipFile(path)
        names = z.namelist()
        kmls = [n for n in names if n.lower().endswith('.kml')]
        if not kmls:
            raise ValueError('No .kml file found inside the KMZ')
        main = 'doc.kml' if 'doc.kml' in names else sorted(kmls, key=lambda n: n.count('/'))[0]
        data, base = z.read(main), posixpath.dirname(main)

        def get_file(href):
            for cand in (posixpath.normpath(posixpath.join(base, href)), href, href.lstrip('./')):
                try:
                    return z.read(cand)
                except KeyError:
                    pass
            return None
    else:
        with open(path, 'rb') as fh:
            data = fh.read()
        base = os.path.dirname(os.path.abspath(path))

        def get_file(href):
            p = os.path.join(base, href)
            if os.path.isfile(p):
                with open(p, 'rb') as fh:
                    return fh.read()
            return None

    root = _parse_xml(data)

    styles, maps = {}, {}
    for st in root.iter('Style'):
        if st.get('id'):
            styles['#' + st.get('id')] = _parse_style(st)
    for sm in root.iter('StyleMap'):
        if not sm.get('id'):
            continue
        for pair in sm.findall('Pair'):
            if (pair.findtext('key') or '').strip() == 'normal':
                inline = pair.find('Style')
                maps['#' + sm.get('id')] = _parse_style(inline) if inline is not None \
                    else (pair.findtext('styleUrl') or '')

    def resolve(url, depth=0):
        key = '#' + url.strip().split('#')[-1]
        if key in maps and depth < 5:
            v = maps[key]
            return v if isinstance(v, dict) else resolve(v, depth + 1)
        return styles.get(key, {})

    features = []
    for pm in root.iter('Placemark'):
        geom = None
        for ch in pm:
            if ch.tag in _GEOM_TAGS:
                geom = _geom(ch)
                break
        if not geom:
            continue
        props = {}
        if pm.findtext('styleUrl'):
            props.update(resolve(pm.findtext('styleUrl')))
        inline = pm.find('Style')
        if inline is not None:
            props.update(_parse_style(inline))
        props['name'] = (pm.findtext('name') or '').strip()
        props['description'] = (pm.findtext('description') or '').strip()
        ed = pm.find('ExtendedData')
        if ed is not None:
            extra = {}
            for d in ed.iter('Data'):
                extra[d.get('name', '')] = d.findtext('value') or ''
            for sd in ed.iter('SimpleData'):
                extra[sd.get('name', '')] = sd.text or ''
            if extra:
                props['data'] = extra
        features.append({'type': 'Feature', 'geometry': geom, 'properties': props})

    images = []
    for go in root.iter('GroundOverlay'):
        href = (go.findtext('Icon/href') or '').strip()
        box = go.find('LatLonBox')
        if not href or box is None:
            continue
        try:
            n, s, e, w = (float(box.findtext(k)) for k in ('north', 'south', 'east', 'west'))
        except (TypeError, ValueError):
            continue
        if re.match(r'^[a-z]+://', href, re.I):
            url = href
        else:
            raw = get_file(href)
            if raw is None:
                continue
            mime = mimetypes.guess_type(href)[0] or 'image/png'
            url = 'data:%s;base64,%s' % (mime, base64.b64encode(raw).decode('ascii'))
        _, a = _kml_color(go.findtext('color'))
        images.append({'url': url, 'north': n, 'south': s, 'east': e, 'west': w,
                       'opacity': a if a is not None else 0.8})

    return {'type': 'FeatureCollection', 'features': features}, images


# ============================================================================
# 3. Map page (Leaflet in QWebEngineView)
# ============================================================================

LEAFLET_CSS_TAG = '<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">'
LEAFLET_JS_TAG = '<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>'


def resource_dir():
    """Folder holding bundled files (PyInstaller extracts to sys._MEIPASS)."""
    return getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))


MAP_HTML = r"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>/*QWEBCHANNEL_JS*/</script>
<style>
  html, body, #map { height: 100%; margin: 0; background: #111; }
  .car { width: 24px; height: 24px; background: #ff3d00;
         clip-path: polygon(50% 0%, 92% 100%, 50% 74%, 8% 100%); }
  .leaflet-popup-content table { border-collapse: collapse; font-size: 12px; margin-top: 4px; }
  .leaflet-popup-content td { border: 1px solid #ccc; padding: 2px 5px; }
</style></head>
<body><div id="map"></div>
<script>
if (typeof L === 'undefined')
  document.getElementById('map').innerHTML = '<p style="color:#ccc;font:14px sans-serif;padding:20px">' +
    'Could not load the map library (Leaflet). An internet connection is needed for the map and tiles.</p>';
</script>
<script>
const map = L.map('map', { preferCanvas: true }).setView([20, 0], 2);
const bases = {
  'Dark': L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png',
          { maxZoom: 20, subdomains: 'abcd', attribution: '&copy; OpenStreetMap &copy; CARTO' }),
  'Streets': L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png',
          { maxZoom: 19, attribution: '&copy; OpenStreetMap contributors' }),
  'Satellite': L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
          { maxZoom: 19, attribution: 'Tiles &copy; Esri' })
};
bases['Dark'].addTo(map);
const layerCtl = L.control.layers(bases, {}, { collapsed: false }).addTo(map);
L.control.scale().addTo(map);

let bridge = null;
if (typeof QWebChannel !== 'undefined' && window.qt && qt.webChannelTransport) {
  new QWebChannel(qt.webChannelTransport, ch => { bridge = ch.objects.bridge; });
}

let track = [], trackGroup = null, doneLine = null, car = null, follow = true;
const esc = s => String(s ?? '').replace(/[&<>"']/g,
  c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));

function clearTrack() {
  if (trackGroup) { layerCtl.removeLayer(trackGroup); map.removeLayer(trackGroup); }
  trackGroup = null; doneLine = null; car = null; track = [];
}

function setTrack(pts) {
  clearTrack();
  track = pts;
  if (!pts.length) return;
  const full = L.polyline(pts, { color: '#4aa3ff', weight: 3, opacity: 0.45, interactive: false });
  doneLine = L.polyline([], { color: '#00e5ff', weight: 5, opacity: 0.95, interactive: false });
  const hit = L.polyline(pts, { color: '#000', weight: 18, opacity: 0 });   // wide click target
  hit.on('click', e => { if (bridge) bridge.mapClicked(e.latlng.lat, e.latlng.lng); });
  hit.bindTooltip('Click to jump video here', { sticky: true });
  const start = L.circleMarker(pts[0], { radius: 6, color: '#000', weight: 1, fillColor: '#2ecc71', fillOpacity: 1, interactive: false });
  const end = L.circleMarker(pts[pts.length - 1], { radius: 6, color: '#000', weight: 1, fillColor: '#e74c3c', fillOpacity: 1, interactive: false });
  car = L.marker(pts[0], { icon: L.divIcon({ className: '', html: '<div class="car"></div>',
        iconSize: [24, 24], iconAnchor: [12, 12] }), zIndexOffset: 1000, interactive: false });
  trackGroup = L.layerGroup([full, doneLine, hit, start, end, car]).addTo(map);
  layerCtl.addOverlay(trackGroup, 'GoPro track');
  map.fitBounds(full.getBounds(), { padding: [30, 30] });
}

function fitTrack() { if (track.length) map.fitBounds(L.latLngBounds(track), { padding: [30, 30] }); }
function setFollow(v) { follow = v; }

function setPos(lat, lon, idx, heading) {
  if (!car) return;
  car.setLatLng([lat, lon]);
  const el = car.getElement();
  if (el && el.firstChild) el.firstChild.style.transform = 'rotate(' + heading + 'deg)';
  doneLine.setLatLngs(track.slice(0, idx + 1).concat([[lat, lon]]));
  if (follow && !map.getBounds().pad(-0.25).contains([lat, lon]))
    map.panTo([lat, lon], { animate: true, duration: 0.3 });
}

function popupHtml(p) {
  let h = '';
  if (p.name) h += '<b>' + esc(p.name) + '</b>';
  if (p.description) h += '<div>' + p.description + '</div>';
  const d = p.data || {}, keys = Object.keys(d);
  if (keys.length) h += '<table>' + keys.map(k =>
      '<tr><td>' + esc(k) + '</td><td>' + esc(d[k]) + '</td></tr>').join('') + '</table>';
  return h || null;
}

function addOverlay(name, gj, images, color) {
  const group = L.featureGroup();
  L.geoJSON(gj, {
    style: f => {
      const p = f.properties || {};
      return {
        color: p.stroke || color,
        weight: p.outline === false ? 0 : (p.strokeWidth ?? 2),
        opacity: p.strokeOpacity ?? 1,
        fillColor: p.fill || color,
        fillOpacity: p.doFill === false ? 0 : (p.fillOpacity ?? 0.25)
      };
    },
    pointToLayer: (f, ll) => L.circleMarker(ll, { radius: 5, color: '#000', weight: 1,
        fillColor: (f.properties || {}).iconColor || color, fillOpacity: 0.9 }),
    onEachFeature: (f, layer) => {
      const h = popupHtml(f.properties || {});
      if (h) layer.bindPopup(h, { maxWidth: 420, maxHeight: 320 });
    }
  }).addTo(group);
  for (const im of images)
    L.imageOverlay(im.url, [[im.south, im.west], [im.north, im.east]], { opacity: im.opacity }).addTo(group);
  group.addTo(map);
  layerCtl.addOverlay(group, esc(name));
  const b = group.getBounds();
  if (b.isValid()) map.fitBounds(b, { padding: [30, 30] });
}
</script></body></html>
"""

# ============================================================================
# 4. GUI
# ============================================================================

if __name__ == '__main__' and len(sys.argv) >= 3 and sys.argv[1] == '--dump-gps':
    pts = read_gopro_gps(sys.argv[2])
    if not pts:
        print('No GPS found.')
    else:
        tr = GpsTrack(pts)
        print('%d points, %.1f Hz, %.1f-%.1f s' % (len(tr), tr.rate_hz, tr.t[0], tr.t[-1]))
        for i in range(0, len(tr), max(1, len(tr) // 10)):
            print('  t=%7.2f  %.6f, %.6f  %.1f km/h' % (tr.t[i], tr.lat[i], tr.lon[i], tr.speed[i] * 3.6))
    sys.exit(0)

from PySide6.QtCore import Qt, QUrl, QTimer, QObject, Signal, Slot, QFile, QIODevice
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QSplitter, QSlider, QPushButton, QLabel,
    QHBoxLayout, QVBoxLayout, QFileDialog, QMessageBox, QComboBox, QDoubleSpinBox,
    QCheckBox, QStyle, QStyleOptionSlider)
from PySide6.QtMultimedia import QMediaPlayer, QAudioOutput
from PySide6.QtMultimediaWidgets import QVideoWidget
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWebChannel import QWebChannel

VIDEO_EXT = ('.mp4', '.mov', '.lrv', '.m4v')
KML_EXT = ('.kmz', '.kml')
OVERLAY_COLORS = ['#ffcc00', '#ff4fd8', '#7cfc00', '#ff8c00', '#00ffd0', '#ffffff']


class Bridge(QObject):
    clicked = Signal(float, float)

    @Slot(float, float)
    def mapClicked(self, lat, lon):
        self.clicked.emit(lat, lon)


class MapView(QWebEngineView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAcceptDrops(False)            # let the main window handle drops
        self.bridge = Bridge(self)
        self.channel = QWebChannel(self.page())
        self.channel.registerObject('bridge', self.bridge)
        self.page().setWebChannel(self.channel)
        self._ready, self._queue = False, []
        self.loadFinished.connect(self._loaded)

        f = QFile(':/qtwebchannel/qwebchannel.js')
        js = bytes(f.readAll()).decode('utf-8') if f.open(QIODevice.ReadOnly) else ''
        html = MAP_HTML.replace('/*QWEBCHANNEL_JS*/', js)
        # Use the bundled copy of Leaflet if present (works without the CDN)
        vendor = os.path.join(resource_dir(), 'vendor')
        try:
            with open(os.path.join(vendor, 'leaflet.js'), encoding='utf-8') as fh:
                ljs = fh.read()
            with open(os.path.join(vendor, 'leaflet.css'), encoding='utf-8') as fh:
                lcss = fh.read()
            html = html.replace(LEAFLET_JS_TAG, '<script>' + ljs + '</script>')
            html = html.replace(LEAFLET_CSS_TAG, '<style>' + lcss + '</style>')
        except OSError:
            pass                              # fall back to the CDN links
        self.setHtml(html, QUrl('https://gopro-map.local/'))

    def _loaded(self, ok):
        self._ready = True
        for code in self._queue:
            self.page().runJavaScript(code)
        self._queue.clear()

    def js(self, code):
        if self._ready:
            self.page().runJavaScript(code)
        else:
            self._queue.append(code)

    def set_track(self, track):
        pts = [[round(a, 7), round(b, 7)] for a, b in zip(track.lat, track.lon)]
        self.js('setTrack(%s)' % json.dumps(pts))

    def clear_track(self):
        self.js('clearTrack()')

    def set_position(self, lat, lon, idx, heading):
        self.js('setPos(%.7f,%.7f,%d,%.1f)' % (lat, lon, idx, heading))

    def add_overlay(self, name, geojson, images, color):
        self.js('addOverlay(%s,%s,%s,%s)' % (json.dumps(name), json.dumps(geojson),
                                              json.dumps(images), json.dumps(color)))


class ClickSlider(QSlider):
    """Slider that jumps straight to where you click."""
    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            opt = QStyleOptionSlider()
            self.initStyleOption(opt)
            handle = self.style().subControlRect(QStyle.CC_Slider, opt, QStyle.SC_SliderHandle, self)
            if not handle.contains(e.position().toPoint()):
                v = QStyle.sliderValueFromPosition(self.minimum(), self.maximum(),
                                                   int(e.position().x()), self.width())
                self.setValue(v)
                self.sliderMoved.emit(v)
        super().mousePressEvent(e)


def fmt_ms(ms):
    s = max(0, int(ms // 1000))
    h, m, s = s // 3600, (s // 60) % 60, s % 60
    return '%d:%02d:%02d' % (h, m, s) if h else '%02d:%02d' % (m, s)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('GoPro GPS Map Player')
        self.resize(1500, 820)
        self.setAcceptDrops(True)
        self.track = None
        self.duration = 0
        self._last_pos = -1
        self._overlay_count = 0

        # --- player
        self.player = QMediaPlayer(self)
        self.audio = QAudioOutput(self)
        self.player.setAudioOutput(self.audio)
        self.video = QVideoWidget()
        self.video.setStyleSheet('background:black')
        self.player.setVideoOutput(self.video)
        self.player.durationChanged.connect(self._on_duration)
        self.player.playbackStateChanged.connect(self._on_state)
        self.player.errorOccurred.connect(
            lambda err, msg: QMessageBox.warning(self, 'Playback error', msg or str(err)))

        # --- map
        self.map = MapView()
        self.map.bridge.clicked.connect(self._on_map_click)

        split = QSplitter(Qt.Horizontal)
        split.addWidget(self.video)
        split.addWidget(self.map)
        split.setSizes([750, 750])

        # --- controls
        self.slider = ClickSlider(Qt.Horizontal)
        self.slider.setRange(0, 0)
        self.slider.sliderMoved.connect(self.seek)

        def btn(text, slot, tip=''):
            b = QPushButton(text)
            b.setFocusPolicy(Qt.NoFocus)
            b.setToolTip(tip)
            b.clicked.connect(slot)
            return b

        self.play_btn = btn('▶ Play', self.toggle_play, 'Space')
        back_btn = btn('⏪ 5s', lambda: self.step(-5000), 'Left arrow')
        fwd_btn = btn('5s ⏩', lambda: self.step(5000), 'Right arrow')
        self.time_lbl = QLabel('00:00 / 00:00')

        self.rate = QComboBox()
        self.rate.setFocusPolicy(Qt.NoFocus)
        for r in ('0.25', '0.5', '1', '2', '4', '8'):
            self.rate.addItem(r + '×', float(r))
        self.rate.setCurrentIndex(2)
        self.rate.currentIndexChanged.connect(lambda _: self.player.setPlaybackRate(self.rate.currentData()))

        self.mute = QCheckBox('Mute')
        self.mute.setFocusPolicy(Qt.NoFocus)
        self.mute.toggled.connect(self.audio.setMuted)

        self.offset = QDoubleSpinBox()
        self.offset.setRange(-60, 60)
        self.offset.setSingleStep(0.1)
        self.offset.setDecimals(1)
        self.offset.setSuffix(' s')
        self.offset.setToolTip('Shift GPS relative to video if they look out of sync')
        self.offset.valueChanged.connect(lambda _: self.update_ui(self.player.position(), force=True))

        self.follow = QCheckBox('Follow')
        self.follow.setChecked(True)
        self.follow.setFocusPolicy(Qt.NoFocus)
        self.follow.toggled.connect(lambda v: self.map.js('setFollow(%s)' % ('true' if v else 'false')))

        self.info_lbl = QLabel('Open a GoPro video (Ctrl+O) or drop files here')

        row = QHBoxLayout()
        for w in (back_btn, self.play_btn, fwd_btn, self.time_lbl, QLabel('  Speed'), self.rate,
                  self.mute, QLabel('  GPS offset'), self.offset, self.follow):
            row.addWidget(w)
        row.addStretch(1)
        row.addWidget(self.info_lbl)

        central = QWidget()
        lay = QVBoxLayout(central)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.addWidget(split, 1)
        lay.addWidget(self.slider)
        lay.addLayout(row)
        self.setCentralWidget(central)

        # --- toolbar / shortcuts
        tb = self.addToolBar('Main')
        tb.setMovable(False)
        for text, key, slot in (('Open video…', 'Ctrl+O', self.open_video),
                                ('Import KMZ/KML…', 'Ctrl+I', self.import_kml),
                                ('Fit track', None, lambda: self.map.js('fitTrack()'))):
            a = QAction(text, self)
            if key:
                a.setShortcut(QKeySequence(key))
            a.triggered.connect(lambda _=False, s=slot: s())
            tb.addAction(a)
        for key, slot in (('Space', self.toggle_play),
                          ('Left', lambda: self.step(-5000)), ('Right', lambda: self.step(5000)),
                          ('Shift+Left', lambda: self.step(-1000)), ('Shift+Right', lambda: self.step(1000))):
            a = QAction(self)
            a.setShortcut(QKeySequence(key))
            a.setShortcutContext(Qt.ApplicationShortcut)
            a.triggered.connect(slot)
            self.addAction(a)

        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self._tick)
        self.timer.start()

    # ---------------------------------------------------------------- files
    def open_video(self, path=None):
        if not path:
            path, _ = QFileDialog.getOpenFileName(
                self, 'Open GoPro video', '',
                'Video (*.mp4 *.MP4 *.mov *.MOV *.lrv *.LRV);;All files (*)')
        if not path:
            return
        self.setWindowTitle('GoPro GPS Map Player — ' + os.path.basename(path))
        try:
            pts = read_gopro_gps(path)
        except Exception as e:  # noqa: BLE001
            pts = []
            QMessageBox.warning(self, 'GPS read error', str(e))
        if pts:
            self.track = GpsTrack(pts)
            self.map.set_track(self.track)
            self.info_lbl.setText('GPS: %d points @ %.1f Hz' % (len(self.track), self.track.rate_hz))
        else:
            self.track = None
            self.map.clear_track()
            self.info_lbl.setText('No GPS data with a fix found in this file')
        self._last_pos = -1
        self.player.setSource(QUrl.fromLocalFile(path))
        self.player.play()

    def import_kml(self, paths=None):
        if not paths:
            paths, _ = QFileDialog.getOpenFileNames(
                self, 'Import KMZ / KML', '', 'Google Earth (*.kmz *.KMZ *.kml *.KML);;All files (*)')
        for p in paths or []:
            try:
                gj, images = load_kml(p)
            except Exception as e:  # noqa: BLE001
                QMessageBox.warning(self, 'KMZ import failed', '%s\n\n%s' % (os.path.basename(p), e))
                continue
            color = OVERLAY_COLORS[self._overlay_count % len(OVERLAY_COLORS)]
            self._overlay_count += 1
            self.map.add_overlay(os.path.basename(p), gj, images, color)
            self.statusBar().showMessage('%s: %d features, %d image overlays' % (
                os.path.basename(p), len(gj['features']), len(images)), 8000)

    def dragEnterEvent(self, e):
        if e.mimeData().hasUrls():
            e.acceptProposedAction()

    def dropEvent(self, e):
        files = [u.toLocalFile() for u in e.mimeData().urls() if u.isLocalFile()]
        kmls = [f for f in files if f.lower().endswith(KML_EXT)]
        vids = [f for f in files if f.lower().endswith(VIDEO_EXT)]
        if vids:
            self.open_video(vids[0])
        if kmls:
            self.import_kml(kmls)

    # ------------------------------------------------------------- playback
    def toggle_play(self):
        if self.player.playbackState() == QMediaPlayer.PlayingState:
            self.player.pause()
        else:
            self.player.play()

    def seek(self, ms):
        ms = max(0, min(int(ms), self.duration or int(ms)))
        self.player.setPosition(ms)
        self.update_ui(ms)

    def step(self, delta_ms):
        self.seek(self.player.position() + delta_ms)

    def _on_duration(self, d):
        self.duration = d
        self.slider.setRange(0, d)

    def _on_state(self, st):
        self.play_btn.setText('⏸ Pause' if st == QMediaPlayer.PlayingState else '▶ Play')

    def _tick(self):
        if self.slider.isSliderDown():
            return
        pos = self.player.position()
        if pos != self._last_pos:
            self.update_ui(pos)

    def update_ui(self, ms, force=False):
        self._last_pos = ms
        if not self.slider.isSliderDown():
            self.slider.blockSignals(True)
            self.slider.setValue(int(ms))
            self.slider.blockSignals(False)
        self.time_lbl.setText('%s / %s' % (fmt_ms(ms), fmt_ms(self.duration)))
        if self.track:
            t = ms / 1000.0 + self.offset.value()
            lat, lon, spd, hdg, idx = self.track.interp(t)
            self.map.set_position(lat, lon, idx, hdg)
            self.info_lbl.setText('%.0f km/h  |  %.0f mph  |  %.6f, %.6f' % (
                spd * 3.6, spd * 2.23694, lat, lon))

    def _on_map_click(self, lat, lon):
        if not self.track:
            return
        i = self.track.nearest(lat, lon)
        self.seek((self.track.t[i] - self.offset.value()) * 1000)


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    args = sys.argv[1:]
    vids = [a for a in args if a.lower().endswith(VIDEO_EXT)]
    kmls = [a for a in args if a.lower().endswith(KML_EXT)]
    if vids:
        w.open_video(vids[0])
    if kmls:
        w.import_kml(kmls)
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
