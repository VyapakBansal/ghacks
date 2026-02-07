import math
import pyproj

# ---------- Geodetic -> ECEF ----------
def geodetic_to_ecef(lat, lon, h, a, f):
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    e2 = f * (2 - f)
    N = a / math.sqrt(1 - e2 * (math.sin(lat_rad) ** 2))

    x = (N + h) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (N + h) * math.cos(lat_rad) * math.sin(lon_rad)
    z = ((1 - e2) * N + h) * math.sin(lat_rad)

    return x, y, z


# ---------- WGS84 -> NAD83 (lat/lon) ----------
def wgs84_to_nad83(lat, lon, h):
    wgs84 = pyproj.CRS.from_epsg(4326)
    nad83 = pyproj.CRS.from_epsg(4269)

    transformer = pyproj.Transformer.from_crs(wgs84, nad83, always_xy=True)
    lon2, lat2, h2 = transformer.transform(lon, lat, h)  # lon, lat order

    return lat2, lon2, h2


# ---------- Ellipsoid constants ----------
WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563

NAD83_A = 6378137.0
NAD83_F = 1 / 298.257222101  # GRS80


# ---------- Input ----------
lat = 51.07900056732
lon = -114.13253478947
h = 1113.9216

# Convert datum
nad_lat, nad_lon, nad_h = wgs84_to_nad83(lat, lon, h)

# ECEF using correct ellipsoids
ecef_nad83 = geodetic_to_ecef(nad_lat, nad_lon, nad_h, NAD83_A, NAD83_F)
ecef_wgs84 = geodetic_to_ecef(lat, lon, h, WGS84_A, WGS84_F)

print("NAD83 geodetic:", nad_lat, nad_lon, nad_h)
print("ECEF (NAD83):", ecef_nad83)
print("ECEF (WGS84):", ecef_wgs84)
