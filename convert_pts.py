import math
import pyproj


class GeoConverter:
    def __init__(self, a=6378137.0, f=1/298.257223563):
        self.a = a
        self.f = f
        self.e2 = f * (2 - f)
        self.geod = pyproj.Geod(a=a, f=f)

    # ---------------- GEODETIC <-> ECEF ----------------
    def geodetic_to_ecef(self, lat, lon, h):
        lat_r = math.radians(lat)
        lon_r = math.radians(lon)

        N = self.a / math.sqrt(1 - self.e2 * math.sin(lat_r)**2)

        x = (N + h) * math.cos(lat_r) * math.cos(lon_r)
        y = (N + h) * math.cos(lat_r) * math.sin(lon_r)
        z = ((1 - self.e2) * N + h) * math.sin(lat_r)
        return x, y, z

    def ecef_to_geodetic(self, x, y, z):
        p = math.sqrt(x**2 + y**2)
        lat = math.atan2(z, p * (1 - self.e2))
        lat_prev = 0

        while abs(lat - lat_prev) > 1e-12:
            lat_prev = lat
            N = self.a / math.sqrt(1 - self.e2 * math.sin(lat)**2)
            h = p / math.cos(lat) - N
            lat = math.atan2(z, p * (1 - self.e2 * (N / (N + h))))

        lon = math.atan2(y, x)
        N = self.a / math.sqrt(1 - self.e2 * math.sin(lat)**2)
        h = p / math.cos(lat) - N

        return math.degrees(lat), math.degrees(lon), h

    # ---------------- ECEF <-> ENU ----------------
    def ecef_to_enu(self, x, y, z, ref_lat, ref_lon, ref_h):
        xr, yr, zr = self.geodetic_to_ecef(ref_lat, ref_lon, ref_h)

        dx = x - xr
        dy = y - yr
        dz = z - zr

        lat_r = math.radians(ref_lat)
        lon_r = math.radians(ref_lon)

        t = [
            [-math.sin(lon_r),              math.cos(lon_r),               0],
            [-math.sin(lat_r)*math.cos(lon_r), -math.sin(lat_r)*math.sin(lon_r), math.cos(lat_r)],
            [ math.cos(lat_r)*math.cos(lon_r),  math.cos(lat_r)*math.sin(lon_r), math.sin(lat_r)]
        ]

        e = t[0][0]*dx + t[0][1]*dy + t[0][2]*dz
        n = t[1][0]*dx + t[1][1]*dy + t[1][2]*dz
        u = t[2][0]*dx + t[2][1]*dy + t[2][2]*dz

        return e, n, u

    def enu_to_ecef(self, e, n, u, ref_lat, ref_lon, ref_h):
        xr, yr, zr = self.geodetic_to_ecef(ref_lat, ref_lon, ref_h)

        lat_r = math.radians(ref_lat)
        lon_r = math.radians(ref_lon)

        t = [
            [-math.sin(lon_r), -math.sin(lat_r)*math.cos(lon_r),  math.cos(lat_r)*math.cos(lon_r)],
            [ math.cos(lon_r), -math.sin(lat_r)*math.sin(lon_r),  math.cos(lat_r)*math.sin(lon_r)],
            [ 0,                math.cos(lat_r),                   math.sin(lat_r)]
        ]

        dx = t[0][0]*e + t[0][1]*n + t[0][2]*u
        dy = t[1][0]*e + t[1][1]*n + t[1][2]*u
        dz = t[2][0]*e + t[2][1]*n + t[2][2]*u

        return xr + dx, yr + dy, zr + dz

    # ---------------- DISTANCE & BEARING ----------------
    def distance_bearing(self, lat1, lon1, lat2, lon2):
        az12, az21, dist = self.geod.inv(lon1, lat1, lon2, lat2)
        return dist, az12  # meters, degrees

    def project_from_point(self, lat, lon, azimuth_deg, distance_m):
        lon2, lat2, _ = self.geod.fwd(lon, lat, azimuth_deg, distance_m)
        return lat2, lon2


# ---------------- USAGE ----------------
geo = GeoConverter()  # WGS84 default

lat, lon, h = 51.07900056732, -114.13253478947, 1113.9216
x, y, z = geo.geodetic_to_ecef(lat, lon, h)

e, n, u = geo.ecef_to_enu(x, y, z, lat, lon, h)

dist, bearing = geo.distance_bearing(lat, lon, 51.08, -114.13)

new_lat, new_lon = geo.project_from_point(lat, lon, 45, 1000)

print(x, y, z)
print(e, n, u)
print(dist, bearing)
print(new_lat, new_lon)
