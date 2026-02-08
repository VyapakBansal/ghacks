import math
import pyproj


class GeoConverter:
    def __init__(self, a=6378137.0, f=1/298.257223563):
        self.a = a
        self.f = f
        self.e2 = f * (2 - f)
        self.geod = pyproj.Geod(a=a, f=f)

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

    def ecef_to_enu(self, x, y, z, ref_lat, ref_lon, ref_h):
        xr, yr, zr = self.geodetic_to_ecef(ref_lat, ref_lon, ref_h)
        dx = x - xr
        dy = y - yr
        dz = z - zr
        lat_r = math.radians(ref_lat)
        lon_r = math.radians(ref_lon)
        t = [
            [-math.sin(lon_r), math.cos(lon_r), 0],
            [-math.sin(lat_r)*math.cos(lon_r), -math.sin(lat_r)*math.sin(lon_r), math.cos(lat_r)],
            [math.cos(lat_r)*math.cos(lon_r), math.cos(lat_r)*math.sin(lon_r), math.sin(lat_r)]
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
            [-math.sin(lon_r), -math.sin(lat_r)*math.cos(lon_r), math.cos(lat_r)*math.cos(lon_r)],
            [math.cos(lon_r), -math.sin(lat_r)*math.sin(lon_r), math.cos(lat_r)*math.sin(lon_r)],
            [0, math.cos(lat_r), math.sin(lat_r)]
        ]
        dx = t[0][0]*e + t[0][1]*n + t[0][2]*u
        dy = t[1][0]*e + t[1][1]*n + t[1][2]*u
        dz = t[2][0]*e + t[2][1]*n + t[2][2]*u
        return xr + dx, yr + dy, zr + dz

    def distance_bearing(self, lat1, lon1, lat2, lon2):
        az12, az21, dist = self.geod.inv(lon1, lat1, lon2, lat2)
        return dist, az12

    def project_from_point(self, lat, lon, azimuth_deg, distance_m):
        lon2, lat2, _ = self.geod.fwd(lon, lat, azimuth_deg, distance_m)
        return lat2, lon2


def main():
    geo = GeoConverter()
    
    print("Geo Coordinate Converter")
    print("=" * 50)
    
    while True:
        print("\n1. Geodetic to ECEF")
        print("2. ECEF to Geodetic")
        print("3. ECEF to ENU")
        print("4. ENU to ECEF")
        print("5. Distance and Bearing")
        print("6. Project from Point")
        print("7. Exit")
        
        choice = input("\nSelect option: ")
        
        if choice == "1":
            lat = float(input("Latitude (degrees): "))
            lon = float(input("Longitude (degrees): "))
            h = float(input("Height (meters): "))
            x, y, z = geo.geodetic_to_ecef(lat, lon, h)
            print(f"ECEF: X={x}, Y={y}, Z={z}")
            
        elif choice == "2":
            x = float(input("X (meters): "))
            y = float(input("Y (meters): "))
            z = float(input("Z (meters): "))
            lat, lon, h = geo.ecef_to_geodetic(x, y, z)
            print(f"Geodetic: Lat={lat}, Lon={lon}, Height={h}")
            
        elif choice == "3":
            x = float(input("X (meters): "))
            y = float(input("Y (meters): "))
            z = float(input("Z (meters): "))
            ref_lat = float(input("Reference Latitude (degrees): "))
            ref_lon = float(input("Reference Longitude (degrees): "))
            ref_h = float(input("Reference Height (meters): "))
            e, n, u = geo.ecef_to_enu(x, y, z, ref_lat, ref_lon, ref_h)
            print(f"ENU: E={e}, N={n}, U={u}")
            
        elif choice == "4":
            e = float(input("East (meters): "))
            n = float(input("North (meters): "))
            u = float(input("Up (meters): "))
            ref_lat = float(input("Reference Latitude (degrees): "))
            ref_lon = float(input("Reference Longitude (degrees): "))
            ref_h = float(input("Reference Height (meters): "))
            x, y, z = geo.enu_to_ecef(e, n, u, ref_lat, ref_lon, ref_h)
            print(f"ECEF: X={x}, Y={y}, Z={z}")
            
        elif choice == "5":
            lat1 = float(input("Point 1 Latitude (degrees): "))
            lon1 = float(input("Point 1 Longitude (degrees): "))
            lat2 = float(input("Point 2 Latitude (degrees): "))
            lon2 = float(input("Point 2 Longitude (degrees): "))
            dist, bearing = geo.distance_bearing(lat1, lon1, lat2, lon2)
            print(f"Distance: {dist} meters, Bearing: {bearing} degrees")
            
        elif choice == "6":
            lat = float(input("Latitude (degrees): "))
            lon = float(input("Longitude (degrees): "))
            azimuth = float(input("Azimuth (degrees): "))
            distance = float(input("Distance (meters): "))
            new_lat, new_lon = geo.project_from_point(lat, lon, azimuth, distance)
            print(f"New Point: Lat={new_lat}, Lon={new_lon}")
            
        elif choice == "7":
            break
            
        else:
            print("Invalid option")


if __name__ == "__main__":
    main()