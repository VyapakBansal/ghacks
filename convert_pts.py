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

    def deg_to_dms(self, deg, is_lon=False):
        direction = ('N' if deg >= 0 else 'S') if not is_lon else ('E' if deg >= 0 else 'W')
        deg = abs(deg)
        d = int(deg)
        m = int((deg - d) * 60)
        s = ((deg - d) * 60 - m) * 60
        return f"{d}°{m}'{s:.4f}\"{direction}"

    def bearing_to_vector(self, bearing_deg):
        bearing_rad = math.radians(bearing_deg)
        east = math.sin(bearing_rad)
        north = math.cos(bearing_rad)
        return east, north

    def calculate_point_b(self, lat_a, lon_a, h_a, bearing_deg, distance_m, delta_h):
        lon_b, lat_b, _ = self.geod.fwd(lon_a, lat_a, bearing_deg, distance_m)
        h_b = h_a + delta_h
        lat_dms = self.deg_to_dms(lat_b, is_lon=False)
        lon_dms = self.deg_to_dms(lon_b, is_lon=True)
        return {
            'geodetic': (lat_b, lon_b, h_b),
            'dms': (lat_dms, lon_dms)
        }

    def calculate_point_c(self, lat_b, lon_b, h_b, dx, dy, dz):
        x_b, y_b, z_b = self.geodetic_to_ecef(lat_b, lon_b, h_b)
        x_c = x_b + dx
        y_c = y_b + dy
        z_c = z_b + dz
        lat_c, lon_c, h_c = self.ecef_to_geodetic(x_c, y_c, z_c)
        lat_dms = self.deg_to_dms(lat_c, is_lon=False)
        lon_dms = self.deg_to_dms(lon_c, is_lon=True)
        return {
            'ecef': (x_c, y_c, z_c),
            'geodetic': (lat_c, lon_c, h_c),
            'dms': (lat_dms, lon_dms)
        }

    def calculate_point_d(self, lat_c, lon_c, h_c, delta_east, delta_north, delta_up):
        x_c, y_c, z_c = self.geodetic_to_ecef(lat_c, lon_c, h_c)
        e_c, n_c, u_c = self.ecef_to_enu(x_c, y_c, z_c, lat_c, lon_c, h_c)
        e_d = e_c + delta_east
        n_d = n_c + delta_north
        u_d = u_c + delta_up
        x_d, y_d, z_d = self.enu_to_ecef(e_d, n_d, u_d, lat_c, lon_c, h_c)
        lat_d, lon_d, h_d = self.ecef_to_geodetic(x_d, y_d, z_d)
        lat_dms = self.deg_to_dms(lat_d, is_lon=False)
        lon_dms = self.deg_to_dms(lon_d, is_lon=True)
        return {
            'enu': (e_d, n_d, u_d),
            'ecef': (x_d, y_d, z_d),
            'geodetic': (lat_d, lon_d, h_d),
            'dms': (lat_dms, lon_dms)
        }

    def calculate_point_e(self, point_d_x, point_d_y, point_d_z, ref_lat, ref_lon, ref_h, 
                         point_p_east, point_p_north, bearing_d_to_e, bearing_e_to_p, height_change):
        d_east, d_north, d_up = self.ecef_to_enu(point_d_x, point_d_y, point_d_z, ref_lat, ref_lon, ref_h)
        
        v1_east, v1_north = self.bearing_to_vector(bearing_d_to_e)
        v2_east, v2_north = self.bearing_to_vector(bearing_e_to_p)
        
        matrix = [[v1_east, v2_east], [v1_north, v2_north]]
        rhs = [point_p_east - d_east, point_p_north - d_north]
        
        det = matrix[0][0]*matrix[1][1] - matrix[0][1]*matrix[1][0]
        
        t = (rhs[0]*matrix[1][1] - rhs[1]*matrix[0][1]) / det
        
        e_east = d_east + t*v1_east
        e_north = d_north + t*v1_north
        e_up = d_up + height_change
        
        e_x, e_y, e_z = self.enu_to_ecef(e_east, e_north, e_up, ref_lat, ref_lon, ref_h)
        e_lat, e_lon, e_height = self.ecef_to_geodetic(e_x, e_y, e_z)
        
        lat_dms = self.deg_to_dms(e_lat, is_lon=False)
        lon_dms = self.deg_to_dms(e_lon, is_lon=True)
        
        return {
            'enu': (e_east, e_north, e_up),
            'ecef': (e_x, e_y, e_z),
            'geodetic': (e_lat, e_lon, e_height),
            'dms': (lat_dms, lon_dms)
        }


def main():
    geo = GeoConverter()
    print("=" * 50)
    print("Geo Coordinate Converter")
    print("=" * 50)
    
    while True:
        print("\n1. Geodetic to ECEF")
        print("2. ECEF to Geodetic")
        print("3. ECEF to ENU")
        print("4. ENU to ECEF")
        print("5. Distance and Bearing")
        print("6. Project from Point")
        print("7. Calculate Point B (bearing + distance + height change)")
        print("8. Calculate Point C (ECEF offset)")
        print("9. Calculate Point D (ENU offset)")
        print("10. Calculate Point E (bearing intersection)")
        print("11. Exit")
        print("-" * 50)
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
            lat_a = float(input("Point A Latitude (degrees): "))
            lon_a = float(input("Point A Longitude (degrees): "))
            h_a = float(input("Point A Height (meters): "))
            bearing = float(input("Bearing from A to B (degrees): "))
            distance = float(input("Distance A to B (meters): "))
            delta_h = float(input("Height change (meters): "))
            result = geo.calculate_point_b(lat_a, lon_a, h_a, bearing, distance, delta_h)
            print(f"Point B Geodetic: {result['geodetic']}")
            print(f"Point B DMS: {result['dms']}")
            
        elif choice == "8":
            lat_b = float(input("Point B Latitude (degrees): "))
            lon_b = float(input("Point B Longitude (degrees): "))
            h_b = float(input("Point B Height (meters): "))
            dx = float(input("Delta X (meters): "))
            dy = float(input("Delta Y (meters): "))
            dz = float(input("Delta Z (meters): "))
            result = geo.calculate_point_c(lat_b, lon_b, h_b, dx, dy, dz)
            print(f"Point C ECEF: {result['ecef']}")
            print(f"Point C Geodetic: {result['geodetic']}")
            print(f"Point C DMS: {result['dms']}")
            
        elif choice == "9":
            lat_c = float(input("Point C Latitude (degrees): "))
            lon_c = float(input("Point C Longitude (degrees): "))
            h_c = float(input("Point C Height (meters): "))
            delta_east = float(input("Delta East (meters): "))
            delta_north = float(input("Delta North (meters): "))
            delta_up = float(input("Delta Up (meters): "))
            result = geo.calculate_point_d(lat_c, lon_c, h_c, delta_east, delta_north, delta_up)
            print(f"Point D ENU: {result['enu']}")
            print(f"Point D ECEF: {result['ecef']}")
            print(f"Point D Geodetic: {result['geodetic']}")
            print(f"Point D DMS: {result['dms']}")
            
        elif choice == "10":
            point_d_x = float(input("Point D X (meters): "))
            point_d_y = float(input("Point D Y (meters): "))
            point_d_z = float(input("Point D Z (meters): "))
            ref_lat = float(input("Reference Latitude (degrees): "))
            ref_lon = float(input("Reference Longitude (degrees): "))
            ref_h = float(input("Reference Height (meters): "))
            point_p_east = float(input("Point P East (meters): "))
            point_p_north = float(input("Point P North (meters): "))
            bearing_d_to_e = float(input("Bearing D to E (degrees): "))
            bearing_e_to_p = float(input("Bearing E to P (degrees): "))
            height_change = float(input("Height change (meters): "))
            result = geo.calculate_point_e(point_d_x, point_d_y, point_d_z, ref_lat, ref_lon, ref_h,
                                          point_p_east, point_p_north, bearing_d_to_e, bearing_e_to_p, height_change)
            print(f"Point E ENU: {result['enu']}")
            print(f"Point E ECEF: {result['ecef']}")
            print(f"Point E Geodetic: {result['geodetic']}")
            print(f"Point E DMS: {result['dms']}")
            
        elif choice >= "11":
            break
            
        else:
            print("Invalid option")


if __name__ == "__main__":
    main()