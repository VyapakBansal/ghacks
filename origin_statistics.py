import statistics

def parse_bestposa_line(line):
    if not line.strip() or line.startswith('#'):
        return None
    parts = line.split(';')
    data = parts[1].split(',')
    try:
        latitude = float(data[2])
        longitude = float(data[3])
        height = float(data[4])
        return {
            'latitude': latitude,
            'longitude': longitude,
            'height': height
        }
    except (ValueError, IndexError):
        return None


def calculate_point_a_from_bestposa(csv_data):
    lines = csv_data.strip().split('\n')
    
    latitudes = []
    longitudes = []
    heights = []
    
    for line in lines:
        coord = parse_bestposa_line(line)
        if coord:
            latitudes.append(coord['latitude'])
            longitudes.append(coord['longitude'])
            heights.append(coord['height'])
    
    avg_lat = statistics.mean(latitudes)
    avg_lon = statistics.mean(longitudes)
    avg_height = statistics.mean(heights)
    
    std_lat = statistics.stdev(latitudes)
    std_lon = statistics.stdev(longitudes)
    std_height = statistics.stdev(heights)
    
    return {
        'point_a': {
            'latitude': avg_lat,
            'longitude': avg_lon,
            'height': avg_height
        },
        'statistics': {
            'num_measurements': len(latitudes),
            'latitude_std': std_lat,
            'longitude_std': std_lon,
            'height_std': std_height,
            'latitude_range': max(latitudes) - min(latitudes),
            'longitude_range': max(longitudes) - min(longitudes),
            'height_range': max(heights) - min(heights)
        },
        'all_measurements': {
            'latitudes': latitudes,
            'longitudes': longitudes,
            'heights': heights
        }
    }


with open("ghacks/parsed_data.csv", "r") as file:
    csv_data = file.read()

result = calculate_point_a_from_bestposa(csv_data)

print("=" * 80)
print("POINT A CALCULATION FROM BESTPOSA DATA")
print("=" * 80)
print()
print(f"Number of measurements: {result['statistics']['num_measurements']}")
print()
print("POINT A COORDINATES (AVERAGE):")
print(f"  Latitude:  {result['point_a']['latitude']:.11f}°")
print(f"  Longitude: {result['point_a']['longitude']:.11f}°")
print(f"  Height:    {result['point_a']['height']:.4f} m")
print()
print("PRECISION STATISTICS:")
print(f"  Latitude Standard Deviation:  {result['statistics']['latitude_std']:.10f}° ({result['statistics']['latitude_std']*111320:.4f} m)")
print(f"  Longitude Standard Deviation: {result['statistics']['longitude_std']:.10f}° ({result['statistics']['longitude_std']*111320*0.6:.4f} m)")
print(f"  Height Standard Deviation:    {result['statistics']['height_std']:.4f} m")
print()
print("MEASUREMENT SPREAD:")
print(f"  Latitude Range:  {result['statistics']['latitude_range']:.10f}° ({result['statistics']['latitude_range']*111320:.4f} m)")
print(f"  Longitude Range: {result['statistics']['longitude_range']:.10f}° ({result['statistics']['longitude_range']*111320*0.6:.4f} m)")
print(f"  Height Range:    {result['statistics']['height_range']:.4f} m")
print()
print("=" * 80)
print("ALL INDIVIDUAL MEASUREMENTS:")
print("=" * 80)
for i in range(len(result['all_measurements']['latitudes'])):
    print(f"Measurement {i+1:2d}: Lat={result['all_measurements']['latitudes'][i]:.11f}°  "
          f"Lon={result['all_measurements']['longitudes'][i]:.11f}°  "
          f"H={result['all_measurements']['heights'][i]:.4f} m")
print("=" * 80)