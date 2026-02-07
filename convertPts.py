import numpy as np
import pyproj
import math

def geodetic2cart(phi, lam, h, flattening): # We specify flattening here to allow for different coord systems
	radPhi = (math.pi/180) * phi
	radLam = (math.pi/180) * lam
	eccentricitySq = flattening * (2 - flattening) # These values are specific to WGS84, thus we specify that for this converter

	N = 6378137 / math.sqrt(1 - (eccentricitySq * (math.sin(radPhi) ** 2)))

	x = (N + h) * math.cos(radPhi) * math.cos(radLam)
	y = (N + h) * math.cos(radPhi) * math.sin(radLam)
	z = ((1 - eccentricitySq) * N + h) * math.sin(radPhi)

	return (x, y, z)

def WGS84toNAD83(coords):
	wgs84 = pyproj.CRS.from_epsg(4326)
	nad83 = pyproj.CRS.from_epsg(3779) # Alberta specific

	transformer = pyproj.Transformer.from_crs(wgs84, nad83)



print(geodetic2cart(51.07900056732, -114.13253478947, 1113.9216, 1/298.257223563))