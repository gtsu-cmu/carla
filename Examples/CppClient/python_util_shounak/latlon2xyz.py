#https://github.com/carla-simulator/carla/issues/2737
import math
import numpy

a = 6378137
b = 6356752.3142
f = (a - b) / a
e_sq = f * (2-f)

#Opposite conv -https://gis.stackexchange.com/questions/265909/converting-from-ecef-to-geodetic-coordinates
def geodetic_to_ecef(lat, lon, h):
    # (lat, lon) in WSG-84 degrees
    # h in meters
    lamb = math.radians(lat)
    phi = math.radians(lon)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)

    x = (h + N) * cos_lambda * cos_phi
    y = (h + N) * cos_lambda * sin_phi
    z = (h + (1 - e_sq) * N) * sin_lambda

    return x, y, z

#Opposite conversion- https://gis.stackexchange.com/questions/308445/local-enu-point-of-interest-to-ecef/308452
def ecef_to_enu(x, y, z, lat0, lon0, h0):
    lamb = math.radians(lat0)
    phi = math.radians(lon0)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)

    x0 = (h0 + N) * cos_lambda * cos_phi
    y0 = (h0 + N) * cos_lambda * sin_phi
    z0 = (h0 + (1 - e_sq) * N) * sin_lambda

    xd = x - x0
    yd = y - y0
    zd = z - z0

    xEast = -sin_phi * xd + cos_phi * yd
    yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd
    zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd

    return xEast, yNorth, zUp

def geodetic_to_enu(lat, lon, h, lat_ref, lon_ref, h_ref):
    x, y, z = geodetic_to_ecef(lat, lon, h)
    
    return ecef_to_enu(x, y, z, lat_ref, lon_ref, h_ref)
    
point_latitude=39.216812
point_longitude=-94.569464
point_altitude=0


origin_latitude=39.199423
origin_longitude=-94.576126
origin_altitude=0.0000

    
# point_latitude=48.99950150520519
# point_longitude=8.002271254544008
# point_altitude=1.9948198795318604


# origin_latitude=49.000000
# origin_longitude=8.00000
# origin_altitude=0.0000



x,y,z=geodetic_to_ecef(point_latitude,point_longitude,point_altitude)
carla_x,carla_y,carla_z=ecef_to_enu(x,y,z,origin_latitude,origin_longitude,origin_altitude)

print ("The converted values from Geodetic to  carla are",carla_x,-carla_y,carla_z)