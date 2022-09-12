import glob
import os
import sys
from copy import copy

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass



import carla
#Read the .osm data
f = open("./maps/kansas.osm", 'r')
osm_data = f.read()
f.close()

# # Define the desired settings. In this case, default values.

settings = carla.Osm2OdrSettings()
# # Set OSM road types to export to OpenDRIVE
# settings.set_osm_way_types(["motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential"])
# # Convert to .xodr

settings.generate_traffic_lights=True
#settings.use_offsets=True
settings.use_offsets=False
#settings.center_map=True
settings.center_map=False
settings.offset_x=0
settings.offset_y=0
#settings2=copy.deepcopy(settings)
settings.proj_string=str("+proj=utm +zone=15 -r -s")

#print(settings.use_offsets)
#print(settings.center_map)
#print(settings.proj_string)

xodr_data = carla.Osm2Odr.convert(osm_data,settings)


# save opendrive file
f = open("./maps/kansas.xodr", 'w')
f.write(xodr_data)
f.close()


# if os.path.exists(args.osm_path):
#             with open(args.osm_path) as od_file:
#                 try:
#                     data = od_file.read()
#                 except OSError:
#                     print('file could not be readed.')
#                     sys.exit()
#             print('Converting OSM data to opendrive')
#             xodr_data = carla.Osm2Odr.convert(data)
