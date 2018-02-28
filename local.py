# path of the routing server
exe_loc = './osrm-backend-5.11.0/build/osrm-routed.exe'
# path of the road network file that the routing server uses
map_loc = './osrm-backend-5.11.0/build/greater-london-latest.osm.pbf'

# host port for routing server
hostport = '127.0.0.1'

# version number of osrm routing server
osrm_version = 'v5.12.0'

# If running on MGHPCC or other environment where singularity image is used, these must be set
use_singularity = False
simg_loc = None