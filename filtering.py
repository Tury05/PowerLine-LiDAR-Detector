NO_GROUND = """ 
[
    {
        "type":"readers.las",
        "filename":"/home/tury/Escritorio/lidar2019-ndp-c14-r8-ll69500-68500-epsg2169/LIDAR2019_NdP_70500_69000_EPSG2169.laz"
    },
    {
        "type":"filters.smrf"
    },
    {
        "type":"writers.las",
        "filename":"./data/input6_ground.laz"
    }
]
"""


import pdal

def main():
    ground = pdal.Pipeline(NO_GROUND)
    
    _ = ground.execute()

if __name__ == '__main__':
    main()