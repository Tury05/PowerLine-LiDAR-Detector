NO_GROUND = """ 
[
    {
        "type":"readers.las",
        "filename":"./data/input.las"
    },
    {
        "type":"filters.smrf"
    },
    {
        "type":"writers.las",
        "filename":"./data/no_ground_smrf.las",
        "where":"Classification == 1"
    }
]"""

K_NEAREST = """
[
    "./data/input.las",
    {
        "type" : "filters.neighborclassifier",
        "domain" : "Classification[1:1]",
        "k" : 50
    },
    "./data/k_nearest.las"
]"""

DBSCAN = """
[   
    {
        "type":"readers.las",
        "filename":"./data/no_ground_smrf.las"
    },
    {
        "type":"filters.dbscan",
        "min_points":6,
        "eps":1.5,
        "dimensions":"X,Y"
    },
    {
        "type":"writers.bpf",
        "filename":"output0.bpf",
        "output_dims":"X,Y,Z,ClusterID"
    }
]
"""

BPF2TXT = """
[   
    {
        "type":"readers.bpf",
        "filename":"output0.bpf"
    },
    {
        "type":"writers.text",
        "order":"X,Y,Z,ClusterID",
        "keep_unspecified":"false",
        "filename":"outputfile0.txt"
    }
]
"""

LAS2TXT = """
[
    {
        "type":"readers.las",
        "filename":"./data/no_ground_smrf.las"
    },
    {
        "type":"writers.text",
        "order":"X,Y,Z",
        "keep_unspecified":"false",
        "filename":"./data/no_ground.txt"
    }
]
"""

TXT2LAS = """
[
    {
        "type":"readers.text",
        "filename":"./data/2D.txt"
    },
    {
        "type":"writers.las",
        "filename":"./data/2D.las"
    }
]
"""

TREES = """
[  
    "./data/k_nearest.las",
    {
        "type":"filters.hag_delaunay"
    },
    {
        "type":"filters.sort",
        "dimension":"HeightAboveGround",
        "order":"DESC"
    },
    {
        "type":"filters.litree",
        "min_points":50,
        "min_height":10.0,
        "radius":200.0
    },
    {
        "type":"writers.las",
        "filename":"output.laz",
        "minor_version":1.4,
        "extra_dims":"all"
    }
]
"""


import pdal
import lidar2D

def main():
    """ground = pdal.Pipeline(NO_GROUND)
    
    _ = ground.execute()

    text = pdal.Pipeline(LAS2TXT)
    _ = text.execute()

    lidar2D.main()

    text = pdal.Pipeline(TXT2LAS)
    _ = text.execute()
    
    dbscan = pdal.Pipeline(DBSCAN)
    
    _ = dbscan.execute()

    bpf2txt = pdal.Pipeline(BPF2TXT)
    
    _ = bpf2txt.execute()

    k_nearest = pdal.Pipeline(K_NEAREST)
    
    _ = k_nearest.execute()"""

    trees = pdal.Pipeline(TREES)
    
    _ = trees.execute()

if __name__ == '__main__':
    main()