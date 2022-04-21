json = """ 
[
    {
        "type":"readers.las",
        "filename":"input.las"
    },
    {
        "type":"filters.smrf"
    },
    {
        "type":"writers.las",
        "filename":"no_ground_smrf.las",
        "where":"Classification != 2"
    }
]"""

import pdal
import time
from WBT.whitebox_tools import WhiteboxTools

def main():
    wbt = WhiteboxTools()
    ground = pdal.Pipeline(json)
    
    start_time = time.time()
    _ = ground.execute()
    end_time = time.time()
    
    print("IO_time: %s\n" % (end_time - start_time))

    """wbt.lidar_segmentation("../no_ground_smrf.las", "../segmentated.las", 
    radius=2.0, 
    num_iter=50, 
    num_samples=10, 
    threshold=0.15, 
    model_size=15, 
    max_slope=80.0, 
    norm_diff=10.0, 
    maxzdiff=1.0, 
    classes=False, 
    ground=False
    )"""

if __name__ == '__main__':
    main()