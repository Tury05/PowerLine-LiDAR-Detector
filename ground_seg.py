GROUND = """
[
    {
        "type":"readers.las",
        "filename":""
    },
    {
        "type":"filters.range",
        "limits":"NumberOfReturns[1:7], ReturnNumber[1:1]"
    },
    {
        "type":"filters.smrf"
    },
    {
        "type":"writers.las",
        "filename":""
    }
]
"""

GROUND_TRUTH = """
[
    {
        "type":"readers.las",
        "filename":""
    },
    {
        "type":"filters.range",
        "limits":"NumberOfReturns[1:7], ReturnNumber[1:1]"
    },
    {
        "type":"writers.las",
        "filename":"",
        "where":"Classification == 15"
    }
]
"""

RADIAL_DENSITY = """
[
    {
        "type":"readers.las",
        "filename":""
    },
    {
        "type":"filters.radialdensity",
        "radius":10.0
    },{
        "type":"filters.covariancefeatures",
        "knn":10,
        "threads": 12,
        "feature_set": "X,Y,Z,RadialDensity"
    },
    {
        "type":"filters.assign",
        "value" :
  [
      "Classification = 15 WHERE (RadialDensity < 0.6) "
      
  ]
        },
        {
            "type":"filters.range",
            "limits":"Classification[15:15]"
        },
        {
        "type":"writers.las",
        "filename":""
    }
]
"""

import pdal
import sys
import os
import time
from os.path import isdir, isfile

def main():
    total_time = 0
    path = sys.argv[2]
    operation = sys.argv[1]
    if operation == "-g":
        if not os.path.exists(os.path.join(path, "grounds")):
            os.makedirs(os.path.join(path, "grounds"))
    elif operation == "-gt":
        if not os.path.exists(os.path.join(path, "ground_truth")):
            os.makedirs(os.path.join(path, "ground_truth"))
    elif operation == "-r":
        if not os.path.exists(os.path.join(path, "radial")):
            os.makedirs(os.path.join(path, "radial"))
    if isdir(path):
        for _root, _dirs, list_of_files in os.walk(path):
            for file in list_of_files:
                if not file.endswith(".laz") and not file.endswith(".las"):
                    continue      
                print("Classifying ground from tile:", file)
                g_string = GROUND[:59] + sys.argv[2] + file + GROUND[59:-10] + sys.argv[2] + "/grounds/G_" + file + GROUND[-10:]
                gt_string = GROUND_TRUTH[:59] + sys.argv[2] + file + GROUND_TRUTH[59:-50] + sys.argv[2] + "/ground_truth/GROUND_TRUTH_" + file + GROUND_TRUTH[-50:]
                r_string = RADIAL_DENSITY[:59] + sys.argv[2] + file + RADIAL_DENSITY[59:-10] + sys.argv[2] + "/radial/RD_" + file[:-1] + 's' + RADIAL_DENSITY[-10:]

                if operation == "-g":
                    ground = pdal.Pipeline(g_string)
                elif operation == "-gt":
                    ground = pdal.Pipeline(gt_string)
                elif operation == "-r":
                    ground = pdal.Pipeline(r_string)
                
                start_time = time.time()
                _ = ground.execute()
                exe_time = time.time() - start_time
                total_time += exe_time
                print("--- %s seconds ---" % exe_time)
            break   #prevent descending into subfolders"""
        print("--- Total time: %s seconds ---" % total_time)

    elif isfile(path):
        filename = os.path.basename(path)
        g_string = GROUND[:59] + path + GROUND[59:-10] + os.path.dirname(path) + "/no_grounds/G_" + filename + GROUND[-10:]
        gt_string = GROUND_TRUTH[:59] + path + GROUND_TRUTH[59:-50] + os.path.dirname(path) + "/ground_truth/GROUND_TRUTH_" + filename + GROUND_TRUTH[-50:]
        r_string = RADIAL_DENSITY[:59] + path + RADIAL_DENSITY[59:-10] + os.path.dirname(path) + "/radial/RD_" + filename + RADIAL_DENSITY[-10:]

        if operation == "-g":
            ground = pdal.Pipeline(g_string)
        elif operation == "-gt":
            ground = pdal.Pipeline(gt_string)
        elif operation == "-r":
            ground = pdal.Pipeline(r_string)
            
        _ = ground.execute()

if __name__ == '__main__':
    main()