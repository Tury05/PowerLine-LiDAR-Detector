NO_GROUND = """
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
        "type":"filters.hag_nn"
    },
    {
        "type":"writers.las",
        "filename":"",
        "where":"Classification != 2"
    }
]
"""

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

import pdal
import json
import sys
import os
from os import listdir
from os.path import isdir, isfile

def listdirs(path):
    files = []
    for file in os.listdir(path):
        d = os.path.join(path, file)
        if isfile(d) and (d.endswith(".las") or d.endswith(".laz")):
            files.append(d)
        elif os.path.isdir(d) and not d.endswith("no_grounds"):
            files = [*files, *listdirs(d)]
    return files

def main():
    path = sys.argv[2]
    operation = sys.argv[1]
    if operation == "-g":
        if not os.path.exists(os.path.join(path, "grounds")):
            os.makedirs(os.path.join(path, "grounds"))
    elif operation == "-ng":
        if not os.path.exists(os.path.join(path, "no_grounds")):
            os.makedirs(os.path.join(path, "no_grounds"))
    elif operation == "-gt":
        if not os.path.exists(os.path.join(path, "ground_truth")):
            os.makedirs(os.path.join(path, "ground_truth"))
    if isdir(path):
        for mypath, mydirs, list_of_files in os.walk(path):
            for file in list_of_files:   
                if not file.endswith(".laz"):
                    continue      
                filename = os.path.basename(file)
                filename = os.path.basename(os.path.join(os.path.basename(mypath), file))
                print("tile:", filename)
                ng_string = NO_GROUND[:59] + sys.argv[2] + file + NO_GROUND[59:-49] + sys.argv[2] + "/no_grounds/NG_" + filename + NO_GROUND[-49:]
                g_string = GROUND[:59] + sys.argv[2] + file + GROUND[59:-10] + sys.argv[2] + "/grounds/G_" + filename + GROUND[-10:]
                gt_string = GROUND_TRUTH[:59] + sys.argv[2] + file + GROUND_TRUTH[59:-50] + sys.argv[2] + "/ground_truth/GROUND_TRUTH_" + filename + GROUND_TRUTH[-50:]

                if operation == "-g":
                    ground = pdal.Pipeline(g_string)
                elif operation == "-gt":
                    ground = pdal.Pipeline(gt_string)
                elif operation == "-ng":
                    # print("JSON", ng_string)
                    ground = pdal.Pipeline(ng_string)
                _ = ground.execute()

    elif isfile(path):
        filename = os.path.basename(path)
        ng_string = NO_GROUND[:59] + path + NO_GROUND[59:-49] + os.path.dirname(path) + "/no_grounds/NG_" + filename + NO_GROUND[-49:]
        g_string = GROUND[:59] + path + GROUND[59:-10] + os.path.dirname(path) + "/no_grounds/G_" + filename + GROUND[-10:]
        gt_string = GROUND_TRUTH[:59] + path + GROUND_TRUTH[59:-50] + os.path.dirname(path) + "/ground_truth/GROUND_TRUTH_" + filename + GROUND_TRUTH[-50:]

        if operation == "-g":
            ground = pdal.Pipeline(g_string)
        elif operation == "-gt":
            ground = pdal.Pipeline(gt_string)
        elif operation == "-ng":
            # print("JSON", ng_string)
            ground = pdal.Pipeline(ng_string)
        _ = ground.execute()

    


if __name__ == '__main__':
    main()