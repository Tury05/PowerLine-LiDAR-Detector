NO_GROUND = """
[
    {
        "type":"readers.las",
        "filename":""
    },
    {
        "type":"filters.range",
        "limits":"NumberOfReturns[1:7]"
    },
    {
        "type":"filters.smrf"
    },
    {
        "type":"writers.las",
        "filename":"",
        "where":"Classification != 2"
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
        "limits":"NumberOfReturns[1:7]"
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
    path = sys.argv[1]
    list_of_files = listdirs(path)
    for file in list_of_files:
        filename = os.path.basename(file)
        ng_string = NO_GROUND[:59] + file + NO_GROUND[59:-49] + sys.argv[1] + "/no_grounds/NG_" + filename + NO_GROUND[-49:]
        gt_string = GROUND_TRUTH[:59] + file + GROUND_TRUTH[59:-50] + sys.argv[1] + "/ground_truth/GROUND_TRUTH_" + filename + GROUND_TRUTH[-50:]
        ground = pdal.Pipeline(gt_string)
        _ = ground.execute()

if __name__ == '__main__':
    main()