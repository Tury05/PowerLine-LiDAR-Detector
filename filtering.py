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

import pdal
import json
import sys
import os
from os import listdir
from os.path import isdir

def main():
    path = sys.argv[1]
    if isdir(path):
        for file in listdir(path):
            string = NO_GROUND[:59] + sys.argv[1] + "/" + file + NO_GROUND[59:-49] + "./grounds/" + file + NO_GROUND[-49:]
            ground = pdal.Pipeline(string)
            _ = ground.execute()

if __name__ == '__main__':
    main()