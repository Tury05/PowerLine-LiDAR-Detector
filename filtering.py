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
        "filename":"ground_smrf.las"
    }
]"""

import pdal

def main():
    ground = pdal.Pipeline(json)
    count = ground.execute()             #Numero de puntos
    arrays = ground.arrays               #Array de todos los puntos
    metadata = ground.metadata           #Metadatos
    log = ground.log                     #Log

if __name__ == '__main__':
    main()