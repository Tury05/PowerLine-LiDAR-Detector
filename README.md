# PowerLine LiDAR Detector

LiDAR technology is one of the main data acquisition techniques in the geospatial field. In
the energy area, this technology is commonly used to capture point clouds of the environment
to later process them and analyze risk situations around power lines. This task, until now,
has been performed in most cases manually. This work consists of automating the processing
of the point clouds in order to detect power lines. In it, an algorithm based on geometric
features is developed.

# Installation
In order to execute this processing algorithm, it's necessary to install
some dependencies.

1. Firstly a python installation is needed:
[a link](https://www.python.org/downloads/)

2. Later conda installation is needed:
In Windows:
[a link](https://www.anaconda.com/products/distribution)

In Linux:
```
bash Anaconda-latest-Linux-x86_64.sh
```

3. Later you need to install PDAL library:
```
conda install -c conda-forge pdal
```


# Execution

In order to execute the full procesing algorithm you need to follow this steps:

1. Execute ground segmentation module:
```
conda activate pdalpy
python ground_seg.py -g <input>
```

2. Execute power line detector module with input being result of ground segmentation module:
```
./lidar-processing <input> <output>
```

# Metrics calculations

Open confusion-matrix folder and execute command:
```
./confusion-matrix <filtered tiles folder> <ground_truth tiles folder> <original tiles folder>
```