


鲁棒地图融合（包括PCM和DCS）
===================================================


Prerequisites
------

- [Pangolin](for visualization): <https://github.com/stevenlovegrove/Pangolin>
- [OpenCV3.x]
- CMake (Ubuntu: `sudo apt-get install cmake`), compilation configuration tool.
- [Boost](http://www.boost.org/)  (Ubuntu: `sudo apt-get install libboost-all-dev`), portable C++ source libraries.
- [GTSAM](https://bitbucket.org/gtborg/gtsam) develop branch, a C++ library that implement smoothing and mapping (SAM) framework in robotics and vision. Here we use factor graph implementations and inference/optimization tools provided by GTSAM. To install a particular commit of GTSAM follow the following instructions: 


```
$ git clone https://bitbucket.org/gtborg/gtsam
$ git checkout b7c695fa71efd43b40972eec154df265617fc07d -b dist-mapper
$ mkdir build
$ cmake ..
$ make -j8
$ sudo make install
```

Compilation & Installation
------

In the ```cpp``` folder excute:

```
$ mkdir build
$ cd build
$ cmake ..
$ make -j3
$ make check  # optonal, run unit tests
$ make install
```
Config
------
In the ```config.yaml```file
```
Dataset.dir: /media/D/robust_distributed_mapper/test_data/datasets/CSAIL/spilt_new50_/（数据集文件路径，更改）
Robots.number: 2（智能体数目）
Max.iter: 1000000（最大迭代次数）
Rotation.threshold: 1e-5（姿态估计阈值）
Pose.threshold: 1e-5（位姿估计阈值）
Confidence.probability: 0.9（PCM置信率）
PCM: 1（PCM开关，1表示开）
DCS: 0（DCS开关）
DCS.phir: 0.01
DCS.phip: 5
PCMHeu: 1（启发式PCM开关）

```

Run on a dataset
------
In the ```cpp/examples/``` folder, run:
```
$ ./robust_distributed_optimization_example_2robots
```


Data Format
-----
Each robot's graph is written in [g2o](https://github.com/RainerKuemmerle/g2o/wiki/File-Format) format and is indexed from 0. For example, for a 4 robot scenario, the directory will contain ```0.g2o```, ```1.g2o```, ```2.g2o``` and ```3.g2o```. An example dataset for 4 robots is given in ```data/example_4robots```. Each robot is specified using a character prefix symbol like 'a', 'b', 'c', 'd' for 4 robot case.

### Vertices ###
All the vertices corresponding to the first robot will be prefixed using 'a' using ```gtsam.Symbol``` like  ```gtsam.Symbol('a', 1)```, ```gtsam.Symbol('a',2)``` etc. Similarly the second robot will be prefixed using 'b' like ```gtsam.Symbol('b', 1)```, ```gtsam.Symbol('b',2)``` etc. 

