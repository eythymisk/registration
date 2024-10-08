

This repository contains two command-line tools. 

The **Registration Tool** performs the registration of two point clouds. It takes as input the names of the files containing the data of the point clouds to be registered and outputs a rigid transformation (a rotation matrix and a translation vector), which represents the estimated transformation of the second point cloud relative to the first. It also outputs two point clouds. The first is the second point cloud transformed in the coordinate system of the first.  The second is a point cloud which contains the merged information of the two registed point clouds. This tool is a foundational step to understand the main tool, which is the Reconstruction tool.

The **Reconstruction Tool** reconstructs a scene from a set of point clouds or point cloud sequence. It takes as input the name of the folder containing the point clouds and outputs a single point cloud which contains the merged information of all the registered scans. It also outputs two txt files. A file of relative poses, which contains the relative pose of each point cloud relative to its previous point cloud, and a file of absolute poses, which contains the absolute pose of each point, which in this case is its pose relative to the the first point cloud in the sequence.

For the Registration Tool, you have three hyperparameters you can tune. **downsample_grid_step** is the voxel size for the voxel grid downsampling in the preprocessing step, **K** is the number of knn of each point to use for analyzing its neighborhood and **epsilon** is a factor for adjusting the anisotropy of error. For the Reconstuction tool, you can also adjust the final resolution of the output map using the **map_grid_step** parameter.

This project depends on the **Point Cloud Library** (PCL) for point cloud processing. 

The software is based on our paper **"Unleashing the Power of Generalized Iterative Closest Point for Swift and Effective Point Cloud Registration"**. You can acces the paper [here](https://ieeexplore.ieee.org/abstract/document/10647551).

You can compile each tool independently. To do that, set your current directory in the command line to the directory containing the respective main.cpp file and use the following command:

g++ -std=c++17 -O3 \
    -I/opt/homebrew/include/pcl-1.14 \
    -I/opt/homebrew/include/eigen3 \
    -L/opt/homebrew/lib \
    -lpcl_common -lpcl_io -lpcl_filters \
    main.cpp \
    -o Test

This command is specific to macOS, where development was done. However, it can be adapted for other platforms like Windows and Linux by adjusting the paths to the header files and libraries accordingly.

Then to run the executables use this format: 

Registration:

./Test <fixed_file> <moving_file>

Reconstruction: 

./Test <folder_path>

Additional comments:

Currently, this tool supports point clouds in PCD format without color and for single-precision points. We plan to extend its capabilities in the near future, including additional formats, color information, and different precision levels.

You can refer to this README file to check the current status of this work in real time. For any feedback, you can reach out at ekoukoulis@ece.upatras.gr. We can also provide you some data to test the algorithms.








