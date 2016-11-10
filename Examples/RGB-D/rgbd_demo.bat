@echo off
set datasetPath=D:\dataSets\TUM-RGBD\
set datasetName=rgbd_dataset_freiburg1_xyz
set associationFileName=rgb_depth_associations.txt
set datasetType=1
@echo on
Release\rgbd_tum.exe ..\..\Vocabulary\ORBvoc.txt TUM%datasetType%.yaml %datasetPath%%datasetName% %datasetPath%%associationFileName%