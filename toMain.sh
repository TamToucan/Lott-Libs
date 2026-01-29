#!/bin/bash
for dir in Cave-LIB DistanceMap-LIB Maze-LIB Tracker-LIB Libs
do
   cd $dir
   git checkout main
   cd ..
done
git checkout main

