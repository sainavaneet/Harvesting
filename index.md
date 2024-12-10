Cucumber harvesting using Object Detecion
================

## Introduction

This repository is dedicated to the development of a cucumber harvesting
system that utilizes custom object detection with `YOLOv11`. After
detecting cucumbers, we generate specific actions for two types of
robots: the `ViperX 300s` arm robot and an `Automated Guided Vehicle`
(AGV) robot.

The primary aim of this project is to create a comprehensive `dataset`
that captures both the actions performed by the robots and the images
taken during the harvesting process. This dataset is a crucial resource
for developing and refining algorithms that will enhance future robotic
harvesting techniques.

By systematically recording a wide array of interactions and scenarios,
we not only improve the efficiency of current systems but also lay a
robust foundation for future advancements in agricultural robotics. This
initiative represents a significant step forward in automating and
optimizing the harvesting process through the integration of advanced
machine learning models and robotic technology.

## 🛠️ Prerequisites

- **Ubuntu 20.04** 🐧

- **Interbotix Packages** 🤖

- **Python** 🐍

- **ROS** 🤖

- **interbotix_ws : -**

## 🚀 Installation

To get started with this frame work, follow these steps:

``` bash
git clone https://github.com/sainavaneet/Harvesting.git

cd Harvesting/

pip install -r requirements.txt
```

## 🗂 Project Structure

## Launch

``` bash
source interbotix_ws/devel.setup.bash

cd ~/Harvesting/launch/

roslaunch robot.launch use_rviz:=false use_sim:=False # if you need in simulation use True
```
