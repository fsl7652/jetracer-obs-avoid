
## Final System

The final system combines 4 key components; a road-following regression model, a road binary classifier, the obstacle avoidance algorithm and the behaviour FSM.

### Road following regression model

The road-following [model](https://github.com/NVIDIA-AI-IOT/jetracer/blob/master/notebooks/interactive_regression.ipynb) is created by following the instructions in the linked notebook.

This model is used to predict and broadcast the desired steering to stay on the road. 

The implementation has been changed to be ROS-compatible in order to interact with other systems

### Road classifier

The road classifier is created using a ResNet18 binary classifier, in order to gather data for this, the data_collection notebook is used. It is recommended that for the road dataset that images from the regression model are transferred and used to maximise accuracy of the model.

After collecting the dataset the train_model notebook should be run, this uses the dataset to create a model trained over 30 epochs and saves it as a .pth file.

After creating the model the optimise_model notebook should be run this converts the model into a trt model which provides faster inference by using GPUs for computation

This trt model can be tested live using the test_model notebook and the probability is shown using live camera images.

This process was inspired and has been adapted from the collsion avoidance classifier from the NVIDIA-AI-IOT JetBot repository

### Obstacle Avoidance Algorithm

The obstacle avoidance algorithm was inspired by and adapted from the ZJU-FAST-LAB [Car-like robotic swarm](https://github.com/ZJU-FAST-Lab/Car-like-Robotic-swarm/tree/main) project, the algorithm was simplified for the lower computational power of the JetRacer, specifically by removing dynamic obstacle calculations to reduce computational load. As well as this the originally decentralised system was changed to directly command the JetRacer using cmd_vel commands based on calculated trajectories.

### Running The Final System

A python 3 virtual environment is necessary to run this project if you are using ROS Noetic or another version which doesn't support Python 3, a guide is found [here](https://docs.python.org/3/library/venv.html)

Make sure the scripts csi_road_follow and the run_py3_venv.sh are in the scripts folder in your catkin project and the paths are set correcty in the .sh file

Parameters are available in the config folder of combined_system. The combined_system & utils folders should be downloaded and moved into your catkin workspace. The workspace should be built using the command 'catkin_make -DC_MAKE_TYPE=Release -DCATKIN_ENABLE_TESTING=0'. After this the system should be ready to use. 

System parameters are available in the config folder, model paths are defined in the launch file

The system is run using the command 'roslaunch combined_system combined.launch'

## Dependencies 

OMPL is used for trajectory calculations - https://ompl.kavrakilab.org

PyTorch, torchvision, torch2trt packages are all required in python



## Acknowledgements

Waveshare JetRacer Wiki - https://www.waveshare.com/wiki/JetRacer_ROS_AI_Kit

Nvidia JetRacer Repositoty - https://github.com/NVIDIA-AI-IOT/jetracer
