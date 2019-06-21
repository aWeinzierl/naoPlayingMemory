knowledgePerceptionWithNao

Instructions:
```
source devel/setup.bash
roscore
roslaunch nao_bringup nao_full_py.launch
roslaunch nao_playing_memory memory.launch 
rosrun nao_playing_memory reasoning
```