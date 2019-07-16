knowledgePerceptionWithNao

Instructions:
``` 
... Build and source correctly
... Prepare Nao so he clearly can see the complete board with all cards on it - with the images on the bottom.
...     Orient yourselves according to the video 
...     If there's an error, Nao will complain after the last step: "The board looks bad, please reposition it"
roslaunch nao_playing_memory memory.launch 
... wait until done ...
rosrun nao_playing_memory vision
rosrun nao_playing_memory speech
rosrun nao_playing_memory reasoning
```