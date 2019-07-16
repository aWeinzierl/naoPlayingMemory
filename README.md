# Nao playing memory

###### Instructions:
1. Build and source correctly
1. Prepare Nao so he clearly can see the complete board with all cards on it - with the images on the bottom.
    -   Orient yourselves according to the video 
    -   If there's an error, Nao will complain after the last step: "The board looks bad, please reposition it"
    -   The cards have to have specific marker ids on the back and front. The configuration can be obtained changed in 
    the vision node: 
        - `card_classes` maps the ids to their class 
        - `top_back_map` maps the id on the top to the back
        - `top_ids` has all top ids in it
1. Execute in the terminal:

    ```roslaunch nao_playing_memory memory.launch```
    
1. Wait until the execution has finished
1. Start the rest of the nodes accordingly:
    ```
    rosrun nao_playing_memory vision
    rosrun nao_playing_memory speech
    rosrun nao_playing_memory reasoning
    ```