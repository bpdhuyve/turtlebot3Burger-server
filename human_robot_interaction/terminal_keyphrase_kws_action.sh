# just adjust HRI_PATH variable to the place where the package is installed
export HRI_PATH=~/catkin_ws/src/human_robot_interaction
export DEMO_PATH=$HRI_PATH/dics_and_kwlists
roslaunch human_robot_interaction keyphrase_kws_action.launch kpdict:=$DEMO_PATH/keyphrase_dictionary.dic hmm:=$HRI_PATH/model/en-us/en-us kwsdict:=$DEMO_PATH/action_dictionary.dic kws:=$DEMO_PATH/action_keywordlist.kwlist
