##### Please Follow these instructions

###### စစ်ဆေးရန်
```
<your_robot>_controller/launch/hardware.launch.py  မှာ string edit ( sed ) အသုံးပြုထားသော
/path/to/<your_robot>_controller/config/<your_robot>_controllers.yaml 
လမ်းကြောင်း နှစ်ခုကို မှန်/မမှန် စစ်ဆေးပါ။
```

###### ဖြည့်စွက်ရန်
```
# set robot's name
export ROM_ROBOT_MODEL=rom2109
#export ROM_ROBOT_MODEL=bobo
#export ROM_ROBOT_MODEL=yoyo

# for joystick
export USE_JOYSTICK=true
export LINEAR_SPEED='0.4'
export ANGULAR_SPEED='0.3'

alias bb='colcon build && source install/setup.bash'
alias delete_workspace='rm -rf build install log; echo "Done"'

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=69
```

###### စတင်ရန်
```
ဒါကဒီလို
```

###### TODO:

```
mapping mode မနှိပ်ရင် save map ဖျောက်ထားရန်
```
```
development script ရေးရန်
```