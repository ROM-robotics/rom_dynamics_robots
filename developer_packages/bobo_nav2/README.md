#### How To Launch
```
ros2 launch bobo_nav2 sim_navigation.launch.py use_composition:=True use_sim_time:=True
#or
ros2 launch bobo_nav2 navigation.launch.py use_composition:=True use_sim_time:=True use_slamtoolbox:=False

```
use_composition ကို True/False ကို capital letter ဖြင့်ရေးပါ။

##### Nav2 ActionServer
```
- /navigate_to_pose actionserver ကို nav2_msgs/action/NavigateToPose action type အသုံးပြုပြီး robot pose ပေးလို့ရ။
- /follow_waypoints actionserver ကို nav2_msgs/action/FollowWaypoints action type အသုံးပြုပြီး waypoints များကိုသွားရန် ပြုလုပ်လို့ရ။
- /spin actionserver ကို nav2_msgs/action/Spin action type အသုံးပြုပြီး spin ပြုလုပ်လို့ရ။
- /wait actionserver ကို nav2_msgs/action/Wait action type အသုံးပြုပြီး wait/delay ပြုလုပ်လို့ရ။
