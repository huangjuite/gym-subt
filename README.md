# gym-subt

install gym-subt
```
$ pip install -e .
```

to open doker
```
$ cd doker && ./run.bash argnctu/subt
```

to open new terminal
```
$ cd docker &&./join.bash argnctu/subt
```

open gazebo in docker
```
# roslaunch subt_gazebo competition.launch scenario:=tunnel_practice_1
```

launch vehicle in docker
```
# X1_SENSOR_CONFIG_5=1 roslaunch subt_example one_robot.launch
```

launch laser scan in docker
```
# roslaunch reinforcement_learning pcToLaser.launch
```

test DQN
```
$ cd dqn && python run_subt_dqn.py
```

rviz
```
$ rviz -d gym-subt-v0.rviz
```
