# Using Autoware with CARLA Scenario Runner

## Example

### Preparation

1. Select a scenario

```
python scenario_runner.py --list
```

In this example we select `FollowLeadingVehicle_1`.

2. Open the scenario definition file to find ego vehicle spawn point

vi srunner/configs/FollowLeadingVehicle.xml

```
<ego_vehicle x="107" y="133" z="0.5" yaw="0" model="vehicle.lincoln.mkz2017" />
```

### Execution

1. Start CARLA

2. Start scenario runner

```
    python scenario_runner.py --waitForEgo --scenario ControlLoss_1
```

3. Start Autoware with CARLA Bridge

Use the previously gathered spawn point during startup of Autoware.

```
    roslaunch $CARLA_AUTOWARE_ROOT/devel.launch role_name:=hero spawn_point:="107,133,0.5,0,0,0"
```


