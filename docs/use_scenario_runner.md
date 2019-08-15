# Using Autoware with CARLA Scenario Runner

## Execution

1. Start CARLA
2. Start scenario runner

```
    python scenario_runner.py --waitForEgo --scenario FollowLeadingVehicle_1
```

3. Start Autoware with CARLA Bridge

```
    roslaunch $CARLA_AUTOWARE_ROOT/devel.launch role_name:=hero
```


