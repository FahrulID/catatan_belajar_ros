# How To Use Custom World

What will you learn here :
* Using custom world

{var} : Change with own var

Prerequisites :
* [MAVROS](INSTALASI_ROS_MAVROS.md)
* [GAZEBO](INSTALASI_GAZEBO.md)

## Masukkan Model World kedalam folder /models

```sh
sudo mv {folder_model_world} path/to/PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/models
```

## Masukkan file .world kedalam folder 

```sh
sudo mv {world_file.world} PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/worlds
```

## Masukkan nama world kedalam

Buka file path/to/PX4-Autopilot/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo.cmake
Tambah nama world kedalam set(worlds)

```
    set(worlds
        none
        baylands
        empty
        ksql_airport
        mcmillan_airfield
        sonoma_raceway
        warehouse
        windy
        yosemite
        {nama_world}
    )
```

Misalkan nama file world adalah template.world, maka {nama_world} diganti dengan "template"

## Running Custom World 

```sh
make px4_sitl gazebo___{nama_world}
```