# Installation

What will you learn here :
* Adding custom world into gazebo

## Copy folder yang berisikan file-file model

Biasanya, folder berisikan :
* meshes/
* model.config
* model.sdf

```sh
sudo cp -r /path/to/custom/models /path/to/PX-Autopilot/Tools/simulation/gazebo/sitl_gazebo/models
```

## Copy file world ke dalam folder worlds

```sh
sudo cp /path/to/custom/custom_world.world /path/to/PX-Autopilot/Tools/simulation/gazebo/sitl_gazebo/worlds
```

## Edit file CMake untuk men-compile world di atas

```sh
gedit /path/to/PX-Autopilot/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo.cmake
```

kemudian tambahkan nama world ke dalam file

```sh
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
	
    //// Input nama world ke sini
    custom_world
)
```

## Cara me-run world 

```sh
cd /path/to/PX-Autopilot/
make px4_{nama_vehicle} gazebo___{nama_world}
```

contoh :

```sh
make px4_sitl gazebo___custom_world
```