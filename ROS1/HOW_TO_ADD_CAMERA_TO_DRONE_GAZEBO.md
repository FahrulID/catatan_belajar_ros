# Installation

What will you learn here :
* Adding camera joint to IRIS default Model

## Buka folder Models

```sh
cd /path/to/PX-Autopilot/Tools/simulation/gazebo/sitl_gazebo/models
```

## Buka model yang akan diganti

Bila ingin mengganti drone IRIS default ( px4_sitl )
```sh
gedit iris/iris.sdf
```

## Tambahkan joint camera
```
    <plugin name='rotors_gazebo_imu_plugin' filename='libgazebo_imu_plugin.so'>
      <robotNamespace/>
      <linkName>/imu_link</linkName>
      <imuTopic>/imu</imuTopic>
      <gyroscopeNoiseDensity>0.00018665</gyroscopeNoiseDensity>
      <gyroscopeRandomWalk>3.8785e-05</gyroscopeRandomWalk>
      <gyroscopeBiasCorrelationTime>1000.0</gyroscopeBiasCorrelationTime>
      <gyroscopeTurnOnBiasSigma>0.0087</gyroscopeTurnOnBiasSigma>
      <accelerometerNoiseDensity>0.00186</accelerometerNoiseDensity>
      <accelerometerRandomWalk>0.006</accelerometerRandomWalk>
      <accelerometerBiasCorrelationTime>300.0</accelerometerBiasCorrelationTime>
      <accelerometerTurnOnBiasSigma>0.196</accelerometerTurnOnBiasSigma>
    </plugin>

    /////// Camera Joint added here 

    <include>
      <uri>model://fpv_cam</uri>
      <pose>0 0 0 0 1.5708 0</pose> // 1.5718 Artinya 90 derajat dalam radian, bila ingin mengarahkan kamera ke bawah
    </include>
    <joint name="fpv_cam_joint" type="fixed">
      <child>fpv_cam::link</child>
      <parent>iris::base_link</parent>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <upper>0</upper>
          <lower>0</lower>
        </limit>
      </axis>
    </joint>

    /////// Camera Joint added here 

  </model>
</sdf>
```