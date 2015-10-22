# Modelo KUKA KR6/2 #

Modelo del robot KUKA KR6/2 para ser empleado en el visualizador RViz integrado en ROS.

![kuka2.png](https://bytebucket.org/isaoparra/ei3001/raw/0998bec3d9b70773a7fce21b8630f6beda2839a6/grupo4/software/ros/kuka_model/kuka_shadow.png?token=dca9fb5d36be5b5264c0af2f49c78e006c4d03f6)

## Uso del modelo

El se tienen distintos tipos de modelos para el robot:

1.	*kuka-model*: Modelo simple del robot.
1.	*kuka-kinect-model*: Posee un sensor Kinect montado en uno de los *links* y define *depth_sensor* para visualizar *cloud points*.
1.	*kuka-shadow-model*: Modelo con transparencia (*alpha*), útil para tareas de teleoperación.

### Lanzar modelo simple
```
$ roslaunch kuka_model kuka-view.launch
```

## Compilar *XACRO* para generar modelo del robot.

Para generar URDF del robot.

```
$ cd kuka_model/urdf
$ rosrun xacro xacro.py kuka-model.xacro alpha_value:=1 use_tool:=0 > kuka-model.urdf
```
Parametros:

- *alpha_value* Valor de transparencia del modelo. Se emplea para generar el modelo KUKA Shadow.

- *use_tool* Define si se decea usar alguna herramienta extra, por defecto al usar *use_tool:=1* se a~nade el modelo de *kinect gripper* sobre el el *link 5*. Para configurar la herramienta se debe editar el archivo *kuka-model.xacro* y definir un archivo *xacro* que la implemente (*joints*, *links*, *meshes*, etc.).

## ROS Bags

```
$ roslaunch kuka_model kuka-bag.launch
```

```
$ cd ros_bags_dir
$ rosbag play bag_name.bag --clock --rate 10
```


## Contacto ##
* Rodrigo Muñoz rorro.mr@gmail.com, rmunozriffo@ing.uchile.cl