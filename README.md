<!-- IDIOMAS ------------------------------------------------------------------------------------------------------------------->
<div>
    <p align="left">
        <a href="/README-en.md">English<a> · <b>Español</b>
    </p>
</div>
            
# Seguimiento autónomo
### *Planificación reactiva de caminos mediante persecución pura en coche a escala utilizando ROS y Gazebo*
  
<br>

## Guía de ejecución

1. Ubicarse en la carpeta de trabajo:
```
cd catkin_racecar
```

2. Generar el entorno de trabajo:
```
source devel/setup.bash:
```

3. Ejeuctar el control mediante mando:
```
roslaunch racecar_control gazebo_sim_joy.launch
```

4. Ejecutar el algoritmo de seguimiento:
```
rosrun path_tracker follower.py
```
