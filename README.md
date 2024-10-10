# paquito-zero
Código de control para el robot paquito-zero

## Requisitos

En la RaspberryPi4 instalar:

```
pip3 install numpy
pip3 install opencv-contrib-python ó pip install opencv-python
pip3 install imutils
pip3 install smbus2
```

## Instalación

El bosquejo en ```bosquejo_completo``` se instala en el arduino.  Es probable que las bibliotecas deban ser copiadas en el directorio ```Arduino``` creado por la IDE.

Es necesario contar con una versión de ROS2 en la RaspberryPi4, dado que el soporte es Tier3 hay que compilarla.  Tras activar el _underlay_ de ROS 2, compilar ```paco_monitor``` en su propio espacio de trabajo y activar este _overlay_. Ejecutar entonces el nodo:

```
ros2 run paquito serial_reader_node
```

Para recibir el video de la cámara y enviar comandos ```http```, ejecutar, en la RaspberryPi, el guión de python:
```
python3 paquito-zero/raspberry_scripts/camera_i2c_control.py
```

En la PC se cuenta con un nodo de prueba para recibir los mensajes en el paquete de ROS 2 ```paco_monitor```.  Desde el espacio de trabajo correspondiente se ejecuta con:
```
ros2 run paco_monitor simple_monitor
```

