# Display IP

En la RaspberryPi4, clonar el repositorio en:
[LCD RaspberryPi Guy](https://github.com/the-raspberry-pi-guy/lcd)

Asegurarse de que en ```/boot/firmware/config.txt``` la línea ```dtparam=i2s=on``` está descomentada y que el módulo ```i2c``` esté activo; verificar con:

```
lsmod | grep i2c
```

```i2c_dev``` debe estar presente.

Para encontrar las conecciones i2c disponibles usar:
```
ls -l /dev/*i2c*
```

El siguiente comando indica si se detectó el dispositivo i2c-1, para el display debe listar un 27.
```
i2cdetect -y 1
```

Asumiendo que este repositorio fue clonado en el directorio hogar de ```paquito```:

* Copiar el archivo ```rpi-host-lcd.service``` a ```/lib/systemd/system/``` como root.
* Si los directorios de instalación cambian es necesario editar ```rpi-host-lcd.service``` para invocar al guión ```service_lcd_host.py``` en la posición correspondiente.
* Activar el servicio e iniciarlo:
```
sudo systemctl enable rpi-host-lcd.service
sudo systemctl start rpi-host-lcd.service
```

