# Display IP

In the RaspberryPi4, clone the repository in:
[LCD RaspberryPi Guy](https://github.com/the-raspberry-pi-guy/lcd)

Assuming that this repository was cloned a paquito's home:

* Copy the file ```rpi-host-lcd.service``` to ```/lib/systemd/system/``` as root.
* Enable the service and start it:
```
sudo systemctl enable rpi-host-lcd.service
sudo systemctl start rpi-host-lcd.service
```

