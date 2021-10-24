# AquilaBot
Решение на базе OmegaBot



Сонар HCSR04





## Подготовка образа raspberry pi 3b+

Качаем версию посередине на 1 Gb, а то задолбаемся с обновлением не нужного софта, вроде целого wolfram-engine
[Raspberry Pi OS with desktop] buster https://www.raspberrypi.com/software/operating-systems/

wifi https://howchoo.com/g/ndy1zte2yjn/how-to-set-up-wifi-on-your-raspberry-pi-without-ethernet

ssh https://phoenixnap.com/kb/enable-ssh-raspberry-pi

raspi-config - переименовать hostname

sudo passwd root - сменить пароль входа пользователя root (вроде, какая-то старая затычка дыры в безопасности, можно поставить одинаковый с pi)

sudo passwd pi - сменить пароль входа пользователя pi (по умолчанию raspberry)

raspi-config - "expand your filesystem" - ускоряет работу в файловой системой, но потом её будет сложнее сжать в один мелкий файл и перенести на другую sd карту.

sudo apt install -y tmux htop - Чтобы можно было обновлять систему и не бояться, что сеть пропадёт.

sudo apt update

sudo apt upgrade

sudo apt dist-upgrade

sudo apt autoremove

sudo reboot

[Ros Desktop Full] http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi

http://wiki.ros.org/melodic/Installation/Ubuntu

Ставить без Arduino IDE http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

raspi-config - включить камеру и другие нужные интерфейсы связи с пишкой.

