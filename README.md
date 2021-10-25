# AquilaBot
Решение на базе OmegaBot

Сонар HCSR04



## Прошивка raspberry pi 3b+

Использован уже готовый дитрибутив для роботов на raspberry pi.

https://downloads.ubiquityrobotics.com/pi.html



{Подключаем кабель из роутера в raspberry pi, а сами коннектимся к этой сети}

[ИЛИ]

{Подключаемся к wifi **ubiquityrobotXXXX** пароль **robotseverywhere**

Добавляем свою сеть с помощью pifi:

https://learn.ubiquityrobotics.com/connect_network}



sudo apt update -y

sudo apt install -y tmux htop



Создать окружение для служебных либ

```bash
mkdir -p ~/service_ws/src
cd ~/service_ws/src
catkin_init_workspace
cd ~/service_ws
catkin_make -j4
# echo "source /home/$USER/service_ws/devel/setup.bash" >> ~/.bashrc
echo "source /home/$USER/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

Собрать rosserial

```bash
cd ~/service_ws/src
git clone https://github.com/ros-drivers/rosserial.git -b jade-devel
cd ..
catkin_make
```

Собрать пакет связи нескольких мастеров для управления через компьютер multimaster_fkie

```bash
cd ~/service_ws/src
git clone https://github.com/fkie/multimaster_fkie.git -b kinetic-devel
cd ..
catkin_make
```

Убрать существующего демона

```bash
sudo systemctl stop magni-base
sudo systemctl disable magni-base
sudo rm -rf /etc/systemd/system/magni-base.service
sudo rm -rf /usr/sbin/magni-base
```

Создать своего демона для автоподключения по serial

```bash
echo "#!/bin/bash" > /usr/sbin/rosserial-arduino
echo "" >> /usr/sbin/rosserial-arduino
echo "source /home/ubuntu/service_ws/devel/setup.bash" >> /usr/sbin/rosserial-arduino
echo "source /etc/ubiquity/env.sh" >> /usr/sbin/rosserial-arduino
echo "" >> /usr/sbin/rosserial-arduino
echo "log_path="/tmp"" >> /usr/sbin/rosserial-arduino
echo "export ROS_HOME=\$(echo ~ubuntu)/.ros" >> /usr/sbin/rosserial-arduino
echo "export ROS_LOG_DIR=\$log_path" >> /usr/sbin/rosserial-arduino
echo "" >> /usr/sbin/rosserial-arduino
echo "rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=250000" >> /usr/sbin/rosserial-arduino

sudo chmod +x /usr/sbin/rosserial-arduino

echo "[Unit]" > /etc/systemd/system/rosserial-arduino.service
echo "Requires=roscore.service" >> /etc/systemd/system/rosserial-arduino.service
echo "PartOf=roscore.service" >> /etc/systemd/system/rosserial-arduino.service
echo "After=NetworkManager.service time-sync.target roscore.service" >> /etc/systemd/system/rosserial-arduino.service
echo "[Service]" >> /etc/systemd/system/rosserial-arduino.service
echo "Type=simple" >> /etc/systemd/system/rosserial-arduino.service
echo "User=ubuntu" >> /etc/systemd/system/rosserial-arduino.service
echo "Restart=always" >> /etc/systemd/system/rosserial-arduino.service
echo "TimeoutSec=1" >> /etc/systemd/system/rosserial-arduino.service
echo "ExecStart=/usr/sbin/rosserial-arduino" >> /etc/systemd/system/rosserial-arduino.service
echo "[Install]" >> /etc/systemd/system/rosserial-arduino.service
echo "WantedBy=multi-user.target" >> /etc/systemd/system/rosserial-arduino.service

sudo systemctl daemon-reload
sudo systemctl disable rosserial-arduino
sudo systemctl stop rosserial-arduino
sudo systemctl enable rosserial-arduino
sudo systemctl start rosserial-arduino
```



## Работа с образами

#### Запись образа

Открываем etcher

Прошиваем sd карту

#### Бекап образа

Открываем gparted

Уменьшаем раздел почти до минимума

https://www.tomshardware.com/how-to/back-up-raspberry-pi-as-disk-image

Посмотреть конец раздела 

`sudo fdisk -l /dev/sdb`

Ищем конец раздела и выполняем следующую операцию END*512/1000000

Именно столько мегабайт мы заняли. Накидываем ещё 100 (всё равно потом сожмём) и копируем образ:

`sudo dd if=/dev/sdb of=my_image.img bs=1M count={NEW_END} conv=sync,noerror` 

`sudo pishrink.sh -z my_image.img`

