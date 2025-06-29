# autonomous-drone-racing

ИНСТРУКЦИЯ ПО УСТАНОВКЕ И ЗАПУСКУ ПО

**1. УСТАНОВКА ВИРТУАЛЬНОЙ МАШИНЫ (Пользователи Windows)**

  Т.к. симулятор Gazebo доступен только в Linux, для Windows подойдёт только запуск виртуальной машины.
  
  Для установки перейдите на документацию clover по ссылке:
  
  Установка виртуальной машины - https://clover.coex.tech/ru/simulation_vm.html (использовалась VMWare Workstation)

  !!! Внимание !!! Проверьте подключение к интернету, нужно будет переключится на Wired connection.

  **2. УСТАНОВКА FIRA-Air-Simulator**

  FIRA-Air-Simulator - сборка симулятора на базе Gazebo, содержащая трассу задания по умолчанию.

  Далее информация, взятая с GitHub - https://github.com/FIRAAir/FIRA-Air-Simulator.

  **Протестированные минимальные требования к локальному оборудованию**
  
  Процессор: Intel® Core™ i5-5257U CPU @ 2.70GHz
  
  Графический процессор: Intel® Iris 6100
  
  ОЗУ: 8 ГБ

  **Требования к программному обеспечению**

  Используются исключительно Ubuntu 20.04 и ROS Noetic. Другие версии официально не поддерживаются. 
  Перед установкой нашего программного обеспечения убедитесь, что установлены инструменты ROS и Catkin: http://wiki.ros.org/noetic/Installation/Ubuntu
  
    sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo apt-get install python3-catkin-tools
    sudo apt install python3-wstool
    sudo apt install ros-noetic-ros-control
    pip3 install catkin_pkg
  
  **Установка**

    # Setup catkin workspace
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin init
    # Add workspace to bashrc.
    echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
    cd src
    git clone https://github.com/FIRAAir/FIRA-Air-Simulator.git
    cd ..
    catkin_make
    source ~/.bashrc

    # Обязательно выполните следующую строку для обновления статических путей (важно)
    rosrun fira_challenge_env model_update.py

  Процесс установки завершён.

  **3. УСТАНОВКА И ЗАПУСК КОДА**

  Поместите файл с кодом в следующую папку: /home/clover/catkin_ws/src/FIRA-Air-Simulator/drone_demo/src/

  **Использование симулятора**

    roscd fira_challenge_env
    cd script
    ./run.sh

  После загрузки симулятора открыть code.py через Visual Studio Code и запустить файл.
