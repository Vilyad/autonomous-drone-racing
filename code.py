#! /usr/bin/env python3

import rospy
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class StartRace(object):

  def __init__(self):
      
    self.ctrl_c = False
    self.rate = rospy.Rate(10)
    
  
  def publish_once_in_cmd_vel(self, cmd):
    """
    Функция для однократной публикации команды в топик cmd_vel.
    Повторяет попытку до тех пор, пока не будет установлено соединение.
    Это важно для систем, которые публикуют команды только один раз.
    """
    while not self.ctrl_c:
        connections = self._pub_cmd_vel.get_num_connections()
        if connections > 0:
            self._pub_cmd_vel.publish(cmd)
            rospy.loginfo("Publish in cmd_vel...")
            break
        else:
            self.rate.sleep()

  def stop_drone(self):
    """Останавливает дрон, обнуляя линейные и угловые скорости."""
    rospy.loginfo("Stopping...")
    self._move_msg.linear.x = 0.0
    self._move_msg.angular.z = 0.0
    self.publish_once_in_cmd_vel(self._move_msg)

  def turn_drone(self, degrees):
    """Поворачивает дрон на заданное количество градусов."""
    rospy.loginfo("Turning...")
    self._move_msg.linear.x = 0.0
    self._move_msg.linear.y = 0.0
    self._move_msg.linear.z = 0.0
    self._move_msg.angular.z = degrees / 105
    self.publish_once_in_cmd_vel(self._move_msg)

  def move_forward_drone(self, move_x, move_y, move_z):
    """Перемещает дрон по вектору, заданному координатами XYZ."""
    rospy.loginfo("Moving forward...")
    self._move_msg.linear.x = move_x
    self._move_msg.linear.y = move_y
    self._move_msg.linear.z = move_z
    self._move_msg.angular.z = 0.0
    self.publish_once_in_cmd_vel(self._move_msg)
    
  def run(self):
    """
    Основная функция, выполняющая последовательность действий:
    - взлет дрона
    - движение по заданной траектории
    - посадка дрона
    """
    
    # Вспомогательные переменные
    r = rospy.Rate(1)
    
    # Инициализация издателей и сообщений для управления дроном
    self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self._move_msg = Twist()
    self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
    self._takeoff_msg = Empty()
    self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)
    self._land_msg = Empty()
    
    # Взлет дрона (3 попытки с интервалом 1 секунда)
    i=0
    while not i == 3:
        self._pub_takeoff.publish(self._takeoff_msg)
        rospy.loginfo('Taking off...')
        time.sleep(1)
        i += 1
    
    # Временные параметры для перемещения и поворотов
    sideSeconds = 2      # время движения по прямой
    turnSeconds = 1.8    # время выполнения поворота

    # Последовательность перемещений по заданной траектории
    self.move_forward_drone(2.3, -0.65, 0.44)
    time.sleep(sideSeconds)
    self.move_forward_drone(1.7, 0.33, -0.53)
    time.sleep(sideSeconds)
    self.turn_drone(-90)
    time.sleep(turnSeconds)
    self.move_forward_drone(3, 0.22, 0.45)
    time.sleep(sideSeconds)
    self.move_forward_drone(1.3, 0, -0.5)
    time.sleep(sideSeconds)
    self.turn_drone(-120)
    time.sleep(turnSeconds)
    self.move_forward_drone(3, 0.2, 0.5)
    time.sleep(sideSeconds)
    self.move_forward_drone(0.7, 0.1, 1)
    time.sleep(sideSeconds)
    self.move_forward_drone(0, -0.05, -1)
    time.sleep(sideSeconds)
    self.move_forward_drone(1.4, 0, 0)
    time.sleep(sideSeconds)
    r.sleep()

    # Остановка и посадка дрона (3 попытки с интервалом 1 секунда)
    self.stop_drone()
    i=0
    while not i == 3:
        self._pub_land.publish(self._land_msg)
        rospy.loginfo('Landing...')
        time.sleep(1)
        i += 1

if __name__ == '__main__':
  # Инициализация ROS-узла и запуск основной программы
  rospy.init_node('move_square')
  drone_attempt = StartRace()
  try:
      drone_attempt.run()
  except rospy.ROSInterruptException:
      pass