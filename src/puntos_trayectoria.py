#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class TrajectorySaver:
    def __init__(self):
        # Inicializa el nodo
        rospy.init_node('trajectory_saver', anonymous=True)
        self.path = None  # Para almacenar la trayectoria del robot

        # Suscriptor al tópico que recibe la trayectoria
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)

    def path_callback(self, msg):
        # Almacena la trayectoria del robot cuando llega
        self.path = msg.poses

        # Guardar la trayectoria en un archivo de texto
        self.save_trajectory()

    def save_trajectory(self):
        if self.path is None:
            return

        # Guardar los puntos x, y en un archivo de texto
        with open('trayectoria.txt', 'w') as f:
            f.write("x\ty\n")
            for pose in self.path:
                x = pose.pose.position.x
                y = pose.pose.position.y
                f.write("{:.4f}\t{:.4f}\n".format(x, y))

        rospy.loginfo("Trayectoria guardada en trayectoria.txt")

    def run(self):
        rospy.spin()  # Mantiene el nodo en ejecución

if __name__ == '__main__':
    try:
        saver = TrajectorySaver()
        saver.run()
    except rospy.ROSInterruptException:
        pass
