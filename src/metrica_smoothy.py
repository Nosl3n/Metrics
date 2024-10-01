#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time

# Función que calcula la métrica smooth
def metrica_smooth(x, y):
    k = len(x) - 1  # Número de segmentos en la trayectoria
    smoothy_value = (1.0 / k) * np.sum(np.square(np.diff(x)) + np.square(np.diff(y)))
    return smoothy_value

class SmoothnessCalculator: 
    def __init__(self):
        # Inicializa el nodo
        rospy.init_node('smoothness_calculator', anonymous=True)
        self.path = None  # Para almacenar la trayectoria del robot
        self.smooth_data = []  # Para almacenar el tiempo y la métrica smooth
        self.rate = rospy.Rate(1)  # Frecuencia de actualización 1Hz (ajusta si es necesario)

        # Suscriptor al tópico que recibe la trayectoria
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)

        # Publicador para la métrica smooth
        self.smooth_pub = rospy.Publisher('smoothness_metric', Float32MultiArray, queue_size=10)

    def path_callback(self, msg):
        # Almacena la trayectoria del robot cuando llega
        self.path = msg.poses

    def calculate_smoothness(self):
        if self.path is None:
            return None

        x_coords = [pose.pose.position.x for pose in self.path]
        y_coords = [pose.pose.position.y for pose in self.path]

        if len(x_coords) < 2:
            return None  # Necesitamos al menos 2 puntos para calcular la métrica smooth

        # Calcular la métrica smooth
        smoothy_value = metrica_smooth(x_coords, y_coords)
        return smoothy_value

    def run(self):
        start_time = time.time()  # Marca el inicio del cálculo

        while not rospy.is_shutdown():
            smoothness = self.calculate_smoothness()

            if smoothness is not None:
                current_time = time.time() - start_time  # Tiempo transcurrido desde el inicio
                rospy.loginfo("Smoothness metric: {}".format(smoothness))

                # Publicar la métrica smooth
                smooth_msg = Float32MultiArray()
                smooth_msg.data = [smoothness]
                self.smooth_pub.publish(smooth_msg)

                # Almacenar tiempo y métrica smooth
                self.smooth_data.append((current_time, smoothness))

            self.rate.sleep()

        # Guardar los datos en un archivo al finalizar
        self.save_data()

    def save_data(self):
        # Guardar los datos en un archivo .txt
        with open('smoothness_metrics.txt', 'w') as f:
            f.write("Tiempo (s)\tSmoothy\n")
            for data in self.smooth_data:
                f.write("{:.2f}\t{:.4f}\n".format(data[0], data[1]))
        rospy.loginfo("Datos guardados en smoothness_metrics.txt")

if __name__ == '__main__':
    try:
        calculator = SmoothnessCalculator()
        calculator.run()
    except rospy.ROSInterruptException:
        # Asegura que los datos se guarden al interrumpir el nodo
        calculator.save_data()
        pass
