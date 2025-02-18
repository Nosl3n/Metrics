#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time
import os  # Para manejar rutas y archivos

# Función que calcula la métrica smooth
def metrica_smooth(x, y):
    k = len(x) - 1  # Número de segmentos en la trayectoria
    if k <= 0:
        return 0.0
    smoothy_value = (1.0 / k) * np.sum(np.square(np.diff(x)) + np.square(np.diff(y)))
    return smoothy_value

class SmoothnessCalculator: 
    def __init__(self):
        # Inicializa el nodo
        rospy.init_node('smoothness_calculator', anonymous=True)
        self.path = None  # Para almacenar la trayectoria del robot
        self.smooth_data = []  # Para almacenar el tiempo y la métrica smooth
        self.rate = rospy.Rate(1)  # Frecuencia de actualización 1Hz

        # Suscriptor al tópico que recibe la trayectoria
        rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.path_callback)

        # Publicador para la métrica smooth
        self.smooth_pub = rospy.Publisher('smoothness_metric', Float32MultiArray, queue_size=10)

    def path_callback(self, msg):
        # Almacena la trayectoria del robot cuando llega
        rospy.loginfo("Path received with {} poses.".format(len(msg.poses)))
        self.path = msg.poses

    def calculate_smoothness(self):
        if self.path is None:
            rospy.logwarn("No path received yet.")
            return None

        x_coords = [pose.pose.position.x for pose in self.path]
        y_coords = [pose.pose.position.y for pose in self.path]

        if len(x_coords) < 2:
            rospy.logwarn("Trajectory too short to calculate smoothness.")
            return None  # Necesitamos al menos 2 puntos para calcular la métrica smooth

        # Calcular la métrica smooth
        smoothy_value = metrica_smooth(x_coords, y_coords)
        rospy.loginfo("Calculated smoothness metric: {:.4f}".format(smoothy_value))
        return smoothy_value

    def run(self):
        start_time = time.time()  # Marca el inicio del cálculo

        while not rospy.is_shutdown():
            smoothness = self.calculate_smoothness()

            if smoothness is not None:
                current_time = time.time() - start_time  # Tiempo transcurrido desde el inicio
                rospy.loginfo("Saving smoothness metric: {:.4f}".format(smoothness))
                self.smooth_data.append((current_time, smoothness))

                # Publicar la métrica smooth
                smooth_msg = Float32MultiArray()
                smooth_msg.data = [smoothness]
                self.smooth_pub.publish(smooth_msg)
            else:
                rospy.logwarn("Smoothness metric not calculated.")

            self.rate.sleep()

        # Guardar los datos en un archivo al finalizar
        self.save_data()

    def save_data(self):
        """
        Guarda los datos en un archivo .txt en la carpeta home del usuario.
        Si el archivo no existe, lo crea automáticamente.
        """
        # Definir la ruta del archivo en la carpeta home
        file_path = os.path.expanduser('~/smoothness_metrics.txt')
        file_exists = os.path.isfile(file_path)

        try:
            with open(file_path, 'a' if file_exists else 'w') as f:
                # Si es un archivo nuevo, agregar la cabecera
                if not file_exists:
                    f.write("Tiempo (s)\tSmoothy\n")

                # Escribir los datos registrados
                for data in self.smooth_data:
                    f.write("{:.2f}\t{:.4f}\n".format(data[0], data[1]))
            rospy.loginfo("Data saved successfully to: {}".format(file_path))
        except Exception as e:
            rospy.logerr("Failed to save data: {}".format(e))

if __name__ == '__main__':
    try:
        calculator = SmoothnessCalculator()
        calculator.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Guardar los datos al cerrar el nodo
        calculator.save_data()
