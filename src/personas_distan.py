#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time
import os  # Para manejar rutas y archivos

# Función para calcular la distancia entre dos puntos
def euclidean_distance(x1, y1, x2, y2):
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

class DistanceCalculator:
    def __init__(self):
        # Inicializa el nodo
        rospy.init_node('distance_calculator', anonymous=True)
        self.positions = None  # Para almacenar las posiciones de las personas
        self.path = None  # Para almacenar la trayectoria del robot
        self.distance_data = []  # Para almacenar el tiempo y la distancia promedio
        self.rate = rospy.Rate(1)  # Frecuencia de actualización 1Hz (ajusta si es necesario)

        # Suscriptores a los tópicos
        rospy.Subscriber('group_positions', Float32MultiArray, self.positions_callback)
        rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.path_callback)

        # Publicador para el promedio de las distancias
        self.distance_pub = rospy.Publisher('average_distance', Float32MultiArray, queue_size=10)

    def positions_callback(self, msg):
        # Almacena las posiciones de las personas cuando llegan
        self.positions = msg.data

    def path_callback(self, msg):
        # Almacena la trayectoria del robot cuando llega
        self.path = msg.poses

    def calculate_distances(self):
        if self.positions is None or self.path is None:
            return None  # Si no hay posiciones o trayectorias, no se hace nada

        distances = []
        num_people = len(self.positions) // 2

        for i in range(num_people):
            x_person = self.positions[i]
            y_person = self.positions[i + num_people]
            min_distance = float('inf')

            for pose in self.path:
                x_robot = pose.pose.position.x
                y_robot = pose.pose.position.y
                distance = euclidean_distance(x_person, y_person, x_robot, y_robot)
                if distance < min_distance:
                    min_distance = distance

            distances.append(min_distance)

        return distances

    def calculate_average_distance(self):
        distances = self.calculate_distances()
        if distances:
            average_distance = np.mean(distances)
            rospy.loginfo("Average distance to the path: {:.4f}".format(average_distance))

            # Publicar el promedio de distancias
            avg_msg = Float32MultiArray()
            avg_msg.data = [average_distance]
            self.distance_pub.publish(avg_msg)

            return average_distance
        return None

    def run(self):
        start_time = time.time()  # Marca el inicio del cálculo

        while not rospy.is_shutdown():
            avg_distance = self.calculate_average_distance()

            if avg_distance is not None:
                current_time = time.time() - start_time  # Tiempo transcurrido desde el inicio
                # Almacenar tiempo y promedio de distancias
                self.distance_data.append((current_time, avg_distance))

            self.rate.sleep()

        # Guardar los datos en un archivo al finalizar
        self.save_data()

    def save_data(self):
        """
        Guarda los datos en un archivo .txt en la carpeta home del usuario.
        Si el archivo no existe, lo crea automáticamente.
        """
        # Definir la ruta del archivo en la carpeta home
        file_path = os.path.expanduser('~/average_distance_metrics.txt')
        
        # Verificar si el archivo ya existe
        file_exists = os.path.isfile(file_path)
        
        # Abrir el archivo en modo de escritura (crear o sobrescribir)
        with open(file_path, 'a' if file_exists else 'w') as f:
            # Si es un archivo nuevo, agregar la cabecera
            if not file_exists:
                f.write("Tiempo (s)\tDistancia Promedio\n")
            
            # Escribir los datos registrados
            for data in self.distance_data:
                f.write("{:.2f}\t{:.4f}\n".format(data[0], data[1]))
        
        # Log para informar dónde se guardaron los datos
        rospy.loginfo("Datos guardados en: {}".format(file_path))

if __name__ == '__main__':
    try:
        calculator = DistanceCalculator()
        calculator.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Guardar los datos al cerrar el nodo
        calculator.save_data()
