#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time

# Función para calcular la métrica de suavidad en base a ángulos
def metrica_smooth(x, y):
    k = len(x) - 1
    angulos = []

    # Calcular los ángulos entre segmentos de la trayectoria
    for i in range(len(x) - 2):
        angulo = ang_entre_rectas(x[i], x[i+1], x[i+2], y[i], y[i+1], y[i+2])
        angulos.append(angulo)

    # Calcular el valor de suavidad
    smoothy_value = (1.0 / k) * np.square(np.sum(angulos))
    return smoothy_value

# Función para calcular el ángulo entre tres puntos (dos segmentos)
def ang_entre_rectas(x1, x2, x3, y1, y2, y3):
    # Vector entre el primer y segundo punto
    v1 = np.array([x2 - x1, y2 - y1])
    # Vector entre el segundo y tercer punto
    v2 = np.array([x3 - x2, y3 - y2])
    # Ángulo entre los vectores
    cos_theta = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    angulo = np.arccos(np.clip(cos_theta, -1.0, 1.0))
    return angulo

# Función que evalúa el error cuadrático medio (ECM)
def errorcm(x, y, error):
    n = len(x)
    for i in range(2, n):
        x_selec = x[:i+1]
        y_selec = y[:i+1]
        # Ajuste de una línea entre el primer y último punto seleccionado
        coef = np.polyfit([x_selec[0], x_selec[-1]], [y_selec[0], y_selec[-1]], 1)
        a, b = coef[0], coef[1]
        y_pred = a * np.array(x_selec) + b
        ecm = np.mean((np.array(y_selec) - y_pred) ** 2)
        if ecm > error or n == 3:
            return i, ecm
    return n, ecm

# Clase principal para el cálculo de suavidad
class SmoothnessCalculator:
    def __init__(self, error_threshold=1.0):
        rospy.init_node('smoothness_calculator', anonymous=True)
        self.path = None
        self.smooth_data = []
        self.rate = rospy.Rate(1)
        self.error_threshold = error_threshold

        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)
        self.smooth_pub = rospy.Publisher('smoothness_metric', Float32MultiArray, queue_size=10)

    def path_callback(self, msg):
        self.path = msg.poses

    def calculate_smoothness(self):
        if self.path is None:
            return None

        x_coords = [pose.pose.position.x for pose in self.path]
        y_coords = [pose.pose.position.y for pose in self.path]

        # Validar que hay al menos 3 puntos en la trayectoria
        if len(x_coords) < 3:
            rospy.logwarn("La trayectoria tiene menos de 3 puntos. No se puede calcular suavidad.")
            return 0.0

        x_filtered, y_filtered = [x_coords[0]], [y_coords[0]]
        x_temp, y_temp = x_coords, y_coords

        while len(x_temp) > 3:
            ni, ecm = errorcm(x_temp, y_temp, self.error_threshold)
            if ni >= len(x_temp):  # Validar índice
                rospy.logwarn("Índice fuera de rango durante el filtrado de puntos.")
                break

            x_filtered.append(x_temp[ni])
            y_filtered.append(y_temp[ni])
            x_temp = x_temp[ni:]
            y_temp = y_temp[ni:]

        # Validar que x_filtered tiene al menos 3 puntos antes de calcular la suavidad
        if len(x_filtered) < 3:
            rospy.logwarn("Menos de 3 puntos en la trayectoria filtrada. No se puede calcular suavidad.")
            return 0.0

        return metrica_smooth(x_filtered, y_filtered)

    def run(self):
        start_time = time.time()

        while not rospy.is_shutdown():
            smoothness = self.calculate_smoothness()

            if smoothness is not None:
                current_time = time.time() - start_time
                rospy.loginfo("Smoothness metric: {}".format(smoothness))

                smooth_msg = Float32MultiArray()
                smooth_msg.data = [smoothness]
                self.smooth_pub.publish(smooth_msg)

                self.smooth_data.append((current_time, smoothness))

            self.rate.sleep()

        self.save_data()

    def save_data(self):
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
        pass