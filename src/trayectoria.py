#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from nav_msgs.msg import Path
import time  # Para obtener la marca de tiempo
import os  # Para manejar archivos y rutas

class MedidorTrayectoria:
    def __init__(self):
        # Inicializamos el nodo ROS llamado 'medidor_trayectoria'
        rospy.init_node('medidor_trayectoria', anonymous=True)
        
        # Variable para almacenar la longitud total de la trayectoria
        self.longitud_trayectoria = 0.0
        
        # Lista para almacenar los datos de tiempo y longitud de la trayectoria
        self.data = []
        
        # Intervalo de tiempo configurable (por defecto 1 segundo)
        self.intervalo_tiempo = rospy.get_param("~intervalo_tiempo", 1.0)  # Parámetro modificable
        
        # Lista para almacenar las posiciones de la trayectoria
        self.posiciones = []

        # Suscribimos al tópico de trayectorias
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.callback_trayectoria)
        
        # Timer para medir la longitud a intervalos regulares
        rospy.Timer(rospy.Duration(self.intervalo_tiempo), self.medir_longitud_trayectoria)
        
    def callback_trayectoria(self, data):
        """
        Función de callback que se ejecuta cada vez que se recibe una nueva trayectoria.
        Almacena los puntos de la trayectoria recibida.
        """
        self.posiciones = data.poses  # Almacena las posiciones de la trayectoria

    def medir_longitud_trayectoria(self, event):
        """
        Función que se ejecuta a intervalos regulares para medir la longitud de la trayectoria.
        """
        if not self.posiciones:
            return  # Si no hay puntos de trayectoria, no se realiza ninguna medición
        
        self.longitud_trayectoria = 0.0  # Reiniciamos la longitud al medir
        
        # Iteramos sobre los puntos de la trayectoria
        for i in range(1, len(self.posiciones)):
            # Obtenemos las coordenadas x, y del punto actual y el punto anterior
            x1, y1 = self.posiciones[i-1].pose.position.x, self.posiciones[i-1].pose.position.y
            x2, y2 = self.posiciones[i].pose.position.x, self.posiciones[i].pose.position.y
            
            # Calculamos la distancia euclidiana entre los dos puntos consecutivos
            distancia = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            # Sumamos la distancia al total
            self.longitud_trayectoria += distancia

        # Obtenemos el tiempo actual (en segundos desde el inicio de la ejecución del nodo)
        tiempo_actual = rospy.get_time()
        
        # Almacenamos los datos (tiempo y longitud de la trayectoria) en la lista
        self.data.append((tiempo_actual, self.longitud_trayectoria))
        
        # Mostramos la longitud de la trayectoria en la consola
        rospy.loginfo("Longitud total de la trayectoria: %.2f metros" % self.longitud_trayectoria)

    def guardar_datos(self):
        """
        Función para guardar los datos de tiempo y longitud en un archivo .txt cuando se cierra el nodo.
        """
        # Definimos la ruta y el nombre del archivo
        ruta_archivo = os.path.expanduser('~/trayectoria_datos.txt')
        
        # Abrimos el archivo en modo escritura
        with open(ruta_archivo, 'w') as archivo:
            archivo.write("Tiempo (s)\tLongitud de Trayectoria (m)\n")  # Cabecera de las columnas
            # Escribimos cada entrada de la lista en el archivo
            for tiempo, longitud in self.data:
                archivo.write("%.2f\t%.2f\n" % (tiempo, longitud))
        
        rospy.loginfo("Datos guardados en: %s" % ruta_archivo)

if __name__ == '__main__':
    try:
        # Instanciamos la clase MedidorTrayectoria
        nodo = MedidorTrayectoria()
        rospy.spin()  # Mantiene el nodo en ejecución
    except rospy.ROSInterruptException:
        pass
    finally:
        # Guardamos los datos en el archivo al terminar el nodo
        nodo.guardar_datos()
