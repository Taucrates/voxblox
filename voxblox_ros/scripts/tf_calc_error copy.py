#!/usr/bin/env python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Point
from tf.transformations import quaternion_multiply, quaternion_inverse

class TFErrorNode:
    def __init__(self):
        rospy.init_node('tf_error_calculator')

        # Parámetros: nombres de los frames a comparar
        self.frame_ref = rospy.get_param('~frame_ref', 'map')
        self.frame_target = rospy.get_param('~frame_target', 'base_link')

        self.listener = tf.TransformListener()
        
        # Variables para la media acumulada
        self.count = 0
        self.total_pos_error = 0.0
        self.total_ang_error = 0.0

        rospy.Timer(rospy.Duration(0.1), self.compute_error) # 10Hz
        rospy.loginfo("Calculando error entre %s y %s", self.frame_ref, self.frame_target)

    def compute_error(self, event):
        try:
            # 1. Obtener la transformada entre los dos frames
            # Queremos la relación relativa entre ellos
            now = rospy.Time(0)
            self.listener.waitForTransform(self.frame_ref, self.frame_target, now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform(self.frame_ref, self.frame_target, now)

            # 2. Calcular Error de Posición (Distancia al origen [0,0,0])
            # Si fueran idénticos, la traslación sería cero.
            pos_error = np.linalg.norm(trans)

            # 3. Calcular Error de Orientación
            # El ángulo de la rotación necesaria para alinear ambos frames
            # La magnitud del error angular se saca del componente 'w' del cuaternión
            # ángulo = 2 * acos(q_w)
            angle_error = 2 * np.arccos(min(1.0, abs(rot[3]))) 

            # 4. Actualizar medias
            self.count += 1
            self.total_pos_error += pos_error
            self.total_ang_error += angle_error

            mean_pos = self.total_pos_error / self.count
            mean_ang = np.degrees(self.total_ang_error / self.count) # En grados para humanos
            
            rospy.loginfo_throttle(1, "Last Error -> Pos: {:.4f}m | Ang: {:.2f} deg      ".format(pos_error, np.degrees(angle_error)))
            rospy.loginfo_throttle(1, "AVG Error -> Pos: {:.4f}m | Ang: {:.2f} deg      \n".format(mean_pos, mean_ang))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Esperando TFs... %s", e)

if __name__ == '__main__':
    try:
        TFErrorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass