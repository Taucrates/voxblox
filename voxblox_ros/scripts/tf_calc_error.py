#!/usr/bin/env python
import rospy
import tf
import numpy as np
from tf.transformations import quaternion_matrix

class TFErrorNode:
    def __init__(self):
        rospy.init_node('tf_error_calculator')

        self.frame_ref = rospy.get_param('~frame_ref', 'map')
        self.frame_est = rospy.get_param('~frame_target', 'base_link')

        self.listener = tf.TransformListener()

        # Almacenamiento de trayectorias
        self.ref_positions = []
        self.est_positions = []
        self.ref_rotations = []
        self.est_rotations = []

        rospy.Timer(rospy.Duration(0.1), self.collect_data)  # 10Hz
        rospy.Timer(rospy.Duration(1.0), self.compute_metrics)  # Reporte cada segundo

        rospy.loginfo("Evaluando ATE entre %s y %s", self.frame_ref, self.frame_est)

    def collect_data(self, event):
        try:
            now = rospy.Time(0)

            self.listener.waitForTransform("mussol/nav", self.frame_ref, now, rospy.Duration(0.5))
            self.listener.waitForTransform("mussol/nav", self.frame_est, now, rospy.Duration(0.5))

            (t_ref, q_ref) = self.listener.lookupTransform("mussol/nav", self.frame_ref, now)
            (t_est, q_est) = self.listener.lookupTransform("mussol/nav", self.frame_est, now)

            self.ref_positions.append(np.array(t_ref))
            self.est_positions.append(np.array(t_est))

            self.ref_rotations.append(q_ref)
            self.est_rotations.append(q_est)

        except:
            pass

    # -------- Umeyama alignment --------
    def align_umeyama(self, X, Y):
        mu_X = np.mean(X, axis=0)
        mu_Y = np.mean(Y, axis=0)

        Xc = X - mu_X
        Yc = Y - mu_Y

        S = Xc.T @ Yc / len(X)

        U, D, Vt = np.linalg.svd(S)

        R = Vt.T @ U.T

        if np.linalg.det(R) < 0:
            Vt[2,:] *= -1
            R = Vt.T @ U.T

        t = mu_Y - R @ mu_X

        return R, t

    def compute_metrics(self, event):

        if len(self.ref_positions) < 5:
            return

        X = np.array(self.est_positions)
        Y = np.array(self.ref_positions)

        # 1️⃣ Alineación rígida
        R_align, t_align = self.align_umeyama(X, Y)

        X_aligned = (R_align @ X.T).T + t_align

        # 2️⃣ ATE
        pos_errors = np.linalg.norm(Y - X_aligned, axis=1)
        ate_rmse = np.sqrt(np.mean(pos_errors**2))
        ate_mean = np.mean(pos_errors)

        # 3️⃣ Error angular
        ang_errors = []

        for q_ref, q_est in zip(self.ref_rotations, self.est_rotations):

            R_ref = quaternion_matrix(q_ref)[:3,:3]
            R_est = quaternion_matrix(q_est)[:3,:3]

            R_err = R_ref.T @ R_est
            angle = np.arccos(
                np.clip((np.trace(R_err) - 1) / 2.0, -1.0, 1.0)
            )
            ang_errors.append(angle)

        ang_errors = np.array(ang_errors)
        ang_rmse = np.degrees(np.sqrt(np.mean(ang_errors**2)))
        ang_mean = np.degrees(np.mean(ang_errors))

        rospy.loginfo("======================================                 ")
        rospy.loginfo("ATE RMSE  : %.4f m                                     ", ate_rmse)
        rospy.loginfo("ATE Mean  : %.4f m                                     ", ate_mean)
        rospy.loginfo("Rot RMSE  : %.3f deg                                   ", ang_rmse)
        rospy.loginfo("Rot Mean  : %.3f deg                                   ", ang_mean)
        rospy.loginfo("Samples   : %d                                         ", len(X))
        rospy.loginfo("======================================               \n")


if __name__ == '__main__':
    try:
        TFErrorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass