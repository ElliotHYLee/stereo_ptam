# pip libs
import numpy as np
import g2o

# custom libs
# none

class PoseGraphOptimization(g2o.SparseOptimizer):
    def __init__(self):
        super().__init__()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCholmodSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super().set_algorithm(solver)

    def optimize(self, max_iterations=20):
        super().initialize_optimization()
        super().optimize(max_iterations)

    def add_vertex(self, id, pose, fixed=False):
        v_se3 = g2o.VertexSE3()
        v_se3.set_id(id)
        v_se3.set_estimate(pose)
        v_se3.set_fixed(fixed)
        super().add_vertex(v_se3)

    def add_edge(self, vertices, measurement=None, information=np.identity(6), robust_kernel=None):
        edge = g2o.EdgeSE3()
        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)

        if measurement is None:
            measurement = (edge.vertex(0).estimate().inverse() * edge.vertex(1).estimate())
        edge.set_measurement(measurement)
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super().add_edge(edge)

    def set_data(self, keyframes, loops):
        super().clear()
        anchor=None
        for kf, *_ in loops:
            if anchor is None or kf < anchor:
                anchor = kf

        for i, kf in enumerate(keyframes):
            pose = g2o.Isometry3d(kf.orientation, kf.position)

            fixed = i == 0
            if anchor is not None:
                fixed = kf <= anchor
            self.add_vertex(kf.id, pose, fixed=fixed)

            if kf.preceding_keyframe is not None:
                self.add_edge(vertices=(kf.preceding_keyframe.id, kf.id), measurement=kf.preceding_constraint)

            if (kf.reference_keyframe is not None and
                kf.reference_keyframe != kf.preceding_keyframe):
                self.add_edge(vertices=(kf.reference_keyframe.id, kf.id), measurement=kf.reference_constraint)

        for kf, kf2, meas in loops:
            self.add_edge((kf.id, kf2.id), measurement=meas)


    def update_poses_and_points(self, keyframes, correction=None, exclude=set()):

        for kf in keyframes:
            if len(exclude) > 0 and kf in exclude:
                continue
            uncorrected = g2o.Isometry3d(kf.orientation, kf.position)
            if correction is None:
                vertex = self.vertex(kf.id)
                if vertex.fixed():
                    continue
                corrected = vertex.estimate()
            else:
                corrected = uncorrected * correction

            delta = uncorrected.inverse() * corrected
            if (g2o.AngleAxis(delta.rotation()).angle() < 0.02 and
                np.linalg.norm(delta.translation()) < 0.03):          # 1°, 3cm
                continue

            for m in kf.measurements():
                if m.from_triangulation():
                    old = m.mappoint.position
                    new = corrected * (uncorrected.inverse() * old)
                    m.mappoint.update_position(new)
                    # update normal ?
            kf.update_pose(corrected)
