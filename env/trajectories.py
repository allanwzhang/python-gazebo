import numpy as np
from multirotor.simulation import Multirotor

from multirotor.vehicle import VehicleParams, SimulationParams, PropellerParams
from multirotor.helpers import control_allocation_matrix


def make_trajectory(waypoints, spatial_resolution=0.01):
    pass



def get_vehicle_ability(vp: VehicleParams, sp: SimulationParams, max_tilt: float=np.pi/12):
    alloc, alloc_inverse = control_allocation_matrix(vp)
    I_roll, I_pitch, I_yaw = vp.inertia_matrix.diagonal()
    # T = I . d_omega
    # s = 0.5 d_omega t^2
    # time to reach max tilt (s):
    # t_m = sqrt(2 s / d_omega) = t = sqrt(2 s I / T)
    # max lateral thrust
    # T_l = Thrust sin(max_tilt)
    # max lateral acc
    # a = T_l / m
    # avg velocity = dist / time
    # x / (t_m + sqrt(2 (x/2) m / T_l) + 2 t_m + sqrt(2 (x/2) m / T_l))
    #      tilt   acc half way          tilt reverse   dec half way to stop



class Trajectory:


    def __init__(
        self, points: np.ndarray, vehicle: Multirotor=None, proximity: float=None,
        resolution: float=None    
    ):
        self.proximity = proximity
        self.vehicle = vehicle
        self.points = np.asarray(points)
        if resolution is not None:
            points = []
            for p1, p2 in zip(self.points[:-1], self.points[1:]):
                dist = np.linalg.norm(p2 - p1)
                num = int(dist / resolution) + 1
                points.extend(np.linspace(p1, p2, num=num, endpoint=True))
            self._points = points
        else:
            self._points = points


    def __len__(self):
        return len(self._points)


    def __getitem__(self, i: int):
        return self._points[i]


    def __iter__(self):
        if self.proximity is not None and self.vehicle is not None:
            for i in range(len(self)):
                while np.linalg.norm((self.vehicle.position - self[i])) >= self.proximity:
                    yield self[i]
        elif self.proximity is None:
            for i in range(len(self)):
                yield self[i]
