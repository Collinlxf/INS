import numpy as np

class GlobalVars:
    def __init__(self):
        self.Re = 6378137  # Earth's semi-major axis
        self.f = 1/298.257223563  # flattening
        self.wie = 7.292115e-5  # Earth's angular rate
        self.b = 6356752.3142  # semi-minor axis
        self.e = np.sqrt(2*self.f - self.f**2)  # first eccentricity
        self.e2 = self.e**2  # square of first eccentricity
        self.ep = np.sqrt(self.Re**2 - self.b**2) / self.b  # second eccentricity
        self.ep2 = self.ep**2  # square of second eccentricity
        self.g0 = 9.7803267714  # gravitational force
        self.deg = np.pi / 180  # arcdeg
        self.min = self.deg / 60  # arcmin
        self.sec = self.min / 60  # arcsec
        self.hur = 3600  # time hour (1hur=3600second)
        self.dps = self.deg / 1  # arcdeg / second
        self.dph = self.deg / self.hur  # arcdeg / hour
        self.dpss = self.deg / np.sqrt(1)  # arcdeg / sqrt(second)
        self.dpsh = self.deg / np.sqrt(self.hur)  # arcdeg / sqrt(hour)
        self.dphpsh = self.dph / np.sqrt(self.hur)  # (arcdeg/hour) / sqrt(hour)
        self.dph2 = self.dph / self.hur  # (arcdeg/hour) / hour

        # Initialize wm_1 and vm_1 to zero vectors
        self.wm_1 = np.zeros(3)
        self.vm_1 = np.zeros(3)

glv = GlobalVars()