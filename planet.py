import matplotlib.pyplot as plt
import numpy as np
from numpy import exp, sqrt, pi

# GLOBAL PARAMETERS

class Planet:
    def __init__(self, name, g, altitude=1, rho=None, viscosity=None):
        self.name = name
        self.g = g
        self.altitude = altitude
        self.viscosity = viscosity
        if rho is None:
            self.rho = self.compute_rho(altitude)
        else:
            self.rho = rho
    
    def compute_rho(self, altitude):
        
        T = -31 - 0.000998 * altitude
        p = 0.699 * exp(-0.00009*altitude)
        
        rho = p/(0.1921 * (T + 273.1))
        return rho


