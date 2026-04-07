from omegaconf import DictConfig
import numpy as np
from utils.track_utils import compute_curvature

class SpeedController:
    
    """
    Module SpeedController : Gère la logique d'accélération
    """

    def __init__(self,config : DictConfig) -> None:
        """Initialise les variables d'instances de l'agent."""
        
        self.c = config
        """@private"""
        self.g = self.c.speed_gain
        """@private"""
        self.amax = self.c.acceleration_max
        """@private"""
    
    def reset(self) -> None:
        """Réinitialise les variables d'instances de l'agent expert"""
        pass
    
    def manage_speed(self,obs:dict) -> tuple[float,bool]:
        """
        Gère l'accélération.

        Args:
            
            obs(dict) : Les données fournies par le simulateur.
        
        Returns:
            
            float: Valeur représentant l'accélération.
            bool: Variable permettant d'activer ou non le brake.

        """
        points = obs.get("paths_start",[]) # On récupère la liste des points
        dx = points[3][0] #on prend le decalage latéral x du troisieme point devant l'agent

        a = abs(dx)
        #print("décal x = ",a)

        if a < 4.0:
            return 0.98, False
        return 1.0, False
        if a < 6.0 :
            return 1.0, False

        if a > 10.0:
            return 0.05, True

        if (a > 5.0 and a < 8.0):
            return 0.55, False

        if (a > 8.0 and a < 10.0):
            return 0.3, True

        return 1.0, False
