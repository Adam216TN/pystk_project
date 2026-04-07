import numpy as np
from omegaconf import DictConfig

class SpeedController:
    """
    Module SpeedController : Gère la logique d'accélération basée sur l'écart latéral.
    """

    def __init__(self, config: DictConfig) -> None:
        """Initialise les variables d'instances de l'agent expert."""
        self.c = config

    def reset(self) -> None:
        """Réinitialise les variables d'instances de l'agent expert."""
        pass

    def manage_speed(self, obs: dict) -> tuple[float, bool]:
        """
        Gère l'accélération en fonction de l'écart latéral du chemin.

        Args:
            obs (dict) : Les données fournies par le simulateur.
        
        Returns:
            tuple[float, bool]: Valeur de l'accélération (float) et activation du frein (bool).
        """
        # 1. Extraction sécurisée de la vitesse (composante z)
        vel = obs.get("velocity", [0.0, 0.0, 0.0])
        vel = np.asarray(vel).reshape(-1)
        speed = float(vel[2]) if vel.size > 2 else 0.0

        # 2. Extraction du décalage latéral (x) du 4ème point (index 3)
        points = obs.get("paths_start", [])
        if len(points) > 3:
            dx = points[3][0]
        else:
            dx = 0.0 
            
        a = abs(dx)

        # 3. Récupération du seuil depuis ta configuration
        v_seuil = self.c.vitesse_seuil

        # 4. Logique d'accélération et de freinage
        if a < 6.0 and speed > v_seuil:
            return 1.0, False

        if a > 10.0 and speed > v_seuil:
            return 0.05, True

        if (5.0 < a < 8.0) and speed > (v_seuil - 2.0):
            return 0.55, False

        if (8.0 < a < 10.0) and speed > (v_seuil - 1.0):
            return 0.3, True

        # Comportement par défaut
        return 1.0, False