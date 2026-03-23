from utils.track_utils import compute_curvature, compute_slope
from agents.kart_agent import KartAgent
from .steering import Steering
from .AgentRescue import AgentRescue
from .speed import SpeedController
from .AgentNitro import AgentNitro
from .AgentBanana import AgentBanana
from .AgentEsquiveAdv import AgentEsquiveAdv
from .AgentDrift import AgentDrift
from .AgentItems import AgentItems
from .AgentEdge import AgentEdge
from omegaconf import OmegaConf
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent # On obtient le chemin absolu vers notre fichier agent4
CONFIG_PATH = BASE_DIR / "configuration.yaml" # On dit que notre fichier de config se trouve aussi ici

__all__ = ["Agent4"]


class BasePilot:
    """
    Module Pilote de Base (Le cœur de l'agent).
    Gère la direction normale, la vitesse, la nitro et l'utilisation des items.
    """

    def __init__(self, config):
        """Initialise les variables d'instances du pilote de base."""
        self.c = config.main_agent
        self.steering = Steering(config.steering)
        self.speedcontroller = SpeedController(config.speed)
        self.expert_nitro = AgentNitro(config.nitro)
        self.expert_items = AgentItems(config.powerup_type, config.steering)

    def reset(self) -> None:
        """Réinitialise les outils de base."""
        self.steering.reset()
        self.speedcontroller.reset()
        self.expert_nitro.reset()
        self.expert_items.reset()

    def choose_action(self, obs: dict) -> dict:
        """Calcule la trajectoire normale et génère le dictionnaire initial."""
        points = obs.get("paths_start", [])
        
        # Ligne d'arrivée proche, on accélère à fond
        if len(points) <= self.c.seuil_lenpoints: 
            return {
                "acceleration": 1.0,
                "steer": 0.0,
                "brake": False,
                "drift": False,
                "nitro": True,
                "rescue": False,
                "fire": False,
            }
        
        target = points[self.c.path_lookahead]
        gx = target[0]
        gz = target[2]

        distance = float(obs.get("distance_down_track", [0.0])[0])
        vel = obs.get("velocity", [0.0, 0.0, 0.0])
        energy = float(obs.get("energy", [0.0])[0])

        gain_volant = self.c.default_gain
        steering = self.steering.manage_pure_pursuit(gx, gz, gain_volant)
        
        acceleration, brake = self.speedcontroller.manage_speed(obs)
        nitro = self.expert_nitro.manage_nitro(obs, steering, energy)
        
        # Au depart on avance tout droit pour eviter de se cogner contre les adversaires
        if distance <= self.c.seuil_distance:
            return {
                "acceleration": 1.0,
                "steer": 0.0,
                "brake": False,
                "drift": False,
                "nitro": False,
                "rescue": False,
                "fire": False,
            }

        # Mécanisme Anti Vibration
        epsilon = self.c.epsilon
        road_straight = abs(points[2][0]) < self.c.seuil_road_straight
        if road_straight and abs(steering) <= epsilon:
            steering = 0.0

        fire, steering = self.expert_items.use_items(obs, steering)

        # CRÉATION DU DICTIONNAIRE DE BASE
        return {
            "acceleration": acceleration,
            "steer": steering,
            "brake": brake,
            "drift": False,  # Initialisé à False, l'AgentDrift le modifiera si besoin
            "nitro": nitro,
            "rescue": False, # Initialisé à False, l'AgentRescue le modifiera si besoin
            "fire": fire,
        }


