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

    def reset(self) -> None:
        """Réinitialise les outils de base."""
        self.steering.reset()
        self.speedcontroller.reset()
    

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

        gain_volant = self.c.default_gain
        steering = self.steering.manage_pure_pursuit(gx, gz, gain_volant)
        
        acceleration, brake = self.speedcontroller.manage_speed(obs)
        
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
            "drift": False,   
            "nitro": False,
            "rescue": False, 
            "fire": False,
        }


class Agent4(KartAgent):
    """
    Module Agent4 : Agent coordinateur (Pattern Decorator).
    Fait appel aux différents wrappers pour gérer la logique générale de pilotage.
    """

    def __init__(self, env, path_lookahead=2):
        """Initialise les variables d'instances de l'agent."""
        
        super().__init__(env)

        self.conf = OmegaConf.load(str(CONFIG_PATH)) # On charge le fichier de config
        """@private"""
        self.c = self.conf.main_agent
        """@private"""
        self.path_lookahead = self.c.path_lookahead
        """@private"""
        self.obs = None
        """@private"""
        self.isEnd = False
        """@private"""
        self.name = "The Winners"
        """@private"""
        
        
        # Architecture en Wrappers
        
        
        # 1. Le Noyau (gère volant, pédales)
        pilote = BasePilot(self.conf)
        
        #2.le Nitro
        pilote=AgentNitro(pilote,self.conf.nitro)
        
        #3.Les items
        pilote=AgentItems(pilote,self.conf.powerup_type,self.conf.steering)
        
        # 4. Le Drift
        pilote = AgentDrift(pilote, self.conf.drift)
        
        # 5. L'Esquive Adversaire 
        pilote = AgentEsquiveAdv(pilote, self.conf.opponent, self.conf.steering)
        
        # 6. L'Esquive Banane 
        pilote = AgentBanana(pilote, self.conf.banana, self.conf.steering)
        
        # 7. La Sécurité Bord de piste (Empêche de tomber dans le vide en esquivant)
        pilote = AgentEdge(pilote, self.conf.edge, self.conf.steering)
        
        # 8. Le Rescue (annule TOUT si on est bloqué dans un mur)
        self.final_pilot = AgentRescue(pilote, self.conf.rescue)
        
        """@private"""
        # (Les experts ne sont plus appelés individuellement ici, ils sont emboîtés)
        
    def reset(self) -> None:
        """Réinitialise les variables d'instances de l'agent en début de course."""
        self.obs, _ = self.env.reset()
        self.isEnd = False
        
        # Réinitialiser la dernière poupée (Rescue) va réinitialiser toutes les autres en cascade !
        self.final_pilot.reset()
        
    def endOfTrack(self) -> bool:
        """Indique si la course est fini."""
        return self.isEnd

    def choose_action(self,obs : dict) -> dict:
        """
        Renvoie les différentes actions à réaliser.
        La décision traverse toutes les couches de wrappers.

        Args:
            obs(dict) : Les données de télémétrie fournies par le simulateur.

        Returns:
            dict : Le dictionnaire d'actions (accélération, direction, nitro, etc.).
        """
        
        return self.final_pilot.choose_action(obs)