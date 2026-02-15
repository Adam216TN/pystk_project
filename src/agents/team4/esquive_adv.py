class esquiveadv:
    
    def esquive_adv(self,obs):

        kart_pos = obs.get('karts_position', []) 
        if len(kart_pos) == 0:
            return False, 0.0, 0.0

        adv=kart_pos[0]
        devant=False
        if -2.5 <= adv[0] <=2.5 and  0.1<= adv[2]<=0.8:
            devant=True

        return devant,adv[0],adv[2]
    
