import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# =====================================================================
# 1. FIZIČKI PARAMETRI ROBOTA
# =====================================================================
M  = 0.5      # Masa točkova i baze (kg)
m  = 0.4      # Masa tela robota (kg)
L  = 0.15     # Visina do centra mase (metri)
Ip = 0.003    # Rotaciona inercija tela (kg*m^2)
g  = 9.81     # Gravitacija (m/s^2)
r  = 0.05     # Radijus točka (metri)

# 0.5 Nm je više nego dovoljno za robota od 400 grama
MAX_TORQUE = 0.5  

# =====================================================================
# 2. PID KONTROLER (Sa ispravljenim smerom i prilagođenim pojačanjima)
# =====================================================================
class FiksniPID:
    def __init__(self):
        # Mnogo pitomije vrednosti prilagođene za tvoj mali/lagan robot
        self.Kp = 4  
        self.Kd = .5   
        self.Ki = 0   # Minimalan integral samo za statičku grešku
        self.integral = 0.0

    def racunaj_snagu(self, ugao, brzina_ugla, dt):
        # ISPRAVLJENO: Ako pada napred (ugao > 0), moment MORA biti pozitivan da točkovi idu napred
        self.integral += ugao * dt
        self.integral = np.clip(self.integral, -1.0, 1.0)
        
        trazena_snaga = (self.Kp * ugao) + (self.Kd * brzina_ugla)
        if ugao < 2.5 or ugao > -2.5:
            trazena_snaga += self.Ki * ugao
        
        return np.clip(trazena_snaga, -MAX_TORQUE, MAX_TORQUE)

pid = FiksniPID()
FIKSNI_DT = 0.01  # 10ms petlja

# =====================================================================
# 3. JEDNAČINE FIZIKE
# =====================================================================
def fizika_sistema(t, stanje):
    x, x_dot, ugao, brzina_ugla = stanje
    
    # Računamo moment sile motora
    T = pid.racunaj_snagu(ugao, brzina_ugla, FIKSNI_DT)
    
    A11 = M + m
    A12 = m * L * np.cos(ugao)
    B1  = (T / r) + m * L * (brzina_ugla**2) * np.sin(ugao)
    
    A21 = m * L * np.cos(ugao)
    A22 = Ip + m * (L**2)
    B2  = -T + m * g * L * np.sin(ugao)
    
    det = (A11 * A22) - (A12 * A21)
    
    ubrzanje_tockova = ((B1 * A22) - (A12 * B2)) / det
    ubrzanje_pada    = ((A11 * B2) - (B1 * A21)) / det
    
    return [x_dot, ubrzanje_tockova, brzina_ugla, ubrzanje_pada]

# =====================================================================
# 4. POKRETANJE SIMULACIJE
# =====================================================================
vreme_trajanja = 10  # Skraćeno vreme jer se lagan robot smiri ultra brzo
vremenski_koraci = np.arange(0, vreme_trajanja, FIKSNI_DT)

# Početno stanje: nagnut 12 stepeni unapred
pocetno_stanje = [0.0, 0.0, np.radians(12.0), 0.0]

istorija_vremena = []
istorija_ugla = []
istorija_pozicije = []

trenutno_stanje = pocetno_stanje

for t in vremenski_koraci:
    istorija_vremena.append(t)
    istorija_ugla.append(np.degrees(trenutno_stanje[2])) 
    istorija_pozicije.append(trenutno_stanje[0])
    
    korak_solvera = solve_ivp(fizika_sistema, [t, t + FIKSNI_DT], trenutno_stanje, method='RK45')
    trenutno_stanje = korak_solvera.y[:, -1]

# =====================================================================
# 5. CRTANJE GRAFIKA
# =====================================================================
plt.figure(figsize=(10, 5))

plt.subplot(2, 1, 1)
plt.plot(istorija_vremena, istorija_ugla, 'r-', linewidth=2, label='Ugao robota (θ)')
plt.axhline(0, color='black', linestyle='--')
plt.ylabel('Ugao (Stepeni)')
plt.title('KONAČNO ISPRAVLJENO: Robot hvata balans i smiruje se na 0°')
plt.grid(True)
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(istorija_vremena, istorija_pozicije, 'b-', linewidth=2, label='Pozicija točkova (x)')
plt.axhline(0, color='black', linestyle='--')
plt.xlabel('Vreme (Sekunde)')
plt.ylabel('Pozicija na podu (Metri)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
