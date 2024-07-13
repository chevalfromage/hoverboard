
# ====== POUR WINDOWS=====
import os
os.chdir('C:/your path')
# ====== POUR WINDOWS=====

import numpy as np
from matplotlib import pyplot as plt
import solve_linear_equation as slv
# data=pd.read_csv("output.csv")


from numpy import genfromtxt
#my_data = genfromtxt("output.csv", delimiter=',')
my_data = genfromtxt("output.csv", delimiter=',')
print(my_data)


vR = my_data.T[0][1:].reshape((1,-1))
vL = my_data.T[1][1:].reshape((1,-1))
commandR = my_data.T[2][1:].reshape((1,-1)) - 1550 ## Pour une réponse impultionelle, l'origine doit être à zéro
commandL = my_data.T[3][1:].reshape((1,-1))


Hz = 1
freq = 100 * Hz
period = 1/freq

Ltemps = np.array(range(len(vR)))*period

fig, (axe_command, axe_vitesse) = plt.subplots(2)

Ltemps = np.linspace(0,1,commandR.shape[1])
axe_command.plot(Ltemps,commandR[0],label="commandR")
axe_command.plot(Ltemps,commandL[0],label="commandL")
axe_vitesse.plot(Ltemps,vR[0],label="vR")
axe_vitesse.plot(Ltemps,vL[0],label="vL")

axe_command.legend(loc="upper left")
axe_vitesse.legend(loc="upper left")
plt.tight_layout()
plt.show()



class Model_une_roue(slv.LinearModel):
    """
    Model :
    ========

           [ wg_km2 ]
           [ wg_km1 ]
    XX_k = [ wg_k   ] -> vitesse une roue

    UU_k = | dk | -> gyroscompe droit

    XX_kp1 = f(XX_k, UU_k)
           = E @ XX_k + F @ UU_k

             [ 0,  1,  0]          [0]
    XX_kp1 = [ 0,  0,  1] @ XX_k + [0] @ UU_k
             [a0, a1, a2]          [c]
    """
    def parameter_sizes(ezlf):
        size_for_symetric_matirices_parameter = []
        size_for_matirices_parameter = [(1,3), (1,1)]
        return [size_for_symetric_matirices_parameter, size_for_matirices_parameter]
    def E(self, list_of_symetric_parameters, list_of_non_symetrice_paramters):
        A, c = list_of_non_symetrice_paramters
        E = np.vstack([
            np.array([[0, 1, 0]]),
            np.array([[0, 0, 1]]),
            A
        ])
        return E
    def F(self, list_of_symetric_parameters, list_of_non_symetrice_paramters):
        A, c = list_of_non_symetrice_paramters
        F = np.vstack([
            np.array([[0]]),
            np.array([[0]]),
            c
        ])
        return F
    def XX_mask(self):
        return np.array(
            [[0],
             [0],
             [1]]
        )


#
    #        [ wg_km2 ]
    #        [ wg_km1 ]
    # XX_k = [ wg_k   ] -> vitesse une roue

XXX = np.vstack([
    vR[:,:-2],    # wg_km2
    vR[:,1:-1],   # wg_km1
    vR[:,2:]      # wg_k
])

UUU = np.hstack([
    commandR[:,2:]   # uL_k
])

print(XXX)
print(UUU)
print(XXX.shape)
print(UUU.shape)

model_une_roue = Model_une_roue()

print(vR[:, 100:])

# On identifie le modèle
print("\nIdentification : ")
E, F, cov_error, rms_error, det = model_une_roue.identify(XXX, UUU)
print(f"E : \n{E}")
print(f"F : \n{F}")
print(f"cov error : \n{cov_error}")
print(f"rms error : \n{rms_error}")
print(f"det : {det}")


print("\n\nSimulation :")


XX_0 = XXX[:,0:1]
XXX_sim = [XX_0]
for i in range(UUU.shape[1]-1) :
    XX_k = XXX_sim[-1]
    UU_k = UUU[:, i:i+1]
    XX_kp1 = model_une_roue.f_model(XX_k, UU_k, [], model_une_roue.list_of_non_symetrice_paramters)
    XXX_sim.append(XX_kp1)
XXX_sim = np.hstack(XXX_sim)

time = np.array(range(XXX_sim.shape[1]))*period

fig, axs = plt.subplots(2, 1, sharex=True)
axs[0].plot( time, XXX[2,:], linestyle = "", marker =".", markersize = 2, label="vitesse_k" )
axs[0].plot( time, XXX_sim[2,:], linestyle = "", marker =".", markersize = 2, label="vitesse_k" )
axs[0].set_ylabel('vitesse angulaire (rad/s)')
axs[1].plot( time, UUU[0,:], linestyle = "", marker =".", markersize = 2, label="command_k" )
axs[1].set_ylabel('inclinaison (rad)')
axs[1].set_xlabel('temps')
for ax in axs:
    ax.legend()
plt.show()



