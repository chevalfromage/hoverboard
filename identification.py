import solve_linear_equation as slv
import numpy as np
from matplotlib import pyplot as plt


class Model_complet_2_roues_ordre_1(slv.LinearModel):
    """
    Model :
    ========

           [ wg_km1 ]
    XX_k = [ wg_k   ] -> vitesse roue gauche
           [ wd_km1 ]
           [ wd_k   ] -> vitesse roue droite

           | gk | -> gyroscope gauche
    UU_k = | dk | -> gyroscompe droit

    XX_kp1 = f(XX_k, UU_k)
           = E @ XX_k + F @ UU_k

             [ 0,  1,  0,  0]          [0,   0]
    XX_kp1 = [a0, a1, a2, a3] @ XX_k + [c1, c2] @ UU_k
             [ 0,  0,  0,  1]          [0,   0]
             [b0, b1, b2, b3]          [d1, d2]

    """
    def parameter_sizes(self):
        size_for_symetric_matirices_parameter = []
        size_for_matirices_parameter = [(1,4), (1,4), (1,2), (1,2)]
        return [size_for_symetric_matirices_parameter, size_for_matirices_parameter]
    def E(self, list_of_symetric_parameters, list_of_non_symetrice_paramters):
        A, B, c, d = list_of_non_symetrice_paramters
        E = np.vstack([
            np.array([[0, 1, 0, 0]]),
            A,
            np.array([[0, 0, 0, 1]]),
            B
        ])
        return E
    def F(self, list_of_symetric_parameters, list_of_non_symetrice_paramters):
        A, B, c, d = list_of_non_symetrice_paramters
        F = np.vstack([
            np.array([[0, 0]]),
            c,
            np.array([[0, 0]]),
            d
        ])
        return F
    def XX_mask(self):
        return np.array(
            [[0],
             [1],
             [0],
             [1]]
        )
    # Uncomment to ignore some parameter in order to identify a model where all
    # wheel are independent.
    #def parameter_mask(self):
    #    sym_mask = []
    #    non_sym_mask = [
    #        np.array([[1, 1, 0, 0]]),  # A
    #        np.array([[0, 0, 1, 1]]),  # B
    #        np.array([[1, 0]]),  # C
    #        np.array([[0, 1]])   # D
    #    ]
    #    return [sym_mask, non_sym_mask]

print("")
print("")
print("")
print("#########################################################")
print("Exemple Modèle deux roues : ")
print("#########################################################")

# Definition du modèle
model = Model_complet_2_roues_ordre_1()

print( "\nOn vérifie le structure du modèle :")
print("E : ")
print(
    model.E(
        [],
        [
            np.array([[1,2,3,4]]), np.array([[5,6,7, 8]]),
            np.array([[9, 10]]), np.array([[11, 12]])
        ]
    )
)
print("F : ")
print(
    model.F(
        [],
        [
            np.array([[1,2,3,4]]), np.array([[5,6,7, 8]]),
            np.array([[9, 10]]), np.array([[11, 12]])
        ])
)

# On récupère les données expérimentales !
data_size = 1000
XXX = np.random.normal(0, 2, (4, data_size)) # Les roues
UUU = np.random.normal(0, 2, (2, data_size)) # Les gyros
print("\nData : ")
print(f"XXX: \n{XXX}")
print(f"UUU: \n{UUU}")

# On identifie le modèle 
print("\nIdentification : ")
E, F, cov_error, rms_error, det = model.identify(XXX, UUU)
print(f"E : \n{E}")
print(f"F : \n{F}")
print(f"cov error : \n{cov_error}")
print(f"rms error : \n{rms_error}")
print(f"det : {det}")





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


print("")
print("")
print("")
print("#########################################################")
print("Exemple Modèle une roue : ")
print("#########################################################")

model_une_roue = Model_une_roue()

print( "\nOn vérifie le structure du modèle :")
print("E : ")
print(
    model_une_roue.E(
        [],
        [
            np.array([[1,2,3]]),
            np.array([[9]])
        ]
    )
)
print("F : ")
print(
    model_une_roue.F(
        [],
        [
            np.array([[1,2,3]]), 
            np.array([[9]])
        ])
)


# On récupère les données expérimentales !
data_size = 1000
XXX = np.random.normal(0, 2, (3, data_size)) # Les roues
UUU = np.random.normal(0, 2, (1, data_size)) # Les gyros
print("\nData : ")
print(f"XXX: \n{XXX}")
print(f"UUU: \n{UUU[:,:3]} ...")

# On identifie le modèle 
print("\nIdentification : ")
E, F, cov_error, rms_error, det = model_une_roue.identify(XXX, UUU)
print(f"E : \n{E}")
print(f"F : \n{F}")
print(f"cov error : \n{cov_error}")
print(f"rms error : \n{rms_error}")
print(f"det : {det}")




print("")
print("")
print("")
print("#########################################################")
print("Exemple Modèle deux roues ordre 2 : ")
print("#########################################################")



class Model_complet_2_roues_ordre_2(slv.LinearModel):
    """
    Model :
    ========

           [ wg_km2 ]
           [ wg_km1 ]
    XX_k = [ wg_k   ] -> vitesse roue gauche
           [ wd_km2 ]
           [ wd_km1 ]
           [ wd_k   ] -> vitesse roue droite

           | gk | -> gyroscope gauche
    UU_k = | dk | -> gyroscompe droit

    XX_kp1 = f(XX_k, UU_k)
           = E @ XX_k + F @ UU_k

             [ 0,  1,  0,  0,  0,  0]          [0,   0]
             [ 0,  0,  1,  0,  0,  0]          [0,   0]
    XX_kp1 = [a0, a1, a2, a3, a4, a5] @ XX_k + [c1, c2] @ UU_k
             [ 0,  0,  0,  0,  1,  0]          [0,   0]
             [ 0,  0,  0,  0,  0,  1]          [0,   0]
             [b0, b1, b2, b3, b4, b5]          [d1, d2]

    """
    def parameter_sizes(self):
        size_for_symetric_matirices_parameter = []
        size_for_matirices_parameter = [(1,6), (1,6), (1,2), (1,2)]
        return [size_for_symetric_matirices_parameter, size_for_matirices_parameter]
    def E(self, list_of_symetric_parameters, list_of_non_symetrice_paramters):
        A, B, c, d = list_of_non_symetrice_paramters
        E = np.vstack([
            np.array([[0, 1, 0, 0, 0, 0]]),
            np.array([[0, 0, 1, 0, 0, 0]]),
            A,
            np.array([[0, 0, 0, 0, 1, 0]]),
            np.array([[0, 0, 0, 0, 0, 1]]),
            B
        ])
        return E
    def F(self, list_of_symetric_parameters, list_of_non_symetrice_paramters):
        A, B, c, d = list_of_non_symetrice_paramters
        F = np.vstack([
            np.array([[0, 0]]),
            np.array([[0, 0]]),
            c,
            np.array([[0, 0]]),
            np.array([[0, 0]]),
            d
        ])
        return F
    def XX_mask(self):
        return np.array(
            [[0],
             [0],
             [1],
             [0],
             [0],
             [1]]
        )
    # Uncomment to ignore some parameter in order to identify a model where all
    # wheel are independent.
    #def parameter_mask(self):
    #    sym_mask = []
    #    non_sym_mask = [
    #        np.array([[1, 1, 1, 0, 0, 0]]),  # A
    #        np.array([[0, 0, 0, 1, 1, 1]]),  # B
    #        np.array([[1, 0]]),  # C
    #        np.array([[0, 1]])   # D
    #    ]
    #    return [sym_mask, non_sym_mask]

model_2_roues_ordre_2 = Model_complet_2_roues_ordre_2()

print( "\nOn vérifie le structure du modèle :")
print("E : ")
print(
    model_2_roues_ordre_2.E(
        [],
        [
            np.array([[1, 2, 3, 4, 5, 6]]),  # A
            np.array([[10, 20, 30, 40, 50, 60]]), # B
            np.array([[90, 100]]),  # C
            np.array([[91, 101]])  # D
        ]
    )
)
print("F : ")
print(
    model_2_roues_ordre_2.F(
        [],
        [
            np.array([[1, 2, 3, 4, 5, 6]]),  # A
            np.array([[10, 20, 30, 40, 50, 60]]), # B
            np.array([[90, 100]]),  # C
            np.array([[91, 101]])  # D
        ])
)


# On récupère les données expérimentales !
data_size = 1000
XXX = np.random.normal(0, 2, (6, data_size)) # Les roues
UUU = np.random.normal(0, 2, (2, data_size)) # Les gyros
print("\nData : ")
print(f"XXX: \n{XXX}")
print(f"UUU: \n{UUU[:,:3]} ...")

# On identifie le modèle 
print("\nIdentification : ")
E, F, cov_error, rms_error, det = model_2_roues_ordre_2.identify(XXX, UUU)
print(f"E : \n{E}")
print(f"F : \n{F}")
print(f"cov error : \n{cov_error}")
print(f"rms error : \n{rms_error}")
print(f"det : {det}")










print("")
print("")
print("")
print("#########################################################")
print("Simulation avec bruit + identification : ")
print("#########################################################")
print("")

# Definition du modèle
print("\nModèle à identifier : ")
model = Model_complet_2_roues_ordre_1()
A = np.array([[ 1/2, 1/4, 1/8, -1/16]])
B = np.array([[-1/8,  1/16, 1/2, 1/4]])
C = np.array([[ 1/2,  1/4 ]])
D = np.array([[ 1/4,  1/3 ]])
print(f"\nE : \n{model.E([], [A, B, C, D])}")
print(f"\nF : \n{model.F([], [A, B, C, D])}")

UUU = np.hstack(
    [ np.array([[0], [0]]) for i in range(100) ] + 
    [ np.array([[1], [0]]) for i in range(100) ] +
    [ np.array([[0], [0]]) for i in range(100) ] +
    [ np.array([[0], [1]]) for i in range(100) ] +
    [ np.array([[0], [0]]) for i in range(100) ]
)

print("\n\nSimulation avec bruit : cf. graphique")

np.random.seed(23298787)
## Caractéristique du Bruit 
mean = np.array([0, 0, 0, 0])
covariance = 2*10**-5 * np.array(
    [[   0,    0,    0,    0], 
     [   0,    1,    0, -1/2],
     [   0,    0,    0,    0],
     [   0, -1/2,    0,    1]]
)

XX_0 = np.array([[0], [0], [0], [0]])
XXX = [XX_0]
for i in range(UUU.shape[1]-1) :
    XX_k = XXX[-1]
    UU_k = UUU[:, i:i+1]
    bruit = np.random.multivariate_normal(mean, covariance).reshape((4, 1))
    XX_kp1 = model.f_model(XX_k, UU_k, [], [A,B,C,D]) + bruit
    XXX.append(XX_kp1)
XXX = np.hstack(XXX)

time = range(XXX.shape[1])

fig, axs = plt.subplots(2, 1, sharex=True)
axs[0].plot( time, XXX[1,:], linestyle = "", marker =".", markersize = 2, label="wg_k" )
axs[0].plot( time, XXX[3,:], linestyle = "", marker =".", markersize = 2, label="wd_k" )
axs[0].set_ylabel('vitesse angulaire (rad/s)')
axs[1].plot( time, UUU[0,:], linestyle = "", marker =".", markersize = 2, label="g_k" )
axs[1].set_ylabel('inclinaison (rad)')
axs[1].plot( time, UUU[1,:], linestyle = "", marker =".", markersize = 2, label="d_k" )
axs[1].set_xlabel('temps')
for ax in axs:
    ax.legend()
plt.show()


# On identifie le modèle 
print("\n\nIdentification : ")
E, F, cov_error, rms_error, det = model.identify(XXX, UUU)
print(f"\nE : \n{E}")
print(f"\nF : \n{F}")
print(f"\ncovariance error : \n{cov_error}")
print(f"rms error : \n{rms_error}")
print(f"\ndet : {det}")




