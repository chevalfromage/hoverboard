"""
Copyright 2023 Adrien Boussicault (boussica@labri.fr)

This file is part of Motor Control Tools.

Motor Control Tools is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the Free
Software Foundation, either version 3 of the License, or (at your option) any
later version.

Motor Control Tools is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
details.

You should have received a copy of the GNU General Public License along with
Motor Control Tools. If not, see <https://www.gnu.org/licenses/>.
"""

import numpy as np
from copy import deepcopy


def Eij(i, j, n, m):
    """
    >>> Eij(1, 2, 3, 4)
    array([[0., 0., 0., 0.],
           [0., 0., 1., 0.],
           [0., 0., 0., 0.]])
    """
    return np.array([
        [
            1.0 if kk == i and ll == j else 0
            for ll in range(m)
        ]
        for kk in range(n)
    ])


def symEij(i, j, n):
    """
    >>> symEij(1, 2, 3)
    array([[0., 0., 0.],
           [0., 0., 1.],
           [0., 1., 0.]])
    """
    if i == j:
        return Eij(i, j, n, n)
    return Eij(i, j, n, n) + Eij(j, i, n, n)


def to_1D(T, mask):
    """
    >>> A = np.array([[1,2],[3,4],[5,6]])
    >>> mask = np.array([[1,0],[1,0],[1,1]])
    >>> to_1D(A, mask).T
    array([[1, 3, 5, 6]])

    >>> A = np.array([[1,2],[3,4],[5,6]])
    >>> mask = np.array([[1,0],[0,1],[0,1]])
    >>> to_1D(A, mask).T
    array([[1, 4, 6]])
    """
    assert(T.shape == mask.shape)
    T = T.reshape(-1)
    mask = mask.reshape(-1)
    res = []
    for i in range(mask.size):
        if mask[i] > 0:
            res.append(T[i])
    res = np.array(res)
    return res.reshape((-1, 1))


def solve_linear_equation(
    A, fct,
    symetric_n=None, rectangular_n=None,
    symetric_mask=None, rectangular_mask=None,
    output_mask=None
):
    """
    A = fct([S1, S2, ...], [M1, M2, ...])

    S1, S2, ... are symetric of size in symetric_n
    M1, M2, ... are rectangular of geometry (x,y sizes) in rectangular_n

    >>> def fct(L1, L2):
    ...     S1, S2, = L1
    ...     M1, M2, = L2
    ...     return (
    ...         np.array([
    ...             [-1,  4],
    ...             [-3,  6],
    ...             [ 1,  1],
    ...             [ 0, -1],
    ...             [ 1,  2],
    ...             [ 4,  1]
    ...         ]) @ S1 @ np.array([
    ...             [-1, 4, 7,  1],
    ...             [-3, 6, 9, -2]
    ...         ])
    ...         +
    ...         np.array([
    ...             [-2,  1,  1],
    ...             [-5,  2,  3],
    ...             [ 1, -2,  0],
    ...             [ 1,  0,  2],
    ...             [ 2,  1,  2],
    ...             [-3,  2, -1]
    ...         ]) @ S2 @ np.array([
    ...             [ -2, 3, 1,  0],
    ...             [ -2, 3, 1, -1],
    ...             [  2, 1, 4,  2]
    ...         ])
    ...         +
    ...         np.array([
    ...             [-1,  2,  3],
    ...             [ 1,  4,  1],
    ...             [ 4,  1, -4],
    ...             [-2, -1,  0],
    ...             [ 0,  3,  1],
    ...             [ 1,  0,  2]
    ...         ]) @ M1 @ np.array([
    ...             [0, 4, 1,  3],
    ...             [-1, 2, 3, 0]
    ...         ])
    ...         +
    ...         np.array([
    ...             [-2,  2,  1],
    ...             [ 4,  2,  1],
    ...             [ 0,  0,  5],
    ...             [ 1,  1,  2],
    ...             [ 1, -3,  1],
    ...             [ 0,  1,  3]
    ...         ]) @ M2 @ np.array([
    ...             [1, 0, -1,  2],
    ...         ])
    ...      )
    >>> S1 = np.array([[2,6],[6,3]])
    >>> S2 = np.array([
    ...     [5,  1, 0],
    ...     [1, -1, 1],
    ...     [0,  1, 1]
    ... ])
    >>> M1 = np.array([
    ...     [-1,  1],
    ...     [ 3, -2],
    ...     [ 2,  5],
    ... ])
    >>> M2 = np.array([
    ...     [  5],
    ...     [ -2],
    ...     [  1],
    ... ])
    >>> A = fct([S1, S2], [M1, M2])
    >>> res_S, res_M, det = solve_linear_equation(
    ...     A, fct, [2, 3], [(3, 2), (3,1)]
    ... )
    >>> np.abs(det) > 10**-3
    True
    >>> np.linalg.norm(A - fct(res_S, res_M)) < 10**-8
    True
    >>> np.linalg.norm(A - fct(res_S, res_M)) < 10**-8
    True

    >>> output_mask = np.array([
    ...     [ 1, 0, 1, 1],
    ...     [ 1, 1, 1, 1],
    ...     [ 1, 1, 0, 1],
    ...     [ 1, 0, 0, 1],
    ...     [ 1, 1, 1, 0],
    ...     [ 1, 1, 1, 0]
    ... ])
    >>> res_S, res_M, det = solve_linear_equation(
    ...     A, fct, [2, 3], [(3, 2), (3,1)], output_mask=output_mask
    ... )
    >>> np.abs(det) > 10**-3
    True
    >>> np.linalg.norm(A - fct(res_S, res_M)) < 10**-8
    True

    >>> output_mask = np.array([
    ...     [ 1, 0, 1, 1],
    ...     [ 0, 0, 1, 1],
    ...     [ 1, 1, 0, 1],
    ...     [ 1, 0, 0, 1],
    ...     [ 1, 1, 1, 0],
    ...     [ 0, 1, 1, 0]
    ... ])
    >>> res_S, res_M, det = solve_linear_equation(
    ...     A, fct, [2, 3], [(3, 2), (3,1)], output_mask=output_mask
    ... )
    >>> np.abs(det) < 10**-3
    True
    >>> np.linalg.norm(A - fct(res_S, res_M)) < 10**-8
    False

    >>> S1 = np.array([[2,6],[6,0]])
    >>> S1_mask = np.array([[1,1],[1,0]])
    >>> S2 = np.array([
    ...     [5,  0, 0],
    ...     [0, -1, 1],
    ...     [0,  1, 0]
    ... ])
    >>> S2_mask = np.array([
    ...     [1,  0, 0],
    ...     [0,  1, 1],
    ...     [0,  1, 0]
    ... ])
    >>> M1 = np.array([
    ...     [ 0,  1],
    ...     [ 0,  0],
    ...     [ 2,  0],
    ... ])
    >>> M1_mask = np.array([
    ...     [ 0,  1],
    ...     [ 1,  0],
    ...     [ 1,  0],
    ... ])
    >>> M2 = np.array([
    ...     [  0],
    ...     [ -2],
    ...     [  1],
    ... ])
    >>> M2_mask = np.array([
    ...     [  0],
    ...     [  1],
    ...     [  1],
    ... ])
    >>> A = fct([S1, S2], [M1, M2])
    >>> res_S, res_M, det = solve_linear_equation(
    ...     A, fct,
    ...     symetric_mask = [S1_mask, S2_mask],
    ...     rectangular_mask = [M1_mask, M2_mask]
    ... )
    >>> np.abs(det) > 10**-3
    True
    >>> np.linalg.norm(A - fct(res_S, res_M)) < 10**-8
    True
    """
    if symetric_n is None:
        symetric_n = [S.shape[0] for S in symetric_mask]
    if rectangular_n is None:
        rectangular_n = [M.shape for M in rectangular_mask]

    if symetric_mask is None:
        symetric_mask = [
            np.full((n, n), 1)
            for n in symetric_n
        ]
    if rectangular_mask is None:
        rectangular_mask = [
            np.full((m, n), 1)
            for m, n in rectangular_n
        ]

    if output_mask is None:
        output_mask = np.full(A.shape, 1)

    zero_S = [np.zeros((n, n)) for n in symetric_n]
    zero_M = [np.zeros((m, n)) for m, n in rectangular_n]

    image_fct_S = []
    nb_parameters_S = [0]
    for k in range(len(symetric_n)):
        n = symetric_n[k]
        images = []
        for i, j in zip(*np.triu_indices(n)):
            if symetric_mask[k][i, j] > 0:
                e_S = deepcopy(zero_S)
                e_M = deepcopy(zero_M)
                e_S[k] = symEij(i, j, n)
                images.append(to_1D(fct(e_S, e_M), output_mask))
        if len(images) != 0:
            image_fct_S.append(np.hstack(images))
        nb_parameters_S.append(nb_parameters_S[-1] + len(images))

    image_fct_M = []
    nb_parameters_M = [0]
    for k in range(len(rectangular_n)):
        m, n = rectangular_n[k]
        images = []
        for i in range(m):
            for j in range(n):
                if rectangular_mask[k][i, j] > 0:
                    e_S = deepcopy(zero_S)
                    e_M = deepcopy(zero_M)
                    e_M[k] = Eij(i, j, m, n)
                    images.append(to_1D(fct(e_S, e_M), output_mask))
        if len(images) != 0:
            image_fct_M.append(np.hstack(images))
        nb_parameters_M.append(nb_parameters_M[-1] + len(images))

    F = np.hstack(image_fct_S + image_fct_M)

    det = np.linalg.det(F.T @ F)

    # sol = np.linalg.inv(F.T @ F) @ F.T @ A.reshape((-1,1))
    reshaped_output = to_1D(A, output_mask)
    sol = np.linalg.pinv(F) @ reshaped_output
    sol = sol.reshape(-1)

    parameter_S, parameter_M = np.split(sol, [nb_parameters_S[-1]])
    parameter_S = np.split(parameter_S, nb_parameters_S)[1:-1]
    parameter_M = np.split(parameter_M, nb_parameters_M)[1:-1]

    sol_S = []
    for k in range(len(symetric_n)):
        n = symetric_n[k]
        S = np.zeros((n, n))
        cpt = 0
        for i, j in zip(*np.triu_indices(n)):
            if(symetric_mask[k][i, j] > 0):
                S[i, j] = parameter_S[k][cpt]
                S[j, i] = parameter_S[k][cpt]
                cpt += 1
        sol_S.append(S)

    sol_M = []
    for k in range(len(rectangular_n)):
        m, n = rectangular_n[k]
        M = np.zeros((m, n))
        cpt = 0
        for i in range(m):
            for j in range(n):
                if rectangular_mask[k][i, j] > 0:
                    M[i, j] = parameter_M[k][cpt]
                    cpt += 1
        sol_M.append(M)

    return [sol_S, sol_M, det]

class Model:
    """
    Modèle :
    ========

    XX_kp1 = f(XX_k, UU_k)

    f is a model linear is some parameters to identify.
    """
    def identify(self, XXX, UUU, store_data=True, store_result=True):
        """
        Identify a model according to some data

        Data are :
        XXX   = [XX_0, XX_1, XX_2, ..., XX_N]
        UUU   = [UU_0, UU_1, UU_2, ..., XX_N]

        Minimized Equation :

        XXX[:, 1:] = f(XX[:, :-1], UU[:, :-1])
        """
        if store_data :
            self.XXX = XXX
            self.UUU = UUU
        def fct(list_of_symetric_parameters, list_of_non_symetrice_paramters):
            return self.f_model(
                XXX[:,:-1], UUU[:,:-1], 
                list_of_symetric_parameters, list_of_non_symetrice_paramters
            )
        fct_output = XXX[:,1:]
        sym_sizes, non_sym_sizes = self.parameter_sizes()
        sym_mask, non_sym_mask = self.parameter_mask()
        x_mask = self.XX_mask()
        if x_mask is None:
            x_mask = np.ones((XXX.shape[0], 1))
        output_mask = np.hstack([ x_mask for i in range(fct_output.shape[1]) ])
        res_S, res_M, det = solve_linear_equation(
            fct_output, fct,
            symetric_n=sym_sizes, rectangular_n=non_sym_sizes,
            symetric_mask=sym_mask, rectangular_mask=non_sym_mask,
            output_mask=output_mask
        )
        error = fct_output - self.f_model(XXX[:,:-1], UUU[:,:-1], res_S, res_M)
        covariance_error = error @ error.T/fct_output.shape[1]
        rms_error = np.trace(covariance_error) / covariance_error.shape[0]
        if store_result :
            self.list_of_symetric_parameters = res_S
            self.list_of_non_symetrice_paramters = res_M
            self.covariance_error = covariance_error
            self.det = det
            self.rms_error = rms_error
        return [res_S, res_M, covariance_error, rms_error, det]
    def parameter_mask(self):
        sym_sizes, non_sym_sizes = self.parameter_sizes()
        sym_mask = [ np.ones((n,n)) for n in sym_sizes ]
        non_sym_mask = [ np.ones(size) for size in non_sym_sizes ]
        return [sym_mask, non_sym_mask]
    def XX_mask(self):
        return None
    def parameter_sizes(ezlf):
        print("Implement the parameter sizes method !")
        raise NotImplemented
    def f_model(
        self, XX_k, UU_k, 
        list_of_symetric_parameters, list_of_non_symetrice_paramters
    ):
        print("Implement the function f of the model that is linear in its parameters!")
        raise NotImplemented

class LinearModel(Model):
    """
    Model :
    ========

    XX_kp1 = f(XX_k, UU_k)
           = E @ XX_k + F @ UU_k

    E and F should be specified by given the E and F matrix
    """
    def identify(self, XXX, UUU, store_data=True, store_result=True):
        """
        Identify a model according to some data

        Data are :
        XXX   = [XX_0, XX_1, XX_2, ..., XX_N]
        UUU   = [UU_0, UU_1, UU_2, ..., XX_N]

        Minimized Equation :

        XXX[:, 1:] = E @ XX[:, :-1] + F @ UU[:, :-1]
        """
        [res_S, res_M, covariance_error, rms_error, det] = Model.identify(
            self, XXX, UUU, store_data, store_result
        )
        E = self.E(res_S, res_M)
        F = self.F(res_S, res_M)
        if store_result:
            self.EE = E
            self.FE = F
        return [E, F, covariance_error, rms_error, det]
    def E(self, list_of_symetric_parameters, list_of_non_symetrice_paramters):
        print("Implement the parameter sizes method !")
        raise NotImplemented
    def F(self, list_of_symetric_parameters, list_of_non_symetrice_paramters):
        print("Implement the parameter sizes method !")
        raise NotImplemented
    def f_model(
        self, XX_k, UU_k, 
        list_of_symetric_parameters, list_of_non_symetrice_paramters
    ):

        E = self.E(list_of_symetric_parameters, list_of_non_symetrice_paramters)
        F = self.F(list_of_symetric_parameters, list_of_non_symetrice_paramters)
        return E @ XX_k + F @ UU_k

if __name__ == "__main__":
    import doctest
    doctest.testmod()
