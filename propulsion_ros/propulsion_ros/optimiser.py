import numpy as np

def pseudo_inverse_old(A):
    """
    Compute the Moore-Penrose pseudo-inverse of a matrix A using SVD.
    """
    A = np.array(A, dtype=float)

    # Compute SVD decomposition of A into U, Sigma, and Vh matrices
    u, sig, vt = np.linalg.svd(A, full_matrices=False)
    print(f"Singular Values: {sig}\n")
    
    sigma_inv = 1 / sig[::-1]
    idx = np.argsort(sig)
    new_sigma_inv = np.zeros_like(sigma_inv)
    new_sigma_inv[idx[:len(sigma_inv)]] = sigma_inv
    print(f"Inverse of singular values: {new_sigma_inv}\n")

    # Create a diagonal matrix with the inverse of singular values
    inv_Sigma = np.diag(new_sigma_inv)

    # Compute the pseudo-inverse (Moore-Penrose inverse) using the formula:
    # pinv_A = V * inv_Sigma * U^T
    pinv_A = np.dot(vt.T,np.dot(inv_Sigma, u.T))
    return pinv_A

def pseudo_inverse(A, eps=1e-9):
    u, s, vt = np.linalg.svd(A, full_matrices=False)

    s_inv = np.array([1/x if x > eps else 0 for x in s])

    return vt.T @ np.diag(s_inv) @ u.T

def setup_kin_mat(alphas: np.ndarray, ells: np.ndarray, phi: float):
    mat = np.array(
             [
                [np.cos(phi), np.cos(phi), np.cos(phi), np.cos(phi)],
                [np.sin(phi), np.sin(phi), np.sin(phi), np.sin(phi)],
                [np.sin(phi - alphas[0])/ells[0], 
                 np.sin(phi - alphas[1])/ells[1], 
                 np.sin(phi - alphas[2])/ells[2], 
                 np.sin(phi - alphas[3])/ells[3]]
             ]
            )
    return mat



best_beta = None
best_cost = 1e9

alphas =  np.array([np.pi/4, 3/4*np.pi, 5/4 * np.pi, 7/4 * np.pi])
ells = np.array([1, 1, 1, 1])
control = np.array([0, 50, 5000])

# create multiple lists of beta and optimise the best combination
for beta in np.linspace(-np.pi, np.pi, 400):
    J = setup_kin_mat(alphas, ells, beta)
    P = J @ pseudo_inverse(J)
    out = P @ control

    cost = np.linalg.norm(control - out)

    if cost < best_cost:
        best_cost = cost
        best_beta = beta

# outputs in units of pi for easy interpretation
print(best_beta/np.pi, best_cost)
