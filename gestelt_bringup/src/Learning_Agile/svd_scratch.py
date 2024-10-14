import numpy as np
import casadi as ca
import copy
def svd_jacobi(M):
  """
  This functions source code is from:
  https://jamesmccaffrey.wordpress.com/2024/01/25/singular-value-decomposition-svd-from-scratch-using-python/
  """
  DBL_EPSILON = 1.0e-15  # approximately
  A = np.copy(M)  # working copy U
  m = len(A)
  n = len(A[0])

  Q = np.eye(n)  # working copy V
  t = np.zeros(n)  # working copy s

  # init counters
  count = 1
  sweep = 0
  sweep_max = max(5 * n, 12)  # heuristic

  tolerance = 10 * m * DBL_EPSILON  # heuristic
  # store the column error estimates in t
  for j in range(n):
    cj = A[:, j]  # get col j
    sj = np.linalg.norm(cj)
    t[j] = DBL_EPSILON * sj

  # orthogonalize A by plane rotations
  while (count > 0 and sweep <= sweep_max):
    # initialize rotation counter
    count = n * (n - 1) / 2
    for j in range(n-1):
      for k in range(j+1, n):
        cj = A[:, j]
        ck = A[:, k]
        p = 2 * np.dot(cj, ck)
        a = np.linalg.norm(cj)
        b = np.linalg.norm(ck)

        # test for columns j,k orthogonal,
        # or dominant errors 
        abserr_a = t[j]
        abserr_b = t[k]

        q = (a * a) - (b * b)
        v = np.sqrt(p**2 + q**2)  # hypot()
 
        sorted = (a >= b)
        orthog = (abs(p) <= tolerance * (a*b))
        noisya = (a < abserr_a)
        noisyb = (b < abserr_b)

        if sorted and (orthog or \
          noisya or noisyb):
          count -= 1
          continue

        # calculate rotation angles
        if v == 0 or sorted == False:
          cosine = 0.0
          sine = 1.0
        else:
          cosine = np.sqrt((v + q) / (2.0 * v))
          sine = p / (2.0 * v * cosine)

        # apply rotation to A (U)
        for i in range(m):
          Aik = A[i][k]
          Aij = A[i][j]
          A[i][j] = Aij * cosine + Aik * sine
          A[i][k] = -Aij * sine + Aik * cosine

        # update singular values
        t[j] = abs(cosine) * abserr_a + \
          abs(sine) * abserr_b
        t[k] = abs(sine) * abserr_a + \
          abs(cosine) * abserr_b

        # apply rotation to Q (V)
        for i in range(n):
          Qij = Q[i][j]
          Qik = Q[i][k]
          Q[i][j] = Qij * cosine + Qik * sine
          Q[i][k] = -Qij * sine + Qik * cosine

    sweep += 1
  # while

  # compute singular values
  prev_norm = -1.0
  for j in range(n):
    column = A[:, j]  # by ref
    norm = np.linalg.norm(column)
    # determine if singular value is zero
    if norm == 0.0 or prev_norm == 0.0 or \
      (j > 0 and norm <= tolerance * prev_norm):
      t[j] = 0.0
      for i in range(len(column)):
        column[i] = 0.0  # updates A indirectly
      prev_norm = 0.0
    else:
      t[j] = norm
      for i in range(len(column)):
        column[i] = column[i] * (1.0 / norm)
      prev_norm = norm

  if count > 0:
    print("Jacobi iterations no converge")

  U = A  # mxn
  s = t
  Vh = np.transpose(Q)

  if m < n:
    U = U[:, 0:m]
    s = t[0:m]
    Vh = Vh[0:m, :]

  return U, s, Vh


def qr_eigen(A, iterations=100):
    """ 
    perform the QR algorithm for eigenvalue decomposition
    source code from https://gist.github.com/edxmorgan/51bdb566592a3bc0e386db1f8c50104b
    """
    pQ = ca.SX.eye(A.size1())
    X = copy.deepcopy(A)
    for _ in range(iterations):
        Q, R = ca.qr(X)  # QR decomposition in CasADi
        pQ = pQ @ Q # Update eigenvector matrix for next iteration
        X = R @ Q  # Update eigenvalue matrix for next iteration
    return ca.diag(X), pQ    
    

# -----------------------------------------------------------
def svd_jacobi_casadi(M):
    """
    Jacobi-based Singular Value Decomposition (SVD) for casadi.SX variables.
    input: M (casadi.SX): mxn matrix
    """

    DBL_EPSILON = 1.0e-15  # approximately
    A = ca.SX(M)  # working copy U
    m = A.size1()
    n = A.size2()

    Q = ca.SX.eye(n)  # working copy V
    t = ca.SX.zeros(n)  # working copy s

    # init counters
    count = 1
    sweep = 0
    sweep_max = max(5 * n, 12)  # heuristic

    tolerance = 10 * m * DBL_EPSILON  # heuristic

    # store the column error estimates in t
    for j in range(n):
        cj = A[:, j]  # get col j
        sj = ca.norm_2(cj)  # Use CasADi norm
        t[j] = DBL_EPSILON * sj

    # orthogonalize A by plane rotations
    while (count > 0 and sweep <= sweep_max):
        # initialize rotation counter
        count = n * (n - 1) / 2
        for j in range(n-1):
            for k in range(j+1, n):
                cj = A[:, j]
                ck = A[:, k]
                p = 2 * ca.dot(cj, ck)
                a = ca.norm_2(cj)
                b = ca.norm_2(ck)

                # test for columns j,k orthogonal,
                # or dominant errors 
                abserr_a = t[j]
                abserr_b = t[k]

                q = (a * a) - (b * b)
                v = ca.sqrt(p**2 + q**2)

                sorted = (a >= b)
                orthog = (ca.fabs(p) <= tolerance * (a * b))
                noisya = (a < abserr_a)
                noisyb = (b < abserr_b)

                if sorted and (orthog or noisya or noisyb):
                    count -= 1
                    continue

                # calculate rotation angles
                if v == 0 or sorted == False:
                    cosine = 0.0
                    sine = 1.0
                else:
                    cosine = ca.sqrt((v + q) / (2.0 * v))
                    sine = p / (2.0 * v * cosine)

                # apply rotation to A (U)
                for i in range(m):
                    Aik = A[i, k]
                    Aij = A[i, j]
                    A[i, j] = Aij * cosine + Aik * sine
                    A[i, k] = -Aij * sine + Aik * cosine

                # update singular values
                t[j] = ca.fabs(cosine) * abserr_a + ca.fabs(sine) * abserr_b
                t[k] = ca.fabs(sine) * abserr_a + ca.fabs(cosine) * abserr_b

                # apply rotation to Q (V)
                for i in range(n):
                    Qij = Q[i, j]
                    Qik = Q[i, k]
                    Q[i, j] = Qij * cosine + Qik * sine
                    Q[i, k] = -Qij * sine + Qik * cosine

        sweep += 1

    # compute singular values
    prev_norm = -1.0
    for j in range(n):
        column = A[:, j]
        norm = ca.norm_2(column)
        if norm == 0.0 or prev_norm == 0.0 or (j > 0 and norm <= tolerance * prev_norm):
            t[j] = 0.0
            for i in range(len(column)):
                A[i, j] = 0.0
            prev_norm = 0.0
        else:
            t[j] = norm
            for i in range(len(column)):
                A[i, j] = A[i, j] / norm
            prev_norm = norm

    if count > 0:
        print("Jacobi iterations did not converge")

    U = A  # mxn
    s = t
    Vh = ca.transpose(Q)

    if m < n:
        U = U[:, 0:m]
        s = t[0:m]
        Vh = Vh[0:m, :]

    return U, s, Vh

def SVD_casadi_func():
    # example usage for a symbolic SX square matrix in casadi    
    
    
    # Define a symbolic matrix
    m = 3
    n = 3
    M = ca.SX.sym('M', m, n)

    # Perform eigenvalue decomposition
    w_sym , v_sym = qr_eigen(M, 100)  # This get the symbolic eigenvalues and eigenvector expression

    # Create a function
    f = ca.Function('f', [M], [w_sym, v_sym])

    # Evaluate the function
    M_val = np.array([
        [1, 2, 3],
        [5, 0, 2],
        [8, 5, 4]
    ])
    w_val, v_val = f(M_val)

    print("\nw_val = "); print(w_val)
    print("\nv_val = "); print(v_val)
    print("\n det(v_val) = ", np.linalg.det(v_val))

def main():
  print("\nBegin SVD from scratch Python ")
  np.set_printoptions(precision=4, suppress=True,
    floatmode='fixed')

  A = np.array([
    [1, 2, 3],
    [5, 0, 2],
    [8, 5, 4]], dtype=np.float64)
    #,
    #[6, 9, 7]
  # m < n example
  # A = np.array([
  #   [1, 2, 3],
  #   [5, 0, 2]], dtype=np.float64)

  print("\nSource matrix: ")
  print(A)

  U, s, Vh = svd_jacobi(A)

  print("\nU = "); print(U)
  print("\n det(U) = ", np.linalg.det(U))
  print("\ns = "); print(s)
  print("\nVh = "); print(Vh)
  print("\n det(Vh) = ", np.linalg.det(Vh))

  U, s, Vh = np.linalg.svd(A, full_matrices=False)
  print("\nUsing linalg.svd(): ")
  print("\nU = "); print(U)
  print("\n det(U) = ", np.linalg.det(U))
  print("\ns = "); print(s)
  print("\nVh = "); print(Vh)
  print("\n det(Vh) = ", np.linalg.det(Vh))
  print("\nEnd demo ")

  SVD_casadi_func()
if __name__ == "__main__":
  main()