from sympy import MatrixSymbol, Matrix, symbols, Inverse, Transpose, integrate, \
    init_printing, latex, simplify, solve, factor

init_printing()

A = Matrix([[0, 0, 1, 0],
            [0, 0, 0, 1],
            [0, 0, 0, 0],
            [0, 0, 0, 0]])

print(latex(A))

B = Matrix([[0, 0],
            [0, 0],
            [1, 0],
            [0, 1]])

print(latex(B))

rho = symbols('rho')

R = Matrix([[rho, 0],
            [0, rho]])

print(latex(R))

tau, t_prime = symbols('tau t_prime')

Id = Matrix([[1, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])

to_integrate = (Id + A * tau - A * t_prime) * B * Inverse(R) * Transpose(B) * (
        Id + Transpose(A) * tau - Transpose(A) * t_prime)

G = integrate(to_integrate, (t_prime, 0, tau))

print(latex(simplify(G)))

G_inv = G.inv()

print(latex(simplify(G_inv)))

x1 = MatrixSymbol('x1', 4, 1)
x_bar = MatrixSymbol('x_bar', 4, 1)

t = symbols('t')

u = Inverse(R) * Transpose(B) * (Id + Transpose(A) * (tau - t)) * G_inv * (x1 - x_bar)

print(latex(simplify(u)))

C_dot = 2 * Transpose(A * x1) * G_inv * (x1 - x_bar) - Transpose(x1 - x_bar) * Transpose(G_inv) * B * Inverse(
    R) * Transpose(B) * G_inv * (x1 - x_bar)

# print(latex(simplify(C_dot)))
