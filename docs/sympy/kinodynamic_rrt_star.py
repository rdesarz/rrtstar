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

x1_vec = MatrixSymbol('x1', 4, 1)
x_bar = MatrixSymbol('x_bar', 4, 1)

t = symbols('t')

u = Inverse(R) * Transpose(B) * (Id + Transpose(A) * (tau - t)) * G_inv * (x1_vec - x_bar)

print(latex(simplify(u)))

# Initial state vector
x_start, y_start, v_x_start, v_y_start = symbols('x_start y_start v_x_start v_y_start')
x_start_vec = Matrix([[x_start],
                      [y_start],
                      [v_x_start],
                      [v_y_start]])

# Final state vector
x_end, y_end, v_x_end, v_y_end = symbols('x_end y_end v_x_end v_y_end')
x_end_vec = Matrix([[x_end],
                    [y_end],
                    [v_x_end],
                    [v_y_end]])

# x_bar, state of the system without control input
x_bar = (Id + A * tau) * x_start_vec

print("x_bar: {}".format(latex(simplify(x_bar))))

# derivative of cost
c_dot = 2 * Transpose(A * x_end_vec) * G_inv * (x_end_vec - x_bar) - Transpose(x_end_vec - x_bar) * Transpose(
    G_inv) * B * Inverse(
    R) * Transpose(B) * G_inv * (x_end_vec - x_bar)

print(latex(simplify(c_dot)))

print(latex(solve(factor((c_dot * tau ** 4), tau, deep=True), tau)))


