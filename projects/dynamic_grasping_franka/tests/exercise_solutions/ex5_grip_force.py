G = 9.81


def min_grip_force(mass, mu):
    return mass * G / (2 * mu)


def holds_while_lifting(force, mass, mu, acceleration):
    return 2 * mu * force >= mass * (G + acceleration)
