import math
    

linspace = lambda x_min, x_max, num_points: [x_min+i*(x_max - x_min)/(num_points-1) for i in range(num_points)]


def path_line(x0, x1, num_points):
    vals = []
    for i in range(len(x0)):
        vals.append(linspace(x0[i], x1[i], num_points))

    vals_t = []
    for i in range(len(vals[0])):
        vals_t.append([vals[j][i] for j in range(len(vals))])    

    return vals_t


def path_spiral(num_points: int):
    phi = (1 + 5**0.5)/2
    y0 = 1/(2+phi)
    x0 = (2*phi + 1)*y0
    theta0 = math.atan2(-y0, -x0)
    k = 2*math.log(phi)/math.pi
    a = -x0 / (math.exp(k*theta0)*math.cos(theta0))

    x = lambda t: x0 + a*math.exp(k*t)*math.cos(t)
    y = lambda t: y0 + a*math.exp(k*t)*math.sin(t)

    linspace = lambda x_min, x_max, num_points: [x_min+i*(x_max - x_min)/(num_points-1) for i in range(num_points)]
    ts = linspace(theta0, -20, num_points)
    xys = [[x(t), y(t)] for t in ts]

    return xys
