import math

transmissionRatio = 2.0
pulleyRatio = 1.0
flyWheelDia = 4.0           # in
launchHeight = 42.0         # in
goalHeight = 8 * 12 + 9     # in
trajectories = []

measuredTerminalVelocity = 66.7  # ft/s
dragCoefficient = 0.25
airDensity = 1.23  # kg/m3
liftCoefficient = 0.13

gravity = -9.8  # m/s2
ballDiameter = 9.5  # in
powerCellMass = 0.27  # kg
dt = 0.005

measuredTerminalVelocityMs = measuredTerminalVelocity * 12 * 2.54 / 100  # m/s
ballDiameterMet = ballDiameter * 2.54 / 100  # m
frontalArea = math.pi * ballDiameterMet * ballDiameterMet / 4  # m2
mud = airDensity * frontalArea * dragCoefficient / 2  # kg/m
mum = airDensity * frontalArea * liftCoefficient / 2  # kg/m
goalHeightMet = goalHeight * 2.54 / 100

def distance1(t):
    return math.sqrt((t['dc'] - t['dg'])*(t['dc'] - t['dg']) + (t['hc'] - goalHeightMet)*(t['hc'] - goalHeightMet))

def distance2(t, d):
    return abs(t['dg'] - d)

def trajectory(motorSpeed, launchAngle):
    wheelCircumference = math.pi * flyWheelDia
    shooterShaftSpeed = motorSpeed / transmissionRatio / pulleyRatio  # rpm
    linearLaunchSpeed = shooterShaftSpeed * wheelCircumference / 12 / 60  # ft/s
    launchSpeed = linearLaunchSpeed * 12 * 2.54 / 100  # m/s

    launchAngleRad = launchAngle * math.pi / 180  # rad
    launchHeightMet = launchHeight * 2.54 / 100  # m

    x = 0.0
    previousX = x
    y = launchHeightMet
    previousY = y
    v = launchSpeed
    vx = launchSpeed * math.cos(launchAngleRad)
    vy = launchSpeed * math.sin(launchAngleRad)
    ax = -(v / powerCellMass) * (mum * vy + mud * vx)
    ay = v / powerCellMass * (mum * vx - mud * vy) + gravity
    t = 0
    hc = -1
    dc = -1
    dg = -1
    ag = 0

    while True:
        vx += ax * dt
        vy += ay * dt
        v = math.sqrt(vx * vx + vy * vy)
        ax = -(v / powerCellMass) * (mum * vy + mud * vx)
        ay = v / powerCellMass * (mum * vx - mud * vy) + gravity
        previousX = x
        x += vx * dt
        previousY = y
        y += vy * dt
        angle = math.atan2(y - previousY, x - previousX)
        t += dt
        # print(f"{t},{x},{y},{goalHeightMet},{v},{vx},{vy},{ax},{ay}")

        if (y < previousY ) & (hc < 0):
            hc = y
            dc = x

        if (y < goalHeightMet) & (hc > 0) & (dg < 0):
            dg = x
            ag = angle

        if y < launchHeightMet:
            break

    ret = dict()
    ret['hc'] = hc
    ret['dc'] = dc
    ret['dg'] = dg
    ret['ag'] = ag
    # ret['x'] = x
    # ret['y'] = y
    # ret['vx'] = vx
    # ret['vy'] = vy
    # ret['v'] = v
    # ret['ax'] = ax
    # ret['ay'] = ay
    ret['t'] = t
    ret['motorSpeed'] = motorSpeed
    ret['launchAngle'] = launchAngle

    return ret

# print(trajectory(2970, 51))
motorSpeed = 2000
while motorSpeed <= 8000:
    launchAngle = 9
    while launchAngle <= 90:
        trajectories.append(trajectory(motorSpeed, launchAngle))
        launchAngle += 1
    motorSpeed += 10

d = 1.0
while d <= 8.0:
    updatedTrajectories = list(map(lambda t: t.update({'d1': distance1(t), 'd2': distance2(t, d)}) or t, trajectories))
    # print(updatedTrajectories)
    filteredTrajectories = list(filter(lambda t: (t['d2'] < 0.05) and (t['hc'] > goalHeightMet + 0.3) and
        (t['dg'] - 2*(t['dg'] - t['dc']) < d - 0.75) and t['ag'] < -0.5, updatedTrajectories))
    sortedTrajectories = sorted(filteredTrajectories, key = lambda t: t['d1'])
    if len(sortedTrajectories) > 0:
        print("%.2f,%4d,%2d" %(d, sortedTrajectories[0]['motorSpeed'], sortedTrajectories[0]['launchAngle']))

    d += 0.1
