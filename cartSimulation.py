import numpy as np
import math
import pygame
import PlotGraph as graph
import UISlider as slide

#Set up state-space of cart=====================================================
#Define state vector
startPosition = 500
startVelocity = 0
startAngle = 0.0
startAngularVelocity = 0
x = np.array([[startPosition], [startVelocity], [startAngle], [startAngularVelocity]]) #Initial conditios of positon and velocity

#Define cart properties
M = 1 #Mass of cart
m = 0.25
b = 0.025 #Damping force (air resistance, friction)
c = 0.5
k = 0 #Spring force (flexibility of surface, elasticity of cart)
u = 0 #Input force on cart
L = 150 #Length of pendulum
g = 0.03 #Gravity

#Define system dynamics of cart -k/m, -b/m, 0, 0 0, 0, -g/L, 0 (m/M)*g
#Newton's 2nd Law: F = ma = spring force (x) + damping force (v) + input force (a)
A = np.array([[0, 1, 0, 0], [0, -b/M, (m/M)*g, 0], [0, 0, 0, -1], [0, 0, (M+m)*g/(L*M), -c/(L*M)]]) #Convert position to velocity and velocity to acceleration
B = np.array([[0], [1/M], [0], [1/(L*M)]]) #Convert input force to acceleration
C = np.array([[1, 0, 0, 0]]) #Output position
D = np.array([[0]]) #Zero for simplicity

#State equations and output state parameter functions
#Derivative of state vector (position -> velocity) & (velocity -> position)
def xDot(A, B, x, u):
    return np.dot(A, x) + np.dot(B, u)

#Output state vector parameter of interest
def yOutput(C, D, x, u):
    return np.dot(C, x) + np.dot(D, u)

#PID controller
setpoint = 0
prevError = 0
error = 0
totalError = 0
timeElapsed = 0
dt = 1 #Milliseconds

def getError(setpoint, currentPoint):
    return setpoint - currentPoint

def PIDOutput(Kp, Kd, Ki, error):
    P = Kp * error
    I = Ki * totalError * dt
    D = Kd * (error - prevError) / dt
    print("Kp: ", Kp, " Ki: ", Ki, " Kd: ", Kd)

    graph.setFuncPoint("P", timeElapsed, setpoint, (255,200,102))
    graph.setFuncPoint("P", timeElapsed, P, (153, 255, 153))
    graph.setFuncPoint("I", timeElapsed, I, (153, 255, 255))
    graph.setFuncPoint("D", timeElapsed, D, (255, 102, 102))
    u = P + I + D
    return np.array([u])

#Create simulation==============================================================
pygame.init()

#Create simulation graphics
screenSizeX = 1500
screenSizeY = 500
screen = pygame.display.set_mode((screenSizeX, screenSizeY))
screen2 = pygame.Surface((screenSizeX, screenSizeY), pygame.SRCALPHA)
screen2 = screen2.convert_alpha()
backgroundColour = (48, 49, 81)
screen.fill(backgroundColour)
pygame.display.set_caption("State-Space Cart Simulation")
pygame.display.flip()

#Draw cart
cartSizeX = 100
cartSizeY = 35
cartPosX = startPosition
cartPosY = screenSizeY/2
cartColour = (255, 200, 102)
pygame.draw.rect(screen, cartColour, pygame.Rect(cartPosX, cartPosY, cartSizeX, cartSizeY))

#Draw pendulum
def getPendulumPoints(theta, length, startPosX, startPosY):
    endPosX = length * math.sin(theta) + startPosX
    endPosY = length* math.cos(theta) + startPosY
    return (endPosX, endPosY)

pendulumPosX = cartPosX + cartSizeX/2
pendulumPosY = cartPosY + cartSizeY/2
pendulumColour = (255,71,58)
bobColour = (245, 140, 88)
circleRadius = 20
pendulumEndPositions = getPendulumPoints(math.pi/4, L, pendulumPosX, pendulumPosY)
pygame.draw.line(screen, pendulumColour, (pendulumPosX, pendulumPosY), pendulumEndPositions, 10)
pygame.draw.circle(screen, bobColour, (pendulumPosX, pendulumPosY), circleRadius/2)
pygame.draw.circle(screen, bobColour, pendulumEndPositions, circleRadius)

#Draw floor
floorColour = (37, 33, 64)
pygame.draw.rect(screen, floorColour, pygame.Rect(0, screenSizeY+1, screenSizeX, screenSizeY/2))

#Simulation parameters
running = True

valueStart = 0
valueEnd = 0.2

sliders = []
letter = ["P", "I", "D"]
for i in range(3):
    sliderSlot, sliderButton = slide.createSlider(screen2, 25, (i + 1) * 50, 200)
    sliders.append([sliderSlot, sliderButton])
    slide.sliderSetting(screen2, sliders[-1], valueStart, valueEnd, letter[i])

#Run cart simulation============================================================
while running:
    u = 0
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    #Get input force direction based on user input
    keys = pygame.key.get_pressed()
    if keys[pygame.K_a]:
        u += -0.1
    if keys[pygame.K_d]:
        u += 0.1
    if keys[pygame.K_w]:
        x[0] = 0

    kValues = {"P": 0, "I": 0, "D": 0}
    i = 0
    for slider in sliders:
        slide.dragSlider(screen2, slider[0], slider[1])
        K =  letter[i]
        if K == "I":
            kValues[K] = slide.getSliderValue(slider, valueStart/100, valueEnd/100)
        else:
            kValues[K] = slide.getSliderValue(slider, valueStart, valueEnd)
        i += 1

    prevError = error
    error = getError(setpoint, x[2])
    totalError += error
    u += PIDOutput(kValues["P"], kValues["D"], kValues["I"], error)
    #Cart simulation code here
    x = x + xDot(A, B, x, u) * dt
    y = yOutput(C, D, x, u)
    if x[2] >= math.pi:
        x[2] = x[2] - 2*math.pi
    elif x[2] <= -math.pi:
        x[2] = x[2] + 2*math.pi
    #Collision detection for next time step
    if y < 0:
        x[0] = 0
        x[1] = 0
    elif y + cartSizeX > screenSizeX:
        x[0] = screenSizeX - cartSizeX
        x[1] = 0
    #Cart simulation code end
    screen.fill(backgroundColour)
    pygame.draw.rect(screen, floorColour, pygame.Rect(0, screenSizeY/2+cartSizeY, screenSizeX, screenSizeY/2))
    pygame.draw.rect(screen, cartColour, pygame.Rect(y.item(), cartPosY, cartSizeX, cartSizeY))
    pendulumEndPositions = getPendulumPoints(x[2], L, y.item() + cartSizeX/2, pendulumPosY)
    pygame.draw.line(screen, pendulumColour, (y.item() + cartSizeX/2, pendulumPosY), pendulumEndPositions, 10)
    pygame.draw.circle(screen, bobColour, (y.item() + cartSizeX/2, pendulumPosY), circleRadius/2)
    pygame.draw.circle(screen, bobColour, pendulumEndPositions, circleRadius)
    screen.blit(screen2, (0, 0))
    pygame.display.update()
    pygame.time.wait(dt)
pygame.quit()
