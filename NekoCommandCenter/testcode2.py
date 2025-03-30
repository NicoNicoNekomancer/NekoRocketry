import serial
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np

# Set up the serial connection
ser = serial.Serial('COM14', 115200)  # replace 'COM14' with your Arduino's port

# Set up the cuboid
vertices = ((1, -1, -1), (1, 1, -1), (-1, 1, -1), (-1, -1, -1), (1, -1, 2), (1, 1, 2), (-1, -1, 2), (-1, 1, 2))
edges = ((0, 1), (0, 3), (0, 4), (2, 1), (2, 3), (2, 7), (6, 3), (6, 4), (6, 7), (5, 1), (5, 4), (5, 7))
faces = [(0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7), (0, 1, 2, 3), (4, 5, 6, 7)]
colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0), (1, 0, 1), (0, 1, 1)]

def Cuboid():
    glBegin(GL_QUADS)
    for face, color in zip(faces, colors):
        glColor3fv(color)
        for vertex in face:
            glVertex3fv(vertices[vertex])
    glEnd()

    glColor3fv((0, 0, 0))
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

# Set up Pygame and OpenGL
pygame.init()
display = (800, 600)
pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
glTranslatef(0.0, 0.0, -5)

# Main loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

    # Read the next line from the serial input
    line = ser.readline().decode('utf-8').strip()

    # Split the line into components
    components = line.split(' ')

    # Parse the quaternion values from the components
    q1 = float(components[0])
    q2 = float(components[1])
    q3 = float(components[2])
    q4 = float(components[3])

    # Convert quaternion to rotation matrix
    R = np.array([
        [1 - 2 * (q2 ** 2 + q3 ** 2), 2 * (q1 * q2 - q3 * q4), 2 * (q1 * q3 + q2 * q4), 0],
        [2 * (q1 * q2 + q3 * q4), 1 - 2 * (q1 ** 2 + q3 ** 2), 2 * (q2 * q3 - q1 * q4), 0],
        [2 * (q1 * q3 - q2 * q4), 2 * (q2 * q3 + q1 * q4), 1 - 2 * (q1 ** 2 + q2 ** 2), 0],
        [0, 0, 0, 1]
    ])

    # Use rotation matrix to rotate cuboid
    glPushMatrix()
    glMultMatrixf(R.T.flatten())
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    Cuboid()
    glPopMatrix()

    pygame.display.flip()
    pygame.time.wait(10)
