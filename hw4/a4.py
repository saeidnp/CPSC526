from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from math import  *
#from matplotlib.pyplot import plot, ion, show
import sys

#  from pyquaternion import Quaternion    ## would be useful for 3D simulation
import numpy as np

window = 0     # number of the glut window
simTime = 0
dT = 0.01
simRun = True
RAD_TO_DEG = 180.0/3.1416

#####################################################
#### Link class, i.e., for a rigid body
#####################################################

class Link:
    color=[0,0,0]    ## draw color
    size=[1,1,1]     ## dimensions
    mass = 1.0       ## mass in kg
    ixx = 1.0
    iyy = 1.0
    izz = 1.0        ## moment of inertia about z-axis
    theta=np.array([0, 0, 0])          ## 3D orientation
    omega=np.array([0, 0, 0])          ## 3D angular velocity
    posn=np.array([0.0,0.0,0.0])     ## 3D position (keep z=0 for 2D)
    vel=np.array([0.0,0.0,0.0])      ## initial velocity
    length = 1.0
    f = np.array([0.0, 0.0, 0.0])
    kinetic_energy = []
    def draw(self):      ### steps to draw a link
        glPushMatrix()                                            ## save copy of coord frame
        glTranslatef(self.posn[0], self.posn[1], self.posn[2])    ## move
        glRotatef(self.theta[2]*RAD_TO_DEG,  0,0,1)                             ## rotate
        glScale(self.size[0], self.size[1], self.size[2])         ## set size
        glColor3f(self.color[0], self.color[1], self.color[2])    ## set colour
        DrawCube()                                                ## draw a scaled cube
        glPopMatrix()                                             ## restore old coord frame

#####################################################
#### main():   launches app
#####################################################

def main():
    global window
    global link
    global plot_ax, plot_line
    global plt
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)     # display mode
    glutInitWindowSize(640, 480)                                  # window size
    glutInitWindowPosition(0, 0)                                  # window coords for mouse start at top-left
    window = glutCreateWindow("CPSC 526 Simulation Template")
    glutDisplayFunc(DrawWorld)       # register the function to draw the world
    # glutFullScreen()               # full screen
    glutIdleFunc(SimWorld)           # when doing nothing, redraw the scene
    glutReshapeFunc(ReSizeGLScene)   # register the function to call when window is resized
    glutKeyboardFunc(keyPressed)     # register the function to call when keyboard is pressed
    InitGL(640, 480)                 # initialize window


    import matplotlib.pyplot as plt
    plt.ion()
    plot_ax = plt.gca()
    plot_ax.set_autoscale_on(True)
    plot_line, = plot_ax.plot([0], [0])
    # https://stackoverflow.com/questions/23058560/plotting-dynamic-data-using-matplotlib

    link = Link();
    resetSim()

    glutMainLoop()                   # start event processing loop

#####################################################
#### keyPressed():  called whenever a key is pressed
#####################################################

def resetSim():
    global link
    global simTime, simRun

    printf("Simulation reset\n")
    simRun = True
    simTime = 0

    link.size=[0.04, 1.0, 0.12]
    link.color=[1,0.9,0.9]
    link.posn=np.array([0.0,0.0,0.0])
    link.vel=np.array([0.0,0.0,0.0])
    link.theta = np.array([0.0, 0.0, pi/4])
    link.omega = np.array([0.0, 0.0, 0.0])        ## radians per second
    link.izz = (link.mass * link.length**2) / 12
    link.f = np.array([0.0, 0.0, 0.0])
    link.kinetic_energy = [0]

#TODO: Cleanup
'''
        link2.size=[0.04, 1.0, 0.12]
        link2.color=[0.9,0.9,1.0]
        link2.posn=np.array([1.0,0.0,0.0])
        link2.vel=np.array([0.0,4.0,0.0])
        link2.theta = -0.2
        link2.omega = 0        ## radians per second
'''

#####################################################
#### keyPressed():  called whenever a key is pressed
#####################################################

def keyPressed(key,x,y):
    global simRun
    global link
    ch = key.decode("utf-8")
    if ch == ' ':                #### toggle the simulation
            if (simRun == True):
                 simRun = False
            else:
                 simRun = True
    elif ch == chr(27):          #### ESC key
            sys.exit()
    elif ch == 'q':              #### quit
            import matplotlib.pyplot as plt
            plt.ion()
            #ax = plt.gca()
            #ax.set_autoscale_on(True)
            #line, = ax.plot(x, y)
            plt.plot(link.kinetic_energy)
            plt.show()
            sys.exit()
    elif ch == 'r':              #### reset simulation
            resetSim()

#####################################################
#### SimWorld():  simulates a time step
#####################################################

def SimWorld():
    global simTime, dT, simRun
    global link

    # Plot variables
    global plot_ax, plot_line, plt

    if (simRun==False):             ## is simulation stopped?
            return

        #### solve for the equations of motion (simple in this case!)
    acc1 = np.array([0,-10,0])       ### linear acceleration = [0, -G, 0]
    omega_dot1 = 0.0                 ### assume no angular acceleration

        ####  for the constrained one-link pendulum, and the 4-link pendulum,
        ####  you will want to build the equations of motion as a linear system, and then solve that.
        ####  Here is a simple example of using numpy to solve a linear system.

    #construct m
    m = np.zeros((3, 3))
    m[0][0] = link.mass
    m[1][1] = link.mass

    #construct I
    I = np.zeros((3, 3))
    I[0][0] = link.ixx
    I[1][1] = link.iyy
    I[2][2] = link.izz

    #construct rhat
    r_x = -link.length/2 * sin(link.theta[2])
    r_y = link.length/2 * cos(link.theta[2])
    r_z = 0
    rhat = np.array([   [0, -r_z, r_y],
                        [r_z, 0, -r_x],
                        [-r_y, r_x, 0] ])

    a = np.concatenate(
        (np.concatenate((m, np.zeros((3, 3)), -np.eye(3)), axis=1),
        np.concatenate((np.zeros((3, 3)), I, -rhat), axis=1),
        np.concatenate((-np.eye(3), rhat, np.zeros((3, 3))), axis=1)),
    axis=0)
    b = np.array([  0, -10*link.mass, 0,
                    0, 0, 0,
                    -r_x*link.omega[2]**2, -r_y*link.omega[2]**2, 0])
    x = np.linalg.solve(a, b)

    acc = x[0:3]
    omega_dot = x[3:6]
    #### explicit Euler integration to update the state
    #link.f += x[6:9]
    print(acc)
    link.posn += link.vel*dT
    link.vel += acc*dT
    link.theta += link.omega*dT
    link.omega += omega_dot*dT

    simTime += dT

        #### draw the updated state
    DrawWorld()
    printf("simTime=%.2f\n",simTime)

    current_kinetic_energy = 0.5 * (link.izz + link.mass * (link.length/2)**2) * link.omega[2]**2 
    link.kinetic_energy.append(current_kinetic_energy)

    plot_line.set_ydata(link.kinetic_energy)
    plot_line.set_xdata(list(range(len(link.kinetic_energy))))
    plot_ax.relim()
    plot_ax.autoscale_view(True,True,True)
    plt.draw()

    #link.kinetic_energy.append(link.kinetic_energy[-1] + 1)
    #print(link.kinetic_energy)

#####################################################
#### DrawWorld():  draw the world
#####################################################

def DrawWorld():
    global link

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	# Clear The Screen And The Depth Buffer
    glLoadIdentity();
    gluLookAt(1,1,3,  0,0,0,  0,1,0)

    DrawOrigin()
    link.draw()

    glutSwapBuffers()                      # swap the buffers to display what was just drawn

#####################################################
#### initGL():  does standard OpenGL initialization work
#####################################################

def InitGL(Width, Height):				# We call this right after our OpenGL window is created.
    glClearColor(1.0, 1.0, 0.9, 0.0)	# This Will Clear The Background Color To Black
    glClearDepth(1.0)					# Enables Clearing Of The Depth Buffer
    glDepthFunc(GL_LESS)				# The Type Of Depth Test To Do
    glEnable(GL_DEPTH_TEST)				# Enables Depth Testing
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);    glEnable( GL_LINE_SMOOTH );
    glShadeModel(GL_SMOOTH)				# Enables Smooth Color Shading
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()					# Reset The Projection Matrix
    gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)

#####################################################
#### ReSizeGLScene():    called when window is resized
#####################################################

def ReSizeGLScene(Width, Height):
    if Height == 0:						# Prevent A Divide By Zero If The Window Is Too Small
	    Height = 1
    glViewport(0, 0, Width, Height)		# Reset The Current Viewport And Perspective Transformation
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)    ## 45 deg horizontal field of view, aspect ratio, near, far
    glMatrixMode(GL_MODELVIEW)

#####################################################
#### DrawOrigin():  draws RGB lines for XYZ origin of coordinate system
#####################################################

def DrawOrigin():
    glLineWidth(3.0);

    glColor3f(1,0.5,0.5)   ## light red x-axis
    glBegin(GL_LINES)
    glVertex3f(0,0,0)
    glVertex3f(1,0,0)
    glEnd()

    glColor3f(0.5,1,0.5)   ## light green y-axis
    glBegin(GL_LINES)
    glVertex3f(0,0,0)
    glVertex3f(0,1,0)
    glEnd()

    glColor3f(0.5,0.5,1)   ## light blue z-axis
    glBegin(GL_LINES)
    glVertex3f(0,0,0)
    glVertex3f(0,0,1)
    glEnd()

#####################################################
#### DrawCube():  draws a cube that spans from (-1,-1,-1) to (1,1,1)
#####################################################

def DrawCube():
	glScalef(0.5,0.5,0.5);                  # dimensions below are for a 2x2x2 cube, so scale it down by a half first
	glBegin(GL_QUADS);			# Start Drawing The Cube

	glVertex3f( 1.0, 1.0,-1.0);		# Top Right Of The Quad (Top)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Left Of The Quad (Top)
	glVertex3f(-1.0, 1.0, 1.0);		# Bottom Left Of The Quad (Top)
	glVertex3f( 1.0, 1.0, 1.0);		# Bottom Right Of The Quad (Top)

	glVertex3f( 1.0,-1.0, 1.0);		# Top Right Of The Quad (Bottom)
	glVertex3f(-1.0,-1.0, 1.0);		# Top Left Of The Quad (Bottom)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Bottom)
	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Bottom)

	glVertex3f( 1.0, 1.0, 1.0);		# Top Right Of The Quad (Front)
	glVertex3f(-1.0, 1.0, 1.0);		# Top Left Of The Quad (Front)
	glVertex3f(-1.0,-1.0, 1.0);		# Bottom Left Of The Quad (Front)
	glVertex3f( 1.0,-1.0, 1.0);		# Bottom Right Of The Quad (Front)

	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Back)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Back)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Right Of The Quad (Back)
	glVertex3f( 1.0, 1.0,-1.0);		# Top Left Of The Quad (Back)

	glVertex3f(-1.0, 1.0, 1.0);		# Top Right Of The Quad (Left)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Left Of The Quad (Left)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Left)
	glVertex3f(-1.0,-1.0, 1.0);		# Bottom Right Of The Quad (Left)

	glVertex3f( 1.0, 1.0,-1.0);		# Top Right Of The Quad (Right)
	glVertex3f( 1.0, 1.0, 1.0);		# Top Left Of The Quad (Right)
	glVertex3f( 1.0,-1.0, 1.0);		# Bottom Left Of The Quad (Right)
	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Right)
	glEnd();				# Done Drawing The Quad

            ### Draw the wireframe edges
	glColor3f(0.0, 0.0, 0.0);
	glLineWidth(1.0);

	glBegin(GL_LINE_LOOP);
	glVertex3f( 1.0, 1.0,-1.0);		# Top Right Of The Quad (Top)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Left Of The Quad (Top)
	glVertex3f(-1.0, 1.0, 1.0);		# Bottom Left Of The Quad (Top)
	glVertex3f( 1.0, 1.0, 1.0);		# Bottom Right Of The Quad (Top)
	glEnd();				# Done Drawing The Quad

	glBegin(GL_LINE_LOOP);
	glVertex3f( 1.0,-1.0, 1.0);		# Top Right Of The Quad (Bottom)
	glVertex3f(-1.0,-1.0, 1.0);		# Top Left Of The Quad (Bottom)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Bottom)
	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Bottom)
	glEnd();				# Done Drawing The Quad

	glBegin(GL_LINE_LOOP);
	glVertex3f( 1.0, 1.0, 1.0);		# Top Right Of The Quad (Front)
	glVertex3f(-1.0, 1.0, 1.0);		# Top Left Of The Quad (Front)
	glVertex3f(-1.0,-1.0, 1.0);		# Bottom Left Of The Quad (Front)
	glVertex3f( 1.0,-1.0, 1.0);		# Bottom Right Of The Quad (Front)
	glEnd();				# Done Drawing The Quad

	glBegin(GL_LINE_LOOP);
	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Back)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Back)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Right Of The Quad (Back)
	glVertex3f( 1.0, 1.0,-1.0);		# Top Left Of The Quad (Back)
	glEnd();				# Done Drawing The Quad

	glBegin(GL_LINE_LOOP);
	glVertex3f(-1.0, 1.0, 1.0);		# Top Right Of The Quad (Left)
	glVertex3f(-1.0, 1.0,-1.0);		# Top Left Of The Quad (Left)
	glVertex3f(-1.0,-1.0,-1.0);		# Bottom Left Of The Quad (Left)
	glVertex3f(-1.0,-1.0, 1.0);		# Bottom Right Of The Quad (Left)
	glEnd();				# Done Drawing The Quad

	glBegin(GL_LINE_LOOP);
	glVertex3f( 1.0, 1.0,-1.0);		# Top Right Of The Quad (Right)
	glVertex3f( 1.0, 1.0, 1.0);		# Top Left Of The Quad (Right)
	glVertex3f( 1.0,-1.0, 1.0);		# Bottom Left Of The Quad (Right)
	glVertex3f( 1.0,-1.0,-1.0);		# Bottom Right Of The Quad (Right)
	glEnd();				# Done Drawing The Quad

####################################################
# printf()
####################################################

def printf(format, *args):
    sys.stdout.write(format % args)

################################################################################
# start the app

print ("Hit ESC key to quit.")
main()

