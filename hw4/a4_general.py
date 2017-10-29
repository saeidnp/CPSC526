#!/usr/bin/python3

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from math import  *
import sys

#  from pyquaternion import Quaternion    ## would be useful for 3D simulation
import numpy as np

window = 0     # number of the glut window
simTime = 0
dT = 0.01
simRun = True
RAD_TO_DEG = 180.0/3.1416
num_links = 4
g_const = 10

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
        glTranslatef(self.posn[0]*0.5, self.posn[1]*0.5, self.posn[2]*0.5)    ## move
        glRotatef(self.theta[2]*RAD_TO_DEG,  0,0,1)                             ## rotate
        glScale(self.size[0]*0.5, self.size[1]*0.5, self.size[2]*0.5)         ## set size
        glColor3f(self.color[0], self.color[1], self.color[2])    ## set colour
        DrawCube()                                                ## draw a scaled cube
        glPopMatrix()                                             ## restore old coord frame

#####################################################
#### main():   launches app
#####################################################
def main():
    global window
    global links
    global plot_ax, plot_line, plt
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)     # display mode
    glutInitWindowSize(640*2, 480*2)                                  # window size
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

    links = []
    for count in range(num_links):
        links.append(Link())

    resetSim()

    glutMainLoop()                   # start event processing loop

#####################################################
#### resetSim():  Restarts the simulation
#####################################################
def resetSim():
    global links
    global simTime, simRun

    printf("Simulation reset\n")
    simRun = True
    simTime = 0

    for idx in range(num_links):
        link = links[idx]
        link.size=[0.04, link.length, 0.12]
        link.color=[1,0.9,0.9]
        link.vel=np.array([0.0,0.0,0.0])
        link.theta = np.array([0.0, 0.0, pi/4])
        link.omega = np.array([0.0, 0.0, 0.0])        ## radians per second
        link.izz = (link.mass * link.length**2) / 12
        link.f = np.array([0.0, 0.0, 0.0])
        link.kinetic_energy = []
        ### Set link's position
        r_vec = np.array([  -link.length/2 * sin(link.theta[2]),
                            link.length/2 * cos(link.theta[2]),
                            0.0])
        if(idx == 0):
            link.posn = -r_vec
        else:
            r_prev_vec = np.array([ -links[idx-1].length/2 * sin(links[idx-1].theta[2]),
                                    links[idx-1].length/2 * cos(links[idx-1].theta[2]),
                                    0.0])
            link.posn=np.array(links[idx-1].posn - r_prev_vec - r_vec)

#####################################################
#### keyPressed():  called whenever a key is pressed
#####################################################
def keyPressed(key,x,y):
    global simRun
    ch = key.decode("utf-8")
    if ch == ' ':                #### toggle the simulation
            if (simRun == True):
                 simRun = False
            else:
                 simRun = True
    elif ch == chr(27):          #### ESC key
            sys.exit()
    elif ch == 'q':              #### quit
            sys.exit()
    elif ch == 'r':              #### reset simulation
            resetSim()

#####################################################
#### SimWorld():  simulates a time step
#####################################################
def SimWorld():
    global simTime, dT, simRun
    global links, numlinks

    # Plot variables
    global plot_ax, plot_line, plt

    if (simRun==False):             ## is simulation stopped?
            return

    a = np.zeros((9*num_links, 9*num_links))
    b = np.zeros(9*num_links)

    for idx in range(num_links):
        link = links[idx]

        ### Some constants!
        kp_stab=20  #virtual "spring"
        kd_stab=1   #virtual "damper"
        kd=0.5      #frictional damping constant


        ### Construct m
        m = np.zeros((3, 3))
        m[0][0] = link.mass
        m[1][1] = link.mass
        m[2][2] = link.mass


        ### Construct I
        I = np.zeros((3, 3))
        I[0][0] = link.ixx
        I[1][1] = link.iyy
        I[2][2] = link.izz


        ### Construct rtilde and r_vec
        r_x = -link.length/2 * sin(link.theta[2])
        r_y = link.length/2 * cos(link.theta[2])
        r_z = 0.0
        r_vec = np.array([r_x, r_y, r_z])
        rtilde = np.array([   [0, -r_z, r_y],
                            [r_z, 0, -r_x],
                            [-r_y, r_x, 0] ])


        ### Construct omegatilde
        omegatilde = np.array([ [0, -link.omega[2], link.omega[1]],
                                [link.omega[2], 0, -link.omega[0]],
                                [-link.omega[1], link.omega[0], 0] ])


        ### Modify the coefficient matrix with previously made blocks.
        #"M" part
        M_start_idx = idx*6
        a[M_start_idx:M_start_idx+3, M_start_idx:M_start_idx+3] = m#np.concatenate((m, np.zeros((3, 3)), -np.eye(3)), axis=1)

        #"I" part
        I_start_idx = M_start_idx+3
        a[I_start_idx:I_start_idx+3, I_start_idx:I_start_idx+3] = I#np.concatenate((np.zeros((3, 3)), I, -rtilde), axis=1)

        #Constraints part -- first 3x6 block (its upper link constraints)
        C_start_row = 6*num_links+idx*3
        C_start_col = idx*6
        a[C_start_row:C_start_row+3, C_start_col:C_start_col+3] = np.eye(3)
        a[C_start_row:C_start_row+3, C_start_col+3:C_start_col+6] = -rtilde

        #Constraints part -- second 3x6 block (its lower link constraints)
        C_start_row += 3
        if C_start_row < 9*num_links:
            a[C_start_row:C_start_row+3, C_start_col:C_start_col+3] = -np.eye(3)
            a[C_start_row:C_start_row+3, C_start_col+3:C_start_col+6] = -rtilde

        #We leave empty the last "few" columns (which are transpose of the last "few" rows) for now
        #After all links modified a, we set that part in one shot.


        ### Modify constants matrix
        #"mg" part
        mg_start_idx = idx*6
        b[mg_start_idx+1] = -g_const*link.mass

        #"-\omega * I * \omega" part
        #it is zero in 2D

        #Constraints part (this object's share!)
        C_start_idx = 6*num_links + 3*idx
        b[C_start_idx:C_start_idx+3] -= np.dot(omegatilde, np.dot(omegatilde, r_vec))
        if C_start_idx+3 < 9*num_links:
            b[C_start_idx+3:C_start_idx+6] += np.dot(omegatilde, np.dot(omegatilde, -r_vec))

        # Constraint stabilization
        CS_start_idx = 6*num_links + 3*idx
        stab = kp_stab*(0.0 - (link.posn+r_vec)) - kd_stab*( 0 - (link.vel + np.dot(omegatilde,r_vec)) ) #stablization term
        if idx > 0:
            prev = links[idx-1]
            r_prev = np.array([ -prev.length/2 * sin(prev.theta[2]),
                                prev.length/2 * cos(prev.theta[2]),
                                0.0])
            omegatilde_prev = np.array([    [0, -prev.omega[2], prev.omega[1]],
                                            [prev.omega[2], 0, -prev.omega[0]],
                                            [-prev.omega[1], prev.omega[0], 0] ])
            stab = kp_stab*( (prev.posn-r_prev) - (link.posn+r_vec) )
            stab += kd_stab*( (prev.vel + np.dot(omegatilde_prev,-r_prev)) - (link.vel + np.dot(omegatilde,r_vec)) )
        b[CS_start_idx:CS_start_idx+3] += stab

        #Frictional Damping
        friction_torque = -kd*link.omega
        if idx > 0:
            friction_torque += kd*links[idx-1].omega
        FD_start_idx = 6*idx+3
        b[FD_start_idx:FD_start_idx+3] += friction_torque
        if FD_start_idx+6 < 6*num_links:
            b[FD_start_idx+6:FD_start_idx+9] -= friction_torque

    a[:, num_links*6:num_links*9] = a[num_links*6:num_links*9, :].T

    ### Solve the linear system
    x = np.linalg.solve(a, b)

    #### explicit Euler integration to update the state of each link
    for idx in range(num_links):
        link = links[idx]
        offset_idx = idx*6
        acc = x[offset_idx:offset_idx+3]
        omega_dot = x[offset_idx+3:offset_idx+6]
        link.posn += link.vel*dT
        link.vel += acc*dT
        link.theta += link.omega*dT
        link.omega += omega_dot*dT

    simTime += dT

    #### draw the updated state
    DrawWorld()
    printf("simTime=%.2f\n",simTime)

    #calculate kinetic_enery and update the history of kinetic energies in link
    current_kinetic_energy = 0.5 * (link.izz + link.mass * (link.length/2)**2) * link.omega[2]**2
    link.kinetic_energy.append(current_kinetic_energy)

    '''
    #https://stackoverflow.com/questions/20130768/retrieve-xy-data-from-matplotlib-figure
    line = plot_ax.lines[0]
    ydata = line.get_ydata()
    if not ydata:
        ydata = [current_kinetic_energy]
    else:
        ydata.append(current_kinetic_energy)
    plot_line.set_ydata(ydata)
    plot_line.set_xdata([x * dT for x in list(range(len(ydata)))])
    '''

    #plot kinetic energy versus time
    plot_line.set_ydata(link.kinetic_energy)
    plot_line.set_xdata([x * dT for x in list(range(len(link.kinetic_energy)))])
    plot_ax.relim()
    plot_ax.autoscale_view(True,True,True)
    plt.draw()

#####################################################
#### DrawWorld():  draw the world
#####################################################
def DrawWorld():
    global links

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	# Clear The Screen And The Depth Buffer
    glLoadIdentity();
    gluLookAt(1,1,3,  0,0,0,  0,1,0)

    DrawOrigin()
    for idx in range(num_links):
        links[idx].draw()

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

