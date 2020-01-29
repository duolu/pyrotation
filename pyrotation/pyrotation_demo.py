'''
This file is part of the pyrotation python module, which is designed to help
teaching and learning the math of 3D rotation. This file contains code to demo
and test the core class and funcitons of 3D rotation.

Author: Duo Lu <duolu.cs@gmail.com>

Version: 0.1
License: GPLv3

Updated on Jan. 28, 2020, version 0.1

Created on Mar 22, 2019, draft

'''

import sys
import math
from math import copysign, fabs, sqrt, pi, sin, cos, asin, acos, atan2, exp, log
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

from matplotlib.widgets import Slider, Button

from pyrotation import *



def rotate_translate_points(p, rotation, t, method):
    '''
    Rotate and translate 3D points given a rotation representation in
    "rotation" and a translation vector "t". The rotation can be either a
    rotation matrix, a quaternion, or an angle-axis vector, indicated by
    the "method". There are three methods, as follows.
    
        (1) angle-axis, indicated by method = "u".
        (2) rotation matrix, indicated by method = "R".
        (3) quaternion, indicated by method = "q". 
    
    The input points "p" must be a numpy 3-by-n matrix, and the translation
    vector must be a numpy 3-by-1 matrix.
    
    '''
    
    if method == 'u': # angle-axis

        p = rotate_points_by_angle_axis(p, rotation) + t
        
    elif method == 'R': # rotation matrix
        
        p = np.matmul(rotation, p) + t
        
    elif method == 'q': # quaternion
        
        p = rotation.rotate_points(p) + t

    else:
        
        raise ValueError('Unknown method: %s' % str(method))
    
    return p


class Arrow3D(FancyArrowPatch):
    '''
    3D arrow object the can be drawn with matplotlib. 
    
    '''
    
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)



class RotationVisualizer3D(object):
    '''
    Abstract visualizer class which contains a bunch of helper functions.
    This class can not be used directly for visualizing a 3D rotation.
    
    '''

        

    def plot_arrow(self, ax, ox, oy, oz, ux, uy, uz, color='k'):
        '''
        Plot an 3D arrow from (ox, oy, oz) to (ux, uy, uz).
        
        '''
        arrow = Arrow3D((ox, ux), (oy, uy), (oz, uz), mutation_scale=15, 
                    lw=2, arrowstyle="-|>", color=color)
    
        ax.add_artist(arrow)

    def plot_vector(self, ax, x, y, z, ox=0, oy=0, oz=0, 
                    style='-', color='k', arrow=False, arrow_rho=0.9):
        '''
        Plot a 3D vector from (ox, oy, oz) to (x, y, z).
        
        '''
        
        ax.plot((ox, x), (oy, y), (oz, z), style, color=color)
        
        if arrow:
            
            aox = ox * (1 - arrow_rho) + x * arrow_rho
            aoy = oy * (1 - arrow_rho) + y * arrow_rho
            aoz = oz * (1 - arrow_rho) + z * arrow_rho
            
            self.plot_arrow(ax, aox, aoy, aoz, x, y, z, color=color)


    def plot_xyz_axes(self, ax, rotation, t, scale=1, style='-', 
                           cx='r', cy='g', cz='b', arrow=False, method='R'):
        '''
        Plot the xyz axes indication using three short line segments in red, green 
        and blue, given the new reference frame indicated by (R, t). 
        
        '''
    
        x = np.asarray([[0, 0, 0], [scale, 0, 0]]).T
        x = rotate_translate_points(x, rotation, t, method)
    
        y = np.asarray([[0, 0, 0], [0, scale, 0]]).T
        y = rotate_translate_points(y, rotation, t, method)
    
        z = np.asarray([[0, 0, 0], [0, 0, scale]]).T
        z = rotate_translate_points(z, rotation, t, method)

        self.plot_vector(ax, x[0, 1], x[1, 1], x[2, 1],
                    x[0, 0], x[1, 0], x[2, 0], 
                    style=style, color=cx, arrow=arrow)

        self.plot_vector(ax, y[0, 1], y[1, 1], y[2, 1],
                    y[0, 0], y[1, 0], y[2, 0], 
                    style=style, color=cy, arrow=arrow)

        self.plot_vector(ax, z[0, 1], z[1, 1], z[2, 1],
                    z[0, 0], z[1, 0], z[2, 0], 
                    style=style, color=cz, arrow=arrow)
    
    

    
    def plot_arc_points(self, ax, x, y, z, rotation, t, 
                        style='-', color='k', arrow=False, method='R'):
        '''
        Plot a sequence of 3D points of an arc.
        
        '''
        
        nr_points = x.shape[0]
        
        p = np.zeros((3, nr_points))
        p[0, :] = x
        p[1, :] = y
        p[2, :] = z
        
        p = rotate_translate_points(p, rotation, t, method)
        
        ax.plot(p[0, :], p[1, :], p[2, :], style, color=color)

        if arrow:
            
            self.plot_arrow(ax, p[0, -2], p[1, -2], p[2, -2], 
                   p[0, -1], p[1, -1], p[2, -1], color=color)

    def plot_surface(self, ax, x, y, z, rotation, t, color='w', method='R'):
        '''
        Fill a surface in the 3D space using a set of points.
        
        CAUTION: matplotlib is not a full-fledged 3D rendering engine!
        There might be problems when multiple surfaces are plotted.
        
        '''
        s = x.shape
        x = x.flatten()
        y = y.flatten()
        z = z.flatten()
        
        p = np.zeros((3, x.shape[0]))
        p[0, :] = x
        p[1, :] = y
        p[2, :] = z
        
        p = rotate_translate_points(p, rotation, t, method)
          
        x = p[0, :].reshape(s)
        y = p[1, :].reshape(s)
        z = p[2, :].reshape(s)
        
        ax.plot_surface(x, y, z, color=color, linewidth=0, antialiased=False)

    
    def generate_arc_angles(self, start, angle, step=0.02):
        '''
        Generate a sequence of angles on an arc.
        
        '''
        
        if fabs(angle) < step * 2:
            
            return None
        
        if angle > 2 * np.pi:
            
            angle = 2 * np.pi
    
        if angle < -2 * np.pi:
            
            angle = -2 * np.pi
    
        a = np.linspace(start, start + angle, fabs(angle) // step)

        return a
    
    def generate_sector_radius_and_angles(self, start, angle, r, step=0.02):
        '''
        Generate a sequence of radius and angles on a sector.
        
        CAUTION: This uses numpy.meshgrid().
        
        '''
        
        if fabs(angle) < 0.04 or fabs(r) < 0.1:
            
            return None
        
        if angle > 2 * np.pi:
            
            angle = 2 * np.pi
    
        if angle < -2 * np.pi:
            
            angle = -2 * np.pi

        rs = np.linspace(0, r, r // 0.1)
        ps = np.linspace(start, start + angle, fabs(angle) // 0.1)
        RS, PS = np.meshgrid(rs, ps)
    
        return RS, PS
    
    def plot_arc(self, ax, start, angle, rotation, t, r, 
            plane='xoy', style='-', color='k', arrow=False, method='R'):
        '''
        Plot an arc in 3D.
        
        '''
        
        a = self.generate_arc_angles(start, angle)
        
        if a is not None:
            
            if plane == 'xoy':
            
                x = r * np.cos(a)
                y = r * np.sin(a)
                z = np.zeros(a.shape)
                
            elif plane == 'yoz':
                
                x = np.zeros(a.shape)
                y = r * np.cos(a)
                z = r * np.sin(a)
                
            elif plane == 'zox':
                
                x = r * np.sin(a)
                y = np.zeros(a.shape)
                z = r * np.cos(a)
                
            else:
                
                raise ValueError('Unknown plane: %s' % str(plane))
                
            self.plot_arc_points(ax, x, y, z, rotation, t, 
                style=style, color=color, arrow=arrow, method=method)

        

    def plot_circle(self, ax, rotation, t, r, 
            plane='xoy', style='-', color='k', arrow=False, method='R'):
        '''
        Plot a circle in 3D.
        
        '''
        
        self.plot_arc(ax, 0, 2 * np.pi, rotation, t, r, 
            plane=plane, style=style, color=color, arrow=arrow, method=method)
    
    def plot_sector(self, ax, start, angle, rotation, t, r, 
                    plane='xoy', color='w', method='R'):
        '''
        Plot a sector in 3D.
        
        '''
        
        tup = self.generate_sector_radius_and_angles(start, angle, r)
        
        if tup is not None:
            
            RS, PS = tup
            
            if plane == 'xoy':
            
                x = RS * np.cos(PS)
                y = RS * np.sin(PS)
                z = np.zeros(x.shape)
                
            elif plane == 'yoz':
                
                y = RS * np.cos(PS)
                z = RS * np.sin(PS)
                x = np.zeros(y.shape)
                
            elif plane == 'zox':
                
                z = RS * np.cos(PS)
                x = RS * np.sin(PS)
                y = np.zeros(z.shape)
                
            else:
                
                raise ValueError('Unknown plane: %s' % str(plane))
            
            self.plot_surface(ax, x, y, z, rotation, t, 
                              color=color, method=method)

    def plot_disk(self, ax, rotation, t, r, plane='xoy', color='w', method='R'):
        '''
        Plot a disk in 3D.
        
        '''

        self.plot_sector(ax, 0, 2 * np.pi, rotation, t, r, 
                         plane=plane, color=color, method=method)
    
    

    
    
    
    def adjust_axes(self, ax, scale=2):
        '''
        Adjust the limite and aspect ratio of a 3D plot.
        
        '''
        
        ax.set_xlim(-scale, scale)
        ax.set_ylim(-scale, scale)
        ax.set_zlim(-scale, scale)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_aspect('equal')

    pass



    def run(self):
        
        plt.show()



class AngleAxisVisualizer3D(RotationVisualizer3D):
    '''
    This class demonstrates 3D rotation in the angle-axis representation with
    a 3D plot.
    
    The user can control the direction of the axis (using alt-azimuth angles)
    and the rotation angle with sliders.
    
    '''

    def __init__(self):
        '''
        Constructor.
        '''
        
        self.reset_states()
        self.update_internal_states()

        self.setup_ui()

    def reset_states(self):
        '''
        Reset to the initial states, where the alt angle is 90 degree, the
        azimuth is 0 degree, and the rotation angle is 0 degree.
        
        This means the axis is pointing to the z-axis of the original frame.
         
        '''
        
        self.alt_degree = 90
        self.azimuth_degree = 0
        self.angle_degree = 0

    def update_internal_states(self):
        '''
        Converting angle states read from the sliders to internal states
        (i.e., angles in radian, and direction in vectors).
        
        '''
        
        self.alt_radian = self.alt_degree * np.pi / 180
        self.azimuth_radian = self.azimuth_degree * np.pi / 180
        self.angle_radian = self.angle_degree * np.pi / 180
        
        self.ud = alt_azimuth_to_axis(self.alt_degree, self.azimuth_degree)
        self.u = self.ud * self.angle_radian
        

    def setup_ui(self):
        '''
        Set up the UI and the 3D plot.
        
        '''
        
        fig = plt.figure(figsize=(8, 10), dpi=100, facecolor='w', edgecolor='k')

        ax3d = fig.add_axes([0.1, 0.3, 0.8, 0.7], projection='3d')
        
        # set up control sliders

        axcolor = 'lightgoldenrodyellow'
        
        ax_alt = fig.add_axes([0.2, 0.1, 0.65, 0.03], facecolor=axcolor)
        ax_azimuth = fig.add_axes([0.2, 0.15, 0.65, 0.03], facecolor=axcolor)
        ax_angle = fig.add_axes([0.2, 0.2, 0.65, 0.03], facecolor=axcolor)

        s_alt = Slider(ax_alt, 'alt', -90, 90, valinit=self.alt_degree, valstep=1)
        s_azimuth = Slider(ax_azimuth, 'azimuth', -180, 180, valinit=self.azimuth_degree, valstep=1)
        s_angle = Slider(ax_angle, 'angle', -180, 180, valinit=self.angle_degree, valstep=1)

        s_alt.on_changed(self.on_angle_axis_slider_update)
        s_azimuth.on_changed(self.on_angle_axis_slider_update)
        s_angle.on_changed(self.on_angle_axis_slider_update)

        self.ax_alt = ax_alt
        self.ax_azimuth = ax_azimuth
        self.ax_angle = ax_angle

        self.s_alt = s_alt
        self.s_azimuth = s_azimuth
        self.s_angle = s_angle

        ax_none = fig.add_axes([0.2, 0.3, 0.65, 0.03])
        ax_none.axis('off')
        

        self.fig = fig
        self.ax3d = ax3d
        
        self.update_ax3d_plot()
        
        

    def update_ax3d_plot(self):
        '''
        Update the 3D plot based on internal states.
        
        Most computations of rotation use rotation matrix for efficiency.
        A few uses angle-axis directly for demo the correctness of those
        functions.
        
        '''
        
        self.ax3d.clear()

        I = np.identity(3)
        O = np.asarray((0, 0, 0)).reshape((3, 1))
        r = 2
        scale = 2
        
        u0 = np.asarray((0, 0, 0))
        ui = np.asarray((scale, 0, 0))
#         uj = np.asarray((0, scale, 0))
#         uk = np.asarray((0, 0, scale))

        u = self.u
        ud = self.ud
        us = ud * scale
        angle = self.angle_radian
        
        u_xoy = np.asarray((scale * cos(self.azimuth_radian), 
                            scale * sin(self.azimuth_radian), 
                            0))

        ux = rotate_a_point_by_angle_axis(ui, u)


        # Plot the original XOY plane.
        self.plot_disk(self.ax3d, I, O, r, 
                       plane='xoy', color='w', method='R')
        self.plot_circle(self.ax3d, I, O, r, 
                         plane='xoy', style='-', color='k', method='R')


        # Plot the rotation axis, its project on the original XOY plane, and 
        # the angle-axis vector u
        self.plot_vector(self.ax3d, us[0], us[1], us[2], 
                         -us[0], -us[1], -us[2], style='-.', arrow=False)
        self.plot_vector(self.ax3d, u_xoy[0], u_xoy[1], u_xoy[2], style=':')
        self.plot_vector(self.ax3d, u[0], u[1], u[2], arrow=True)
 
 
        # Plot the conic structure along the rotation axis
        utx = np.dot(ui, self.ud) * self.ud
        tx = utx.reshape((3, 1))
        rx = np.linalg.norm(ui - utx)
        Rx = rotation_matrix_from_zx(ud, ui)
 
        self.plot_circle(self.ax3d, Rx, tx, rx, 
                      plane='xoy', style='--', color='k', method='R')
        self.plot_arc(self.ax3d, 0, angle, Rx, tx, rx, 
                      plane='xoy', style='-', color='r', arrow=True, method='R')
 
        self.plot_vector(self.ax3d, ui[0], ui[1], ui[2],
                         utx[0], utx[1], utx[2],
                         style='--', color='k', arrow=True)
 
        self.plot_vector(self.ax3d, ux[0], ux[1], ux[2],
                         utx[0], utx[1], utx[2],
                         style='--', color='k', arrow=True)
 
 
 
        # Plot the original axes.
        self.plot_xyz_axes(self.ax3d, u0, O, scale=scale,
            style='-', cx='k', cy='k', cz='k', arrow=True, method='u')
        self.plot_xyz_axes(self.ax3d, u0, O, scale=scale,
            style='--', cx='r', cy='g', cz='b', arrow=False, method='u')

        # Plot the rotated axes.        
        self.plot_xyz_axes(self.ax3d, u, O, scale=scale, arrow=True, method='u')
        
        self.adjust_axes(self.ax3d, scale=scale * 0.75)

    def reset_sliders(self):
        '''
        Reset the slider to inital values.
        
        '''
        
        self.s_alt.reset()
        self.s_azimuth.reset()
        self.s_angle.reset()

    def on_angle_axis_slider_update(self, val):
        '''
        Event handler of the sliders.
        
        '''
        
        del val
        
        self.alt_degree = self.s_alt.val
        self.azimuth_degree = self.s_azimuth.val
        self.angle_degree = self.s_angle.val
        
        self.update_internal_states()
        
        self.update_ax3d_plot()



class EulerZYXVisualizer3D(RotationVisualizer3D):
    '''
    This class demonstrates 3D rotation in the Euler angle representation in
    z-y'-x" (intrinsic) convention with a 3D plot.
    
    The user can control the three angles with sliders.
    
    '''

    def __init__(self):
        '''
        Constructor.
        '''

        self.reset_states()
        self.update_internal_states()

        self.setup_ui()

    def reset_states(self):
        '''
        Reset to the initial states, where all three angles are zero, i.e.,
        no rotation.
        
        '''

        self.euler_z_degree = 0
        self.euler_y_degree = 0
        self.euler_x_degree = 0

    def update_internal_states(self):
        '''
        Converting angle states read from the sliders to internal states
        (i.e., angles in radian, etc.).
        
        Internally, a rotation matrix is used for calculation and plot.
        
        '''
        
        self.euler_z_radian = self.euler_z_degree * np.pi / 180
        self.euler_y_radian = self.euler_y_degree * np.pi / 180
        self.euler_x_radian = self.euler_x_degree * np.pi / 180

        self.R = euler_zyx_to_rotation_matrix( \
            self.euler_z_radian, self.euler_y_radian, self.euler_x_radian)


    def setup_ui(self):
        '''
        Set up the UI and the 3D plot.
        
        '''
        
        fig = plt.figure(figsize=(8, 10), dpi=100, facecolor='w', edgecolor='k')

        ax3d = fig.add_axes([0.1, 0.3, 0.8, 0.7], projection='3d')
        
        # set up control sliders
        
        axcolor = 'lightgoldenrodyellow'
        
        ax_ez = fig.add_axes([0.2, 0.1, 0.65, 0.03], facecolor=axcolor)
        ax_ey = fig.add_axes([0.2, 0.15, 0.65, 0.03], facecolor=axcolor)
        ax_ex = fig.add_axes([0.2, 0.2, 0.65, 0.03], facecolor=axcolor)
        
        s_ez = Slider(ax_ez, '$\psi$ (yaw)', -180, 180, valinit=self.euler_z_degree, valstep=1)
        s_ey = Slider(ax_ey, '$\\theta$ (pitch)', -180, 180, valinit=self.euler_y_degree, valstep=1)
        s_ex = Slider(ax_ex, '$\phi$ (roll)', -180, 180, valinit=self.euler_x_degree, valstep=1)
        
        s_ez.on_changed(self.on_euler_angles_slider_update)
        s_ey.on_changed(self.on_euler_angles_slider_update)
        s_ex.on_changed(self.on_euler_angles_slider_update)


        self.ax_ez = ax_ez
        self.ax_ey = ax_ey
        self.ax_ex = ax_ex

        self.s_ez = s_ez
        self.s_ey = s_ey
        self.s_ex = s_ex


        self.fig = fig
        self.ax3d = ax3d
        
        
        self.update_ax3d_plot()

    def update_ax3d_plot(self):
        '''
        Update the 3D plot based on internal states.
        
        All computations of rotation use rotation matrix.
        
        '''
        
        self.ax3d.clear()

        I = np.identity(3)
        r = 2
        scale = 2
        O = np.asarray((0, 0, 0)).reshape((3, 1))

#         ui = np.asarray((scale, 0, 0))
#         uj = np.asarray((0, scale, 0))
#         uk = np.asarray((0, 0, scale))
        
        R = self.R

        # Calculate x-axis and y-axis after the yaw rotation. This is needed
        # to plot the pitch angle.
        Rz = euler_zyx_to_rotation_matrix( \
            self.euler_z_radian, 0, 0)
        nx = np.asarray((r, 0, 0)).reshape((3, 1))
        nx = np.matmul(Rz, nx).flatten()
        ny = np.asarray((0, r, 0)).reshape((3, 1))
        ny = np.matmul(Rz, ny).flatten()

        # Calculate z-axis after the yaw rotation and the pitch rotation.
        # This is needed to plot the pitch angle.
        Rzy = euler_zyx_to_rotation_matrix( \
            self.euler_z_radian, self.euler_y_radian, 0)
        mz = np.asarray((0, 0, r)).reshape((3, 1))
        mz = np.matmul(Rzy, mz).flatten()



        # Plot the original XOY plane.
        self.plot_disk(self.ax3d, I, O, r, plane='xoy', color='w')
        self.plot_circle(self.ax3d, I, O, r, plane='xoy', style='-', color='k')
        
        
        # ---------- yaw -----------------
        
        # Plot the yaw angle
        self.plot_arc(self.ax3d, 0, self.euler_z_radian, I, O, r, 
                          style='-', color='b', arrow=True)

        

        # ---------- pitch -----------------
        

        # Plot the pitch angle on the ZOX plane after the yaw rotation.
        self.plot_arc(self.ax3d, np.pi / 2, self.euler_y_radian, Rz, O, r, 
                          plane='zox', style='-', color='g', arrow=True)
        
        # Plot x-axis and y-axis after the yaw rotation.
        self.plot_vector(self.ax3d, nx[0], nx[1], nx[2], 
                                style=':', color='r')
        self.plot_vector(self.ax3d, ny[0], ny[1], ny[2], 
                                style=':', color='g')


        # ---------- roll -----------------
       
        
        # Plot the roll angle on the YOZ plane after yaw and pitch.
        self.plot_circle(self.ax3d, R, O, r, plane='yoz', style='--', color='k')
        self.plot_arc(self.ax3d, 0, self.euler_x_radian, Rzy, O, r, 
                          plane='yoz', style='-', color='r', arrow=True)

        # Plot z-axis after the yaw rotation and the pitch rotation.
        self.plot_vector(self.ax3d, mz[0], mz[1], mz[2], 
                                style=':', color='b')


        # Plot the original axes.
        self.plot_xyz_axes(self.ax3d, I, O, scale=scale,
                                style='-', cx='k', cy='k', cz='k', arrow=True)
        self.plot_xyz_axes(self.ax3d, I, O, scale=scale,
                                style='--', cx='r', cy='g', cz='b', arrow=False)


        # Plot the rotated reference frame.
        self.plot_xyz_axes(self.ax3d, R, O, scale=2, arrow=True)
        
        
        self.adjust_axes(self.ax3d, scale=scale * 0.75)
        

    def on_euler_angles_slider_update(self, val):
        '''
        Event handler of the sliders.
        
        '''
        
        del val
        
        self.euler_z_degree = self.s_ez.val
        self.euler_y_degree = self.s_ey.val
        self.euler_x_degree = self.s_ex.val
        
        self.update_internal_states()
        
        self.update_ax3d_plot()
        



class RotationMatrixVisualizer3D(RotationVisualizer3D):
    '''
    This class demonstrates 3D rotation in the rotation matrix representation
    with a 3D plot.
    
    The user can control the direction of a pair of basis vectors after the 
    rotation with sliders. Since it is generally difficult to directly
    manipulate 3D vectors through the UI, instead, this demo program uses
    alt-azimuth angles to control the direction of the basis vectors.
    
    NOTE: There three ways to manipulate a pair of basis vectors after the
    rotation, as follows.
    
        (1) ux-uy, i.e., manipulating the x-axis and y-axis
        (2) uy-uz, i.e., manipulating the y-axis and z-axis
        (3) uz-ux, i.e., manipulating the z-axis and x-axis
    
    '''

    def __init__(self):
        '''
        Constructor.
        '''

        self.modes = ['ux-uy', 'uy-uz', 'uz-ux']
        self.mode_idx = 0
        self.mode = self.modes[self.mode_idx]

        self.reset_states()
        self.update_internal_states()

        self.setup_ui()

    def reset_states(self):
        '''
        Reset to the initial states, where the three basis vectors after 
        rotation is the same as before rotation, i.e., the identity rotation 
        is applied.
         
        '''

        self.ux_alt_degree = 0
        self.ux_azimuth_degree = 0

        self.uy_alt_degree = 0
        self.uy_azimuth_degree = 90

        self.uz_alt_degree = 90
        self.uz_azimuth_degree = 0


    def update_internal_states(self):
        '''
        Converting angle states read from the sliders to internal states
        (i.e., angles in radian, and direction in vectors).
        
        The rotation is internally represented by a rotation matrix.
        
        '''
        
        self.ux_alt_radian = self.ux_alt_degree * np.pi / 180
        self.ux_azimuth_radian = self.ux_azimuth_degree * np.pi / 180

        self.uy_alt_radian = self.uy_alt_degree * np.pi / 180
        self.uy_azimuth_radian = self.uy_azimuth_degree * np.pi / 180

        self.uz_alt_radian = self.uz_alt_degree * np.pi / 180
        self.uz_azimuth_radian = self.uz_azimuth_degree * np.pi / 180

        self.ux = alt_azimuth_to_axis(self.ux_alt_degree, self.ux_azimuth_degree)
        self.uy = alt_azimuth_to_axis(self.uy_alt_degree, self.uy_azimuth_degree)
        self.uz = alt_azimuth_to_axis(self.uz_alt_degree, self.uz_azimuth_degree)

        if self.mode == 'ux-uy':
            
            self.R = rotation_matrix_from_xy(self.ux, self.uy)

        elif self.mode == 'uy-uz':
            
            self.R = rotation_matrix_from_yz(self.uy, self.uz)
            
        elif self.mode == 'uz-ux':
            
            self.R = rotation_matrix_from_zx(self.uz, self.ux)
            
        else:
            
            raise ValueError('Unknown mode: %s' % str(self.mode))
        
        

    def setup_ui(self):
        '''
        Set up the UI and the 3D plot.
        
        '''
        
        fig = plt.figure(figsize=(8, 10), dpi=100, facecolor='w', edgecolor='k')

        ax3d = fig.add_axes([0.1, 0.3, 0.8, 0.7], projection='3d')
        
        # set up control sliders

        axcolor = 'lightgoldenrodyellow'
        
        ax_ux_alt       = fig.add_axes([0.1, 0.2, 0.3, 0.03], facecolor=axcolor)
        ax_ux_azimuth   = fig.add_axes([0.6, 0.2, 0.3, 0.03], facecolor=axcolor)
        ax_uy_alt       = fig.add_axes([0.1, 0.15, 0.3, 0.03], facecolor=axcolor)
        ax_uy_azimuth   = fig.add_axes([0.6, 0.15, 0.3, 0.03], facecolor=axcolor)
        ax_uz_alt       = fig.add_axes([0.1, 0.1, 0.3, 0.03], facecolor=axcolor)
        ax_uz_azimuth   = fig.add_axes([0.6, 0.1, 0.3, 0.03], facecolor=axcolor)

        ax_mode_prev    = fig.add_axes([0.1, 0.25, 0.1, 0.05])
        ax_mode_next    = fig.add_axes([0.8, 0.25, 0.1, 0.05])
        ax_mode_str     = fig.add_axes([0.2, 0.25, 0.6, 0.05])

        ax_rm_str       = fig.add_axes([0.1, 0.03, 0.8, 0.05])

        s_ux_alt = Slider(ax_ux_alt, 'alt-ux', -90, 90, 
                          valinit=self.ux_alt_degree, valstep=1)
        s_ux_azimuth = Slider(ax_ux_azimuth, 'azimuth-ux', -180, 180, 
                              valinit=self.ux_azimuth_degree, valstep=1)
        s_uy_alt = Slider(ax_uy_alt, 'alt-uy', -90, 90, 
                          valinit=self.uy_alt_degree, valstep=1)
        s_uy_azimuth = Slider(ax_uy_azimuth, 'azimuth-uy', -180, 180, 
                              valinit=self.uy_azimuth_degree, valstep=1)
        s_uz_alt = Slider(ax_uz_alt, 'alt-uz', -90, 90, 
                          valinit=self.uz_alt_degree, valstep=1)
        s_uz_azimuth = Slider(ax_uz_azimuth, 'azimuth-uz', -180, 180, 
                              valinit=self.uz_azimuth_degree, valstep=1)

        b_mode_prev = Button(ax_mode_prev, '<-')
        b_mode_next = Button(ax_mode_next, '->')

        s_ux_alt.on_changed(self.on_rotm_slider_update)
        s_ux_azimuth.on_changed(self.on_rotm_slider_update)
        s_uy_alt.on_changed(self.on_rotm_slider_update)
        s_uy_azimuth.on_changed(self.on_rotm_slider_update)
        s_uz_alt.on_changed(self.on_rotm_slider_update)
        s_uz_azimuth.on_changed(self.on_rotm_slider_update)

        b_mode_prev.on_clicked(self.on_mode_prev)
        b_mode_next.on_clicked(self.on_mode_next)

        self.ax_ux_alt = ax_ux_alt
        self.ax_ux_azimuth = ax_ux_azimuth
        self.ax_uy_alt = ax_uy_alt
        self.ax_uy_azimuth = ax_uy_azimuth
        self.ax_uz_alt = ax_uz_alt
        self.ax_uz_azimuth = ax_uz_azimuth

        self.ax_mode_prev = ax_mode_prev
        self.ax_mode_next = ax_mode_next
        self.ax_mode_str = ax_mode_str

        self.ax_rm_str = ax_rm_str

        self.s_ux_alt = s_ux_alt
        self.s_ux_azimuth = s_ux_azimuth
        self.s_uy_alt = s_uy_alt
        self.s_uy_azimuth = s_uy_azimuth
        self.s_uz_alt = s_uz_alt
        self.s_uz_azimuth = s_uz_azimuth
        
        self.b_mode_prev = b_mode_prev
        self.b_mode_next = b_mode_next

        self.fig = fig
        self.ax3d = ax3d
        
        self.update_ax3d_plot()
        self.update_mode_label()

        self.fig.canvas.draw()
    
    
    def plot_ux_uy(self, R, r, scale):
        '''
        Update the 3D plot based on internal states (ux-uy case).

        '''

        O = np.asarray((0, 0, 0)).reshape((3, 1))

        ux_xoy = np.asarray((scale * cos(self.ux_azimuth_radian), 
                            scale * sin(self.ux_azimuth_radian), 
                            0))

        uy_xoy = np.asarray((scale * cos(self.uy_azimuth_radian), 
                            scale * sin(self.uy_azimuth_radian), 
                            0))
        
        z = np.cross(self.ux, self.uy)
        degenerated = fabs(np.linalg.norm(z)) < EPSILON
        
        if not degenerated:

            self.plot_circle(self.ax3d, R, O, r, 
                      plane='xoy', style='--', color='k')
        
            self.plot_xyz_axes(self.ax3d, R, O, scale=scale, arrow=True)
            
        else:
            
            x = self.ux * scale
            self.plot_vector(self.ax3d, x[0], x[1], x[2],
                             style='-', color='r', arrow=True)

        y = self.uy * scale
        self.plot_vector(self.ax3d, y[0], y[1], y[2],
                         style='--', color='g', arrow=True)

        # Plot the project of ux and uy on the original XOY plane.
        self.plot_vector(self.ax3d, ux_xoy[0], ux_xoy[1], ux_xoy[2], 
                         style=':', color='r')
        self.plot_vector(self.ax3d, uy_xoy[0], uy_xoy[1], uy_xoy[2], 
                         style=':', color='g')
        
        self.update_rotation_matrix_text(degenerated)

    def plot_uy_uz(self, R, r, scale):
        '''
        Update the 3D plot based on internal states (uy-uz case).

        '''
        
        O = np.asarray((0, 0, 0)).reshape((3, 1))

        uy_xoy = np.asarray((scale * cos(self.uy_azimuth_radian), 
                            scale * sin(self.uy_azimuth_radian), 
                            0))

        uz_xoy = np.asarray((scale * cos(self.uz_azimuth_radian), 
                            scale * sin(self.uz_azimuth_radian), 
                            0))
        
        x = np.cross(self.uy, self.uz)
        degenerated = fabs(np.linalg.norm(x)) < EPSILON
        
        if not degenerated:

            self.plot_circle(self.ax3d, R, O, r, 
                      plane='yoz', style='--', color='k')
        
            self.plot_xyz_axes(self.ax3d, R, O, scale=scale, arrow=True)
            
        else:
            
            y = self.uy * scale
            self.plot_vector(self.ax3d, y[0], y[1], y[2],
                             style='-', color='g', arrow=True)

        z = self.uz * scale
        self.plot_vector(self.ax3d, z[0], z[1], z[2],
                         style='--', color='b', arrow=True)

        # Plot the project of uy and uz on the original XOY plane.
        self.plot_vector(self.ax3d, uy_xoy[0], uy_xoy[1], uy_xoy[2], 
                         style=':', color='g')
        self.plot_vector(self.ax3d, uz_xoy[0], uz_xoy[1], uz_xoy[2], 
                         style=':', color='b')

        self.update_rotation_matrix_text(degenerated)

    def plot_uz_ux(self, R, r, scale):
        '''
        Update the 3D plot based on internal states (uz-ux case).

        '''
        
        O = np.asarray((0, 0, 0)).reshape((3, 1))
        
        uz_xoy = np.asarray((scale * cos(self.uz_azimuth_radian), 
                            scale * sin(self.uz_azimuth_radian), 
                            0))

        ux_xoy = np.asarray((scale * cos(self.ux_azimuth_radian), 
                            scale * sin(self.ux_azimuth_radian), 
                            0))
        
        
        y = np.cross(self.uz, self.ux)
        degenerated = fabs(np.linalg.norm(y)) < EPSILON
        
        if not degenerated:

            self.plot_circle(self.ax3d, R, O, r, 
                      plane='zox', style='--', color='k')
        
            self.plot_xyz_axes(self.ax3d, R, O, scale=scale, arrow=True)
            
        else:
            
            z = self.uz * scale
            self.plot_vector(self.ax3d, z[0], z[1], z[2],
                             style='--', color='b', arrow=True)

        x = self.ux * scale
        self.plot_vector(self.ax3d, x[0], x[1], x[2],
                         style='--', color='r', arrow=True)

        # Plot the project of uz and ux on the original XOY plane.
        self.plot_vector(self.ax3d, uz_xoy[0], uz_xoy[1], uz_xoy[2], 
                         style=':', color='b')
        self.plot_vector(self.ax3d, ux_xoy[0], ux_xoy[1], ux_xoy[2], 
                         style=':', color='r')

        self.update_rotation_matrix_text(degenerated)

    def update_ax3d_plot(self):
        '''
        Update the 3D plot based on internal states.

        '''
        
        
        self.ax3d.clear()

        R = self.R

        r = 2
        scale = 2

        I = np.identity(3)
        O = np.asarray((0, 0, 0)).reshape((3, 1))


        # Plot the original axes.
        #self.plot_xyz_axes(self.ax3d, I, O, scale=scale)
        self.plot_xyz_axes(self.ax3d, I, O, scale=scale, style='-',
                                cx='k', cy='k', cz='k', arrow=True)
        self.plot_xyz_axes(self.ax3d, I, O, scale=scale, style='--', 
                                cx='r', cy='g', cz='b', arrow=False)

        # Plot the original XOY plane.
        self.plot_disk(self.ax3d, I, O, r, plane='xoy', color='w')
        self.plot_circle(self.ax3d, I, O, r, plane='xoy', style='-', color='k')



        if self.mode == 'ux-uy':

            self.plot_ux_uy(R, r, scale)


        elif self.mode == 'uy-uz':

            self.plot_uy_uz(R, r, scale)
            
        elif self.mode == 'uz-ux':

            self.plot_uz_ux(R, r, scale)
            
        else:
            
            raise ValueError('Unknown mode: %s' % str(self.mode))


        
        self.adjust_axes(self.ax3d, scale=scale * 0.75)

    def reset_sliders(self):
        '''
        Reset the slider to inital values.
        
        '''
        
        self.s_ux_alt.reset()
        self.s_ux_azimuth.reset()
        self.s_uy_alt.reset()
        self.s_uy_azimuth.reset()
        self.s_uz_alt.reset()
        self.s_uz_azimuth.reset()

    def update_mode_label(self):
        '''
        Update label string indicating the current mode according to the 
        internal states.
        
        '''
        
        self.ax_mode_str.clear()
        self.ax_mode_str.text(0.5, 0.5, str(self.mode),
                      horizontalalignment='center',
                      verticalalignment='center')
        self.ax_mode_str.axis('off')

    def update_rotation_matrix_text(self, degenerated):
        '''
        Update text string indicating the three basis vectors after the
        rotation. These are essentially the three columns of the rotation
        matrix.
        
        '''
        
        if degenerated:
            
            rm_str = 'Degenerated.'
            
        else:
        
            R = self.R
            x_str = 'x = (%.3f, %.3f, %.3f)' % (R[0, 0], R[1, 0], R[2, 0])
            y_str = 'y = (%.3f, %.3f, %.3f)' % (R[0, 1], R[1, 1], R[2, 1])
            z_str = 'z = (%.3f, %.3f, %.3f)' % (R[0, 2], R[1, 2], R[2, 2])
            rm_str = x_str + ', ' + y_str + ', ' + z_str
        
        self.ax_rm_str.clear()
        self.ax_rm_str.text(0.5, 0.5, rm_str,
                      horizontalalignment='center',
                      verticalalignment='center')
        self.ax_rm_str.axis('off')
        

    def on_rotm_slider_update(self, val):
        '''
        Event handler of the sliders.
        
        '''
        
        del val
        
        self.ux_alt_degree = self.s_ux_alt.val
        self.ux_azimuth_degree = self.s_ux_azimuth.val
        self.uy_alt_degree = self.s_uy_alt.val
        self.uy_azimuth_degree = self.s_uy_azimuth.val
        self.uz_alt_degree = self.s_uz_alt.val
        self.uz_azimuth_degree = self.s_uz_azimuth.val


        self.update_internal_states()

        self.update_ax3d_plot()

    def update_mode(self):
        '''
        Update the current mode (triggered by clicking buttons).
        
        '''
        
        self.reset_states()
        self.update_internal_states()

        self.reset_sliders()
        self.update_ax3d_plot()
        self.update_mode_label()

        self.fig.canvas.draw()


    def on_mode_prev(self, val):
        '''
        Event handler of the button for going to the previous mode.
        
        '''
        
        del val
        
        self.mode_idx -= 1
        if self.mode_idx < 0:
            self.mode_idx = 0
            
        self.mode = self.modes[self.mode_idx]
        
        self.update_mode()


    def on_mode_next(self, val):
        '''
        Event handler of the button for going to the next mode.
        
        '''
        
        del val
        
        nr_modes = len(self.modes)
        
        self.mode_idx += 1
        if self.mode_idx >= nr_modes:
            self.mode_idx = nr_modes - 1
            
        self.mode = self.modes[self.mode_idx]
                    
        self.update_mode()


class QuaternionVisualizer3D(RotationVisualizer3D):
    '''
    This class demonstrates 3D rotation in the unit quaternion representation
    with a 3D plot.
    
    The user can control the four components of the unit quaternion with 
    sliders.
    
    NOTE: Since it is a unit quaternion, the four components are coupled and
    the user can not change one component without influence to others. This
    demo program provides two manipulations in general, as follows.
    
        (1) Changing the rotation angle while maintaining the axis, by
            manipulating qw.
        (2) Changing the axis direction while maintaining the angle, by
            manipulating qx, qy, qz and setting qw to 0.
    
    '''


    def __init__(self):
        '''
        Constructor.
        
        '''


        self.reset_states()
        self.update_internal_states()

        self.setup_ui()

    def reset_states(self):
        '''
        Reset to the inital states, where no rotation is applied.
        '''

        # These are the four components of the unit quaternion.
        self.qw = 1
        self.qx = 0
        self.qy = 0
        self.qz = 0

        # These indicates which slider is triggered. 
        self.qw_update = False
        self.qx_update = False
        self.qy_update = False
        self.qz_update = False
        self.quaternion_updating = False
        

    def update_internal_states(self):
        '''
        Converting the values read from the sliders to internal states.
        
        Internally, a unit quaternion is used for calculation and plot.
        
        '''
        
        self.q = Quaternion(self.qw, self.qx, self.qy, self.qz)
        
    def setup_ui(self):
        '''
        Set up the UI and the 3D plot.
        
        '''
        
        
        fig = plt.figure(figsize=(8, 10), dpi=100, facecolor='w', edgecolor='k')

        ax3d = fig.add_axes([0.1, 0.3, 0.8, 0.7], projection='3d')
        
        # set up control sliders

        axcolor = 'lightgoldenrodyellow'

        ax_qw = fig.add_axes([0.2, 0.25, 0.65, 0.03], facecolor=axcolor)
        ax_qx = fig.add_axes([0.2, 0.2, 0.65, 0.03], facecolor=axcolor)
        ax_qy = fig.add_axes([0.2, 0.15, 0.65, 0.03], facecolor=axcolor)
        ax_qz = fig.add_axes([0.2, 0.1, 0.65, 0.03], facecolor=axcolor)

        s_qw = Slider(ax_qw, 'w', -1.0, 1.0, valinit=self.qw, valstep=0.01)
        s_qx = Slider(ax_qx, 'x', -1.0, 1.0, valinit=self.qx, valstep=0.01)
        s_qy = Slider(ax_qy, 'y', -1.0, 1.0, valinit=self.qy, valstep=0.01)
        s_qz = Slider(ax_qz, 'z', -1.0, 1.0, valinit=self.qz, valstep=0.01)

        s_qw.on_changed(self.on_quaternion_slider_update_qw)
        s_qx.on_changed(self.on_quaternion_slider_update_qx)
        s_qy.on_changed(self.on_quaternion_slider_update_qy)
        s_qz.on_changed(self.on_quaternion_slider_update_qz)


        self.ax_qw = ax_qw
        self.ax_qx = ax_qx
        self.ax_qy = ax_qy
        self.ax_qz = ax_qz

        self.s_qw = s_qw
        self.s_qx = s_qx
        self.s_qy = s_qy
        self.s_qz = s_qz

        self.fig = fig
        self.ax3d = ax3d

        self.update_ax3d_plot()


    def update_ax3d_plot(self):
        '''
        Update the 3D plot based on internal states.
        
        All computations of rotation use unit quaternion.
        
        '''
        
        self.ax3d.clear()

        qi = Quaternion.identity()
        O = np.asarray((0, 0, 0)).reshape((3, 1))
        r = 2
        scale = 2

        u = self.q.to_angle_axis()
        #nu = np.linalg.norm(u)

        # Plot the original XOY plane.
        self.plot_disk(self.ax3d, qi, O, r, 
            plane='xoy', color='w', method='q')
        self.plot_circle(self.ax3d, qi, O, r, 
            plane='xoy', style='-', color='k', method='q')

        # Plot the original axes.
        self.plot_xyz_axes(self.ax3d, qi, O, scale=scale,
            style='-', cx='k', cy='k', cz='k', arrow=True, method='q')
        self.plot_xyz_axes(self.ax3d, qi, O, scale=scale,
            style='--', cx='r', cy='g', cz='b', arrow=False, method='q')

#         self.plot_circle(self.ax3d, self.q, O, r, 
#                          plane='xoy', style='--', color='b', method='q')

        self.plot_circle(self.ax3d, self.q, O, r, 
                         plane='yoz', style='--', color='k', method='q')

#         self.plot_circle(self.ax3d, self.q, O, r, 
#                          plane='zox', style='--', color='g', method='q')

        # Plot the rotated axes.        
        self.plot_xyz_axes(self.ax3d, self.q, O, scale=scale,
            style='-', cx='r', cy='g', cz='b', arrow=True, method='q')

        # Plot the rotation axis
        self.plot_vector(self.ax3d, u[0], u[1], u[2], arrow=True)

        self.adjust_axes(self.ax3d, scale=scale * 0.75)

    def on_quaternion_slider_update_qw(self, val):
        '''
        Event handler of the slider qw.
        
        NOTE: It is needed to track which slider is triggered.
        
        '''
        
        self.qw_update = True
        self.on_quaternion_slider_update(val)
        self.qw_update = False

    def on_quaternion_slider_update_qx(self, val):
        '''
        Event handler of the slider qx.
        
        NOTE: It is needed to track which slider is triggered.
        
        '''
        
        self.qx_update = True
        self.on_quaternion_slider_update(val)
        self.qx_update = False

    def on_quaternion_slider_update_qy(self, val):
        '''
        Event handler of the slider qy.
        
        NOTE: It is needed to track which slider is triggered.
        
        '''
        
        self.qy_update = True
        self.on_quaternion_slider_update(val)
        self.qy_update = False

    def on_quaternion_slider_update_qz(self, val):
        '''
        Event handler of the slider qz.
        
        NOTE: It is needed to track which slider is triggered.
        
        '''
        
        self.qz_update = True
        self.on_quaternion_slider_update(val)
        self.qz_update = False

    def construct_unit_quaternion(self, qw, qx, qy, qz):
        '''
        Construct a unit quaternion from the slider values.
        
        NOTE: Since all four sliders can be changed, the four values may not
        be a unit quaternion. This function adjusts the components to make a
        unit quaternion based on the slider values. There are two types of
        manipulations: (1) change the angle and keep the axis, or (2) change
        the axis and keep the angle.
        
        '''
        
        qw2 = qw * qw
        qx2 = qx * qx
        qy2 = qy * qy
        qz2 = qz * qz
        
        if self.qw_update:
            
            res = 1 - qw2
            
            if fabs(qx) < EPSILON and fabs(qy) < EPSILON and fabs(qz) < EPSILON:
                
                qx = sqrt(res / 3)
                qy = sqrt(res / 3)
                qz = sqrt(res / 3)
                
            else:
            
                more = qx2 + qy2 + qz2 + EPSILON
                k = sqrt(res / more)
                qx = k * qx
                qy = k * qy
                qz = k * qz

            qw2_res = 1 - qx * qx - qy * qy - qz * qz
            qw = copysign(sqrt(qw2_res), qw)
        
        if self.qx_update:
            
            res = 1 - qx * qx
            more = qy2 + qz2 + EPSILON
            k = sqrt(res / more)
            qy = k * qy
            qz = k * qz

            qw2_res = 1 - qx * qx - qy * qy - qz * qz
            qw = sqrt(qw2_res)

        if self.qy_update:
            
            res = 1 - qy * qy
            more = qx2 + qz2 + EPSILON
            k = sqrt(res / more)
            qx = k * qx
            qz = k * qz

            qw2_res = 1 - qx * qx - qy * qy - qz * qz
            qw = sqrt(qw2_res)

        if self.qz_update:
            
            res = 1 - qz * qz
            more = qx2 + qy2 + EPSILON
            k = sqrt(res / more)
            qx = k * qx
            qy = k * qy
        
            qw2_res = 1 - qx * qx - qy * qy - qz * qz
            qw = sqrt(qw2_res)
        
        return qw, qx, qy, qz


    def on_quaternion_slider_update(self, val):
        '''
        Event handler of the sliders.
        
        '''
        

        del val
        
        # This prevents recursion triggered by Sider.set_val().
        if self.quaternion_updating:
            return
        
        self.quaternion_updating = True
        
        qw = self.s_qw.val
        qx = self.s_qx.val
        qy = self.s_qy.val
        qz = self.s_qz.val
        
        qw, qx, qy, qz = self.construct_unit_quaternion(qw, qx, qy, qz)

        self.qw = qw
        self.qx = qx
        self.qy = qy
        self.qz = qz
        
        self.update_internal_states()

        # CAUTION: The values on the slider may need adjustment after 
        # normalizing the four components to a unit quaternion.
        
        self.s_qw.set_val(qw)
        self.s_qx.set_val(qx)
        self.s_qy.set_val(qy)
        self.s_qz.set_val(qz)
        
        self.update_ax3d_plot()

        self.quaternion_updating = False




def test_visualizer():
    
    vis = AngleAxisVisualizer3D()
    #vis = EulerZYXVisualizer3D()
    
    #vis = RotationMatrixVisualizer3D()
    #vis = QuaternionVisualizer3D()
    
    #vis = RotationConversionVisualizer3D()
    #vis = AngularSpeedVisualizer()
    
    vis.run()
    

def print_usage():
    
    print('Usage: "pyrotation_demo [mode]", mode can be:')
    print('\tu - angle-axis')
    print('\te - Euler angles (z-y\'-x")')
    print('\tr - rotation matrix')
    print('\tq - quaternion')



if __name__ == '__main__':
    
    argc = len(sys.argv)
    
    if argc < 2:
        
        mode = 'u'
        
    elif argc == 2:
        
        mode = sys.argv[1]
        
    else:
        
        print_usage()
    
    if mode == 'u' or mode == 'angle_axis':
        
        vis = AngleAxisVisualizer3D()
        
    elif mode == 'e' or mode == 'euler':
    
        vis = EulerZYXVisualizer3D()
        
    elif mode == 'r' or mode == 'rotation_matrix':
        
        vis = RotationMatrixVisualizer3D()
        
    elif mode == 'q' or mode == 'quaternion':
        
        vis = QuaternionVisualizer3D()
    
    
    
    vis.run()
    
    pass














