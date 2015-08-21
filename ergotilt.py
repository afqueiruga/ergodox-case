#!/opt/local/bin/python

from solid import *
from solid.utils import *

import numpy as np

hole_dia = 6.28-0.75
fil_dia = 10.
height = 20.
pin_height = 5.
Hh = 96.66+hole_dia
Wh = 134.04
Ah = 98.24
Bh = 126.81
ch = np.sqrt(Hh**2. + Wh**2.)
t1h = np.arcsin( Hh/ch )
t2h = np.arccos( (ch**2. + Bh**2. - Ah**2.)/(2.0*ch*Bh) )
t3h = np.pi/2. - t1h - t2h
xh = Bh * np.sin(t3h)
yh = Bh * np.cos(t3h)


#
# The baseplate
#
def stand1():
    pts = [ [0,0,0],[xh,0,0],[0,yh,0] ]
    cyl = lambda p,r,h : translate(p)(cylinder(r=r,h=h,segments=50))
    R1 = fil_dia/2.0
    HIT = lambda x : hull()( union()( x ) )
    cornersbase = [ cyl(p,R1,5.0) for p in pts]
    cornershigh = [  cyl(p,R1,height) for p in [[0,0,0],[0,yh-R1,0]]]

    L1 = HIT([ cyl(p,R1,5.0) for p in [[0,0,0],[xh,0,0]] ])
    L2 = HIT([ cyl(p,R1,5.0) for p in [[0,0,0],[0,yh,0]]])
    Legs = HIT([ cyl(p,R1,height) for p in [[0,0,0],[0,yh-2.0*R1,0]]])
    
    body = L1 + L2 + Legs
    cutL = (Wh+fil_dia)
    omega = 180./np.pi * np.arcsin( height / (Wh+fil_dia) )
    cut = back(hole_dia)(right( (cutL-.75*fil_dia))(
        rotate(v=[0,1,0],a=omega)(
            left(cutL)(
                cube([cutL,1.5*Hh,2.*height])
            )
        )
    ))
    cutrad = 70.0
    cutout = rot_z_to_x(cylinder(r=cutrad,h=3.0*R1,segments=100,center=True))
    pins = union()([ translate(p)(cylinder(r=hole_dia/2.,h=height+pin_height,segments=50)) for p in [[xh,0,0],[0,yh,0]] ])
    return body - cut - down(1.0)(pins) - translate([0,(yh-2.0*R1)/2.0,cutrad+5.0])(cutout)

final = stand1()

scad_render_to_file( final, "ergotitl.scad")
