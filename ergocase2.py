#!/opt/local/bin/python

from solid import *
from solid.utils import *

import numpy as np


thick = 3.5
gap = 2.0
plate_thick = 1.0

floorgap = 3.0
standoff_dia = 10.0
height = 15.

brd = scale(1.0/110.0)(import_dxf("board/ErgoDOX-B_Adhes.dxf", layer = "0"))
brd = right(185)(mirror([1,0,0])(brd))

plate = minkowski()(circle(r=thick,segments=50),brd)
incut = minkowski()(circle(r=gap,segments=50),brd)
body=linear_extrude(height=height)(plate-incut) +\
    down(plate_thick)(linear_extrude(height=plate_thick)(plate))

board = linear_extrude(height=2.5)(brd)


hole_dia = 6.28-0.75
fil_dia = 10.
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

holespt = [ [0,-Hh,0],[Wh,0,0],[0,0,0], [Wh-xh, -yh,0] ]
holes = union()([
    translate(p)(
        cylinder(r=hole_dia/2.,h=height+pin_height,center=True,segments=50)
        )
    for p in holespt ])
holefloors = union()([
    translate(p)(
        cylinder(r=standoff_dia/2.,h=floorgap,segments=50)
        )
    for p in holespt ])

jack_pos = 25.0
jack_dia = 8.0
jackhole = translate([Wh+12-jack_pos,0,jack_dia/2.+floorgap])(
    rot_z_to_y( cylinder(r=jack_dia/2.0,h=2.0*thick,center=True,segments=50) )
    )
usb_pos = 77.0
usb_dia = 8.0
usbhole = translate([Wh+12-usb_pos,0,usb_dia/2.+floorgap])(
    rot_z_to_y( cylinder(r=usb_dia/2.0,h=2.0*thick,center=True,segments=50) )
    )

final = body \
   + translate([standoff_dia/2.,-standoff_dia/2.-4,0])(holefloors) \
      - translate([standoff_dia/2.,-standoff_dia/2.-4,0])(holes) \
      - (jackhole + usbhole) 

def cut(geo, a,b, s1=0.25,s2=0.75):
    t1 = 0.1
    t2 = 0.08
    d = 5.0
    norm = lambda x: np.array([ x[1],-x[0] ])/np.linalg.norm(x)
    an = np.array(a)
    bn = np.array(b)
    n = norm(bn-an)
    
    def nppoly(nda):
        return polygon([list(x) for x in nda])
    def jig(s):
        return [ an+(bn-an)*s,
                 an+(bn-an)*(s-(t2-t1)/2.0) + d*n,
                 an+(bn-an)*(s+t1+(t2-t1)/2.0) + d*n,
                an+(bn-an)*(s+t1) ]
    pts = [a]+jig(s1)+jig(s2)+[b,[b[0]+1000.0,b[1]],[b[0]+1000.0,a[1]-1000.0],[a[0],a[1]-1000.0]]
    male = nppoly(pts)

    
    malecut = down(500.0)(linear_extrude(height=1000.0)(male))

    return geo*malecut, geo-geo*malecut

male1,female1 = cut(final, [80.0,-180],[80,10.0], s1=0.4,s2=0.75)
ycut = -70
male2,female2 = cut(male1, [-10.0,ycut],[160,ycut])
male3,female3 = cut(female1, [-10.0,ycut],[160,ycut])

scad_render_to_file( male3 #+ color(White)(up(1)(board))
                    ,"ergocase.scad")
