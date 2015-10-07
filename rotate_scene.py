from visual import *
scene.material = materials.wood # default material for all objects
box(pos=(-2,0,0), color=color.red)
box(pos=(2,0,0), color=color.green, material=materials.marble)
cylinder(pos=(0,-0.5,0), radius=1, axis=(0,1,0), color=color.orange)
s = sphere(pos=(-2,0.8,0), radius=0.3, color=color.cyan,
           material=materials.emissive)
local_light(pos=s.pos, color=s.color)

lframe = frame()
for obj in scene.lights:
    if isinstance(obj, distant_light):
        obj.frame = lframe # put distant lights in a frame
old = vector(scene.forward) # keep a copy of the old forward
while 1:
    rate(50)
    if scene.forward != old:
        new = scene.forward
        axis = cross(old,new)
        angle = new.diff_angle(old)
        lframe.rotate(axis=axis, angle=angle)
        old = vector(new)