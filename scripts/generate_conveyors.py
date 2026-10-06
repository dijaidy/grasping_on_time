"""Rebuild the two closed-loop MJCF assets using convex annular segments."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def generate(kind):
    root = ET.Element('mujoco', model=f'{kind}_conveyor')
    ET.SubElement(root, 'compiler', angle='radian')
    custom = ET.SubElement(root, 'custom')
    # centerline radius (circle) or half-side and corner radius (rounded square)
    parameters = [0.23, 0.06] if kind == 'circular' else [0.23, 0.10, 0.06]
    ET.SubElement(custom, 'numeric', name='conveyor_path_parameters',
                  data=' '.join(map(str, parameters)))
    asset = ET.SubElement(root, 'asset')
    world = ET.SubElement(root, 'worldbody')
    body = ET.SubElement(world, 'body', name='conveyor')
    ET.SubElement(body, 'site', name='conveyor_origin', size='0.002', rgba='0 0 0 0')
    body.append(ET.Comment('Fixed surface: conveyor.py supplies contact-gated transport forces.'))
    count = 0

    def arc(cx, cy, radius, start, end, n):
        nonlocal count
        for i in range(n):
            a, b = start+(end-start)*i/n, start+(end-start)*(i+1)/n
            points = [(cx+r*math.cos(t), cy+r*math.sin(t))
                      for r,t in [(radius-.06,a),(radius+.06,a),
                                  (radius+.06,b),(radius-.06,b)]]
            vertices = [v for z in [-.015,.015] for x,y in points for v in [x,y,z]]
            name = f'belt_mesh_{count}'
            ET.SubElement(asset, 'mesh', name=name,
                          vertex=' '.join(f'{v:.9f}' for v in vertices))
            ET.SubElement(body, 'geom', name=f'conveyor_belt_{count}', type='mesh',
                          mesh=name, pos='0 0 0.235', friction='0.8 0.005 0.0001',
                          rgba='0.12 0.17 0.20 1')
            count += 1

    if kind == 'circular':
        arc(0,0,.23,0,2*math.pi,96)
    else:
        a,r = .23,.10
        c = a-r
        for cx,cy,start in [(c,c,0),(-c,c,math.pi/2),
                            (-c,-c,math.pi),(c,-c,3*math.pi/2)]:
            arc(cx,cy,r,start,start+math.pi/2,24)
        for name,pos,size in [
            ('top',f'0 {a} 0.235',f'{c} .06 .015'),
            ('bottom',f'0 {-a} 0.235',f'{c} .06 .015'),
            ('left',f'{-a} 0 0.235',f'.06 {c} .015'),
            ('right',f'{a} 0 0.235',f'.06 {c} .015')]:
            ET.SubElement(body,'geom',name=f'conveyor_belt_{name}',type='box',
                          pos=pos,size=size,friction='0.8 0.005 0.0001',rgba='0.12 0.17 0.20 1')
    for i,(x,y) in enumerate([(0,.23),(0,-.23),(.23,0),(-.23,0)]):
        ET.SubElement(body,'geom',name=f'conveyor_leg_{i}',type='box',
                      pos=f'{x} {y} .11',size='.025 .025 .11',rgba='.5 .55 .6 1')
    ET.indent(root)
    ET.ElementTree(root).write(ROOT/'models'/'conveyors'/f'{kind}.xml',encoding='unicode')


if __name__ == '__main__':
    for kind in ['circular','rounded_square']:
        generate(kind)
