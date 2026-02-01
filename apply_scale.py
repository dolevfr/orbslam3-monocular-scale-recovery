#!/usr/bin/env python3
import argparse, numpy as np

def scale_tum(inf, outf, s):
    with open(inf) as f, open(outf,'w') as o:
        for l in f:
            l=l.strip()
            if not l: continue
            parts=l.split()
            ts = parts[0]
            tx,ty,tz = map(float, parts[1:4])
            q = parts[4:8]
            o.write("{:.9f} {:.9f} {:.9f} {:.9f} {} {} {} {}\n".format(
                float(ts), s*tx, s*ty, s*tz, q[0], q[1], q[2], q[3]
            ))

def scale_ply(inply, outply, s):
    with open(inply) as f:
        header=[]
        verts=[]
        while True:
            line = f.readline()
            if not line:
                break
            header.append(line)
            if line.strip().startswith('end_header'):
                break
        for l in f:
            if not l.strip(): continue
            x,y,z = map(float, l.split()[:3])
            verts.append((s*x, s*y, s*z))
    with open(outply,'w') as f:
        for h in header:
            f.write(h)
        for v in verts:
            f.write("{:.6f} {:.6f} {:.6f}\n".format(v[0], v[1], v[2]))

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument("--scale", type=float, required=True)
    parser.add_argument("--kfin", default="KeyFrameTrajectory_unscaled.tum")
    parser.add_argument("--kfout", default="KeyFrameTrajectory_scaled.tum")
    parser.add_argument("--plyin", default="map_unscaled.ply")
    parser.add_argument("--plyout", default="map_scaled.ply")
    args=parser.parse_args()

    scale_tum(args.kfin, args.kfout, args.scale)
    scale_ply(args.plyin, args.plyout, args.scale)
    print("Wrote", args.kfout, "and", args.plyout)
