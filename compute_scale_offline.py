#!/usr/bin/env python3
# compute_scale_offline.py
import numpy as np, sys, math, argparse
from pathlib import Path

def load_ply_xyz(path):
    pts=[]
    with open(path,'r') as f:
        # skip header
        line=f.readline()
        while not line.strip().startswith('end_header'):
            line=f.readline()
        for l in f:
            toks=l.split()
            if len(toks)>=3:
                pts.append([float(toks[0]),float(toks[1]),float(toks[2])])
    return np.array(pts)

def load_tum(path):
    out=[]
    for l in open(path):
        l=l.strip()
        if not l: continue
        parts=l.split()
        ts=float(parts[0])
        tx,ty,tz = map(float, parts[1:4])
        qx,qy,qz,qw = map(float, parts[4:8])
        out.append((ts, np.array([tx,ty,tz]), np.array([qx,qy,qz,qw])))
    return out

def load_times(path):
    return [float(l.strip()) for l in open(path) if l.strip()]

def quat_to_rot(q):
    qx,qy,qz,qw = q
    x,y,z,w = qx,qy,qz,qw
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w),   2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w),   1-2*(x*x+y*y)]
    ])
    return R

def find_closest_keyframe(kfs, ts):
    best=None; bd=1e9
    for (kts,t, q) in kfs:
        d=abs(kts-ts)
        if d<bd:
            bd=d; best=(kts,t,q)
    return best

def project_points(pts_w, Rcw, tcw, K):
    Xc = (Rcw @ pts_w.T).T + tcw
    valid = Xc[:,2] > 0.001
    uv = (K @ Xc[valid].T).T
    uv = uv[:, :2] / uv[:, 2:3]
    return uv, Xc[valid]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--ply", required=True)
    parser.add_argument("--kftum", required=True)
    parser.add_argument("--times", required=True)
    parser.add_argument("--lanes", required=True)
    parser.add_argument("--frame", type=int, required=True)
    parser.add_argument("--fx", type=float, required=True)
    parser.add_argument("--fy", type=float, required=True)
    parser.add_argument("--cx", type=float, required=True)
    parser.add_argument("--cy", type=float, required=True)
    parser.add_argument("--pix_radius", type=float, default=20.0)
    args=parser.parse_args()

    pts = load_ply_xyz(args.ply)
    kfs = load_tum(args.kftum)
    times = load_times(args.times)

    clicks={}
    for l in open(args.lanes):
        if l.strip()=='' or l.strip().startswith('#'): continue
        p=l.split()
        fid=int(p[0]); uL=float(p[1]); vL=float(p[2]); uR=float(p[3]); vR=float(p[4]); w=float(p[5])
        clicks[fid]=(uL,vL,uR,vR,w)
    if args.frame not in clicks:
        print("frame not in lane file"); sys.exit(1)
    uL,vL,uR,vR,w = clicks[args.frame]
    ts = times[args.frame]

    kf = find_closest_keyframe(kfs, ts)
    if kf is None:
        print("no keyframe close"); sys.exit(1)
    kts, t_w, q = kf
    R_w_c = quat_to_rot(q)
    Rcw = R_w_c.T
    tcw = -Rcw @ t_w

    K = np.array([[args.fx,0,args.cx],[0,args.fy,args.cy],[0,0,1.0]])

    uv, Xc = project_points(pts, Rcw, tcw, K)

    def find_match(uv_pts, Xc_pts, u, v, radius):
        d2 = np.sum((uv_pts - np.array([u,v]))**2,axis=1)
        idx = np.where(d2 <= radius*radius)[0]
        if idx.size==0: return None
        best = idx[np.argmin(d2[idx])]
        return Xc_pts[best]

    pL = find_match(uv, Xc, uL, vL, args.pix_radius)
    pR = find_match(uv, Xc, uR, vR, args.pix_radius)
    if pL is None or pR is None:
        print("Could not find cached map points near clicks with radius", args.pix_radius)
        sys.exit(2)

    Rwc = Rcw.T
    Pw = Rwc @ (pL[:3] - tcw)
    Pr = Rwc @ (pR[:3] - tcw)

    dslam = np.linalg.norm(Pw - Pr)
    scale = w / dslam
    print("frame",args.frame,"d_slam=",dslam,"scale=",scale)

if __name__=='__main__':
    main()
