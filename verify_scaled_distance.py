#!/usr/bin/env python3
# verify_scaled_distance.py
import numpy as np, argparse, math

def load_ply_xyz(path):
    pts=[]
    with open(path,'r') as f:
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
        p=l.split()
        if len(p)<8: continue
        ts=float(p[0]); tx,ty,tz = map(float,p[1:4]); qx,qy,qz,qw = map(float,p[4:8])
        out.append((ts,np.array([tx,ty,tz]),np.array([qx,qy,qz,qw])))
    return out

def quat_to_rot(q):
    qx,qy,qz,qw=q; x,y,z,w=qx,qy,qz,qw
    R=np.array([[1-2*(y*y+z*z),2*(x*y - z*w),2*(x*z + y*w)],
                [2*(x*y + z*w),1-2*(x*x+z*z),2*(y*z - x*w)],
                [2*(x*z - y*w),2*(y*z + x*w),1-2*(x*x+y*y)]])
    return R

def find_closest(kfs, ts):
    bd=1e9; idx=0
    for i,(kts,_,_) in enumerate(kfs):
        d=abs(kts-ts)
        if d<bd: bd=d; idx=i
    return kfs[idx]

def project(pts_w, Rcw, tcw, K):
    Xc = (Rcw @ pts_w.T).T + tcw
    valid = Xc[:,2] > 1e-3
    uv = (K @ Xc[valid].T).T
    uv = uv[:,:2] / uv[:,2:3]
    return uv, Xc[valid]

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument("--ply", required=True)
    p.add_argument("--kftum", required=True)
    p.add_argument("--times", required=True)
    p.add_argument("--lanes", required=True)
    p.add_argument("--frame", type=int, required=True)
    p.add_argument("--fx", type=float, required=True)
    p.add_argument("--fy", type=float, required=True)
    p.add_argument("--cx", type=float, required=True)
    p.add_argument("--cy", type=float, required=True)
    p.add_argument("--pix_radius", type=float, default=20.0)
    args=p.parse_args()

    pts = load_ply_xyz(args.ply)
    kfs = load_tum(args.kftum)
    times = [float(l.strip()) for l in open(args.times) if l.strip()]
    clicks={}
    for l in open(args.lanes):
        if l.strip()=='' or l.strip().startswith('#'): continue
        a=l.split(); fid=int(a[0]); clicks[fid]=tuple(map(float,a[1:6]))
    if args.frame not in clicks:
        raise SystemExit("frame not in lane file")
    uL,vL,uR,vR,w = clicks[args.frame]
    ts = times[args.frame]
    kts, t_w, q = find_closest(kfs, ts)
    R_w_c = quat_to_rot(q); Rcw = R_w_c.T; tcw = -Rcw @ t_w
    K = np.array([[args.fx,0,args.cx],[0,args.fy,args.cy],[0,0,1]])
    uv, Xc = project(pts, Rcw, tcw, K)

    def find_match(uv_pts, Xc_pts, u,v,radius):
        d2 = np.sum((uv_pts - np.array([u,v]))**2,axis=1)
        idx = np.where(d2 <= radius*radius)[0]
        if idx.size==0: return None
        best = idx[np.argmin(d2[idx])]
        return Xc_pts[best]

    pL = find_match(uv, Xc, uL, vL, args.pix_radius)
    pR = find_match(uv, Xc, uR, vR, args.pix_radius)
    if pL is None or pR is None:
        print("No match, increase pix_radius")
        raise SystemExit(2)

    # convert camera->world
    Rwc = Rcw.T
    Pw = Rwc @ (pL[:3] - tcw)
    Pr = Rwc @ (pR[:3] - tcw)
    d = np.linalg.norm(Pw - Pr)
    print("Measured distance in map_scaled.ply =", d, "meters (expected ~", w, ")")
