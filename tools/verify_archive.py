"""Stdlib integrity check plus independent CSV metric replay; no MATLAB needed."""
from pathlib import Path
import csv,hashlib,json,math,argparse
ROOT=Path(__file__).resolve().parents[1]

def require(condition,message):
    if not condition:raise RuntimeError(message)

def close(a,b,name,tol=1e-7):
    require((math.isnan(a) and math.isnan(b)) or abs(a-b)<=tol*max(1,abs(b)),f'{name}: {a} != {b}')

def hashes():
    records=json.loads((ROOT/'manifest.json').read_text(encoding='utf-8'))['files']
    for r in records:
        p=ROOT/r['path'];require(p.is_file(),f'Missing file: {r["path"]}')
        require(p.stat().st_size==r['bytes'],f'Size mismatch: {r["path"]}. Run git lfs pull if this is an LFS pointer.')
        h=hashlib.sha256()
        with p.open('rb') as f:
            for chunk in iter(lambda:f.read(4*1024*1024),b''):h.update(chunk)
        require(h.hexdigest()==r['sha256'],f'Hash mismatch: {r["path"]}')
    print(f'PASS: {len(records)} SHA-256 file hashes',flush=True)

def metrics():
    data=ROOT/'study/discrete_results'
    with (data/'summary.csv').open() as f:summary={r['controller']:r for r in csv.DictReader(f) if r['variant']=='discrete_01ms'}
    for label,expected in summary.items():
        peaks=[0.,0.,0.];area=0.;prev=None;count=0;lastbad=[None,None];loss=math.nan
        with (data/f'discrete_01ms_{label}_kick.csv').open() as fk,(data/f'discrete_01ms_{label}_baseline.csv').open() as fb:
            kick=csv.DictReader(fk);base=csv.DictReader(fb)
            for k in kick:
                b=next(base,None);require(b is not None,'Baseline shorter than kick')
                t=float(k['t']);close(t,count*1e-4,'time grid',1e-10);close(float(b['t']),t,'paired time')
                x=float(k['x']);theta=float(k['theta']);u=float(k['u']);dx=x-float(b['x']);dt=theta-float(b['theta'])
                require(all(math.isfinite(v) for v in (x,theta,u,dx,dt)),f'{label} nonfinite');require(abs(u)<=10+1e-8,'Actuator saturation')
                close(float(k['bob_x']),x-.3*math.sin(theta),'bob x');close(float(k['bob_y']),.3*math.cos(theta),'bob y')
                if math.isnan(loss) and abs(theta)>=math.pi/2:loss=t
                if t>=8:
                    peaks=[max(peaks[0],abs(theta)*180/math.pi),max(peaks[1],abs(dt)*180/math.pi),max(peaks[2],abs(dx))]
                    if prev is not None:area+=(t-prev[0])*(u*u+prev[1]*prev[1])/2
                    prev=(t,u)
                if t>=8.1:
                    if abs(dt)>=math.pi/180:lastbad[0]=t
                    if abs(dx)>=.01:lastbad[1]=t
                count+=1
            require(next(base,None) is None,'Baseline longer than kick')
        require(count==200001,f'{label} sample count: {count}')
        settle=[0 if v is None else (math.nan if v+1e-4>19 else v+1e-4-8.1) for v in lastbad]
        vals=peaks+[x,theta*180/math.pi,math.sqrt(area/12)]+settle+[loss]
        keys=['postkick_peak_angle_deg','pulse_peak_angle_deg','pulse_peak_x_m','final_x_m','final_angle_deg','postkick_rms_force_N','pulse_angle_settle_s','pulse_x_settle_s','first90deg_s']
        for key,value in zip(keys,vals):close(value,float(expected[key]),f'{label}/{key}')
        print(f'PASS: {label}, {count} samples, geometry, limits and 9 metrics',flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--hash-only',action='store_true');args=parser.parse_args()
    hashes()
    if not args.hash_only:metrics()
