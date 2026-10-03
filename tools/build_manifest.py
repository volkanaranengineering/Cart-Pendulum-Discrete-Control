"""Record all Git-visible archive files except manifest.json itself."""
from pathlib import Path
import hashlib,json,csv,subprocess
ROOT=Path(__file__).resolve().parents[1]

def digest(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()

def main():
    data=ROOT/'study/discrete_results';catalog=ROOT/'docs/DATA_CATALOG.csv'
    with catalog.open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['path','bytes','role','snapshot_date'])
        for p in sorted(data.iterdir()):
            if not p.is_file():continue
            role='full-resolution trajectory' if p.suffix=='.csv' and p.name not in ('summary.csv','differences.csv') else {'results.mat':'complete MATLAB results struct','summary.csv':'18-row controller metric table','differences.csv':'same-rate and fast-rate comparison','validation.txt':'MATLAB validation record'}.get(p.name,{'png':'figure/preview','fig':'editable MATLAB figure','mp4':'six-controller animation'}.get(p.suffix[1:],'study interpretation'))
            w.writerow([p.relative_to(ROOT).as_posix(),p.stat().st_size,role,'2026-10-03'])
    proc=subprocess.run(['git','ls-files','--cached','--others','--exclude-standard','-z'],cwd=ROOT,check=True,capture_output=True)
    paths=sorted(set(x.decode('utf-8') for x in proc.stdout.split(b'\0') if x))
    records=[]
    for name in paths:
        if name=='manifest.json':continue
        p=ROOT/name
        if p.is_file():records.append({'path':name,'bytes':p.stat().st_size,'sha256':digest(p)})
    (ROOT/'manifest.json').write_text(json.dumps({'snapshot_date':'2026-10-03','timezone':'Europe/Istanbul','description':'Discrete cart-pendulum study; SHA-256 of hydrated file contents','files':records},indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(f'Manifest: {len(records)} files, {sum(r["bytes"] for r in records):,} bytes')

if __name__=='__main__':main()
