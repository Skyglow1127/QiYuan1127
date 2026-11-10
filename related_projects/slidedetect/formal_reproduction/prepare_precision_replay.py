"""Independent integer truth for unchanged SFLS core + optional nHz readout."""
import hashlib,json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'results/feasibility/precision_rtl'


def main():
    OUT.mkdir(parents=True,exist_ok=True);profiles=[]
    for m in [22,96,213,2130]:
        path=OUT/f'vectors_m{m}.txt';events=checks=0
        with path.open('w') as out:
            for trial,offset in enumerate([0,2**62,2**63]):
                out.write('3 0 0 0 0 0\n');rng=np.random.default_rng(2026111115+m+trial)
                j=np.arange(3*m+7,dtype=np.longdouble)
                f=np.longdouble('2128.8700005')
                t=j/f+rng.uniform(0,5e-9)+rng.normal(0,.5e-9,len(j))
                ticks=[offset+100+int(x) for x in np.ceil(t*200000000)]
                for index,tick in enumerate(ticks):
                    events+=1
                    if index>=m-1 and (index-m+1)%max(1,m//4)==0:
                        w=ticks[index-m+1:index+1]
                        q=sum((2*k-m+1)*value for k,value in enumerate(w));total=sum(w)
                        numerator=200000000*(m*(m*m-1)//6)
                        uf=(2*numerator*10**6+q)//(2*q)
                        nf=(2*numerator*10**9+q)//(2*q)
                        out.write(f'2 {tick:x} {q:x} {total:x} {uf:x} {nf:x}\n');checks+=1
                    else:out.write(f'1 {tick:x} 0 0 0 0\n')
        profiles.append(dict(m=m,file=path.name,events=events,checks=checks,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    (OUT/'vectors.json').write_text(json.dumps(dict(profiles=profiles,source_sha256={str(Path(__file__).name):hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}),indent=2)+'\n')


if __name__=='__main__':main()
