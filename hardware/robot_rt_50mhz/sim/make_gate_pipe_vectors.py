#!/usr/bin/env python3
"""Pure-Python independent products/cache + legacy Fraction gate oracle.
No RTL expression is used as the expected pass/fail/value source.
"""
import argparse,copy,csv,hashlib,json,re,sys
from fractions import Fraction
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'sim'))
from player_reference import Bounds,PlayerFault,checked_clock_hz,round_even,to_q16,encode_can
import make_gate_vectors
M=(1<<63)-1;MIN=-(1<<63);S=1<<32
NAMES=['q0','v0','a0','j','q1','v1','a1','lower','upper','velocity_limit','acceleration_limit','jerk_limit','dq_max']
def pack(v,bits):return sum((x&((1<<bits)-1))<<(bits*i) for i,x in enumerate(v))
def unpack(x,bits):return [((x>>(i*bits))&((1<<bits)-1))-(1<<bits) if (x>>(i*bits+bits-1))&1 else ((x>>(i*bits))&((1<<bits)-1)) for i in range(7)]
def oracle(raw,dt,previous,previous_valid,clock):
    q,v,a,j,qe,ve,ae,lo,hi,vl,al,jl,dq=raw
    try:
        b=Bounds(*(tuple(x) for x in (lo,hi,vl,al,jl,dq)))
        if dt==0:raise PlayerFault('dt')
        h=Fraction(dt,clock)
        for i in range(7):
            if abs(j[i])>jl[i]:raise PlayerFault('j')
            qcontrol=(Fraction(q[i]),q[i]+v[i]*h/3,q[i]+2*v[i]*h/3+a[i]*h*h/6,q[i]+v[i]*h+a[i]*h*h/2+j[i]*h**3/6)
            vcontrol=(Fraction(v[i]),v[i]+a[i]*h/2,v[i]+a[i]*h+j[i]*h*h/2)
            acontrol=(Fraction(a[i]),a[i]+j[i]*h)
            low,high=max(lo[i],q[i]-dq[i]),min(hi[i],q[i]+dq[i])
            if any(x<low or x>high for x in qcontrol):raise PlayerFault('q hull')
            if any(abs(x)>vl[i] for x in vcontrol):raise PlayerFault('v hull')
            if any(abs(x)>al[i] for x in acontrol):raise PlayerFault('a hull')
            if not low<=qe[i]<=high or abs(ve[i])>vl[i] or abs(ae[i])>al[i]:raise PlayerFault('provided endpoint')
        q16=tuple(to_q16(qe[i],lo[i],hi[i]) for i in range(7))
        current=tuple(to_q16(q[i],lo[i],hi[i]) for i in range(7))
        if any(abs(q16[i]-current[i])*65536>dq[i] for i in range(7)):raise PlayerFault('Q16 DQ')
        wire=encode_can(q16,b,previous if previous_valid else None)
        clamp=any(q16[i]!=round_even(Fraction(qe[i],65536)) for i in range(7)) or any(wire[i]!=round_even(Fraction(q16[i]*scale,65536)) for i,scale in enumerate((1000,)*6+(1000000,)))
        return 1,int(clamp),pack(q16,32),pack(wire,32)
    except (PlayerFault,ValueError):return 0,0,0,0

def bundles(raw,dt,C):
    q,v,a,j,_,_,_,lo,hi,vl,al,_,dq=raw
    Q,V=6*C**3,2*C*C
    products=[[q[i]*Q for i in range(7)],[v[i]*6*dt*C*C for i in range(7)],
              [a[i]*3*dt*dt*C for i in range(7)],[j[i]*dt**3 for i in range(7)],
              [v[i]*V for i in range(7)],[a[i]*2*dt*C for i in range(7)],
              [j[i]*dt*dt for i in range(7)],[a[i]*C for i in range(7)],[j[i]*dt for i in range(7)]]
    caches=[[x*Q for x in lo],[x*Q for x in hi],[x*Q for x in dq],[x*V for x in vl],[x*C for x in al]]
    for vec in products+caches:
        assert all(-(1<<191)<=x<(1<<191) for x in vec)
    return products+caches

def directed(C):
    # Parse the existing22 scalar directed cases, not the new RTL. Check their
    # expected tuples against the independent Fraction oracle below.
    text=(ROOT/'sim/tb_limit_gate.sv').read_text().split('    initial begin',1)[1].split('        if ($value$plusargs',1)[0]
    values={};result=[]
    def ev(s):
        s=s.strip()
        if s in values:return values[s]
        if s=='CLOCK_HZ':return C
        if s=="'0":return 0
        sign=-1 if s.startswith('-') else 1
        s=s.lstrip('-')
        m=re.fullmatch(r"(\d+)'s?([dh])([0-9a-fA-F]+)",s)
        if m:return sign*int(m.group(3),10 if m.group(2)=='d' else 16)
        return sign*int(s)
    for statement in re.sub(r'//[^\n]*','',text).split(';'):
        statement=statement.strip()
        if not statement:continue
        if statement=='base()':
            values={name:0 for name in NAMES}
            values.update(previous_wire=0,previous_wire_valid=1,expected_q16=0,expected_wire=0,dt_cycles=1000000)
            for name in ['lower']:values[name]=pack([-10*S]*7,64)
            for name in ['upper','velocity_limit','acceleration_limit','jerk_limit']:values[name]=pack([10*S]*7,64)
            values['dq_max']=pack([S]*7,64)
        elif statement.startswith('check('):
            args=next(csv.reader([statement[6:-1]],skipinitialspace=True))
            raw=[unpack(values[n],64) for n in NAMES]
            expected=tuple(ev(x) for x in args[1:])
            got=oracle(raw,values['dt_cycles'],unpack(values['previous_wire'],32),values['previous_wire_valid'],C)
            assert got==expected,(args[0],got,expected)
            result.append((args[0],raw,values['dt_cycles'],unpack(values['previous_wire'],32),values['previous_wire_valid'],got,(1,1,0)))
        else:
            m=re.fullmatch(r'(\w+)(?:\[(\d+)\+:(\d+)\])?=(.+)',statement)
            assert m,statement
            name,offset,bits,expr=m.groups();value=ev(expr)
            if bits:
                offset,bits=int(offset),int(bits);mask=((1<<bits)-1)<<offset
                values[name]=(values[name]&~mask)|((value&((1<<bits)-1))<<offset)
            else:values[name]=value
    assert len(result)==22,len(result)
    return result

def rows(C,count):
    result=directed(C)
    text,_=make_gate_vectors.generate(2601102,count,clock_hz=C)
    for n,line in enumerate(text.splitlines()):
        f=[int(x,16) for x in line.split()]
        raw=[unpack(x,64) for x in f[:13]];prev=unpack(f[14],32)
        got=oracle(raw,f[13],prev,f[15],C)
        assert got==tuple(f[16:20]),('existing vector',n,got,f[16:20])
        result.append((f'existing_python_{n}',raw,f[13],prev,f[15],got,(1,1,0)))
    def add(name,raw,dt=1,prev=None,pv=0,flags=(1,1,0)):
        got=oracle(raw,dt,prev or [0]*7,pv,C)
        result.append((name,copy.deepcopy(raw),dt,prev or [0]*7,pv,got,flags))
    stationary=copy.deepcopy(result[0][1])
    for label,flags in [('terms_invalid',(0,1,0)),('cache_invalid',(1,0,0)),('math_overflow',(1,1,1))]:
        add(label,stationary,flags=flags)
    base=[[0]*7 for _ in range(13)]
    base[7]=[-1000*S]*7;base[8]=[1000*S]*7
    for index in (9,10,11):base[index]=[M]*7
    base[12]=[1000*S]*7
    for axisstate in (1,2,3):
        for value in (M,-M,MIN):
            raw=copy.deepcopy(base);raw[axisstate][0]=value
            h=Fraction(1,C)
            for i in range(7):
                q,v,a,j=(raw[k][i] for k in range(4))
                ends=(q+v*h+a*h*h/2+j*h**3/6,v+a*h+j*h*h/2,a+j*h)
                for k,x in enumerate(ends,4):raw[k][i]=round_even(x)
            add(f'full_i64_state{axisstate}_{value}',raw)
    for sign in (1,-1):
        raw=copy.deepcopy(stationary);raw[0][0]=raw[4][0]=sign*2129921
        add(f'double_RNE_{sign}',raw,dt=1)
        expected=oracle(raw,1,[0]*7,0,C)
        assert unpack(expected[3],32)[0]==sign and round_even(Fraction(raw[4][0]*1000,S))==0
    raw=copy.deepcopy(stationary);raw[0][0]=32*65536;raw[4][0]=33*65536
    raw[1][0]=raw[5][0]=65536;raw[12][0]=(6*S)//10000
    add('Q16_32_to33_CAN_0_to1_DQ_reject',raw,dt=C,prev=[0]*7,pv=1)
    raw=copy.deepcopy(base)
    for k in range(4):raw[k]=[MIN if (k+i)%2 else M for i in range(7)]
    raw[7]=[MIN]*7;raw[8]=[M]*7;raw[4]=raw[5]=raw[6]=[0]*7
    add('all_axes_full_width_max_dt',raw,dt=0xffffffff,flags=(1,1,1))
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--clock-hz',type=int,default=99999001);p.add_argument('--count',type=int,default=700);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists() or a.output.with_suffix('.json').exists():p.error('output and companion manifest must be new files')
    if a.count<1:p.error('--count must be positive')
    try:checked_clock_hz(a.clock_hz)
    except ValueError as exc:p.error(str(exc))
    result=rows(a.clock_hz,a.count);lines=[str(len(result))];descriptions=[]
    for name,raw,dt,prev,pv,old,flags in result:
        bundles_values=bundles(raw,dt,a.clock_hz)
        f=[f'{pack(x,64):0112x}' for x in raw]+[f'{dt:08x}',f'{pack(prev,32):056x}',str(pv),str(old[0]),str(old[1]),f'{old[2]:056x}',f'{old[3]:056x}']
        f += [f'{pack(x,192):0336x}' for x in bundles_values]+[str(x) for x in flags]
        assert len(f)==37
        lines.append(' '.join(f));descriptions.append(dict(name=name,legacy_allow=old[0],flags=flags))
    payload='\n'.join(lines)+'\n';a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as handle:handle.write(payload)
    manifest=dict(provenance='synthetic_design_only',clock_hz=a.clock_hz,seed=2601102,count=len(result),vectors_sha256=hashlib.sha256(payload.encode()).hexdigest(),
       oracle='direct Fraction legacy Bernstein + player_reference Q16/CAN; products/cache independently integer formed',sources={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in [ROOT/'sim/player_reference.py',ROOT/'sim/make_gate_vectors.py',ROOT/'sim/tb_limit_gate.sv',Path(__file__)]},cases=descriptions)
    with a.output.with_suffix('.json').open('x') as handle:handle.write(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(output=str(a.output.resolve()),count=len(result),clock_hz=a.clock_hz)))
if __name__=='__main__':main()
