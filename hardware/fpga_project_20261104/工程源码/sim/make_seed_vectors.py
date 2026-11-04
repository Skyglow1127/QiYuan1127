#!/usr/bin/env python3
"""Independent exact-Fraction oracle; no simulator or hardware access."""
import argparse
import hashlib
import json
import random
from fractions import Fraction
from pathlib import Path

S32=1<<32
I32MIN=-(1<<31)
I32MAX=(1<<31)-1
I64MIN=-(1<<63)
I64MAX=(1<<63)-1
FIELDS=('seed_q','lower','upper','velocity_limit','acceleration_limit','jerk_limit','dq_max')

def floor(x): return x.numerator//x.denominator

def ceil(x): return -floor(-x)

def rne(x):
    n=floor(x); rest=x-n
    return n+int(rest>Fraction(1,2) or (rest==Fraction(1,2) and n%2))

def oracle(fields):
    """Same declared contract, rational arithmetic independent of RTL widths."""
    codes=[]; wires=[]; clamped=False
    for axis in range(7):
        q,lo,hi,vl,al,jl,dq=(fields[f][axis] for f in FIELDS)
        if not all(I64MIN<=v<=I64MAX for v in (q,lo,hi,vl,al,jl,dq)):
            raise ValueError('raw input outside signed64')
        if not(lo<hi and min(vl,al,jl,dq)>0 and lo<=q<=hi):
            return 0,0,[0]*7,[0]*7
        qlo=ceil(Fraction(lo,1<<16)); qhi=floor(Fraction(hi,1<<16))
        if qlo>qhi or qlo<I32MIN or qhi>I32MAX:
            return 0,0,[0]*7,[0]*7
        nearest=rne(Fraction(q,1<<16)); code=min(max(nearest,qlo),qhi)
        if abs(code-nearest)>1 or not(lo<=code*(1<<16)<=hi):
            return 0,0,[0]*7,[0]*7
        clamped|=code!=nearest
        unit=1000000 if axis==6 else 1000
        wlo=ceil(Fraction(lo*unit,S32)); whi=floor(Fraction(hi*unit,S32))
        if wlo>whi or wlo<I32MIN or whi>I32MAX:
            return 0,0,[0]*7,[0]*7
        nearest=rne(Fraction(code*unit,1<<16)); wire=min(max(nearest,wlo),whi)
        if abs(wire-nearest)>1 or not(lo*unit<=wire*S32<=hi*unit):
            return 0,0,[0]*7,[0]*7
        clamped|=wire!=nearest
        codes.append(code); wires.append(wire)
    return 1,int(clamped),codes,wires

def base():
    return dict(seed_q=[0]*7,lower=[-10*S32]*6+[0],
                upper=[10*S32]*6+[S32//10],
                velocity_limit=[S32]*7,acceleration_limit=[S32]*7,
                jerk_limit=[S32]*7,dq_max=[1]*7)

def clone(d): return {k:list(v) for k,v in d.items()}

def directed():
    out=[('zero',base())]
    def case(name,axis,**changes):
        d=base()
        for k,v in changes.items(): d[k][axis]=v
        out.append((name,d))
    for a in range(7):
        for f in FIELDS:
            for val in (I64MIN,I64MAX):
                case(f'{f}_{a}_{val}',a,**{f:val})
        for f in FIELDS[3:]:
            for val in (0,-1): case(f'{f}_{a}_{val}',a,**{f:val})
        for val in (-10*S32-1,10*S32+1):
            case(f'outside_{a}_{val}',a,seed_q=val)
        case(f'equal_bounds_{a}',a,lower=0,upper=0)
        case(f'inverted_bounds_{a}',a,lower=1,upper=-1)
        case(f'empty_q16_{a}',a,lower=1,upper=2,seed_q=1)
        for val in (32768,-32768,98304,-98304,65536,-65536,268435456,-268435456,805306368,-805306368):
            if a==6:
                case(f'tie_unit_{a}_{val}',a,seed_q=val,lower=-S32,upper=S32)
            else: case(f'tie_unit_{a}_{val}',a,seed_q=val)
        for edge in (I32MIN*(1<<16),I32MAX*(1<<16)):
            for shift in (-65537,-65536,-1,0,1,65536,65537):
                q=edge+shift
                case(f'q32_edge_{a}_{edge}_{shift}',a,seed_q=q,lower=q-65536,upper=q+65536)
    case('gripper_positive_inward',6,seed_q=429496729,upper=429496729)
    case('gripper_negative_inward',6,seed_q=-429496729,lower=-429496729,upper=0)
    case('gripper_lower_inward',6,seed_q=1,lower=1,upper=S32)
    case('joint_wire_inward',0,seed_q=2555904,lower=0,upper=2576980)
    case('joint_empty_can',0,seed_q=644245,lower=429497,upper=858993)
    case('gripper_empty_can',6,seed_q=65536,lower=65536,upper=65537)
    # Explicit accepted signed32 Q16 endpoints and lower-ceil overflow.
    case('q16_i32min_exact',0,seed_q=I32MIN*65536,lower=I32MIN*65536,upper=I32MIN*65536+S32)
    case('q16_i32max_exact',0,seed_q=I32MAX*65536,lower=I32MAX*65536-S32,upper=I32MAX*65536)
    case('q16_lower_ceil_above_i32max',0,seed_q=I32MAX*65536+1,
         lower=I32MAX*65536+1,upper=I32MAX*65536+65537)
    for code in (-4096,-12288):
        case(f'joint_scaled_negative_half_{code}',0,seed_q=code*65536)
    for code in (-512,-1536):
        case(f'gripper_scaled_negative_half_{code}',6,seed_q=code*65536,lower=-S32,upper=S32)
    unit=1000000
    for edge in (I32MIN,I32MAX):
        raw=floor(Fraction(edge*S32,unit))
        for shift in (-S32,-1,0,1,S32):
            q=raw+shift
            case(f'gripper_wire_i32_{edge}_{shift}',6,
                 seed_q=q,lower=q-S32,upper=q+S32)
    d=base(); d['seed_q']=[65536]*7; out.append(('axis_unit_separation',d))
    return out

def make_cases(count,seed):
    rng=random.Random(seed); cases=directed()
    for n in range(count):
        d=base()
        if n%4:
            for a in range(7):
                # Broad signed motion units with a nonempty code/wire domain.
                unit=S32 if a<6 else S32//1000
                lo=rng.randint(-100,50)*unit; hi=lo+rng.randint(2,50)*unit
                d['lower'][a]=lo; d['upper'][a]=hi
                d['seed_q'][a]=rng.randint(lo,hi)
                for f in FIELDS[3:]: d[f][a]=rng.randint(1,I64MAX)
        else:
            # Invalid and extreme cases remain in the same raw64 contract.
            for a in range(7):
                for f in FIELDS:
                    d[f][a]=rng.randint(I64MIN,I64MAX)
        cases.append((f'random_{n}',d))
    return cases

def pack(values,bits):
    return sum((v&((1<<bits)-1))<<(bits*a) for a,v in enumerate(values))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--count',type=int,default=1024)
    p.add_argument('--seed',type=int,default=20261102)
    p.add_argument('--clock-hz',type=int,default=99999001)
    args=p.parse_args()
    if args.count<0 or not 0<args.clock_hz<=(1<<31)-1: p.error('invalid count/positive signed32 clock')
    cases=make_cases(args.count,args.seed); rows=[str(len(cases))]; counts={'allow':0,'reject':0,'clamp':0}
    names=[]
    for name,fields in cases:
        allow,clamp,q,w=oracle(fields)
        counts['allow' if allow else 'reject']+=1; counts['clamp']+=clamp
        row=[f'{pack(fields[f],64):0112x}' for f in FIELDS]
        row += [str(allow),str(clamp),f'{pack(q,32):056x}',f'{pack(w,32):056x}']
        rows.append(' '.join(row)); names.append(name)
    args.output.write_text('\n'.join(rows)+'\n')
    metadata={'schema':'stationary_seed_vectors.v1','design_only':True,'hardware_verified':False,
              'clock_hz':args.clock_hz,'count':len(cases),'random_count':args.count,'seed':args.seed,
              'counts':counts,'oracle':'fractions.Fraction rational floor/ceil/nearest ties-even',
              'sha256':hashlib.sha256(args.output.read_bytes()).hexdigest(),'names':names,
              'generator_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    args.output.with_suffix(args.output.suffix+'.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps({k:metadata[k] for k in ('count','counts','clock_hz','sha256')}))
if __name__=='__main__': main()
