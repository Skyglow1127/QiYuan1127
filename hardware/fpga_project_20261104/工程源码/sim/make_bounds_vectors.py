#!/usr/bin/env python3
"""Generate explicit synthetic pure-bounds golden vectors; no HDL/device access."""
from pathlib import Path
from fractions import Fraction
import argparse,hashlib,json,random
B=Path(__file__).resolve().parent
I64=1<<63;Q=1<<32;B16=1<<16;M=1<<31

def ceil(v):return -((-v.numerator)//v.denominator)
def predicate(lo,hi,unit):
    # Independent original full-raw64 rational predicate. No raw49 truncation
    # or candidate's floor/remainder implementation is used by this oracle.
    qlo=ceil(Fraction(lo,B16));qhi=Fraction(hi,B16).__floor__()
    wlo=ceil(Fraction(lo*unit,Q));whi=Fraction(hi*unit,Q).__floor__()
    return lo<hi and qlo<=qhi and -M<=qlo and qhi<M and wlo<=whi and -M<=wlo and whi<M

def encode(values):return sum((v&((1<<64)-1))<<(64*i) for i,v in enumerate(values))
def baseline():return [-10*Q]*6+[0],[10*Q]*6+[Q]
def points(unit):
    values={-I64,-I64+1,I64-2,I64-1,-(1<<48)-1,-(1<<48),-(1<<48)+1,(1<<48)-1,1<<48,(1<<48)+1,
        -(1<<47)-65537,-(1<<47)-65536,-(1<<47)-65535,-(1<<47)-1,-(1<<47),-(1<<47)+1,
        (1<<47)-65537,(1<<47)-65536,(1<<47)-65535,(1<<47)-1,1<<47,(1<<47)+1,
        -65537,-65536,-65535,-32769,-32768,-32767,-1,0,1,32767,32768,32769,65535,65536,65537}
    for edge in (Fraction((-M-1)*Q,unit),Fraction(M*Q,unit)):
        for delta in (-2,-1,0,1,2):
            value=edge.__floor__()+delta
            if -I64<=value<I64:values.add(value)
    assert len(values)==47
    return sorted(values)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True,help='new .mem file; companion .json must also not exist')
    ap.add_argument('--clock-hz',type=int,required=True,help='positive nominal test profile only; oracle is clock independent')
    ap.add_argument('--seed',type=int,default=20261102)
    ap.add_argument('--count',type=int,default=256,help='random pairs per axis in addition to47 directed boundary points')
    a=ap.parse_args();manifest=Path(str(a.output)+'.json')
    if not 0<a.clock_hz<(1<<31) or a.count<0:ap.error('invalid positive signed32 clock/count')
    if a.output.exists() or manifest.exists():ap.error('output or companion manifest exists')
    rng=random.Random(a.seed);rows=[];labels=[];counts={'allow':0,'reject':0};per_axis={}
    def add(axis,lo,hi,label):
        lows,highs=baseline();lows[axis]=lo;highs[axis]=hi
        assert all(-I64<=v<I64 for v in lows+highs)
        expected=int(all(predicate(l,h,1000000 if i==6 else 1000) for i,(l,h) in enumerate(zip(lows,highs))))
        raw=encode(lows)|(encode(highs)<<448)|(expected<<896)
        rows.append(f'{raw:0225x}\n');labels.append({'axis':axis,'name':label,'expected_bounds_valid':expected})
        counts['allow' if expected else 'reject']+=1
    for axis in range(7):
        unit=1000000 if axis==6 else 1000;p=points(unit);low,high=baseline();begin=len(rows)
        for v in p:
            add(axis,v,min(I64-1,v+1),f'boundary_adjacent_{v}')
            add(axis,v,high[axis],f'boundary_lower_{v}')
            add(axis,low[axis],v,f'boundary_upper_{v}')
            add(axis,v,max(-I64,v-1),f'boundary_inverted_{v}')
        for label,lo,hi in [('negative_fraction',-65537,1),('empty_q16',1,65535),('empty_can_with_q16_integer',65536,65537),('equal',0,0),('inverted',1,-1),
            ('q16_min_fraction',-(1<<47)-65535,0),('q16_below_min',-(1<<47)-65536,0),('q16_max_fraction',0,(1<<47)-1),('q16_above_max',0,1<<47)]:add(axis,lo,hi,label)
        for bit in range(49,63):
            add(axis,-(1<<bit),0,f'illegal_raw64_lower_bit{bit}')
            add(axis,0,1<<bit,f'illegal_raw64_upper_bit{bit}')
        for i in range(a.count):
            if i%3==0:lo=rng.randrange(-I64,I64);hi=rng.randrange(-I64,I64)
            elif i%3==1:lo=rng.randrange(-(1<<48),1<<48);hi=rng.randrange(-(1<<48),1<<48)
            else:lo=rng.randrange(-10*Q,10*Q);hi=min(I64-1,lo+rng.randrange(0,1<<24))
            add(axis,lo,hi,f'random_{i}')
        per_axis[str(axis)]={'unit':unit,'boundary_points':47,'vectors':len(rows)-begin}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as f:f.writelines(rows)
    result={'schema':'robot_rt.seed_bounds_vectors.v1','design_only':True,'synthetic_only':True,'hardware_verified':False,'HDL_executed':False,'clock_hz':a.clock_hz,
        'seed':a.seed,'random_count_per_axis':a.count,'count':len(rows),'counts':counts,'per_axis':per_axis,'layout':'897 bits: bit896 expected bounds_valid, bits895:448 upper7*signed64, bits447:0 lower7*signed64; axis0 low',
        'oracle':'independent Fraction ceil/floor over original full signed64 domain, U=1000 for0..5 and1000000 for6','sha256':sha(a.output),'generator_sha256':sha(__file__),
        'candidate_source_sha256':{name:sha(B.parent/'rtl'/name) for name in ['stationary_seed_gate.sv','robot_rt_top.sv']},'labels':labels}
    with manifest.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k!='labels'},indent=2))
if __name__=='__main__':main()
