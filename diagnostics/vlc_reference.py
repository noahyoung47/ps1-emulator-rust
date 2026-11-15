import struct,re,pathlib
for path in sorted(pathlib.Path('diagnostics/ff7-vlc').glob('vlc_*.bin')):
 d=path.read_bytes();regs=[int(x,16) for x in re.findall('[0-9a-f]{8}',path.with_suffix('.txt').read_text())];src=regs[4]&0x1fffff;dst=regs[5]&0x1fffff
 def h(a):return struct.unpack_from('<H',d,a&0x1fffff)[0]
 def w(a):return struct.unpack_from('<I',d,a&0x1fffff)[0]
 print(path.name,hex(src),hex(dst),'header',d[src:src+16].hex())
 if not src:continue
 q=h(src+4)<<10;ver=h(src+6);v=(h(src+8)<<16)|h(src+10);p=src+12;bits=0;out=[];phase=1 if ver>=3 else 0;dc=[0,0,0]
 def consume(n):
  global v,bits,p
  v=(v<<n)&0xffffffff;bits+=n
  if bits&16:
   bits &=15;v|=h(p)<<bits;p+=2
 def escape():
  global v,p
  value=v>>16;v=(v<<16)&0xffffffff;v|=h(p)<<bits;p+=2;return value
 try:
  for step in range(200000):
   if phase:
    if v>>22==0x3ff:break
    base=0x52390-(0x800 if phase>=3 else 0x400)+(v>>24)*4
    n,b=h(base),h(base+2);consume(n);value=0
    if b:
     value=v>>(32-b)
     if v&0x80000000==0:value-=(1<<b)-1
     consume(b)
    k=0 if phase==1 else 1 if phase==2 else 2;dc[k]+=value;out.append(q|((dc[k]*4)&0x3ff));phase=phase%6+1
   else:
    value=v>>22
    if value==0x1ff:break
    out.append(q|value);consume(10)
   while True:
    base=0x52390+(v>>19)*8;t=w(base);packed=0
    if t:packed=w(base+4)
    else:
     consume(8);t=w(0x62390+(v>>23)*4)
    consume(t&255);codes=[t>>16]
    if packed&65535:codes.append(packed&65535)
    if packed>>16:codes.append(packed>>16)
    end=False
    for code in codes:
     if code==0x7c1f:out.append(escape())
     else:out.append(code)
     if code==0xfe00:end=True;break
    if end:break
  else:print('LOOP LIMIT')
  print('ref halfwords',len(out),'EOB',out.count(0xfe00),'srcused',p-src,'declaredhalfwords',h(src)*2)
  pathlib.Path(str(path)+'.rle').write_bytes(struct.pack('<%dH'%len(out),*out))
 except Exception as e:print('ERROR',e)
