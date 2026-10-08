import argparse, concurrent.futures, datetime, ipaddress, json, pathlib, subprocess, time, urllib.request
p=argparse.ArgumentParser()
p.add_argument('--hotspot',required=True)
p.add_argument('--phone-subnet',required=True)
p.add_argument('--primary-ip',required=True)
p.add_argument('--log',required=True)
a=p.parse_args()
start=time.monotonic()
log=pathlib.Path(a.log)
def event(stage,**fields):
    row={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'elapsedS':round(time.monotonic()-start,2),'stage':stage,**fields}
    with log.open('a',encoding='utf-8') as f:f.write(json.dumps(row)+'\n')
    print(json.dumps(row),flush=True)
def get(ip,path,timeout=.5):
    try:
        with urllib.request.urlopen(f'http://{ip}{path}',timeout=timeout) as r:return json.load(r)
    except Exception:return None
def status(ip,stage):
    s=get(ip,'/api/status',2)
    if s:
        r=s['robot']
        event(stage,ip=ip,mode=r['mode'],pwm=r['pwm'],rpm=r['rpm'],net=s['net'],build=s['sys']['build'],uptimeS=s['sys']['uptimeS'])
        if r['mode']!='halt' or r['pwm']!=0:raise RuntimeError('Robot is not halted')
    return s
def hotspot(action):
    r=subprocess.run(['powershell.exe','-NoProfile','-File',a.hotspot,action],capture_output=True,text=True,timeout=25)
    if r.returncode:raise RuntimeError(f'Hotspot {action} failed')
    d=json.loads(r.stdout.strip())
    event('pc-hotspot-'+action.lower(),**d)
def find(subnet):
    ips=map(str,ipaddress.ip_network(subnet).hosts())
    def probe(ip):
        d=get(ip,'/api/whoami',.35)
        return ip if d and d.get('type')=='morluam' else None
    with concurrent.futures.ThreadPoolExecutor(max_workers=96) as pool:
        return [ip for ip in pool.map(probe,ips) if ip]
log.write_text('',encoding='utf-8')
if not status(a.primary_ip,'initial-primary'):raise RuntimeError('Primary robot unreachable')
backup_ip=None
try:
    hotspot('Off')
    deadline=time.monotonic()+65
    while time.monotonic()<deadline:
        hits=find(a.phone_subnet)
        for ip in hits:
            s=status(ip,'fallback-observed')
            if s and s['net']['ssid']=='eieiei':backup_ip=ip;break
        if backup_ip:break
        event('waiting-for-backup')
        time.sleep(3)
    if backup_ip:
        time.sleep(5)
        status(backup_ip,'fallback-stable')
    else:event('fallback-not-reachable',note='No API found on phone LAN; association/isolation must be distinguished')
finally:
    hotspot('On')
    deadline=time.monotonic()+80
    returned=False
    while time.monotonic()<deadline:
        s=get(a.primary_ip,'/api/status',1)
        if s and s['net']['ssid']=='manny':
            status(a.primary_ip,'primary-returned');returned=True;break
        if backup_ip:
            s=get(backup_ip,'/api/status',1)
            if s and s['net']['ssid']=='manny':
                status(s['net']['ip'],'primary-returned');returned=True;break
        hits=find('192.168.137.0/24')
        for ip in hits:
            s=status(ip,'primary-return-probe')
            if s and s['net']['ssid']=='manny':
                status(ip,'primary-returned');returned=True;break
        if returned:break
        time.sleep(2)
    if not returned:
        for ip in find('192.168.137.0/24'):
            s=status(ip,'primary-rediscovered')
            if s and s['net']['ssid']=='manny':returned=True;break
    event('result',fallback=bool(backup_ip),primaryReturned=returned)
    if not returned:raise RuntimeError('Primary reconnect not confirmed')
