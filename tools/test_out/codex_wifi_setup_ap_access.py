import concurrent.futures, datetime, ipaddress, json, pathlib, re, subprocess, tempfile, time, urllib.request
from xml.sax.saxutils import escape
HOTSPOT=r'C:\Users\manma\Documents\Codex\2026-10-08\you-are-continuing-work-on-my\work\mor_luam_hotspot.ps1'
AP='mor-luam-D570'
log=pathlib.Path('tools/test_out/codex_wifi_setup_ap_access.jsonl')
start=time.monotonic()
log.write_text('',encoding='utf-8')
def event(stage,**fields):
    row={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'elapsedS':round(time.monotonic()-start,2),'stage':stage,**fields}
    with log.open('a',encoding='utf-8') as f:f.write(json.dumps(row)+'\n')
    print(json.dumps(row),flush=True)
def get(ip,path='/api/status',timeout=1):
    try:
        with urllib.request.urlopen(f'http://{ip}{path}',timeout=timeout) as r:return json.load(r)
    except Exception:return None
def report(ip,stage):
    s=get(ip)
    if s:
        event(stage,ip=ip,mode=s['robot']['mode'],pwm=s['robot']['pwm'],net=s['net'],uptimeS=s['sys']['uptimeS'])
        if s['robot']['mode']!='halt' or s['robot']['pwm']!=0:raise RuntimeError('Robot is not halted')
    return s
def hotspot(action):
    r=subprocess.run(['powershell.exe','-NoProfile','-File',HOTSPOT,action],capture_output=True,text=True,timeout=25)
    if r.returncode:raise RuntimeError('PC hotspot operation failed')
    event('pc-hotspot-'+action.lower(),**json.loads(r.stdout.strip()))
def connect(profile):
    r=subprocess.run(['netsh','wlan','connect',f'name={profile}','interface=Wi-Fi'],capture_output=True,text=True,timeout=15)
    event('pc-wifi-connect',ssid=profile,accepted=r.returncode==0,detail=r.stdout.strip())
def find_primary():
    def probe(ip):
        d=get(ip,'/api/whoami',.35)
        return ip if d and d.get('type')=='morluam' else None
    with concurrent.futures.ThreadPoolExecutor(max_workers=96) as pool:
        return next((x for x in pool.map(probe,map(str,ipaddress.ip_network('192.168.137.0/24').hosts())) if x),None)
if not report('192.168.137.148','initial-primary'):raise RuntimeError('Robot unreachable')
text=pathlib.Path('firmware/config/conf_network.h').read_text(encoding='utf-8')
match=re.search(r'(?m)^[^\r\n/]*\bDEFAULT_AP_PASS\b[^\r\n"]*"([^"\r\n]+)"',text)
if not match:raise RuntimeError('AP credential not available')
secret=match.group(1)
xml=f'''<?xml version="1.0"?><WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1"><name>{AP}</name><SSIDConfig><SSID><name>{AP}</name></SSID></SSIDConfig><connectionType>ESS</connectionType><connectionMode>manual</connectionMode><MSM><security><authEncryption><authentication>WPA2PSK</authentication><encryption>AES</encryption><useOneX>false</useOneX></authEncryption><sharedKey><keyType>passPhrase</keyType><protected>false</protected><keyMaterial>{escape(secret)}</keyMaterial></sharedKey></security></MSM></WLANProfile>'''
profile_added=False
try:
    with tempfile.NamedTemporaryFile(mode='w',suffix='.xml',delete=False,encoding='utf-8') as f:
        f.write(xml);temp=f.name
    try:
        r=subprocess.run(['netsh','wlan','add','profile',f'filename={temp}','user=current'],capture_output=True,text=True,timeout=15)
        if r.returncode:raise RuntimeError('Could not prepare setup AP profile')
        profile_added=True
    finally:pathlib.Path(temp).unlink(missing_ok=True)
    secret=None;xml=None;text=None;match=None
    hotspot('Off')
    time.sleep(32)
    connect(AP)
    deadline=time.monotonic()+25
    ap_status=None
    while time.monotonic()<deadline:
        ap_status=report('192.168.4.1','setup-ap-access')
        if ap_status:break
        time.sleep(2)
    if not ap_status:raise RuntimeError('Setup AP HTTP not reachable')
    event('setup-ap-access-confirmed',connected=ap_status['net']['connected'],clients=ap_status['net']['ap']['clients'])
    time.sleep(5)
finally:
    connect('eieiei')
    time.sleep(8)
    hotspot('On')
    if profile_added:subprocess.run(['netsh','wlan','delete','profile',f'name={AP}','interface=Wi-Fi'],capture_output=True,text=True,timeout=15)
    deadline=time.monotonic()+50
    final_ip=None
    while time.monotonic()<deadline:
        final_ip=find_primary()
        if final_ip:break
        time.sleep(3)
    if final_ip:
        report(final_ip,'primary-restored')
        time.sleep(16)
        report(final_ip,'ap-client-left')
    else:event('recovery-not-confirmed')
