'use strict';
// BeyondHome ALFA 0.07 — camera-only monocular spatial mapper.
// No GPS is used by the mapper. No depth sensor, IMU, ARCore, WebXR or external libraries.
// Monocular scale is relative: a single RGB camera cannot recover absolute metres by itself.
const $=id=>document.getElementById(id);
const SCREENS=['splash','home','cameraScreen','createScreen','spacesScreen','localScreen','infoScreen','simScreen','arScreen'];
const APP_VERSION='0.09';
const KEY='beyondHome.v28';
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const uid=()=>crypto?.randomUUID?.()||'bh-'+Date.now().toString(36)+'-'+Math.random().toString(36).slice(2);
const esc=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
let db=loadDB(); let stream=null;
function loadDB(){try{const now=localStorage.getItem(KEY);if(now)return JSON.parse(now);const old=localStorage.getItem('beyondHome.v27')||localStorage.getItem('beyondHome.v25')||localStorage.getItem('beyondHome.v21');return old?JSON.parse(old):{spaces:[],active:null}}catch{return{spaces:[],active:null}}}
function saveDB(){try{localStorage.setItem(KEY,JSON.stringify(db));return true}catch{toast('No se pudo guardar el mapa.');return false}}
function activeSpace(){return db.spaces.find(s=>s.id===db.active)||null}
function toast(m){const t=$('arToast'); if(t){t.textContent=m;t.classList.add('show');clearTimeout(toast.t);toast.t=setTimeout(()=>t.classList.remove('show'),2600)}else console.log(m)}
function show(id){stopScanner();if(id!=='arScreen')stopAR();if(id!=='cameraScreen'&&id!=='createScreen'&&id!=='arScreen')stopCamera();SCREENS.forEach(s=>$(s)?.classList.toggle('active',s===id));if(id==='spacesScreen')renderSpaces();if(id==='localScreen')renderLocal();if(id==='infoScreen')startInfo();else stopInfo();updateSupport()}
$('enter').onclick=()=>show('home');
async function openCamera(video){
  if(!navigator.mediaDevices?.getUserMedia)throw Error('Este navegador no permite cámara. Usa Chrome/Edge con HTTPS.');
  if(!video)throw Error('Visor de cámara no encontrado.');
  if(stream)stopCamera();
  stream=await navigator.mediaDevices.getUserMedia({video:{facingMode:{ideal:'environment'},width:{ideal:640,max:1280},height:{ideal:480,max:960},frameRate:{ideal:20,max:30}},audio:false});
  video.srcObject=stream; video.muted=true; video.setAttribute('playsinline','');
  await new Promise((resolve,reject)=>{
    if(video.readyState>=2)return resolve();
    const ok=()=>{cleanup();resolve()}, bad=()=>{cleanup();reject(new Error('La cámara no entregó frames.'))};
    const cleanup=()=>{video.removeEventListener('loadeddata',ok);video.removeEventListener('canplay',ok);video.removeEventListener('error',bad)};
    video.addEventListener('loadeddata',ok,{once:true});video.addEventListener('canplay',ok,{once:true});video.addEventListener('error',bad,{once:true});
    setTimeout(()=>{if(video.readyState>=2){cleanup();resolve()}},1800);
  });
  await video.play();
  if(video.readyState<2)throw Error('La cámara está activa pero todavía no entrega imágenes.');
}
function stopCamera(){if(stream){stream.getTracks().forEach(t=>t.stop());stream=null}['camera','scanCamera','arCamera'].forEach(id=>{const v=$(id);if(v)v.srcObject=null})}
$('previewCamera').onclick=async()=>{show('cameraScreen');try{await openCamera($('camera'));$('cameraInfo').textContent='CÁMARA ACTIVA · SOLO RGB'}catch(e){$('cameraInfo').textContent='ERROR DE CÁMARA · '+e.message;toast(e.message)}};
$('stopCamera').onclick=()=>{stopCamera();show('home')};document.querySelectorAll('.back').forEach(b=>b.onclick=()=>show('home'));
$('createSpace').onclick=()=>{show('createScreen');resetScannerUI()};$('previewLocal').onclick=()=>{show('localScreen');renderLocal()};$('spaces').onclick=()=>show('spacesScreen');$('engineInfo').onclick=()=>show('infoScreen');$('simulation').onclick=()=>activeSpace()?show('simScreen'):show('createScreen');

// ---------- Small linear algebra ----------
const I3=()=>[[1,0,0],[0,1,0],[0,0,1]];
function mm(A,B){const n=A.length,m=B.length,p=B[0].length,C=Array.from({length:n},()=>Array(p).fill(0));for(let i=0;i<n;i++)for(let k=0;k<m;k++){const a=A[i][k];for(let j=0;j<p;j++)C[i][j]+=a*B[k][j]}return C}
function mt(A){return A[0].map((_,j)=>A.map(r=>r[j]))}
function mv(A,v){return A.map(r=>r.reduce((s,x,i)=>s+x*v[i],0))}
function dot(a,b){return a.reduce((s,v,i)=>s+v*b[i],0)}
function cross(a,b){return[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]}
function norm(v){const n=Math.hypot(...v)||1;return v.map(x=>x/n)}
function solve(A,b){const n=b.length,M=A.map((r,i)=>[...r,b[i]]);for(let k=0;k<n;k++){let p=k;for(let i=k+1;i<n;i++)if(Math.abs(M[i][k])>Math.abs(M[p][k]))p=i;if(Math.abs(M[p][k])<1e-9)return null;[M[k],M[p]]=[M[p],M[k]];const q=M[k][k];for(let j=k;j<=n;j++)M[k][j]/=q;for(let i=0;i<n;i++){if(i===k)continue;const f=M[i][k];for(let j=k;j<=n;j++)M[i][j]-=f*M[k][j]}}return M.map(r=>r[n])}
function inv3(A){const d=A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])-A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])+A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);if(Math.abs(d)<1e-9)return null;const q=1/d;return [[(A[1][1]*A[2][2]-A[1][2]*A[2][1])*q,(A[0][2]*A[2][1]-A[0][1]*A[2][2])*q,(A[0][1]*A[1][2]-A[0][2]*A[1][1])*q],[(A[1][2]*A[2][0]-A[1][0]*A[2][2])*q,(A[0][0]*A[2][2]-A[0][2]*A[2][0])*q,(A[0][2]*A[1][0]-A[0][0]*A[1][2])*q],[(A[1][0]*A[2][1]-A[1][1]*A[2][0])*q,(A[0][1]*A[2][0]-A[0][0]*A[2][1])*q,(A[0][0]*A[1][1]-A[0][1]*A[1][0])*q]]}
function mul3(A,B){return mm(A,B)}
function rotX(a){const c=Math.cos(a),s=Math.sin(a);return[[1,0,0],[0,c,-s],[0,s,c]]}
function rotY(a){const c=Math.cos(a),s=Math.sin(a);return[[c,0,s],[0,1,0],[-s,0,c]]}
function rotZ(a){const c=Math.cos(a),s=Math.sin(a);return[[c,-s,0],[s,c,0],[0,0,1]]}
function compose(rx,ry,rz){return mul3(mul3(rotZ(rz),rotY(ry)),rotX(rx))}
function transform(pose,p){const q=mv(pose.R,p);return[q[0]+pose.t[0],q[1]+pose.t[1],q[2]+pose.t[2]]}
function project(p,pose){const q=transform(pose,p);if(q[2]<=.02)return null;return {x:FX*q[0]/q[2]+CX,y:FY*q[1]/q[2]+CY,z:q[2]};}
function ray(x,y){return norm([(x-CX)/FX,(y-CY)/FY,1])}
function camCenter(pose){const R=mt(pose.R),q=mv(R,pose.t);return[-q[0],-q[1],-q[2]]}

// ---------- Camera image ----------
const SW=192,SH=144,FX=170,FY=170,CX=SW/2,CY=SH/2;
const work=document.createElement('canvas'),wctx=work.getContext('2d',{willReadFrequently:true});
function frame(video){if(!video||video.readyState<2)return null;work.width=SW;work.height=SH;wctx.drawImage(video,0,0,SW,SH);const d=wctx.getImageData(0,0,SW,SH).data,g=new Uint8Array(SW*SH);for(let i=0,j=0;i<d.length;i+=4,j++)g[j]=(77*d[i]+150*d[i+1]+29*d[i+2])>>8;return g}
function descriptor(g,x,y){const a=[];let m=0;for(let j=-6;j<=6;j+=2)for(let i=-6;i<=6;i+=2){const xx=Math.max(1,Math.min(SW-2,x+i)),yy=Math.max(1,Math.min(SH-2,y+j));const v=g[yy*SW+xx];a.push(v);m+=v}m/=a.length;return a.map(v=>clamp(v-m+128,0,255))}
function scoreCorner(g,x,y){let sxx=0,syy=0,sxy=0;for(let j=-2;j<=2;j++)for(let i=-2;i<=2;i++){const xx=x+i,yy=y+j;const gx=g[yy*SW+xx+1]-g[yy*SW+xx-1];const gy=g[(yy+1)*SW+xx]-g[(yy-1)*SW+xx];sxx+=gx*gx;syy+=gy*gy;sxy+=gx*gy}const det=sxx*syy-sxy*sxy,tr=sxx+syy;return det-.04*tr*tr}
function detect(g){
  const cand=[];
  // Fast, resolution-independent corner/texture score. The threshold is relative to
  // the current frame so cheap phone cameras with different exposure still produce candidates.
  for(let y=8;y<SH-8;y+=3){
    for(let x=8;x<SW-8;x+=3){
      const gx=Math.abs(g[y*SW+x+3]-g[y*SW+x-3]);
      const gy=Math.abs(g[(y+3)*SW+x]-g[(y-3)*SW+x]);
      const d1=Math.abs(g[(y+3)*SW+x+3]-g[(y-3)*SW+x-3]);
      const d2=Math.abs(g[(y+3)*SW+x-3]-g[(y-3)*SW+x+3]);
      const score=gx+gy+0.5*(d1+d2);
      cand.push({x,y,s:score});
    }
  }
  cand.sort((a,b)=>b.s-a.s);
  const max=cand[0]?.s||0, threshold=Math.max(9,max*.20);
  const out=[];
  for(const p of cand){
    if(p.s<threshold)break;
    let ok=true;
    for(const q of out)if((p.x-q.x)**2+(p.y-q.y)**2<64){ok=false;break}
    if(ok){out.push({...p,d:descriptor(g,p.x,p.y)});if(out.length>=96)break}
  }
  // Always expose a small set of high-gradient samples. They are visual evidence, not 3D points.
  return out;
}
function patchSSD(g,a,b,r=3){let e=0,n=0;for(let j=-r;j<=r;j++)for(let i=-r;i<=r;i++){const av=g[(a.y+j)*SW+a.x+i],bv=g[(b.y+j)*SW+b.x+i];e+=Math.abs(av-bv);n++}return e/n}
function trackFeatures(prev,g,features){const out=[];for(const p of features){let best=null,be=1e9;for(let j=-12;j<=12;j+=2)for(let i=-12;i<=12;i+=2){const b={x:p.x+i,y:p.y+j};if(b.x<8||b.y<8||b.x>=SW-8||b.y>=SH-8)continue;const e=patchSSD(g,p,b);if(e<be){be=e;best=b}}if(best&&be<32)out.push({a:p,b:best,e:be})}return out}
function median(a){if(!a.length)return 0;const b=[...a].sort((x,y)=>x-y);return b[b.length>>1]}
function robustFlow(ms){if(ms.length<8)return{dx:0,dy:0,mag:0,coh:0};const dx=ms.map(m=>m.b.x-m.a.x),dy=ms.map(m=>m.b.y-m.a.y),mx=median(dx),my=median(dy),rs=ms.map((m,i)=>Math.hypot(dx[i]-mx,dy[i]-my)),mr=median(rs),gate=Math.max(1.5,mr*2.5),inl=rs.filter(v=>v<gate).length;return{dx:mx,dy:my,mag:Math.hypot(mx,my),coh:inl/ms.length}}
// Estimate a similarity motion from image tracks. This is only used to obtain a rotation prior;
// translation direction is kept arbitrary because monocular scale is unobservable.
function similarity(ms){if(ms.length<8)return null;const ax=median(ms.map(m=>m.a.x)),ay=median(ms.map(m=>m.a.y)),bx=median(ms.map(m=>m.b.x)),by=median(ms.map(m=>m.b.y));let num=0,den=0,cr=0;for(const m of ms){const x=m.a.x-ax,y=m.a.y-ay,u=m.b.x-bx,v=m.b.y-by;num+=x*u+y*v;cr+=x*v-y*u;den+=x*x+y*y}const ang=Math.atan2(cr,num),sc=Math.hypot(num,cr)/Math.max(1,den);return{ang,scale:clamp(sc,.85,1.18),dx:bx-ax,dy:by-ay}}

// ---------- Mapping ----------
function triangulate(p1,p2,R,t){
  // Two-view triangulation in the FIRST camera coordinate frame.
  // Camera 1: X = a*d1. Camera 2: X = C2 + b*d2, where
  // C2 = -R^T t and d2 = R^T ray2.
  const d1=ray(p1.x,p1.y), d2=mv(mt(R),ray(p2.x,p2.y)), C2=mv(mt(R),t).map(v=>-v);
  const a11=dot(d1,d1), a12=-dot(d1,d2), a22=dot(d2,d2);
  const b1=dot(d1,C2), b2=-dot(d2,C2), det=a11*a22-a12*a12;
  if(Math.abs(det)<1e-10)return null;
  const a=(b1*a22-a12*b2)/det, b=(a11*b2-a12*b1)/det;
  const A=d1.map(v=>v*a), B=C2.map((v,i)=>v+d2[i]*b), P=A.map((v,i)=>(v+B[i])*.5);
  const err=Math.hypot(A[0]-B[0],A[1]-B[1],A[2]-B[2]);
  if(a<=0||b<=0||P[2]<.04||err>.38*Math.max(.08,P[2]))return null;
  return{p:P,err};
}
function keyFor(p){return `${Math.round(p.x/.045)},${Math.round(p.y/.045)},${Math.round(p.z/.045)}`}
function fuse(map,p,d,quality=0.25,err=0.02){
  const k=keyFor(p),old=map.get(k);
  if(!old){map.set(k,{x:p.x,y:p.y,z:p.z,n:1,confidence:clamp(quality,0.05,.95),d,err});return}
  const n=old.n+1;
  const w=1/n;
  old.x+=(p.x-old.x)*w;old.y+=(p.y-old.y)*w;old.z+=(p.z-old.z)*w;
  old.n=Math.min(255,n);
  const q=clamp(quality*(1-clamp(err/.08,0,.9)),0.02,.95);
  old.confidence=clamp(old.confidence+(q-old.confidence)*.22,0,1);
  old.err=old.err+(err-old.err)*.18;
  if(!old.d)old.d=d;
}
function pointReliability(p){return clamp(p.confidence*(1-Math.min(1,(p.err||0)/.08)),0,1)}
function estimateRelative(ms){const f=robustFlow(ms),s=similarity(ms);if(!s||f.coh<.55||f.mag<1.8)return null;const yaw=-s.dx/FX*.55,pitch=s.dy/FY*.25;const radial=median(ms.map(m=>{const x=m.a.x-CX,y=m.a.y-CY,l=Math.hypot(x,y)||1;return (m.b.x-m.a.x)*(x/l)+(m.b.y-m.a.y)*(y/l)}));let v=[-s.dx/FX,s.dy/FY,-radial/Math.max(20,SH)];const n=Math.hypot(...v)||1;v=v.map(x=>x/n);return{R:compose(pitch,yaw,0),t:v.map(x=>x*.12),flow:f,sim:s}}
function poseError(pose,corr){let e=0,n=0;for(const c of corr){const q=project(c.p,pose);if(!q)continue;const d=Math.hypot(q.x-c.x,q.y-c.y);if(d<35){e+=d;n++}}return n?e/n:999}
function refinePose(pose,corr){if(corr.length<6)return{pose,error:999};let cur={R:pose.R.map(r=>r.slice()),t:[...pose.t]};for(let it=0;it<3;it++){const H=Array.from({length:6},()=>Array(6).fill(0)),g=Array(6).fill(0);let used=0;for(const c of corr){const q=project(c.p,cur);if(!q)continue;const rx=c.x-q.x,ry=c.y-q.y;if(Math.hypot(rx,ry)>40)continue;const J=[];for(let k=0;k<6;k++){const pp={R:cur.R.map(r=>r.slice()),t:[...cur.t]},eps=k<3?.004:.003;if(k<3)pp.t[k]+=eps;else pp.R=mul3(compose(k===3?eps:0,k===4?eps:0,k===5?eps:0),pp.R);const qq=project(c.p,pp);J.push(qq?[(qq.x-q.x)/eps,(qq.y-q.y)/eps]:[0,0])}for(let a=0;a<6;a++){g[a]+=J[a][0]*rx+J[a][1]*ry;for(let b=0;b<6;b++)H[a][b]+=J[a][0]*J[b][0]+J[a][1]*J[b][1]}used++}const d=solve(H.map((r,i)=>r.map((v,j)=>v+(i===j?.002:0))),g);if(!d||used<6)break;cur.t=cur.t.map((v,i)=>v+d[i]);cur.R=mul3(compose(d[3],d[4],d[5]),cur.R)}return{pose:cur,error:poseError(cur,corr)}}
function matchDescriptors(features,map){const arr=[...map.values()].filter(p=>p.d&&p.n>=2),out=[];for(const f of features){let best=null,be=999;for(const p of arr){let e=0;for(let i=0;i<f.d.length;i++)e+=Math.abs(f.d[i]-p.d[i]);e/=f.d.length;if(e<be){be=e;best=p}}if(best&&be<24)out.push({f,p:best,e:be})}return out}

const scan={running:false,started:0,last:0,prev:null,features:[],pose:{R:I3(),t:[0,0,0]},map:new Map(),keyframes:0,lastKey:0,baseline:0,good:0,coverage:new Map(),zones:new Map(),motion:0,coh:0,status:'READY',raf:0,phase:'HOLD',motionKind:'QUIETO',lastMotion:{dx:0,dy:0,mag:0},stable:[]};
function resetScannerUI(){Object.assign(scan,{running:false,started:0,last:0,prev:null,features:[],pose:{R:I3(),t:[0,0,0]},map:new Map(),keyframes:0,lastKey:0,baseline:0,good:0,coverage:new Map(),zones:new Map(),motion:0,coh:0,status:'READY',raf:0,phase:'HOLD',motionKind:'QUIETO',lastMotion:{dx:0,dy:0,mag:0},stable:[]});$('scanStart').disabled=false;$('scanFinish').disabled=true;$('scanStart').textContent='INICIAR ESCANEO';$('scanPercent').textContent='0%';$('scanPts').textContent='0';$('scanRefs').textContent='0';$('scanTime').textContent='0.0';$('scanQuality').textContent='Esperando';$('qualityBar').style.width='0%'}
function scanTime(){return scan.started?(performance.now()-scan.started)/1000:0}
function secured(){let n=0;for(const p of scan.map.values())if(pointReliability(p)>=.55&&p.n>=2)n++;return n}
function zoneKey(x,y){return Math.min(2,Math.max(0,Math.floor(x/(SW/3))))+','+Math.min(1,Math.max(0,Math.floor(y/(SH/2))));}
function zoneCount(){let n=0;for(const z of scan.zones.values())if(z.points>=2)n++;return n}
function coverageScore(){let sum=0,n=0;for(const c of scan.coverage.values()){sum+=c.score;n++}return n?sum/n:0}
function readiness(){const a=clamp(scan.keyframes/4,0,1),b=clamp(scan.map.size/24,0,1),c=clamp(secured()/6,0,1),d=clamp(scan.baseline/.012,0,1),e=clamp(zoneCount()/6,0,1),f=clamp(coverageScore(),0,1);return Math.round(100*(a*.14+b*.24+c*.22+d*.10+e*.22+f*.08))}
function canSave(){return scanTime()>=4&&scan.keyframes>=2&&scan.map.size>=12&&secured()>=3&&zoneCount()>=3}
function motionLabel(flow){const ax=Math.abs(flow.dx),ay=Math.abs(flow.dy),m=flow.mag;if(m<1.2)return 'QUIETO';if(ax>ay*1.35)return flow.dx>0?'DERECHA':'IZQUIERDA';if(ay>ax*1.35)return flow.dy>0?'ABAJO / TILT':'ARRIBA / TILT';return 'DESPLAZAMIENTO / ROTACIÓN'}
function updateCoverage(features){for(const f of features){const gx=Math.floor(f.x/16),gy=Math.floor(f.y/16),k=gx+','+gy,c=scan.coverage.get(k)||{seen:0,stable:0,parallax:0,score:0};c.seen++;c.score=clamp(c.score*.85+.12,0,1);scan.coverage.set(k,c)}}
function markCoverageFromPoint(p,q){if(!q)return;const gx=Math.floor(q.x/16),gy=Math.floor(q.y/16),k=gx+','+gy,c=scan.coverage.get(k)||{seen:0,stable:0,parallax:0,score:0};c.stable++;c.parallax++;c.score=clamp(c.score+.12,0,1);scan.coverage.set(k,c)}
function markZone(q){if(!q)return;const k=zoneKey(q.x,q.y),z=scan.zones.get(k)||{seen:0,points:0,last:0};z.seen++;z.points=Math.min(99,z.points+1);z.last=performance.now();scan.zones.set(k,z)}
function colorForReliability(r){return r>=.82?'#54f2a2':r>=.5?'#ffd166':'#ff4d6d'}
function updateStableTracks(matches){
  // Visual lock is deliberately independent from successful 3D triangulation.
  // A normal RGB camera can track a feature reliably before monocular depth is
  // solved. Keeping these two states separate prevents every point remaining red.
  const next=[];
  for(const m of matches){
    const x=m.b.x,y=m.b.y;
    let best=null,bd=18;
    for(const a of scan.stable){const d=Math.hypot(a.x-x,a.y-y);if(d<bd){bd=d;best=a}}
    if(best){next.push({x:x*.65+best.x*.35,y:y*.65+best.y*.35,age:Math.min(30,best.age+1),confidence:Math.min(1,best.confidence+.08)});best._used=true}
    else next.push({x,y,age:1,confidence:.25});
  }
  // Keep a few recently lost locks alive, so a brief tracker miss does not flash red.
  for(const a of scan.stable)if(!a._used&&a.age>=5)next.push({x:a.x,y:a.y,age:Math.max(0,a.age-2),confidence:a.confidence*.88});
  scan.stable=next.filter(a=>a.age>0).slice(0,120);
  for(const a of scan.stable)delete a._used;
}
function stableAt(x,y){let best=null,bd=12;for(const a of scan.stable){const d=Math.hypot(a.x-x,a.y-y);if(d<bd){bd=d;best=a}}return best}
function drawScan(features){
  const c=$('scanPreview'),d=devicePixelRatio||1,w=c.clientWidth||320,h=c.clientHeight||360;
  if(c.width!==w*d||c.height!==h*d){c.width=w*d;c.height=h*d}
  const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);x.clearRect(0,0,w,h);

  // Two-stage visual state:
  // RED = newly detected image feature.
  // GREEN = same image feature tracked consistently for several frames.
  // A green point does NOT claim solved depth; 3D confidence is drawn separately.
  for(const f of features){
    const a=stableAt(f.x,f.y),locked=!!a&&a.age>=4;
    const sx=f.x/SW*w,sy=f.y/SH*h;
    x.fillStyle=locked?'#54f2a2':'#ff4d6d';x.shadowColor=x.fillStyle;x.shadowBlur=locked?9:7;
    x.beginPath();x.arc(sx,sy,locked?3.2:2.5,0,Math.PI*2);x.fill();x.shadowBlur=0;
    if(locked){x.strokeStyle='#54f2a288';x.lineWidth=1;x.beginPath();x.arc(sx,sy,6,0,Math.PI*2);x.stroke()}
  }

  const projected=[];
  for(const p of scan.map.values()){
    const q=project([p.x,p.y,p.z],scan.pose);if(!q)continue;
    const sx=q.x/SW*w,sy=q.y/SH*h;if(sx<0||sy<0||sx>w||sy>h)continue;
    projected.push({p,q,sx,sy,r:pointReliability(p)});
  }
  // Stable 3D geometry is drawn over the red live features.
  for(let i=0;i<projected.length;i++){
    const a=projected[i];let links=0;
    for(let j=i+1;j<projected.length&&links<3;j++){
      const b=projected[j],dist=Math.hypot(a.p.x-b.p.x,a.p.y-b.p.y,a.p.z-b.p.z);
      if(dist>.24||dist<.015)continue;
      if(Math.hypot(a.sx-b.sx,a.sy-b.sy)>90)continue;
      x.strokeStyle=colorForReliability(Math.min(a.r,b.r))+'66';x.lineWidth=1;x.beginPath();x.moveTo(a.sx,a.sy);x.lineTo(b.sx,b.sy);x.stroke();links++;
    }
  }
  for(const a of projected){x.fillStyle=a.r>=.55?'#71e8ff':a.r>=.3?'#ffd166':'#ff4d6d';x.shadowColor=x.fillStyle;x.shadowBlur=7;x.beginPath();x.arc(a.sx,a.sy,a.r>=.82?3:a.r>=.5?2.5:2.3,0,Math.PI*2);x.fill();x.shadowBlur=0}

  // Red coverage probes mark image regions that have been observed but still lack reliable 3D.
  for(const [k,cov] of scan.coverage){if(cov.score>=.78)continue;const [gx,gy]=k.split(',').map(Number),cx=(gx*16+8)/SW*w,cy=(gy*16+8)/SH*h;x.fillStyle=`rgba(255,77,109,${.18+.25*(1-cov.score)})`;x.beginPath();x.arc(cx,cy,2.4,0,Math.PI*2);x.fill()}

  const p=readiness();
  $('scanPercent').textContent=p+'%';$('scanPts').textContent=scan.map.size;$('scanRefs').textContent=features.length;$('scanStatus').textContent=`ROJOS ${features.filter(f=>!stableAt(f.x,f.y)||stableAt(f.x,f.y).age<4).length} · FIJADOS ${scan.stable.filter(a=>a.age>=4).length} · CIAN 3D ${secured()}`;$('scanTime').textContent=scanTime().toFixed(1);$('scanCoverage').textContent=`${zoneCount()}/6 zonas`;
  $('scanQuality').textContent=canSave()?'MAPA ESTABLE':scan.map.size>0?'CONSTRUYENDO 3D':features.length>=12?'REFERENCIAS DETECTADAS':'BUSCANDO TEXTURA';
  $('qualityBar').style.width=p+'%';$('scanFinish').disabled=!canSave();$('scanReady').textContent=canSave()?'✓ suficiente evidencia 3D':'Añade más vistas';
  const motionHUD=$('scanMotion'); if(motionHUD)motionHUD.textContent=`${scan.motionKind} · ${scan.motion.toFixed(1)} px · ${Math.round(scan.coh*100)}% coherencia · ${features.length} refs`;
  const t=scanTime();
  if(t<2){$('scanGuide').textContent='QUÉDATE QUIETO';$('scanHint').textContent='Estamos fijando las primeras referencias visuales.'}
  else if(features.length<12){$('scanGuide').textContent='BUSCA TEXTURA';$('scanHint').textContent='Apunta a esquinas, muebles, libros, marcos o superficies con detalle.'}
  else {$('scanGuide').textContent=scan.motionKind==='QUIETO'?'MUÉVETE EN CUALQUIER DIRECCIÓN':'SIGUE MOVIÉNDOTE';$('scanHint').textContent='Rojo = nuevo · verde = fijado · cian = 3D. Recorre 3 zonas con un pequeño desplazamiento lateral; no hace falta caminar.'}
}
function scannerLoop(){
  if(!scan.running)return;
  const now=performance.now();
  if(now-scan.last<85){scan.raf=requestAnimationFrame(scannerLoop);return}
  scan.last=now;
  try{
    const video=$('scanCamera');
    if(!video||video.readyState<2||video.videoWidth<2){
      $('scanGuide').textContent='INICIALIZANDO CÁMARA';
      $('scanHint').textContent='Esperando el primer frame…';
      $('scanQuality').textContent='CÁMARA';
      scan.raf=requestAnimationFrame(scannerLoop);return;
    }
    const g=frame(video);
    if(!g)throw new Error('frame vacío');
    const f=detect(g);
    const matches=scan.prev?trackFeatures(scan.prev,g,scan.features):[];
    const flow=robustFlow(matches);
    scan.motion=flow.mag;scan.coh=flow.coh;scan.motionKind=motionLabel(flow);
    updateCoverage(f);
    const t=scanTime();scan.phase=t<1.5?'HOLD':t<6?'EXPLORE':'REFINE';
    if(flow.mag<1.2&&flow.coh>.45)scan.status='STILL';else if(flow.mag>1.4&&flow.coh>.35)scan.status='MOVING';else scan.status=f.length?'TRACKING':'LOW TEXTURE';
    // Seed the first frame as image references. They stay red until 3D evidence exists.
    if(!scan.features.length&&f.length){scan.features=f;}
    if(matches.length>=6&&flow.coh>.25&&flow.mag>.65&&(!scan.lastKey||now-scan.lastKey>650)){
      const rel=estimateRelative(matches);
      if(rel){
        const oldPose={R:scan.pose.R.map(r=>r.slice()),t:[...scan.pose.t]};
        const newR=mul3(rel.R,oldPose.R);
        const newT=mv(rel.R,oldPose.t).map((v,i)=>v+rel.t[i]);
        scan.pose={R:newR,t:newT};
        let added=0;
        for(const m of matches){
          const q=triangulate(m.a,m.b,rel.R,rel.t);if(!q)continue;
          const pw=mv(mt(oldPose.R),q.p.map((v,i)=>v-oldPose.t[i]));
          const quality=clamp(.82-Math.min(.65,q.err/Math.max(.025,q.p[2]*.35)),.12,.9);
          fuse(scan.map,pw,m.a.d,quality,q.err);
          const qq=project(pw,scan.pose);markCoverageFromPoint(pw,qq);markZone(qq);added++;
        }
        if(added>0){scan.keyframes++;scan.lastKey=now;scan.baseline=Math.max(scan.baseline,Math.hypot(...rel.t));scan.good+=added}
      }
    }
    if(scan.map.size>=4){
      const locks=matchDescriptors(f,scan.map),corr=[];
      for(const a of locks.slice(0,30))corr.push({p:[a.p.x,a.p.y,a.p.z],x:a.f.x,y:a.f.y});
      if(corr.length>=6){const rr=refinePose(scan.pose,corr);if(rr.error<30)scan.pose=rr.pose}
    }
    updateStableTracks(matches);scan.prev=g;scan.features=f;drawScan(f);
    $('scanGuide').textContent=t<1.5?'QUIETO · FIJANDO REFERENCIAS':(canSave()?'MAPA LISTO':'EXPLORA EL ENTORNO');
    $('scanHint').textContent=t<1.5?'Mantén el móvil quieto un instante. Después muévelo lentamente en cualquier dirección.':(f.length<10?'Busca textura, esquinas y objetos con detalle.':'Rojo = referencia nueva; verde = referencia visual fijada; cian = 3D consolidado.');
  }catch(e){
    console.warn('BeyondHome scanner:',e);
    $('scanGuide').textContent='ESCÁNER ACTIVO';
    $('scanHint').textContent='Procesando cámara…';
  }
  scan.raf=requestAnimationFrame(scannerLoop);
}
async function startScanner(){
  try{
    resetScannerUI();
    $('scanGuide').textContent='INICIALIZANDO CÁMARA';
    $('scanHint').textContent='Solicitando acceso a la cámara…';
    await openCamera($('scanCamera'));
    scan.running=true;scan.started=performance.now();scan.last=0;scan.prev=null;scan.features=[];
    $('scanStart').disabled=true;$('scanStart').textContent='ESCANEANDO…';
    const waitHUD=$('scanWait'); if(waitHUD)waitHUD.textContent='Sólo cámara · quieto 1 s y después haz micro-desplazamientos por zonas';
    toast('Cámara activa. Quieto 1 s; después haz pequeños desplazamientos laterales y cambia de zona.');
    scan.raf=requestAnimationFrame(scannerLoop);
  }catch(e){
    scan.running=false;
    $('scanGuide').textContent='CÁMARA NO DISPONIBLE';
    $('scanHint').textContent=e.message||'Comprueba los permisos de cámara y HTTPS.';
    toast(e.message||'No se pudo iniciar la cámara.');
  }
}
function stopScanner(){scan.running=false;cancelAnimationFrame(scan.raf)}
function finishScan(){if(!canSave()){toast('Aún falta evidencia 3D. Sigue moviéndote y reforzando zonas rojas.');return}stopScanner();const pts=[...scan.map.values()].filter(p=>p.n>=2&&pointReliability(p)>.45);const s={id:uid(),name:(($('spaceName').value||'Mi espacio').trim()),created:new Date().toLocaleString(),method:'camera-only-monocular-sfm-v28-bogon-zones',scale:'relative',version:28,appVersion:APP_VERSION,coverage:readiness(),points:pts.length,secured:secured(),samples:pts,objects:[],camera:{fx:FX,fy:FY,width:SW,height:SH}};db.spaces.push(s);db.active=s.id;saveDB();renderSpaces();renderLocal();updateSupport();toast(`Mapa guardado · ${pts.length} puntos 3D relativos`);setTimeout(()=>show('simScreen'),200)}
$('scanStart').onclick=startScanner;$('scanFinish').onclick=finishScan;

// ---------- Spaces / map ----------
function renderSpaces(){const el=$('spaceList');if(!el)return;el.innerHTML=db.spaces.length?db.spaces.map(s=>`<div class="spaceItem"><div><b>${esc(s.name)}</b><span>${s.points||0} puntos · ${s.secured||0} consolidados · ${esc(s.created)}</span></div><div class="spaceBtns"><button data-open="${s.id}">USAR</button><button data-del="${s.id}">BORRAR</button></div></div>`).join(''):`<div class="empty"><h2>No hay espacios</h2><p>Crea el primero usando únicamente la cámara.</p></div>`;el.querySelectorAll('[data-open]').forEach(b=>b.onclick=()=>{db.active=b.dataset.open;saveDB();show('simScreen')});el.querySelectorAll('[data-del]').forEach(b=>b.onclick=()=>{db.spaces=db.spaces.filter(s=>s.id!==b.dataset.del);if(db.active===b.dataset.del)db.active=db.spaces[0]?.id||null;saveDB();renderSpaces();renderLocal()})}
let mapView={yaw:.5,pitch:-.35,zoom:1};
function renderLocal(){const c=$('localMap');if(!c)return;const s=activeSpace();$('localTitle').textContent=s?s.name:'Sin espacio';const d=devicePixelRatio||1,w=c.clientWidth||innerWidth,h=c.clientHeight||innerHeight;c.width=w*d;c.height=h*d;const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);x.fillStyle='#05070b';x.fillRect(0,0,w,h);if(!s){$('localMeta').textContent='Crea y guarda un espacio para ver su nube 3D.';return}const cy=Math.cos(mapView.yaw),sy=Math.sin(mapView.yaw),cp=Math.cos(mapView.pitch),sp=Math.sin(mapView.pitch),pts=s.samples||[],q=[];for(const p of pts){let X=p.x*cy-p.z*sy,Z=p.x*sy+p.z*cy,Y=p.y*cp-Z*sp;Z=p.y*sp+Z*cp;q.push({x:w/2+X*320,y:h/2-Y*320,z:Z,r:pointReliability(p)})}q.sort((a,b)=>b.z-a.z);for(let i=0;i<q.length;i++){let links=0;for(let j=i+1;j<q.length&&links<4;j++){const dx=q[i].x-q[j].x,dy=q[i].y-q[j].y;if(Math.hypot(dx,dy)<85){x.strokeStyle=colorForReliability(Math.min(q[i].r,q[j].r))+'55';x.lineWidth=1;x.beginPath();x.moveTo(q[i].x,q[i].y);x.lineTo(q[j].x,q[j].y);x.stroke();links++}}}for(const p of q){const r=clamp(3-p.z*.7,.9,3.4);x.fillStyle=colorForReliability(p.r);x.fillRect(p.x-r/2,p.y-r/2,r,r)}$('localMeta').textContent=`${pts.length} puntos 3D · ${s.secured||0} consolidados · rojo=necesita evidencia · verde=alta confianza · escala relativa · sólo cámara`}
$('mapReset').onclick=()=>{mapView={yaw:.5,pitch:-.35,zoom:1};renderLocal()};let drag=null;$('localMap').addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY,yaw:mapView.yaw,pitch:mapView.pitch}});$('localMap').addEventListener('pointermove',e=>{if(!drag)return;mapView.yaw=drag.yaw+(e.clientX-drag.x)*.01;mapView.pitch=clamp(drag.pitch+(e.clientY-drag.y)*.01,-1.4,1.4);renderLocal()});['pointerup','pointercancel','pointerleave'].forEach(ev=>$('localMap').addEventListener(ev,()=>drag=null));

// ---------- Explanation ----------
let infoRAF=0;function stopInfo(){cancelAnimationFrame(infoRAF)}function startInfo(){stopInfo();const c=$('bogonExplain'),d=devicePixelRatio||1;const f=t=>{const w=c.clientWidth||340,h=c.clientHeight||260;c.width=w*d;c.height=h*d;const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);x.clearRect(0,0,w,h);const ph=(t%9000)/9000;const n=Math.floor(20+ph*600),cx=w/2,cy=h*.48,s=Math.min(w,h)*.24;for(let i=0;i<n;i++){const a=i/n*Math.PI*2*3.8,r=s*(.25+.75*i/n),xx=cx+Math.cos(a)*r,yy=cy+Math.sin(a)*r*.65;x.fillStyle=i/n>.7?'#ff5ea8':i/n>.4?'#a35cff':'#71e8ff';x.fillRect(xx,yy,1.4,1.4)}x.fillStyle='#fff';x.font='800 11px system-ui';x.fillText(ph<.33?'UNA FUNCIÓN':ph<.66?'MUESTRAS': 'DETALLE ADAPTATIVO',14,h-16);infoRAF=requestAnimationFrame(f)};infoRAF=requestAnimationFrame(f)}
$('bogonLab').onclick=()=>show('infoScreen');$('lab').onclick=()=>show('infoScreen');

// ---------- AR ----------
const ar={running:false,prev:null,features:[],pose:{R:I3(),t:[0,0,0]},objects:[],preview:null,showMap:true,showObjects:true,edit:true,size:.12,spacing:.02,mode:'surface',foveated:true,raf:0,last:0};
const arWork=document.createElement('canvas'),arCtx=arWork.getContext('2d',{willReadFrequently:true});
function arFrame(){const v=$('arCamera');if(!v||v.readyState<2)return null;arWork.width=SW;arWork.height=SH;arCtx.drawImage(v,0,0,SW,SH);const d=arCtx.getImageData(0,0,SW,SH).data,g=new Uint8Array(SW*SH);for(let i=0,j=0;i<d.length;i+=4,j++)g[j]=(77*d[i]+150*d[i+1]+29*d[i+2])>>8;return g}
function matchMap(features,s){const arr=(s?.samples||[]).filter(p=>p.d&&p.n>=2),out=[];for(const f of features){let best=null,be=999;for(const p of arr){let e=0;for(let i=0;i<f.d.length;i++)e+=Math.abs(f.d[i]-p.d[i]);e/=f.d.length;if(e<be){be=e;best=p}}if(best&&be<24){const q=project([best.x,best.y,best.z],ar.pose);if(q&&Math.hypot(q.x-f.x,q.y-f.y)<38)out.push({f,p:best})}}return out}
function bogonPoints(o){const n=clamp(Math.round((o.size||.12)/(o.spacing||.02)),3,28),h=(o.size||.12)/2,pts=[];if(o.mode==='edge'){for(let i=0;i<n;i++){const a=-h+2*h*i/(n-1);pts.push([a,-h,-h],[a,h,-h],[a,-h,h],[a,h,h])}}else{for(let face=0;face<6;face++)for(let j=0;j<n;j++)for(let i=0;i<n;i++){const a=-h+2*h*i/(n-1),b=-h+2*h*j/(n-1);pts.push(face===0?[a,b,-h]:face===1?[a,b,h]:face===2?[-h,a,b]:face===3?[h,a,b]:face===4?[a,-h,b]:[a,h,b])}}return pts}
function drawBogon(x,w,h,o,pose){const a=o.anchor,c=project([a.x,a.y,a.z],pose);if(!c)return;const R=compose(o.rx||0,o.ry||0,o.rz||0),pts=[];for(const p of bogonPoints(o)){const q=mv(R,p),s=project([a.x+q[0],a.y+q[1],a.z+q[2]],pose);if(s)pts.push(s)}for(const p of pts){if(p.x<0||p.y<0||p.x>SW||p.y>SH)continue;const near=clamp(1-(p.z-c.z)/(o.size||.12),0,1);x.fillStyle=o.foveated?(near>.66?'#ff5ea8':near>.33?'#a35cff':'#71e8ff'):'#71e8ff';x.fillRect(p.x/SW*w-1,p.y/SH*h-1,2,2)}const h3=(o.size||.12)/2,verts=[[-h3,-h3,-h3],[h3,-h3,-h3],[h3,h3,-h3],[-h3,h3,-h3],[-h3,-h3,h3],[h3,-h3,h3],[h3,h3,h3],[-h3,h3,h3]],edges=[[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]],vp=verts.map(p=>{const q=mv(R,p);return project([a.x+q[0],a.y+q[1],a.z+q[2]],pose)});x.strokeStyle='#71e8ff';x.lineWidth=1;for(const [i,j] of edges){const p=vp[i],q=vp[j];if(p&&q){x.beginPath();x.moveTo(p.x/SW*w,p.y/SH*h);x.lineTo(q.x/SW*w,q.y/SH*h);x.stroke()}}}
function cameraToScreen(px,py,w,h){const scale=Math.max(w/SW,h/SH),dw=SW*scale,dh=SH*scale;return{x:(w-dw)/2+px*scale,y:(h-dh)/2+py*scale}}
function arRender(){
  if(!ar.running)return;
  const c=$('arCanvas'),d=devicePixelRatio||1,w=innerWidth,h=innerHeight;
  if(c.width!==w*d||c.height!==h*d){c.width=w*d;c.height=h*d}
  const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);x.clearRect(0,0,w,h);
  const s=activeSpace(),locks=ar.locks||[],features=ar.features||[];

  // The camera is a normal RGB camera: these markers are visual tracking evidence,
  // not depth/plane detection. Cyan = usable visual texture, green = matched 3D map point.
  const matched=new Set(locks.map(a=>a.f));
  for(const f of features){
    const q=cameraToScreen(f.x,f.y,w,h);
    if(q.x<-8||q.y<-8||q.x>w+8||q.y>h+8)continue;
    const isMatched=matched.has(f),r=isMatched?4:2.6;
    x.strokeStyle=isMatched?'#54f2a2cc':'#71e8ffbb';x.lineWidth=isMatched?1.5:1;
    x.beginPath();x.arc(q.x,q.y,r,0,Math.PI*2);x.stroke();
    if(!isMatched){x.fillStyle='#71e8ff99';x.fillRect(q.x-1,q.y-1,2,2)}
  }
  if(ar.showMap)for(const a of locks){
    const q=project([a.p.x,a.p.y,a.p.z],ar.pose);
    if(q){const p=cameraToScreen(q.x,q.y,w,h);x.fillStyle='#54f2a2';x.shadowColor='#54f2a2';x.shadowBlur=9;x.beginPath();x.arc(p.x,p.y,3.5,0,Math.PI*2);x.fill();x.shadowBlur=0}
  }
  // Highlight the strongest currently visible reference: this tells the user exactly
  // where to aim the phone instead of presenting a generic AR reticle.
  const target=features.length?features[0]:null;
  if(target){
    const q=cameraToScreen(target.x,target.y,w,h),pulse=7+Math.sin(performance.now()/180)*2;
    x.strokeStyle='#fff';x.lineWidth=1.5;x.beginPath();x.arc(q.x,q.y,pulse,0,Math.PI*2);x.stroke();
    x.fillStyle='#fff';x.font='700 9px system-ui';x.fillText('ENFOCA AQUÍ',q.x+10,q.y-8);
  }
  if(ar.showObjects)for(const o of ar.objects)drawBogon(x,w,h,o,ar.pose);
  if(ar.preview)drawBogon(x,w,h,ar.preview,ar.pose);
  $('cloudCount').textContent=`${ar.objects.length} BOGONES · ${locks.length} REFERENCIAS 3D · ${features.length} MARCAS`;
  ar.raf=requestAnimationFrame(arLoop);
}
function arLoop(){if(!ar.running)return;const now=performance.now();if(now-ar.last<90){ar.raf=requestAnimationFrame(arLoop);return}ar.last=now;const g=arFrame(),s=activeSpace();if(g&&s){const f=detect(g),locks=matchMap(f,s);ar.locks=locks;if(locks.length>=6){const corr=locks.slice(0,30).map(a=>({p:[a.p.x,a.p.y,a.p.z],x:a.f.x,y:a.f.y}));const rr=refinePose(ar.pose,corr);if(rr.error<25)ar.pose=rr.pose}$('track').textContent=locks.length>=10?'● MAPA RECONOCIDO':locks.length>=4?'● REFERENCIAS 3D':'○ BUSCANDO TEXTURA';
    $('reticleText').textContent=locks.length>=10?'Punto verde = referencia 3D reconocida':f.length?'Enfoca esquinas, bordes o textura visible':'Busca una zona con detalle';
    $('arToast').textContent=locks.length>=10?'Mapa reconocido · mantén el teléfono estable':'Apunta a una esquina, borde o textura y muévete lentamente';
    ar.features=f;ar.prev=g}arRender()}
async function startAR(){const s=activeSpace();if(!s){toast('Primero crea un espacio');show('createScreen');return}SCREENS.forEach(id=>$(id)?.classList.toggle('active',id==='arScreen'));try{await openCamera($('arCamera'))}catch(e){toast(e.message);return}ar.running=true;ar.pose={R:I3(),t:[0,0,0]};ar.objects=(s.objects||[]).map(o=>({...o}));ar.preview=null;ar.locks=[];ar.last=0;$('arSpaceName').textContent=s.name.toUpperCase();$('arToast').textContent='Sólo cámara · busca el espacio guardado';arLoop()}
function stopAR(){ar.running=false;cancelAnimationFrame(ar.raf);ar.preview=null;ar.locks=[]}
function placeAt(x,y){if(!ar.edit)return toast('Modo visualización: no se puede crear.');const s=activeSpace(),arr=s?.samples||[];let best=null,bd=999;for(const p of arr){if(p.n<3||p.confidence<.58)continue;const q=project([p.x,p.y,p.z],ar.pose);if(!q)continue;const d=Math.hypot(q.x-x*SW,q.y-y*SH);if(d<bd){bd=d;best=p}}if(!best||bd>30)return toast('Apunta a un punto verde consolidado.');ar.preview={anchor:{x:best.x,y:best.y,z:best.z},size:ar.size,spacing:ar.spacing,mode:ar.mode,foveated:ar.foveated};$('confirmPos').textContent=`Anclaje 3D · ${best.n} observaciones`;$('confirm').classList.remove('hidden')}
$('enterAR').onclick=startAR;$('placeAction').onclick=()=>placeAt(SW/2,SH/2);$('arCanvas').addEventListener('pointerup',e=>{if(ar.running&&!e.target.closest?.('.arUI'))placeAt(e.clientX/innerWidth,e.clientY/innerHeight)});$('place').onclick=()=>{if(!ar.preview)return;ar.objects.push({...ar.preview});ar.preview=null;$('confirm').classList.add('hidden');const s=activeSpace();s.objects=ar.objects;saveDB();toast('Bogon 3D guardado')};$('cancelPlace').onclick=()=>{$('confirm').classList.add('hidden');ar.preview=null};$('exitXR').onclick=()=>{stopAR();stopCamera();show('simScreen')};$('arExit').onclick=()=>$('exitXR').click();$('arSave').onclick=()=>{const s=activeSpace();if(s){s.objects=ar.objects;saveDB();toast('Estado guardado')}};$('arMenu').onclick=()=>$('arMenuPanel').classList.toggle('hidden');$('closeArMenu').onclick=()=>$('arMenuPanel').classList.add('hidden');$('arMapToggle').onclick=()=>{ar.showMap=!ar.showMap;$('geo').innerHTML=`MAPA <b>${ar.showMap?'ON':'OFF'}</b>`};$('geo').onclick=()=>$('arMapToggle').click();$('objects').onclick=()=>{ar.showObjects=!ar.showObjects;$('objects').innerHTML=`OBJETOS <b>${ar.showObjects?'ON':'OFF'}</b>`};$('clear').onclick=()=>{ar.objects=[];const s=activeSpace();if(s){s.objects=[];saveDB()}toast('Bogones eliminados')};$('size').oninput=e=>{ar.size=+e.target.value/100;$('sizeText').textContent=e.target.value+' cm'};$('detail').oninput=e=>{ar.spacing=+e.target.value/100;$('detailText').textContent=e.target.value+' cm'};['edge','surface','volume'].forEach(id=>$(id).onclick=()=>{ar.mode=id;document.querySelectorAll('#edge,#surface,#volume').forEach(b=>b.classList.remove('selected'));$(id).classList.add('selected')});$('foveated').onclick=()=>{ar.foveated=!ar.foveated;$('foveated').classList.toggle('selected',ar.foveated)};$('deleteNearest').onclick=()=>{if(ar.objects.length){ar.objects.pop();const s=activeSpace();if(s){s.objects=ar.objects;saveDB()}}};$('arLab').onclick=()=>show('infoScreen');$('closeLab').onclick=()=>show('arScreen');
$('interactive').onclick=()=>{ar.edit=true;$('interactive').classList.add('selected');$('visualMode').classList.remove('selected')};$('creationMode').onclick=()=>{ar.edit=true;$('creationMode').classList.add('selected');$('visualMode').classList.remove('selected')};$('visualMode').onclick=()=>{ar.edit=false;$('visualMode').classList.add('selected');$('creationMode').classList.remove('selected')};
$('reset').onclick=()=>{if(confirm('¿Borrar todos los espacios?')){localStorage.removeItem(KEY);db={spaces:[],active:null};renderSpaces();renderLocal();updateSupport()}};
function updateSupport(){const cam=!!navigator.mediaDevices?.getUserMedia;$('capBadge').textContent=cam?'CÁMARA':'CÁMARA NO DISPONIBLE';$('deviceReport').innerHTML=`CÁMARA <b>${cam?'✓':'—'}</b> · PROFUNDIDAD <b>NO</b> · GPS <b>NO USADO</b> · TRACKER <b>MONOCULAR + SFM</b>`;const s=activeSpace();$('simStatus').textContent=s?`Espacio activo: ${s.name} · ${s.points||0} puntos 3D · ${s.objects?.length||0} Bogones`:'Crea y guarda un espacio para entrar en AR'}
updateSupport();renderSpaces();
