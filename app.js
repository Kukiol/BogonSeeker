'use strict';
// BeyondHome ALFA 0.13.5 — coarse spatial Bogon mapper with IMU-assisted orientation and sparse map display.
// RGB remains the primary source. Gyroscope/compass orientation is used when the browser exposes it;
// visual references remain the fallback. No depth sensor or ARCore/WebXR dependency is required.
// Monocular scale is relative: a single RGB camera cannot recover absolute metres by itself.
const $=id=>document.getElementById(id);
const SCREENS=['splash','home','cameraScreen','createScreen','spacesScreen','localScreen','infoScreen','simScreen','arScreen'];
const APP_VERSION='0.13.5';
const BUILD_ID='2026-08-16.13.5';
const KEY='beyondHome.v35';
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const uid=()=>crypto?.randomUUID?.()||'bh-'+Date.now().toString(36)+'-'+Math.random().toString(36).slice(2);
const esc=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
let db=loadDB(); let stream=null;
function loadDB(){try{const now=localStorage.getItem(KEY);if(now)return JSON.parse(now);const old=localStorage.getItem('beyondHome.v34')||localStorage.getItem('beyondHome.v33')||localStorage.getItem('beyondHome.v32')||localStorage.getItem('beyondHome.v30')||localStorage.getItem('beyondHome.v28')||localStorage.getItem('beyondHome.v27')||localStorage.getItem('beyondHome.v25')||localStorage.getItem('beyondHome.v21');return old?JSON.parse(old):{spaces:[],active:null}}catch{return{spaces:[],active:null}}}
// Old service workers/caches can keep a previous HTML/JS build on a phone. This build deliberately removes them.
(async()=>{try{if('serviceWorker' in navigator){for(const r of await navigator.serviceWorker.getRegistrations())await r.unregister()}if('caches' in window){for(const k of await caches.keys())await caches.delete(k)}}catch(e){console.debug('cache cleanup',e)}})();
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
  if(a<=0||b<=0||P[2]<.025)return null;
  // The mapper is intentionally tolerant: this is a provisional Bogon point,
  // not a photogrammetry-grade measurement. Keep points with moderate ray error
  // and let repeated observations/fusion stabilize them later.
  if(err>.85*Math.max(.08,P[2]))return null;
  return{p:P,err};
}
function keyFor(p){return `${Math.round(p.x/.045)},${Math.round(p.y/.045)},${Math.round(p.z/.045)}`}
function fuse(map,p,d,quality=0.25,err=0.02,trackerId=null){
  // A tracker identity is a stronger key than a noisy provisional XYZ estimate.
  // Once a tracker becomes spatial, all later observations reinforce the same
  // Bogon point instead of spawning a new nearby point.
  const remembered=trackerId?scan.trackKeys.get(trackerId):null;
  const k=remembered||keyFor(p),old=map.get(k);
  if(!old){map.set(k,{x:p.x,y:p.y,z:p.z,n:1,obs:1,trackers:trackerId?[trackerId]:[],confidence:clamp(quality,0.05,.95),d,err});if(trackerId)scan.trackKeys.set(trackerId,k);return}
  const n=old.n+1;
  const w=1/n;
  old.x+=(p.x-old.x)*w;old.y+=(p.y-old.y)*w;old.z+=(p.z-old.z)*w;
  old.n=Math.min(255,n);old.obs=(old.obs||1)+1;if(trackerId&&!(old.trackers||[]).includes(trackerId)){old.trackers=[...(old.trackers||[]),trackerId].slice(-12)}
  const q=clamp(quality*(1-clamp(err/.08,0,.9)),0.02,.95);
  old.confidence=clamp(old.confidence+(q-old.confidence)*.22,0,1);
  old.err=old.err+(err-old.err)*.18;
  if(!old.d)old.d=d;
  if(trackerId)scan.trackKeys.set(trackerId,k);
}
function pointReliability(p){return clamp(p.confidence*(1-Math.min(1,(p.err||0)/.08)),0,1)}
function estimateRelative(ms){
  const f=robustFlow(ms),s=similarity(ms);
  // We deliberately do NOT wait for a high-precision SfM solution here.
  // A normal RGB phone camera only gives us image motion; for the Bogon mapper
  // that motion is enough to create a provisional local spatial hypothesis.
  if(!s||f.coh<.25||f.mag<.45)return null;
  const yaw=-s.dx/FX*.55,pitch=s.dy/FY*.25;
  const radial=median(ms.map(m=>{const x=m.a.x-CX,y=m.a.y-CY,l=Math.hypot(x,y)||1;return (m.b.x-m.a.x)*(x/l)+(m.b.y-m.a.y)*(y/l)}));
  let v=[-s.dx/FX,s.dy/FY,-radial/Math.max(20,SH)];
  // If the observed motion is close to pure rotation, choose a stable lateral
  // baseline instead of throwing the whole frame away.
  if(Math.hypot(...v)<.08)v=[s.dx>=0?-1:1,0,0];
  const n=Math.hypot(...v)||1;v=v.map(x=>x/n);
  const baseline=clamp(.018+f.mag/1400,.018,.065);
  return{R:compose(pitch,yaw,0),t:v.map(x=>x*baseline),flow:f,sim:s,baseline};
}
function relativeDepthForMatch(m,rel){
  // Coarse monocular depth: remove the dominant image similarity and interpret the
  // remaining parallax as translation-induced motion. Absolute metres are not claimed;
  // the value only needs to preserve near/far ordering for the Bogon map.
  const ax=m.a.x-CX,ay=m.a.y-CY,bx=m.b.x-CX,by=m.b.y-CY;
  const c=rel.sim,ca=Math.cos(c.ang),sa=Math.sin(c.ang);
  const px=c.scale*(ca*ax-sa*ay),py=c.scale*(sa*ax+ca*ay);
  const predX=px+c.dx,predY=py+c.dy;
  const par=Math.hypot((bx-predX),(by-predY));
  const depth=clamp((FX*Math.max(rel.baseline,.012))/Math.max(par,1.2),.28,6.5);
  return {depth,parallax:par};
}
function fallbackSpatialPoint(m,rel){
  const z=relativeDepthForMatch(m,rel).depth;
  const d=ray(m.a.x,m.a.y);
  return {p:d.map(v=>v*z),err:Math.max(.06,z*.16),depth:z};
}

function poseError(pose,corr){let e=0,n=0;for(const c of corr){const q=project(c.p,pose);if(!q)continue;const d=Math.hypot(q.x-c.x,q.y-c.y);if(d<35){e+=d;n++}}return n?e/n:999}
function refinePose(pose,corr){if(corr.length<6)return{pose,error:999};let cur={R:pose.R.map(r=>r.slice()),t:[...pose.t]};for(let it=0;it<3;it++){const H=Array.from({length:6},()=>Array(6).fill(0)),g=Array(6).fill(0);let used=0;for(const c of corr){const q=project(c.p,cur);if(!q)continue;const rx=c.x-q.x,ry=c.y-q.y;if(Math.hypot(rx,ry)>40)continue;const J=[];for(let k=0;k<6;k++){const pp={R:cur.R.map(r=>r.slice()),t:[...cur.t]},eps=k<3?.004:.003;if(k<3)pp.t[k]+=eps;else pp.R=mul3(compose(k===3?eps:0,k===4?eps:0,k===5?eps:0),pp.R);const qq=project(c.p,pp);J.push(qq?[(qq.x-q.x)/eps,(qq.y-q.y)/eps]:[0,0])}for(let a=0;a<6;a++){g[a]+=J[a][0]*rx+J[a][1]*ry;for(let b=0;b<6;b++)H[a][b]+=J[a][0]*J[b][0]+J[a][1]*J[b][1]}used++}const d=solve(H.map((r,i)=>r.map((v,j)=>v+(i===j?.002:0))),g);if(!d||used<6)break;cur.t=cur.t.map((v,i)=>v+d[i]);cur.R=mul3(compose(d[3],d[4],d[5]),cur.R)}return{pose:cur,error:poseError(cur,corr)}}
function matchDescriptors(features,map){const arr=[...map.values()].filter(p=>p.d&&p.n>=2),out=[];for(const f of features){let best=null,be=999;for(const p of arr){let e=0;for(let i=0;i<f.d.length;i++)e+=Math.abs(f.d[i]-p.d[i]);e/=f.d.length;if(e<be){be=e;best=p}}if(best&&be<24)out.push({f,p:best,e:be})}return out}

const scan={running:false,started:0,last:0,prev:null,features:[],pose:{R:I3(),t:[0,0,0]},map:new Map(),keyframes:0,lastKey:0,baseline:0,good:0,coverage:new Map(),zones:new Map(),motion:0,coh:0,status:'READY',raf:0,phase:'HOLD',motionKind:'QUIETO',lastMotion:{dx:0,dy:0,mag:0},stable:[],trackKeys:new Map(),bogonAnchors:new Map(),sensorRLast:null};
function resetScannerUI(){Object.assign(scan,{running:false,started:0,last:0,prev:null,features:[],pose:{R:I3(),t:[0,0,0]},map:new Map(),keyframes:0,lastKey:0,baseline:0,good:0,coverage:new Map(),zones:new Map(),motion:0,coh:0,status:'READY',raf:0,phase:'HOLD',motionKind:'QUIETO',lastMotion:{dx:0,dy:0,mag:0},stable:[],trackKeys:new Map(),bogonAnchors:new Map(),sensorRLast:null});$('scanStart').disabled=false;$('scanFinish').disabled=true;$('scanStart').textContent='INICIAR ESCANEO';$('scanPercent').textContent='0%';$('scanPts').textContent='0';$('scanRefs').textContent='0';$('scanTime').textContent='0.0';$('scanQuality').textContent='Esperando';$('qualityBar').style.width='0%'}
function scanTime(){return scan.started?(performance.now()-scan.started)/1000:0}
function secured(){return scan.bogonAnchors.size}
function greenCount(){return scan.stable.filter(a=>a.hits>=4).length}
function worldPointForTracker(a){
  if(!a)return null;
  // First choice: the tracker has already been fused into the spatial map.
  const direct=spatialForTracker(a.id);
  if(direct&&direct.n>=2)return [direct.x,direct.y,direct.z];

  // Second choice: associate the green tracker with the closest projected 3D sample.
  let best=null,bd=28;
  for(const p of scan.map.values()){
    if((p.n||0)<2)continue;
    const q=project([p.x,p.y,p.z],scan.pose);if(!q)continue;
    const d=Math.hypot(q.x-a.x,q.y-a.y);
    if(d<bd){bd=d;best=p;}
  }
  if(best)return [best.x,best.y,best.z];

  // Last-resort Bogon spatial hypothesis. It is still in the SAME world frame:
  // camera centre + a camera ray transformed by the current pose. The depth is
  // deliberately coarse; later observations can replace/fuse it.
  const C=camCenter(scan.pose),d=mv(mt(scan.pose.R),ray(a.x,a.y));
  const depth=clamp(a.depth||(.75+1.15*(1-a.confidence)),.35,6.5);
  return C.map((v,i)=>v+d[i]*depth);
}
function buildBogonAnchors(){
  const greens=scan.stable.filter(a=>a.hits>=4&&a.confidence>=.45);
  const groups=new Map();
  for(const a of greens){const k=zoneKey(a.x,a.y);if(!groups.has(k))groups.set(k,[]);groups.get(k).push(a)}

  // Every blue Bogon anchor is now derived from WORLD positions, never from
  // the 2D image grid. The image zone is only its label/coverage bucket.
  for(const [zone,arr] of groups){
    if(arr.length<2)continue;
    const used=new Set();
    for(const seed of arr){
      if(used.has(seed.id))continue;
      const cluster=arr.filter(b=>!used.has(b.id)&&Math.hypot(b.x-seed.x,b.y-seed.y)<52);
      if(cluster.length<2)continue;
      cluster.forEach(b=>used.add(b.id));
      const ids=cluster.map(b=>b.id).sort();
      const positions=cluster.map(worldPointForTracker).filter(Boolean);
      if(positions.length<2)continue;
      const pos=[0,0,0];
      positions.forEach(q=>{for(let i=0;i<3;i++)pos[i]+=q[i]/positions.length});
      const spread=positions.length>1?positions.reduce((s,q)=>s+Math.hypot(q[0]-pos[0],q[1]-pos[1],q[2]-pos[2]),0)/positions.length:0;
      const id='bogon-'+ids.slice(0,4).join('-');
      const old=scan.bogonAnchors.get(id);
      const alpha=old?.n?0.22:1;
      const fused=old?[old.x+(pos[0]-old.x)*alpha,old.y+(pos[1]-old.y)*alpha,old.z+(pos[2]-old.z)*alpha]:pos;
      scan.bogonAnchors.set(id,{id,x:fused[0],y:fused[1],z:fused[2],trackers:ids,zone,n:old?Math.min(255,old.n+1):1,confidence:Math.min(1,(old?.confidence||.45)+.08),spread});
    }
  }

  // Once there are enough green trackers, ensure sparse room coverage. These
  // anchors are still world-space: no fake X/Z coordinates are generated from
  // screen columns. This gives us a solid general room before detail scanning.
  if(greens.length>=8){
    for(const a of greens.slice(0,24)){
      const ids=greens.filter(b=>Math.hypot(b.x-a.x,b.y-a.y)<70).slice(0,4).map(b=>b.id).sort();
      if(ids.length<2)continue;
      const positions=ids.map(id=>worldPointForTracker(scan.stable.find(b=>b.id===id))).filter(Boolean);
      if(positions.length<2)continue;
      const pos=[0,0,0];positions.forEach(q=>{for(let i=0;i<3;i++)pos[i]+=q[i]/positions.length});
      const id='surface-'+ids.slice(0,3).join('-');
      const old=scan.bogonAnchors.get(id);
      scan.bogonAnchors.set(id,{id,x:pos[0],y:pos[1],z:pos[2],trackers:ids,zone:zoneKey(a.x,a.y),n:(old?.n||0)+1,confidence:Math.min(1,(old?.confidence||.48)+.04),spread:old?.spread||0});
    }
  }
}
function zoneKey(x,y){return Math.min(2,Math.max(0,Math.floor(x/(SW/3))))+','+Math.min(1,Math.max(0,Math.floor(y/(SH/2))));}
function zoneCount(){let n=0;for(const z of scan.zones.values())if(z.seen>=3)n++;return n}
function coverageScore(){let sum=0,n=0;for(const c of scan.coverage.values()){sum+=c.score;n++}return n?sum/n:0}
function readiness(){const a=clamp(greenCount()/12,0,1),b=clamp(secured()/8,0,1),c=clamp(zoneCount()/4,0,1),d=clamp(scan.map.size/10,0,1),e=clamp(coverageScore(),0,1);return Math.round(100*(a*.20+b*.35+c*.30+d*.05+e*.10))}
function canSave(){return scanTime()>=2&&zoneCount()>=2&&secured()>=4}
function motionLabel(flow){const ax=Math.abs(flow.dx),ay=Math.abs(flow.dy),m=flow.mag;if(m<1.2)return 'QUIETO';if(ax>ay*1.35)return flow.dx>0?'DERECHA':'IZQUIERDA';if(ay>ax*1.35)return flow.dy>0?'ABAJO / TILT':'ARRIBA / TILT';return 'DESPLAZAMIENTO / ROTACIÓN'}
function updateCoverage(features){for(const f of features){const gx=Math.floor(f.x/16),gy=Math.floor(f.y/16),k=gx+','+gy,c=scan.coverage.get(k)||{seen:0,stable:0,parallax:0,score:0};c.seen++;c.score=clamp(c.score*.85+.12,0,1);scan.coverage.set(k,c)}}
function markCoverageFromPoint(p,q){if(!q)return;const gx=Math.floor(q.x/16),gy=Math.floor(q.y/16),k=gx+','+gy,c=scan.coverage.get(k)||{seen:0,stable:0,parallax:0,score:0};c.stable++;c.parallax++;c.score=clamp(c.score+.12,0,1);scan.coverage.set(k,c)}
function markZone(q){if(!q)return;const k=zoneKey(q.x,q.y),z=scan.zones.get(k)||{seen:0,points:0,last:0};z.seen++;z.points=Math.min(99,z.points+1);z.last=performance.now();scan.zones.set(k,z)}
function spatialForTracker(id){if(!id)return null;const k=scan.trackKeys.get(id);return k?scan.map.get(k)||null:null}
function colorForReliability(r){return r>=.82?'#54f2a2':r>=.5?'#ffd166':'#ff4d6d'}
function updateStableTracks(matches){
  // Tracker state is intentionally much less demanding than 3D reconstruction.
  // The UI describes the life-cycle of a reference, not its geometric precision:
  // RED -> ORANGE -> YELLOW -> GREEN -> BLUE.
  const next=[];
  for(const m of matches){
    const x=m.b.x,y=m.b.y;
    let best=null,bd=20;
    for(const a of scan.stable){const d=Math.hypot(a.x-x,a.y-y);if(d<bd){bd=d;best=a}}
    if(best){
      next.push({id:best.id,x:x*.65+best.x*.35,y:y*.65+best.y*.35,hits:Math.min(60,best.hits+1),miss:0,confidence:Math.min(1,best.confidence+.12),group:best.group||zoneKey(x,y)});
      best._used=true;
    }else{
      next.push({id:uid(),x,y,hits:1,miss:0,confidence:.25,group:zoneKey(x,y)});
    }
  }
  // Recently lost references remain known, so if they reappear they are ORANGE/
  // YELLOW/GREEN rather than becoming a brand-new RED reference.
  for(const a of scan.stable){
    if(!a._used && a.miss<8) next.push({...a,miss:a.miss+1,confidence:a.confidence*.97});
  }
  scan.stable=next.filter(a=>a.miss<8).slice(0,160);
  for(const a of scan.stable)delete a._used;
}
function stableAt(x,y){let best=null,bd=15;for(const a of scan.stable){const d=Math.hypot(a.x-x,a.y-y);if(d<bd){bd=d;best=a}}return best}
function trackerState(a){
  if(!a)return 'red';
  if(a.hits<=1)return 'orange';
  if(a.hits===2)return 'yellow';
  return 'green';
}
function stateColor(state){
  return state==='blue'?'#4da6ff':state==='green'?'#54f2a2':state==='yellow'?'#ffd84d':state==='orange'?'#ff9d3d':'#ff4d6d';
}
function drawScan(features){
  const c=$('scanPreview'),d=devicePixelRatio||1,w=c.clientWidth||320,h=c.clientHeight||360;
  if(c.width!==w*d||c.height!==h*d){c.width=w*d;c.height=h*d}
  const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);x.clearRect(0,0,w,h);

  // Reference lifecycle:
  // RED    = never seen before.
  // ORANGE = seen previously.
  // YELLOW = one more successful observation and it becomes a pattern.
  // GREEN  = fixed in an abstract environment/group.
  // BLUE   = triangulated fixed spatial point; part of the final Bogon mesh.
  for(const f of features){
    const a=stableAt(f.x,f.y);
    const blue=!!(a&&[...scan.bogonAnchors.values()].some(b=>b.trackers.includes(a.id)));
    // Blue is a coarse Bogon spatial anchor, not a photogrammetry-grade measurement.
    const state=blue?'blue':trackerState(a),color=stateColor(state);
    const sx=f.x/SW*w,sy=f.y/SH*h;
    x.fillStyle=color;x.shadowColor=color;x.shadowBlur=state==='blue'||state==='green'?9:6;
    x.beginPath();x.arc(sx,sy,state==='blue'?4:state==='green'?3.2:state==='yellow'?3:2.6,0,Math.PI*2);x.fill();x.shadowBlur=0;
    if(state==='green'||state==='blue'){x.strokeStyle=color+'88';x.lineWidth=1;x.beginPath();x.arc(sx,sy,state==='blue'?7:6,0,Math.PI*2);x.stroke()}
  }

  // BLUE: coarse spatial anchors projected back onto the current camera image.
  // They are built from several stable trackers and form the solid first-pass Bogon mesh.
  for(const a of scan.bogonAnchors.values()){
    const members=a.trackers.map(id=>scan.stable.find(s=>s.id===id)).filter(Boolean);
    const sx=members.length?members.reduce((s,v)=>s+v.x,0)/members.length/SW*w:(a.x+.0);
    const sy=members.length?members.reduce((s,v)=>s+v.y,0)/members.length/SH*h:(a.y+.0);
    x.fillStyle='#4da6ff';x.shadowColor='#4da6ff';x.shadowBlur=12;x.beginPath();x.arc(sx,sy,5,0,Math.PI*2);x.fill();x.shadowBlur=0;x.strokeStyle='#4da6ff99';x.lineWidth=1;x.beginPath();x.arc(sx,sy,9,0,Math.PI*2);x.stroke();
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
  for(const a of projected){const blue=a.p.n>=2; x.fillStyle=blue?'#4da6ff':'#54f2a2';x.shadowColor=x.fillStyle;x.shadowBlur=7;x.beginPath();x.arc(a.sx,a.sy,blue?3.4:2.8,0,Math.PI*2);x.fill();x.shadowBlur=0}

  // Red coverage probes mark image regions that have been observed but still lack reliable 3D.
  for(const [k,cov] of scan.coverage){if(cov.score>=.78)continue;const [gx,gy]=k.split(',').map(Number),cx=(gx*16+8)/SW*w,cy=(gy*16+8)/SH*h;x.fillStyle=`rgba(255,77,109,${.18+.25*(1-cov.score)})`;x.beginPath();x.arc(cx,cy,2.4,0,Math.PI*2);x.fill()}

  const p=readiness();
  $('scanPercent').textContent=p+'%';$('scanPts').textContent=scan.map.size;$('scanRefs').textContent=features.length;const counts={red:0,orange:0,yellow:0,green:0,blue:secured()}; for(const f of features){const a=stableAt(f.x,f.y),st=trackerState(a); if(st!=='blue')counts[st]++;} $('scanStatus').textContent=`ROJOS ${counts.red} · NARANJAS ${counts.orange} · AMARILLOS ${counts.yellow} · VERDES ${counts.green} · AZULES ${scan.bogonAnchors.size}`;$('scanTime').textContent=scanTime().toFixed(1);$('scanCoverage').textContent=`${zoneCount()}/6 zonas`;
  $('scanQuality').textContent=canSave()?'MAPA GENERAL LISTO':scan.bogonAnchors.size>0?'CONSTRUYENDO MALLA BOGON':greenCount()>0?'FIJANDO ENTORNO':'BUSCANDO REFERENCIAS';
  $('qualityBar').style.width=p+'%';$('scanFinish').disabled=!canSave();$('scanReady').textContent=canSave()?'✓ suficiente evidencia 3D':'Añade más vistas';
  const motionHUD=$('scanMotion'); if(motionHUD)motionHUD.textContent=`${scan.motionKind} · ${scan.motion.toFixed(1)} px · ${Math.round(scan.coh*100)}% coherencia · ${features.length} refs`;
  const t=scanTime();
  if(t<1.2){$('scanGuide').textContent='QUÉDATE QUIETO';$('scanHint').textContent='Estamos fijando las primeras referencias visuales.'}
  else if(features.length<12){$('scanGuide').textContent='BUSCA TEXTURA';$('scanHint').textContent='Apunta a esquinas, muebles, libros, marcos o superficies con detalle.'}
  else {$('scanGuide').textContent=scan.motionKind==='QUIETO'?'GIRA LENTAMENTE POR LA HABITACIÓN':'SIGUE SUAVEMENTE';$('scanHint').textContent='Buscamos la forma general por zonas. No necesitas acercarte ni escanear el detalle; el detalle se hará después.'}
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
    // Room scanning is coarse: a zone becomes known from repeated visual
    // occupation, even before a precise 3D solution exists.
    for(const ff of f)markZone(ff);
    const t=scanTime();scan.phase=t<1.5?'HOLD':t<6?'EXPLORE':'REFINE';
    if(flow.mag<1.2&&flow.coh>.45)scan.status='STILL';else if(flow.mag>1.4&&flow.coh>.35)scan.status='MOVING';else scan.status=f.length?'TRACKING':'LOW TEXTURE';
    // Seed the first frame as image references. They stay red until 3D evidence exists.
    if(!scan.features.length&&f.length){scan.features=f;}
    if(matches.length>=6&&flow.coh>.20&&flow.mag>.45&&(!scan.lastKey||now-scan.lastKey>480)){
      const visualRel=estimateRelative(matches);
      if(visualRel){
        const imuRel=sensorRelativeRotation();
        const rel={...visualRel,R:imuRel||visualRel.R};
        const oldPose={R:scan.pose.R.map(r=>r.slice()),t:[...scan.pose.t]};
        const newR=mul3(rel.R,oldPose.R);
        const newT=mv(rel.R,oldPose.t).map((v,i)=>v+rel.t[i]);
        scan.pose={R:newR,t:newT};
        let added=0;
        for(const m of matches){
          let q=triangulate(m.a,m.b,rel.R,rel.t);
          // Fallback: estimate a provisional depth directly from parallax.
          // This is deliberately coarse and is later stabilized by fusion.
          if(!q){
            const par=Math.hypot(m.b.x-m.a.x,m.b.y-m.a.y);
            if(par<.35)continue;
            const depth=clamp((FX*rel.baseline)/par,.18,5.5);
            const d=ray(m.a.x,m.a.y);
            q={p:d.map(v=>v*depth),err:Math.min(.22,par/100)};
          }
          const pw=mv(mt(oldPose.R),q.p.map((v,i)=>v-oldPose.t[i]));
          const quality=clamp(.72-Math.min(.45,q.err/Math.max(.05,q.p[2]*.6)),.18,.82);
          const tracker=stableAt(m.b.x,m.b.y); fuse(scan.map,pw,m.a.d,quality,q.err,tracker?.id||null);
          const qq=project(pw,scan.pose);markCoverageFromPoint(pw,qq);added++;
        }
        if(added>0){scan.keyframes++;scan.lastKey=now;scan.baseline=Math.max(scan.baseline,Math.hypot(...rel.t));scan.good+=added}
      }
    }
    if(scan.map.size>=4){
      const locks=matchDescriptors(f,scan.map),corr=[];
      for(const a of locks.slice(0,30))corr.push({p:[a.p.x,a.p.y,a.p.z],x:a.f.x,y:a.f.y});
      if(corr.length>=6){const rr=refinePose(scan.pose,corr);if(rr.error<30)scan.pose=rr.pose}
    }
    updateStableTracks(matches);buildBogonAnchors();scan.prev=g;scan.features=f;drawScan(f);
    $('scanGuide').textContent=t<1.5?'QUIETO · FIJANDO REFERENCIAS':(canSave()?'MAPA GENERAL LISTO':'ESCANEANDO ZONAS');
    $('scanHint').textContent=t<1.5?'Mantén el móvil quieto un instante para registrar referencias.':(canSave()?'Ya tenemos una representación general; el detalle puede hacerse después.':'Gira lentamente para cubrir izquierda, centro y derecha. No hace falta caminar ni buscar cada detalle.');
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
    scan.sensorRLast=null;
    await startOrientationSensors();
    $('scanStart').disabled=true;$('scanStart').textContent='ESCANEANDO…';
    const waitHUD=$('scanWait'); if(waitHUD)waitHUD.textContent='Sólo cámara · quieto 1 s y después gira suavemente por la habitación';
    toast('Cámara activa. Quieto 1 s; después gira suavemente y apunta a distintas zonas. No necesitas caminar.');
    scan.raf=requestAnimationFrame(scannerLoop);
  }catch(e){
    scan.running=false;
    $('scanGuide').textContent='CÁMARA NO DISPONIBLE';
    $('scanHint').textContent=e.message||'Comprueba los permisos de cámara y HTTPS.';
    toast(e.message||'No se pudo iniciar la cámara.');
  }
}
function stopScanner(){scan.running=false;cancelAnimationFrame(scan.raf);stopOrientationSensors();scan.sensorRLast=null}
function finishScan(){if(!canSave()){toast('Aún falta evidencia 3D. Sigue moviéndote y reforzando zonas rojas.');return}stopScanner();const pts=[...scan.map.values()].filter(p=>p.n>=2&&pointReliability(p)>.35);const anchors=[...scan.bogonAnchors.values()].map(a=>{const src=a.trackers.map(id=>spatialForTracker(id)).find(p=>p&&p.d)||null;return {x:a.x,y:a.y,z:a.z,n:Math.max(2,a.n),obs:a.n,trackers:a.trackers,confidence:a.confidence,err:Math.max(.06,a.spread||.12),d:src?.d||null,depth:Math.max(.28,...a.trackers.map(id=>scan.stable.find(t=>t.id===id)?.depth||0)),bogon:true,anchorId:a.id,zone:a.zone};});const sparsePts=sparsifyWorldPoints(pts,48);const samples=[...sparsePts,...anchors];const s={id:uid(),name:(($('spaceName').value||'Mi espacio').trim()),created:new Date().toLocaleString(),method:'camera-monocular-bogon-coarse-v15-imu-depth',scale:'abstract-relative',version:35,appVersion:APP_VERSION,build:BUILD_ID,coverage:readiness(),points:samples.length,secured:anchors.length,samples,objects:[],camera:{fx:FX,fy:FY,width:SW,height:SH}};db.spaces.push(s);db.active=s.id;saveDB();renderSpaces();renderLocal();updateSupport();toast(`Mapa guardado · ${samples.length} puntos Bogon`);setTimeout(()=>show('simScreen'),200)}
function sparsifyWorldPoints(points,maxCount=48){
  const sorted=[...points].sort((a,b)=>pointReliability(b)-pointReliability(a));
  const out=[];
  const cell=.22;
  const used=new Set();
  for(const p of sorted){
    const k=`${Math.floor(p.x/cell)},${Math.floor(p.y/cell)},${Math.floor(p.z/cell)}`;
    if(used.has(k))continue;
    used.add(k);out.push(p);
    if(out.length>=maxCount)break;
  }
  return out;
}
$('scanStart').onclick=startScanner;$('scanFinish').onclick=finishScan;

// ---------- Spaces / map ----------
function renderSpaces(){const el=$('spaceList');if(!el)return;el.innerHTML=db.spaces.length?db.spaces.map(s=>`<div class="spaceItem"><div><b>${esc(s.name)}</b><span>${s.points||0} puntos · ${s.secured||0} consolidados · ${esc(s.created)}</span></div><div class="spaceBtns"><button data-open="${s.id}">USAR</button><button data-del="${s.id}">BORRAR</button></div></div>`).join(''):`<div class="empty"><h2>No hay espacios</h2><p>Crea el primero usando únicamente la cámara.</p></div>`;el.querySelectorAll('[data-open]').forEach(b=>b.onclick=()=>{db.active=b.dataset.open;saveDB();show('simScreen')});el.querySelectorAll('[data-del]').forEach(b=>b.onclick=()=>{db.spaces=db.spaces.filter(s=>s.id!==b.dataset.del);if(db.active===b.dataset.del)db.active=db.spaces[0]?.id||null;saveDB();renderSpaces();renderLocal()})}
let mapView={yaw:.5,pitch:-.35,zoom:1};
function renderLocal(){
  const c=$('localMap');if(!c)return;
  const s=activeSpace();$('localTitle').textContent=s?s.name:'Sin espacio';
  const d=devicePixelRatio||1,w=c.clientWidth||innerWidth,h=c.clientHeight||innerHeight;
  c.width=w*d;c.height=h*d;
  const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);
  x.fillStyle='#05070b';x.fillRect(0,0,w,h);
  if(!s){$('localMeta').textContent='Crea y guarda un espacio para ver su mapa Bogon.';return}
  const cy=Math.cos(mapView.yaw),sy=Math.sin(mapView.yaw),cp=Math.cos(mapView.pitch),sp=Math.sin(mapView.pitch);
  // The editor intentionally shows a sparse representation: anchors first, then a
  // small reliability-filtered sample. The dense tracking cloud remains in the logic
  // but is never dumped wholesale into this canvas.
  const anchors=(s.samples||[]).filter(p=>p.bogon);
  const rest=(s.samples||[]).filter(p=>!p.bogon&&pointReliability(p)>=.55).slice(0,24);
  const pts=[...anchors,...rest];
  if(!pts.length){$('localMeta').textContent='Mapa vacío.';return}
  const span=Math.max(.8,...pts.map(p=>Math.max(Math.abs(p.x),Math.abs(p.y),Math.abs(p.z))));
  const scale=Math.min(w,h)*.34/(span*1.25)*mapView.zoom;
  const q=[];
  for(const p of pts){
    let X=p.x*cy-p.z*sy,Z=p.x*sy+p.z*cy,Y=p.y*cp-Z*sp;Z=p.y*sp+Z*cp;
    q.push({x:w/2+X*scale,y:h/2-Y*scale,z:Z,r:pointReliability(p),bogon:!!p.bogon,p});
  }
  q.sort((a,b)=>b.z-a.z);
  // Only connect nearby Bogon anchors; this keeps the visual map readable.
  for(let i=0;i<q.length;i++){
    if(!q[i].bogon)continue;
    let links=0;
    for(let j=i+1;j<q.length&&links<3;j++){
      if(!q[j].bogon)continue;
      const dx=q[i].p.x-q[j].p.x,dy=q[i].p.y-q[j].p.y,dz=q[i].p.z-q[j].p.z;
      if(Math.hypot(dx,dy,dz)<.65){
        x.strokeStyle='#4da6ff44';x.lineWidth=1;x.beginPath();x.moveTo(q[i].x,q[i].y);x.lineTo(q[j].x,q[j].y);x.stroke();links++;
      }
    }
  }
  for(const p of q){
    const perspective=clamp(1.25/(1.15+p.z*.65),.55,1.5);
    const r=p.bogon?clamp(5*perspective,2.2,7):clamp(2.5*perspective,1.2,3.5);
    x.fillStyle=p.bogon?'#4da6ff':colorForReliability(p.r);
    x.globalAlpha=p.bogon?1:.7;
    x.beginPath();x.arc(p.x,p.y,r,0,Math.PI*2);x.fill();
    x.globalAlpha=1;
  }
  $('localMeta').textContent=`${anchors.length} Bogones espaciales + ${rest.length} referencias · mapa visual reducido · profundidad relativa · escala abstracta`;
}
$('mapReset').onclick=()=>{mapView={yaw:.5,pitch:-.35,zoom:1};renderLocal()};let drag=null;$('localMap').addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY,yaw:mapView.yaw,pitch:mapView.pitch}});$('localMap').addEventListener('pointermove',e=>{if(!drag)return;mapView.yaw=drag.yaw+(e.clientX-drag.x)*.01;mapView.pitch=clamp(drag.pitch+(e.clientY-drag.y)*.01,-1.4,1.4);renderLocal()});['pointerup','pointercancel','pointerleave'].forEach(ev=>$('localMap').addEventListener(ev,()=>drag=null));

// ---------- Explanation ----------
let infoRAF=0;function stopInfo(){cancelAnimationFrame(infoRAF)}function startInfo(){stopInfo();const c=$('bogonExplain'),d=devicePixelRatio||1;const f=t=>{const w=c.clientWidth||340,h=c.clientHeight||260;c.width=w*d;c.height=h*d;const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);x.clearRect(0,0,w,h);const ph=(t%9000)/9000;const n=Math.floor(20+ph*600),cx=w/2,cy=h*.48,s=Math.min(w,h)*.24;for(let i=0;i<n;i++){const a=i/n*Math.PI*2*3.8,r=s*(.25+.75*i/n),xx=cx+Math.cos(a)*r,yy=cy+Math.sin(a)*r*.65;x.fillStyle=i/n>.7?'#ff5ea8':i/n>.4?'#a35cff':'#71e8ff';x.fillRect(xx,yy,1.4,1.4)}x.fillStyle='#fff';x.font='800 11px system-ui';x.fillText(ph<.33?'UNA FUNCIÓN':ph<.66?'MUESTRAS': 'DETALLE ADAPTATIVO',14,h-16);infoRAF=requestAnimationFrame(f)};infoRAF=requestAnimationFrame(f)}
$('bogonLab').onclick=()=>show('infoScreen');$('lab').onclick=()=>show('infoScreen');

// ---------- Optional device orientation / compass ----------
const imu={supported:false,active:false,absolute:false,permission:false,initial:null,current:null,heading:null,screen:0,handler:null,screenHandler:null};
function angleDelta(a,b){let d=(a-b+540)%360-180;return d}
function orientationSample(e){
  const heading=Number.isFinite(e.webkitCompassHeading)?e.webkitCompassHeading:(Number.isFinite(e.alpha)?e.alpha:null);
  const sample={alpha:Number.isFinite(e.alpha)?e.alpha:0,beta:Number.isFinite(e.beta)?e.beta:0,gamma:Number.isFinite(e.gamma)?e.gamma:0,heading};
  imu.current=sample;if(sample.heading!=null)imu.heading=sample.heading;
}
function screenAngle(){return Number((screen.orientation?.angle ?? window.orientation ?? 0))||0}
function orientationDelta(){
  if(!imu.initial||!imu.current)return null;
  const a=imu.initial,b=imu.current;
  const yaw=a.heading!=null&&b.heading!=null?angleDelta(b.heading,a.heading):angleDelta(b.alpha,a.alpha);
  const pitch=b.beta-a.beta;
  const roll=b.gamma-a.gamma;
  return {yaw,pitch,roll};
}
function sensorRotation(){
  const d=orientationDelta();if(!d)return null;
  // Pose is world -> camera, therefore the camera's measured orientation is inverted.
  return compose(-d.pitch*Math.PI/180,-d.yaw*Math.PI/180,-d.roll*Math.PI/180);
}
function sensorRelativeRotation(){
  const absolute=sensorRotation();
  if(!absolute)return null;
  const previous=scan.sensorRLast||I3();
  const relative=mul3(absolute,mt(previous));
  scan.sensorRLast=absolute;
  return relative;
}
async function startOrientationSensors(){
  imu.supported=('DeviceOrientationEvent' in window);
  if(!imu.supported)return false;
  try{
    if(typeof DeviceOrientationEvent.requestPermission==='function'){
      const p=await DeviceOrientationEvent.requestPermission();imu.permission=p==='granted';if(!imu.permission)return false;
    }else imu.permission=true;
  }catch(e){imu.permission=false;return false}
  if(imu.handler){window.removeEventListener('deviceorientationabsolute',imu.handler,true);window.removeEventListener('deviceorientation',imu.handler,true)}
  if(imu.screenHandler)window.removeEventListener('orientationchange',imu.screenHandler,true);
  imu.handler=e=>orientationSample(e);imu.screenHandler=()=>{imu.screen=screenAngle()};
  window.addEventListener('deviceorientationabsolute',imu.handler,true);
  window.addEventListener('deviceorientation',imu.handler,true);
  window.addEventListener('orientationchange',imu.screenHandler,true);
  imu.active=true;imu.absolute=('ondeviceorientationabsolute' in window);imu.screen=screenAngle();
  await new Promise(r=>setTimeout(r,180));
  imu.initial=imu.current?{...imu.current}:null;
  return !!imu.initial;
}
function stopOrientationSensors(){
  if(imu.handler){window.removeEventListener('deviceorientationabsolute',imu.handler,true);window.removeEventListener('deviceorientation',imu.handler,true)}
  if(imu.screenHandler)window.removeEventListener('orientationchange',imu.screenHandler,true);
  imu.active=false;imu.initial=null;imu.current=null;imu.heading=null;
}
function sensorInfo(){return imu.active?'IMU ✓':'IMU visual'}

// ---------- AR ----------
const ar={running:false,prev:null,features:[],pose:{R:I3(),t:[0,0,0]},objects:[],preview:null,showMap:true,showObjects:true,edit:true,size:.12,spacing:.02,mode:'surface',foveated:true,raf:0,last:0};
const arWork=document.createElement('canvas'),arCtx=arWork.getContext('2d',{willReadFrequently:true});
function arFrame(){const v=$('arCamera');if(!v||v.readyState<2)return null;arWork.width=SW;arWork.height=SH;arCtx.drawImage(v,0,0,SW,SH);const d=arCtx.getImageData(0,0,SW,SH).data,g=new Uint8Array(SW*SH);for(let i=0,j=0;i<d.length;i+=4,j++)g[j]=(77*d[i]+150*d[i+1]+29*d[i+2])>>8;return g}
function matchMap(features,s){const arr=(s?.samples||[]).filter(p=>p.d&&p.n>=2),out=[];for(const f of features){let best=null,be=999;for(const p of arr){let e=0;for(let i=0;i<f.d.length;i++)e+=Math.abs(f.d[i]-p.d[i]);e/=f.d.length;if(e<be){be=e;best=p}}if(best&&be<24){const q=project([best.x,best.y,best.z],ar.pose);if(q&&Math.hypot(q.x-f.x,q.y-f.y)<38)out.push({f,p:best})}}return out}
function bogonPoints(o){const n=clamp(Math.round((o.size||.12)/(o.spacing||.02)),3,28),h=(o.size||.12)/2,pts=[];if(o.mode==='edge'){for(let i=0;i<n;i++){const a=-h+2*h*i/(n-1);pts.push([a,-h,-h],[a,h,-h],[a,-h,h],[a,h,h])}}else{for(let face=0;face<6;face++)for(let j=0;j<n;j++)for(let i=0;i<n;i++){const a=-h+2*h*i/(n-1),b=-h+2*h*j/(n-1);pts.push(face===0?[a,b,-h]:face===1?[a,b,h]:face===2?[-h,a,b]:face===3?[h,a,b]:face===4?[a,-h,b]:[a,h,b])}}return pts}
function drawBogon(x,w,h,o,pose){const a=o.anchor,c=project([a.x,a.y,a.z],pose);if(!c)return;const worldSize=o.size||.12;const size=depthVisualScale(c.z,worldSize);const R=compose(o.rx||0,o.ry||0,o.rz||0),pts=[];for(const p of bogonPoints({...o,size})){const q=mv(R,p),s=project([a.x+q[0],a.y+q[1],a.z+q[2]],pose);if(s)pts.push(s)}for(const p of pts){if(p.x<0||p.y<0||p.x>SW||p.y>SH)continue;const near=clamp(1-(p.z-c.z)/(o.size||.12),0,1);x.fillStyle=o.foveated?(near>.66?'#ff5ea8':near>.33?'#a35cff':'#71e8ff'):'#71e8ff';x.fillRect(p.x/SW*w-1,p.y/SH*h-1,2,2)}const h3=size/2,verts=[[-h3,-h3,-h3],[h3,-h3,-h3],[h3,h3,-h3],[-h3,h3,-h3],[-h3,-h3,h3],[h3,-h3,h3],[h3,h3,h3],[-h3,h3,h3]],edges=[[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]],vp=verts.map(p=>{const q=mv(R,p);return project([a.x+q[0],a.y+q[1],a.z+q[2]],pose)});x.strokeStyle='#71e8ff';x.lineWidth=1;for(const [i,j] of edges){const p=vp[i],q=vp[j];if(p&&q){x.beginPath();x.moveTo(p.x/SW*w,p.y/SH*h);x.lineTo(q.x/SW*w,q.y/SH*h);x.stroke()}}}
function cameraToScreen(px,py,w,h){const scale=Math.max(w/SW,h/SH),dw=SW*scale,dh=SH*scale;return{x:(w-dw)/2+px*scale,y:(h-dh)/2+py*scale}}
function applySensorPose(){
  const sr=sensorRotation();
  if(!sr)return;
  // Keep the translation solved visually, while using the phone's orientation as a strong
  // rotation prior. This stops a placed Bogon from behaving like a HUD sticker when the user turns.
  ar.pose.R=sr;
}
function visualTranslationFromLocks(locks){
  if(locks.length<4)return;
  // Solve a small translation correction from reprojection residuals with the current rotation.
  // This is deliberately damped: it gives the map a stable spatial drift model without pretending
  // that monocular RGB provides metric SLAM.
  let A=[],B=[];
  for(const l of locks.slice(0,24)){
    const p=transform(ar.pose,l.p);if(p[2]<=.05)continue;
    const u=FX*p[0]/p[2]+CX,v=FY*p[1]/p[2]+CY,rx=l.f.x-u,ry=l.f.y-v;
    const z=Math.max(.25,Math.min(6.5,p[2]));
    A.push([-FX/z,0,FX*p[0]/(z*z),0,-FY/z,FY*p[1]/(z*z)]);
    B.push(rx,ry);
  }
  if(A.length<4)return;
  const H=Array.from({length:3},()=>Array(3).fill(0)),g=Array(3).fill(0);
  for(let i=0;i<A.length;i+=2){const ax=A[i],ay=A[i+1],bx=B[i],by=B[i+1];for(let j=0;j<3;j++){g[j]+=ax[j]*bx+ay[j]*by;for(let k=0;k<3;k++)H[j][k]+=ax[j]*ax[k]+ay[j]*ay[k]}}
  const d=solve(H.map((r,i)=>r.map((v,j)=>v+(i===j?.02:0))),g);if(d){for(let i=0;i<3;i++)ar.pose.t[i]+=clamp(d[i],-.035,.035)*.45}
}
function depthVisualScale(z,size){return clamp(size*(1.8/Math.max(.25,z)),.045,size*2.8)}

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
  // The saved room itself is made of coarse Bogon cubes. They are NOT screen-space
  // decorations: their anchors are the world-space coordinates created during scan.
  if(ar.showMap && s){
    const roomBogons=(s.samples||[]).filter(p=>p.bogon);
    for(const p of roomBogons){
      drawBogon(x,w,h,{anchor:{x:p.x,y:p.y,z:p.z},size:Math.max(.10,Math.min(.34,(p.spread||.18)*1.8)),spacing:.035,mode:'surface',foveated:false,rx:0,ry:0,rz:0},ar.pose);
    }
  }
  if(ar.showObjects)for(const o of ar.objects)drawBogon(x,w,h,o,ar.pose);
  if(ar.preview)drawBogon(x,w,h,ar.preview,ar.pose);
  const roomBogonCount=s?(s.samples||[]).filter(p=>p.bogon).length:0; $('cloudCount').textContent=`${roomBogonCount} BOGONES DE MAPA · ${ar.objects.length} OBJETOS · ${locks.length} REFERENCIAS 3D · ${sensorInfo()}`;
  ar.raf=requestAnimationFrame(arLoop);
}
function arLoop(){if(!ar.running)return;const now=performance.now();if(now-ar.last<70){ar.raf=requestAnimationFrame(arLoop);return}ar.last=now;const g=arFrame(),s=activeSpace();if(g&&s){applySensorPose();const f=detect(g),locks=matchMap(f,s);ar.locks=locks;if(locks.length>=4){visualTranslationFromLocks(locks)}if(locks.length>=6){const corr=locks.slice(0,30).map(a=>({p:[a.p.x,a.p.y,a.p.z],x:a.f.x,y:a.f.y}));const rr=refinePose(ar.pose,corr);if(rr.error<28){ar.pose=rr.pose}}$('track').textContent=locks.length>=10?'● MAPA RECONOCIDO':locks.length>=4?'● REFERENCIAS 3D':'○ BUSCANDO TEXTURA';
    $('reticleText').textContent=locks.length>=10?'Punto verde = referencia 3D reconocida':f.length?'Enfoca esquinas, bordes o textura visible':'Busca una zona con detalle';
    $('arToast').textContent=locks.length>=10?'Mapa reconocido · mantén el teléfono estable':'Apunta a una esquina, borde o textura y muévete lentamente';
    ar.features=f;ar.prev=g}arRender()}
async function startAR(){const s=activeSpace();if(!s){toast('Primero crea un espacio');show('createScreen');return}SCREENS.forEach(id=>$(id)?.classList.toggle('active',id==='arScreen'));try{await openCamera($('arCamera'))}catch(e){toast(e.message);return}ar.running=true;ar.pose={R:I3(),t:[0,0,0]};ar.objects=(s.objects||[]).map(o=>({...o}));ar.preview=null;ar.locks=[];ar.last=0;await startOrientationSensors();$('arSpaceName').textContent=s.name.toUpperCase();$('arToast').textContent=imu.active?'Mapa cargado · orientación del teléfono activa · busca referencias para fijar posición':'Mapa cargado · tracking visual · busca referencias para fijar posición';arLoop()}
function stopAR(){ar.running=false;cancelAnimationFrame(ar.raf);ar.preview=null;ar.locks=[];stopOrientationSensors()}
function placeAt(x,y){if(!ar.edit)return toast('Modo visualización: no se puede crear.');const s=activeSpace(),arr=s?.samples||[];let best=null,bd=999;for(const p of arr){if(p.n<3||p.confidence<.58)continue;const q=project([p.x,p.y,p.z],ar.pose);if(!q)continue;const d=Math.hypot(q.x-x*SW,q.y-y*SH);if(d<bd){bd=d;best=p}}if(!best||bd>30)return toast('Apunta a un punto verde consolidado.');ar.preview={anchor:{x:best.x,y:best.y,z:best.z},size:ar.size,spacing:ar.spacing,mode:ar.mode,foveated:ar.foveated};$('confirmPos').textContent=`Anclaje 3D · ${best.n} observaciones`;$('confirm').classList.remove('hidden')}
$('enterAR').onclick=startAR;$('placeAction').onclick=()=>placeAt(SW/2,SH/2);$('arCanvas').addEventListener('pointerup',e=>{if(ar.running&&!e.target.closest?.('.arUI'))placeAt(e.clientX/innerWidth,e.clientY/innerHeight)});$('place').onclick=()=>{if(!ar.preview)return;ar.objects.push({...ar.preview});ar.preview=null;$('confirm').classList.add('hidden');const s=activeSpace();s.objects=ar.objects;saveDB();toast('Bogon 3D guardado')};$('cancelPlace').onclick=()=>{$('confirm').classList.add('hidden');ar.preview=null};$('exitXR').onclick=()=>{stopAR();stopCamera();show('simScreen')};$('arExit').onclick=()=>$('exitXR').click();$('arSave').onclick=()=>{const s=activeSpace();if(s){s.objects=ar.objects;saveDB();toast('Estado guardado')}};$('arMenu').onclick=()=>$('arMenuPanel').classList.toggle('hidden');$('closeArMenu').onclick=()=>$('arMenuPanel').classList.add('hidden');$('arMapToggle').onclick=()=>{ar.showMap=!ar.showMap;$('geo').innerHTML=`MAPA <b>${ar.showMap?'ON':'OFF'}</b>`};$('geo').onclick=()=>$('arMapToggle').click();$('objects').onclick=()=>{ar.showObjects=!ar.showObjects;$('objects').innerHTML=`OBJETOS <b>${ar.showObjects?'ON':'OFF'}</b>`};$('clear').onclick=()=>{ar.objects=[];const s=activeSpace();if(s){s.objects=[];saveDB()}toast('Bogones eliminados')};$('size').oninput=e=>{ar.size=+e.target.value/100;$('sizeText').textContent=e.target.value+' cm'};$('detail').oninput=e=>{ar.spacing=+e.target.value/100;$('detailText').textContent=e.target.value+' cm'};['edge','surface','volume'].forEach(id=>$(id).onclick=()=>{ar.mode=id;document.querySelectorAll('#edge,#surface,#volume').forEach(b=>b.classList.remove('selected'));$(id).classList.add('selected')});$('foveated').onclick=()=>{ar.foveated=!ar.foveated;$('foveated').classList.toggle('selected',ar.foveated)};$('deleteNearest').onclick=()=>{if(ar.objects.length){ar.objects.pop();const s=activeSpace();if(s){s.objects=ar.objects;saveDB()}}};$('arLab').onclick=()=>show('infoScreen');$('closeLab').onclick=()=>show('arScreen');
$('interactive').onclick=()=>{ar.edit=true;$('interactive').classList.add('selected');$('visualMode').classList.remove('selected')};$('creationMode').onclick=()=>{ar.edit=true;$('creationMode').classList.add('selected');$('visualMode').classList.remove('selected')};$('visualMode').onclick=()=>{ar.edit=false;$('visualMode').classList.add('selected');$('creationMode').classList.remove('selected')};
$('reset').onclick=()=>{if(confirm('¿Borrar todos los espacios?')){localStorage.removeItem(KEY);db={spaces:[],active:null};renderSpaces();renderLocal();updateSupport()}};
function updateSupport(){const cam=!!navigator.mediaDevices?.getUserMedia;$('capBadge').textContent=cam?'CÁMARA':'CÁMARA NO DISPONIBLE';$('deviceReport').innerHTML=`CÁMARA <b>${cam?'✓':'—'}</b> · PROFUNDIDAD <b>NO</b> · ORIENTACIÓN <b>${('DeviceOrientationEvent' in window)?'DISPONIBLE':'VISUAL'}</b> · GPS <b>NO USADO</b> · TRACKER <b>MONOCULAR + ORIENTACIÓN</b>`;const s=activeSpace();$('simStatus').textContent=s?`Espacio activo: ${s.name} · ${s.points||0} puntos 3D · ${s.objects?.length||0} Bogones`:'Crea y guarda un espacio para entrar en AR'}
updateSupport();renderSpaces();
