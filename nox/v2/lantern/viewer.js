(function(){
const PATHS=__PATHS__;
const FOCUS=__FOCUS__;
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const stage=document.getElementById('stage'),canvas=document.getElementById('c'),note=document.getElementById('load-note');
const renderer=new THREE.WebGLRenderer({canvas,antialias:true,alpha:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.setClearColor(0,0);
const scene=new THREE.Scene();
const cam=new THREE.PerspectiveCamera(30,1,5,4000);
const root=new THREE.Group();root.rotation.x=-Math.PI/2;scene.add(root);   // drone (x nose, y left, z up) -> three (y up)
const groups={cage:new THREE.Group(),props:new THREE.Group(),body:new THREE.Group(),funnel:new THREE.Group()};
Object.values(groups).forEach(g=>root.add(g));
// part category by name prefix -> theme token, opacity, group
const CAT={rod:['--carbon',1,'cage'],node:['--node',1,'cage'],tpu:['--tpu',1,'cage'],strip:['--cat',1,'cage'],
  frame:['--fg',.6,'body'],prop:['--prop',.16,'props'],fc:['--fc',.95,'body'],radxa:['--radxa',.95,'body'],ubec:['--ubec',.95,'body'],
  cam:['--fc',.9,'body'],flow:['--radxa',.9,'body'],pack:['--pack',.5,'body'],recv:['--pack',.45,'body']};
const meshes=[];let nodesOnly=false;
function paint(){meshes.forEach(({m,cat})=>{const [tok,op]=CAT[cat];const dim=nodesOnly&&cat!=='node';
  m.material.color.set(css(tok));m.material.opacity=dim?Math.min(op,.12):op;m.material.transparent=m.material.opacity<1;m.material.needsUpdate=true;});
  funnelMat.color.set(css('--muted'));}
const funnelMat=new THREE.MeshLambertMaterial({color:0x888888,transparent:true,opacity:.16,side:THREE.DoubleSide});
// box funnel: four 45 deg collar faces from the 168 opening at z -0.8 to the 320 mouth at z 75.1
[[0,1],[0,-1],[1,0],[-1,0]].forEach(([sx,sy])=>{const g=new THREE.PlaneGeometry(320,107.5);const m=new THREE.Mesh(g,funnelMat);
  const c=new THREE.Vector3(sx*(84+38),sy*(84+38),37.15);m.position.copy(c);m.lookAt(c.clone().add(new THREE.Vector3(sx,sy,-1).normalize()));groups.funnel.add(m);});
scene.add(new THREE.HemisphereLight(0xffffff,0x444444,.9));
const dl=new THREE.DirectionalLight(0xffffff,.55);dl.position.set(300,500,200);scene.add(dl);
const dl2=new THREE.DirectionalLight(0xffffff,.25);dl2.position.set(-300,200,-400);scene.add(dl2);
new THREE.GLTFLoader().load('scout_v21.json',gltf=>{
  gltf.scene.traverse(o=>{if(!o.isMesh)return;const name=o.name||(o.parent&&o.parent.name)||'';const cat=Object.keys(CAT).find(k=>name.startsWith(k));
    if(!CAT[cat])return;const [tok,op,grp]=CAT[cat];if(!o.geometry.attributes.normal)o.geometry.computeVertexNormals();
    o.material=new THREE.MeshLambertMaterial({color:css(tok),transparent:op<1,opacity:op,side:THREE.DoubleSide});o.userData.cat=cat;
    meshes.push({m:o,cat});});
  // reparent by category (keep world transforms: the glb is in drone coords, root does the axis swap)
  gltf.scene.updateMatrixWorld(true);
  meshes.forEach(({m,cat})=>{const g=groups[CAT[cat][2]];m.matrix.copy(m.matrixWorld);m.matrix.decompose(m.position,m.quaternion,m.scale);g.add(m);});
  note.textContent='';paint();draw();
},undefined,err=>{note.textContent='model did not load';console.error(err);});
let az=.7,el=.5,dist=600,target=new THREE.Vector3(0,15,0);
function place(){cam.position.set(target.x+dist*Math.cos(el)*Math.sin(az),target.y+dist*Math.sin(el),target.z+dist*Math.cos(el)*Math.cos(az));cam.lookAt(target);}
function resize(){const w=stage.clientWidth,h=stage.clientHeight;renderer.setSize(w,h,false);cam.aspect=w/h;cam.updateProjectionMatrix();}
new ResizeObserver(()=>{resize();draw();}).observe(stage);
function draw(){place();renderer.render(scene,cam);}
let drag=null;
canvas.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY};canvas.setPointerCapture(e.pointerId);});
canvas.addEventListener('pointermove',e=>{if(!drag)return;az-=(e.clientX-drag.x)*.008;el=Math.max(-1.5,Math.min(1.5,el+(e.clientY-drag.y)*.008));drag={x:e.clientX,y:e.clientY};draw();});
canvas.addEventListener('pointerup',()=>drag=null);
canvas.addEventListener('wheel',e=>{e.preventDefault();dist=Math.max(40,Math.min(1400,dist*(1+e.deltaY*.0012)));draw();},{passive:false});
const views={iso:[.7,.5],top:[0,1.5],side:[Math.PI/2,0.02],nose:[0,0.02]};
function setView(k){[az,el]=views[k];dist=600;target.set(0,15,0);Object.keys(views).forEach(j=>document.getElementById('v-'+j).setAttribute('aria-pressed',j===k));
  document.querySelectorAll('[data-focus]').forEach(b=>b.setAttribute('aria-pressed','false'));draw();}
Object.keys(views).forEach(k=>document.getElementById('v-'+k).onclick=()=>setView(k));
// node close-ups: camera flies to the node point (drone coords -> three)
document.querySelectorAll('[data-focus]').forEach(b=>b.onclick=()=>{const f=FOCUS[b.dataset.focus];target.set(f[0],f[2],-f[1]);dist=f[3]||90;az=f[4]||.8;el=f[5]||.4;
  document.querySelectorAll('[data-focus]').forEach(x=>x.setAttribute('aria-pressed',x===b));Object.keys(views).forEach(j=>document.getElementById('v-'+j).setAttribute('aria-pressed','false'));draw();});
[['cage','cage'],['props','props'],['funnel','funnel'],['body','body']].forEach(([id,g])=>{const b=document.getElementById('t-'+id);if(!b)return;groups[g].visible=b.getAttribute('aria-pressed')==='true';
  b.onclick=()=>{groups[g].visible=!groups[g].visible;b.setAttribute('aria-pressed',groups[g].visible);draw();};});
const nb=document.getElementById('t-nodes');nb.onclick=()=>{nodesOnly=!nodesOnly;nb.setAttribute('aria-pressed',nodesOnly);paint();draw();};
matchMedia('(prefers-color-scheme: dark)').addEventListener('change',()=>{paint();draw();});
new MutationObserver(()=>{paint();draw();}).observe(document.documentElement,{attributes:true,attributeFilter:['data-theme']});
resize();draw();
(function(){const g=document.getElementById('pe');const col={nose:'var(--carbon)',side:'var(--fc)',diagonal:'var(--cat)'};
  Object.entries(PATHS).forEach(([k,v])=>{const pts=v.map((pe,i)=>[40+i*360/(v.length-1),150-(pe-40)*130/120]);
    const p=document.createElementNS('http://www.w3.org/2000/svg','path');p.setAttribute('d','M'+pts.map(q=>q[0].toFixed(1)+' '+q[1].toFixed(1)).join('L'));p.setAttribute('stroke',col[k]);g.appendChild(p);});})();
})();
