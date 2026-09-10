'use strict';
const $=id=>document.getElementById(id),K=window.K;
let q=[0,0,0,0,0,0],g={l1:.3,l2:.3,l3:.1,h:0,d:.12,e56:0,cx:.06,cy:0,cz:.04,cr:0,cp:0,cw:0,control:'tool'};
let limits=Array.from({length:6},()=>[-Math.PI,Math.PI]),mode='pose',solutions=[],target=null;
let camera={az:-1.1,el:.24,zoom:1},selected=-1;
const names=['Base','Hombro','Codo','Tercer eslabón','Muñeca','Herramienta'],axisNames=['+Z base','−Y local','−Y local','−Y local','+Z local','+X local'];
const fmt=(v,n=2)=>{let x=Math.abs(v)<.5*10**(-n)?0:v;return x.toFixed(n);};
function clearSolutions(){window.MotionUI?.invalidate();solutions=[];$('brancheswrap').hidden=true;}
function message(title,body,warn=false){$('result').className='result'+(warn?' warn':'');$('result').replaceChildren();const t=document.createElement('strong'),p=document.createElement('p');t.textContent=title;p.textContent=body;$('result').append(t,p);}
function field(container,id,label,value,step,min,max){
 const div=document.createElement('div');div.className='field';const lab=document.createElement('label');lab.htmlFor=id;lab.textContent=label;
 const inp=document.createElement('input');inp.type='number';inp.id=id;inp.value=value;inp.step=step;if(min!==undefined)inp.min=min;if(max!==undefined)inp.max=max;div.append(lab,inp);$(container).append(div);return inp;
}
for(let i=0;i<6;i++){
 const div=document.createElement('div');div.className='joint';
 div.innerHTML=`<div class="jointhead"><label for="q${i}"><strong>J${i+1}</strong>${names[i]}</label><input aria-label="Ángulo J${i+1} en grados" id="n${i}" type="number" min="-180" max="180" step="any"></div><input id="q${i}" aria-label="Deslizador J${i+1}: ${names[i]}" type="range" min="-180" max="180" step="0.1"><small>${axisNames[i]}</small>`;
 $('joints').append(div);
 for(const id of ['q','n'])$(id+i).addEventListener('input',e=>{
   if(e.target.value===''||!e.target.checkValidity())return;
   q[i]=Number(e.target.value)*K.rad;selected=i;clearSolutions();message('Directa actualizada','La pose se calcula a partir de los seis ángulos. Copia esta pose al objetivo para comprobar la inversa.');update();
 });
 const row=document.createElement('div');row.className='limitrow';row.innerHTML=`<span>J${i+1}</span><input id="lo${i}" type="number" aria-label="Límite mínimo J${i+1}, grados" value="-180" min="-360" max="360" step="1"><input id="hi${i}" type="number" aria-label="Límite máximo J${i+1}, grados" value="180" min="-360" max="360" step="1">`;$('limits').append(row);
 for(const side of ['lo','hi'])$(side+i).addEventListener('change',()=>{
   const lo=Number($('lo'+i).value),hi=Number($('hi'+i).value);
   if(!$('lo'+i).value||!$('hi'+i).value||!$('lo'+i).checkValidity()||!$('hi'+i).checkValidity()||lo>=hi){$('limitmsg').textContent='El mínimo debe ser menor que el máximo, entre −360° y 360°. No se aplicó el cambio.';return;}
   $('limitmsg').textContent='Límites aplicados.';limits[i]=[lo*K.rad,hi*K.rad];q[i]=K.clamp(q[i],...limits[i]);
   for(const p of ['q','n']){$(p+i).min=lo;$(p+i).max=hi;}clearSolutions();message('Límites actualizados','La inversa respetará los nuevos intervalos articulares.');update();
 });
}
for(const [key,label] of [['l1','L₁ · primer eslabón'],['l2','L₂ · segundo'],['l3','L₃ · tercero'],['h','h · altura base'],['d','d · barra / punta'],['e56','e₅₆ · axial J5–J6']]){
 const inp=field('geometry',key,label,g[key]*100,.1,key==='l1'||key==='l2'?.1:0,200);
 inp.addEventListener('input',()=>{if(!inp.value||!inp.checkValidity())return;g[key]=Number(inp.value)/100;clearSolutions();message('Geometría actualizada','La posición actual y los errores usan las nuevas longitudes.');update();});
}
for(const [key,label,isAngle] of [['cx','cₓ · cm',false],['cy','cᵧ · cm',false],['cz','c_z · cm',false],['cr','Roll montaje · °',true],['cp','Pitch montaje · °',true],['cw','Yaw montaje · °',true]]){
 const inp=field('cameramount',key,label,g[key]*(isAngle?K.deg:100),'any',isAngle?-360:-200,isAngle?360:200);
 inp.addEventListener('input',()=>{if(inp.value===''||!inp.checkValidity())return;g[key]=Number(inp.value)*(isAngle?K.rad:.01);clearSolutions();message('Montaje de cámara actualizado','Su posición y orientación son constantes respecto a J6, no respecto a la base. Los valores no constituyen una calibración medida.');update();});
}
$('controlpoint').onchange=()=>{g.control=$('controlpoint').value;capture();message('Punto controlado actualizado',g.control==='camera'?'La inversa y la recta controlan ahora el centro y el marco de la cámara.':'La inversa y la recta controlan ahora la punta de la barra.');};
['X','Y','Z'].forEach((v,i)=>field('targetpos','t'+i,v,0,'any',-1000,1000));
['Roll','Pitch','Yaw'].forEach((v,i)=>field('targetrot','r'+i,v,0,'any',-360,360));
function readTarget(){
 const els=[0,1,2].map(i=>$('t'+i));if(mode==='pose')els.push(...[0,1,2].map(i=>$('r'+i)));
 if(els.some(e=>e.value===''||!e.checkValidity()))return false;
 target={p:[0,1,2].map(i=>Number($('t'+i).value)/100),R:mode==='pose'?K.rpyMatrix(...[0,1,2].map(i=>Number($('r'+i).value)*K.rad)):K.fk(q,g).R};return true;
}
for(const prefix of ['t','r'])for(let i=0;i<3;i++)$(prefix+i).addEventListener('input',()=>{clearSolutions();if(readTarget()){message('Objetivo actualizado','Pulsa “Resolver inversa” para buscar una configuración.');update();}});
function capture(){const f=K.fk(q,g),rpy=K.matrixRPY(f.R);f.p.forEach((v,i)=>$('t'+i).value=fmt(v*100,7));rpy.forEach((v,i)=>$('r'+i).value=fmt(v*K.deg,7));readTarget();clearSolutions();update();}
$('capture').onclick=()=>{capture();message('Pose copiada','El objetivo coincide con la pose actual. Cambia los ángulos o resuelve la inversa para reconstruirla.');};
function changeMode(next){mode=next;clearSolutions();$('orientation').hidden=mode==='position';$('posemode').classList.toggle('active',mode==='pose');$('posmode').classList.toggle('active',mode==='position');$('posemode').setAttribute('aria-pressed',mode==='pose');$('posmode').setAttribute('aria-pressed',mode==='position');$('modehint').textContent=mode==='pose'?'Especifica posición y orientación. Se calculan las ramas analíticas compatibles con tus límites.':'Especifica X, Y y Z. Un método numérico busca una solución desde la postura actual; la orientación queda libre.';message('Modo actualizado',mode==='pose'?'Se controlan posición y orientación.':'La orientación de la herramienta no forma parte del objetivo.');update();}
$('posemode').onclick=()=>changeMode('pose');$('posmode').onclick=()=>changeMode('position');
function applySolution(i){if(!solutions[i])return;window.MotionUI?.invalidate();q=solutions[i].q.slice();$('solutionangles').textContent=q.map((a,i)=>`J${i+1} ${fmt(a*K.deg,2)}°`).join('   ');update();}
$('branches').onchange=e=>applySolution(Number(e.target.value));
function solve(roundtrip=false){
 if(!readTarget()){message('Revisa el objetivo','Introduce valores numéricos válidos en todos los campos del objetivo.',true);return;}
 clearSolutions();const seed=roundtrip?limits.map(([lo,hi])=>K.clamp(0,lo,hi)):q.slice();
 if(mode==='pose'){
  const result=K.ikPose(target,g,limits,seed);solutions=result.solutions;
  if(solutions.length){
   $('branches').replaceChildren();solutions.forEach((s,i)=>{const opt=document.createElement('option');opt.value=i;opt.textContent=`${i+1}. ${s.label}`;$('branches').append(opt);});$('brancheswrap').hidden=false;applySolution(0);
   const suffix=result.sampled?' Configuración singular: se muestran representantes obtenidos por muestreo, no toda la familia continua.':' Se muestra primero la más cercana a la postura de partida.';
   message(roundtrip?'Ciclo verificado':`${solutions.length} solución${solutions.length===1?'':'es'} encontrada${solutions.length===1?'':'s'}`,`${roundtrip?'La directa reconstruye la pose de partida. ':''}${solutions.length} configuración(es) cumplen la pose y los límites.${suffix}`);
  }else{
   message('No se encontró una solución',result.sampled?'El objetivo es singular. El muestreo de la familia no encontró una postura dentro de los límites; esto no demuestra que sea imposible.':result.unlimited?'Hay soluciones geométricas, pero ninguna respeta los límites establecidos.':'La posición y orientación combinadas no son alcanzables con esta geometría. Prueba “Solo posición” o cambia la orientación.',true);update();
  }
 }else{
  const result=K.ikPosition(target.p,g,limits,seed);
  if(result.converged){q=result.q;message('Posición alcanzada','Solución numérica con orientación libre. Error de posición menor que 0,01 mm.');}
  else message('La búsqueda no convergió',`Mejor error encontrado: ${fmt(result.position*1000,2)} mm. No se aplicó esa aproximación. Puede ser un objetivo inalcanzable, un límite articular o un mínimo local.`,true);
  update();
 }
}
$('solve').onclick=()=>solve();$('roundtrip').onclick=()=>{changeMode('pose');capture();solve(true);};
$('home').onclick=()=>{q=limits.map(([lo,hi])=>K.clamp(0,lo,hi));capture();message('Inicial vertical · cero articular',q.every(v=>Math.abs(v)<1e-10)?'J5 gira sobre Z global y J6 sobre X global en esta postura. El brazo completamente extendido es singular.':'Algún límite impide llegar a cero: se aplicó el valor permitido más cercano.',true);};
$('zero').onclick=()=>{q=[20,-50,40,10,25,15].map((v,i)=>K.clamp(v*K.rad,...limits[i]));capture();message('Postura de prueba','Postura flexionada para explorar desplazamientos locales y trayectorias. Los ángulos se ajustan a tus límites.');};
function update(){
 const f=K.fk(q,g),rpy=K.matrixRPY(f.R);
 q.forEach((v,i)=>{if(document.activeElement!==$('n'+i))$('n'+i).value=fmt(v*K.deg,3);$('q'+i).value=v*K.deg;});
 $('readouts').innerHTML=[...f.p.map((v,i)=>[('XYZ')[i],v*100,'cm']),...rpy.map((v,i)=>[['ROLL','PITCH','YAW'][i],v*K.deg,'°'])].map(([l,v,u])=>`<div class="readout"><span>${l}</span><strong>${fmt(v)}<em>${u}</em></strong></div>`).join('');
 $('matrix').innerHTML=f.T.flat().map(v=>`<span>${fmt(v,5)}</span>`).join('');
 const phi=q[1]+q[2]+q[3],r=-g.l1*Math.sin(q[1])-g.l2*Math.sin(q[1]+q[2])-g.l3*Math.sin(phi);
 const singular=Math.abs(Math.sin(q[2]))<.015||Math.abs(Math.cos(q[4]))<.015||Math.abs(r)<.005;
 $('singularity').textContent=singular?'CERCA DE SINGULARIDAD':'6 ARTICULACIONES';$('singularity').style.color=singular?'#ffb36a':'';
 if(target){const e=K.errors(f,target);$('poserror').textContent=fmt(e.position*1000,3)+' mm';$('roterror').textContent=mode==='pose'?fmt(e.orientation*K.deg,4)+'°':'Libre';}
 $('posetitle').textContent=g.control==='camera'?'Pose actual · centro de cámara':'Pose actual · punta de barra';
 $('activepointtag').textContent=g.control==='camera'?'MUEVE EL CENTRO DE CÁMARA':'MUEVE LA PUNTA DE BARRA';
 $('framesummary').innerHTML=[['Punta de barra',f.tool],['Centro de cámara',f.camera]].map(([name,pose])=>`<div><strong>${name}</strong><span>XYZ: ${pose.p.map(v=>fmt(v*100,2)).join(' · ')} cm</span><span>RPY: ${K.matrixRPY(pose.R).map(v=>fmt(v*K.deg,2)).join(' · ')}°</span></div>`).join('');
 $('cameramatrix').innerHTML=K.homogeneous(K.rpyMatrix(g.cr,g.cp,g.cw),[g.cx,g.cy,g.cz]).flat().map(v=>`<span>${fmt(v,5)}</span>`).join('');
 draw();window.MotionUI?.update();
}
const canvas=$('scene'),ctx=canvas.getContext('2d');let W=0,H=0;
function resize(){const r=canvas.getBoundingClientRect();W=r.width;H=r.height;const dpr=Math.min(devicePixelRatio||1,2);canvas.width=Math.round(W*dpr);canvas.height=Math.round(H*dpr);ctx.setTransform(dpr,0,0,dpr,0,0);draw();}
function project(p){
 const reach=g.l1+g.l2+g.l3+g.e56+Math.max(g.d,K.norm([g.cx,g.cy,g.cz]))+g.h+.05,s=Math.min(W,H)*.68/Math.max(reach,.1)*camera.zoom,ca=Math.cos(camera.az),sa=Math.sin(camera.az),ce=Math.cos(camera.el),se=Math.sin(camera.el);
 const z=p[2]-g.h*.4-.2*(g.l1+g.l2+g.l3),right=-sa*p[0]+ca*p[1],up=-se*(ca*p[0]+sa*p[1])+ce*z;
 return [W*.48+s*right,H*.57-s*up,ce*(ca*p[0]+sa*p[1])+se*z];
}
function line(a,b,color,width=1,dash=[]){const A=project(a),B=project(b);ctx.beginPath();ctx.moveTo(A[0],A[1]);ctx.lineTo(B[0],B[1]);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.lineCap='round';ctx.stroke();ctx.setLineDash([]);}
function label(p,text,color='#cad7e9',dx=9,dy=-10){const a=project(p);ctx.font='12px ui-monospace,monospace';ctx.fillStyle='#0e1725';ctx.fillRect(a[0]+dx-3,a[1]+dy-12,ctx.measureText(text).width+6,17);ctx.fillStyle=color;ctx.fillText(text,a[0]+dx,a[1]+dy);}
function arrow(p,v,color,name,len){const end=K.add(p,K.scale(v,len));line(p,end,color,2);const a=project(p),b=project(end),theta=Math.atan2(b[1]-a[1],b[0]-a[0]);ctx.beginPath();ctx.moveTo(b[0],b[1]);ctx.lineTo(b[0]-7*Math.cos(theta-.4),b[1]-7*Math.sin(theta-.4));ctx.moveTo(b[0],b[1]);ctx.lineTo(b[0]-7*Math.cos(theta+.4),b[1]-7*Math.sin(theta+.4));ctx.strokeStyle=color;ctx.lineWidth=2;ctx.stroke();if(name)label(end,name,color,5,-4);}
function triad(p,R,len,ghost=false){K.tr(R).forEach((v,i)=>arrow(p,v,ghost?'#ffb36a':['#ff777f','#69d591','#6baaff'][i],ghost?'':['xₜ','yₜ','zₜ'][i],len));}
function cuboid(center,R,size,colors){
 const pts=Array.from({length:8},(_,i)=>K.add(center,K.mv(R,size.map((v,j)=>(i&(1<<j)?1:-1)*v/2))));
 const faces=[[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]];
 faces.map((ids,i)=>({ids,i,depth:ids.reduce((s,j)=>s+project(pts[j])[2],0)/4})).sort((a,b)=>a.depth-b.depth).forEach(({ids,i})=>{
  ctx.beginPath();ids.forEach((j,k)=>{const p=project(pts[j]);if(k)ctx.lineTo(p[0],p[1]);else ctx.moveTo(p[0],p[1]);});ctx.closePath();ctx.fillStyle=colors[i%colors.length];ctx.fill();ctx.lineWidth=1;ctx.strokeStyle='#576779';ctx.stroke();
 });
}
function jointRing(p,axis,r,color){const helper=Math.abs(axis[2])<.9?[0,0,1]:[0,1,0];let u=K.cross(axis,helper);u=K.scale(u,1/K.norm(u));const v=K.cross(axis,u);let last=null;for(let i=0;i<=32;i++){const a=i/32*Math.PI*2,point=K.add(p,K.add(K.scale(u,r*Math.cos(a)),K.scale(v,r*Math.sin(a))));if(last)line(last,point,color,2);last=point;}}
function draw(){
 if(!W||!H)return;ctx.clearRect(0,0,W,H);const f=K.fk(q,g),reach=g.l1+g.l2+g.l3+g.d,grid=Math.max(.2,Math.ceil(reach*10)/10),step=grid>1?.2:.1;
 for(let x=-grid;x<=grid+.0001;x+=step){line([x,-grid,0],[x,grid,0],'#233149',1);line([-grid,x,0],[grid,x,0],'#233149',1);}
 const unit=Math.max(.1,Math.min(.25,reach*.35));arrow([0,0,0],[1,0,0],'#e6757d','X',unit);arrow([0,0,0],[0,1,0],'#68bd85','Y',unit);arrow([0,0,0],[0,0,1],'#71a2eb','Z',unit);
 window.MotionUI?.drawPath();
 if(target){line([target.p[0],target.p[1],0],target.p,'#a77144',1,[4,5]);if(mode==='pose')triad(target.p,target.R,unit*.65,true);const a=project(target.p);ctx.strokeStyle='#ffb36a';ctx.lineWidth=2;ctx.beginPath();ctx.arc(a[0],a[1],7,0,Math.PI*2);ctx.moveTo(a[0]-12,a[1]);ctx.lineTo(a[0]+12,a[1]);ctx.moveTo(a[0],a[1]-12);ctx.lineTo(a[0],a[1]+12);ctx.stroke();label(target.p,'objetivo B','#ffb36a',12,22);}
 line([f.p[0],f.p[1],0],f.p,'#41696c',1,[3,6]);
 const segments=f.points.slice(1,5).map((p,i)=>({a:f.points[i],b:p,i,depth:(project(p)[2]+project(f.points[i])[2])/2})).sort((a,b)=>a.depth-b.depth);
 for(const s of segments){if(K.norm(K.sub(s.a,s.b))<1e-7)continue;line(s.a,s.b,'#09111b',17);line(s.a,s.b,s.i===4?'#cbd9ec':s.i===0?'#586981':'#359f9c',12);line(s.a,s.b,s.i===4?'#e3edf9':s.i===0?'#8b9aad':'#75ece0',3);}
 if(g.e56>1e-8)line(f.wrist,f.flange.p,'#b7c7d5',7);
 if(g.d>1e-8)cuboid(K.add(f.flange.p,K.mv(f.flange.R,[g.d/2,0,0])),f.flange.R,[g.d,.035,.008],['#e8bb25','#fff27a','#f1d044']);
 const mountAnchor=K.add(f.flange.p,K.mv(f.flange.R,[g.cx,0,0]));line(mountAnchor,f.camera.p,'#abb6c8',2,[3,3]);
 cuboid(f.camera.p,f.camera.R,[.04,.025,.035],['#c4cedd','#ffffff','#e6edf5']);
 const ringSize=Math.max(.012,reach*.028);
 f.origins.map((p,i)=>({p,i,depth:project(p)[2]})).sort((a,b)=>a.depth-b.depth).forEach(({p,i})=>{
  jointRing(p,f.axes[i],ringSize*(i===4?1.4:1),selected===i?'#ffcc8e':'#d9eeee');
  const axislen=ringSize*2.4;line(K.sub(p,K.scale(f.axes[i],axislen)),K.add(p,K.scale(f.axes[i],axislen)),i===0||i===4?'#75aaff':i===5?'#ff818b':'#7cd49c',1.5,[3,3]);
  if(i!==5||g.e56>1e-8)label(p,i===4&&g.e56<1e-8?'J5 · J6':g.h===0&&i===0?'J1 · J2':i===1&&g.h===0?'':`J${i+1}`,'#d5e8ed',i===3?-36:10,i===1?24:-13);
 });
 triad(f.tool.p,f.tool.R,unit*.4);label(f.tool.p,g.control==='tool'?'punta · activa':'punta','#f4e382',12,24);
 K.tr(f.camera.R).forEach((v,i)=>arrow(f.camera.p,v,['#ff777f','#69d591','#6baaff'][i],['x꜀','y꜀','z꜀'][i],unit*.25));label(f.camera.p,g.control==='camera'?'cámara · activa':'cámara','#ffffff',10,-27);
 ctx.fillStyle='#8a9bb6';ctx.font='12px ui-monospace,monospace';ctx.fillText(`Cuadrícula: ${fmt(step*100,0)} cm`,16,H-18);
 if(Math.min(...f.points.map(p=>p[2]))<-1e-6){ctx.fillStyle='#ffb36a';ctx.fillText('Parte del brazo está bajo z = 0',16,25);}
}
let drag=null;canvas.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY};canvas.setPointerCapture(e.pointerId);canvas.style.cursor='grabbing';});
canvas.addEventListener('pointermove',e=>{if(!drag)return;camera.az-=(e.clientX-drag.x)*.009;camera.el=K.clamp(camera.el+(e.clientY-drag.y)*.008,-1.5,1.56);drag={x:e.clientX,y:e.clientY};draw();});
for(const ev of ['pointerup','pointercancel'])canvas.addEventListener(ev,()=>{drag=null;canvas.style.cursor='grab';});
canvas.addEventListener('wheel',e=>{e.preventDefault();camera.zoom=K.clamp(camera.zoom*Math.exp(-e.deltaY*.001),.4,3.5);draw();},{passive:false});
canvas.addEventListener('keydown',e=>{if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','+','-'].includes(e.key))return;e.preventDefault();if(e.key==='ArrowLeft')camera.az-=.1;if(e.key==='ArrowRight')camera.az+=.1;if(e.key==='ArrowUp')camera.el=K.clamp(camera.el+.1,-1.5,1.56);if(e.key==='ArrowDown')camera.el=K.clamp(camera.el-.1,-1.5,1.56);if(e.key==='+')camera.zoom=K.clamp(camera.zoom*1.1,.4,3.5);if(e.key==='-')camera.zoom=K.clamp(camera.zoom/1.1,.4,3.5);draw();});
document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>{const v=b.dataset.view;camera={az:v==='front'?-Math.PI/2:-1,el:v==='front'?0:v==='top'?Math.PI/2:.48,zoom:1};document.querySelectorAll('[data-view]').forEach(btn=>btn.classList.toggle('active',btn===b));draw();});
capture();message('Modelo corregido · inicial vertical','Los seis ángulos cero corresponden a la postura de referencia. La barra amarilla y la cámara blanca giran juntas con J6. Las medidas del montaje son ejemplos editables.');new ResizeObserver(resize).observe(canvas);
