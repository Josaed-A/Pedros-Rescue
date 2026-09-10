'use strict';
(function(){
 const M=window.Motion,colors=['#52dfd0','#f3b768','#81adff','#e795d3','#b7e57d','#e88085'];
 let plan=null,elapsed=0,playing=false,lastFrame=0,raf=null,worker=null,generation=0,plotCache=null;
 const plot=$('jointplot'),plotctx=plot.getContext('2d');
 function status(text,warn=false){$('pathstatus').textContent=text;$('pathstatus').classList.toggle('warn',warn);}
 function pause(){playing=false;if(raf!==null)cancelAnimationFrame(raf);raf=null;$('playpath').textContent=elapsed>0&&plan&&elapsed<plan.duration?'Continuar':'Reproducir';}
 function invalidate(){
  pause();generation++;if(worker){worker.terminate();worker=null;}plan=null;elapsed=0;plotCache=null;
  $('pathplayback').hidden=true;$('generatepath').disabled=false;$('generatepath').textContent='Generar A → B';
  status('Genera un recorrido desde la postura actual hasta el objetivo B.');
 }
 function setTarget(next){
  changeMode('pose');
  next.p.forEach((v,i)=>$('t'+i).value=fmt(v*100,9));K.matrixRPY(next.R).forEach((v,i)=>$('r'+i).value=fmt(v*K.deg,9));
  readTarget();clearSolutions();update();
 }
 function valid(id){const e=$(id);return e.value!==''&&e.checkValidity()&&Number.isFinite(Number(e.value));}
 function relative(animate,delta=null){
  if(!delta){if(['dx','dy','dz'].some(id=>!valid(id))){status('Introduce desplazamientos válidos entre −200 y 200 cm.',true);return;}delta=['dx','dy','dz'].map(id=>Number($(id).value)/100);}
  pause();const start=K.fk(q,g),frame=$('moveframe').value,next=M.relativeTarget(start,delta,frame);
  setTarget(next);$('pathtype').value='cartesian';kindHint();
  message('Objetivo relativo calculado',`Se mueve ${g.control==='camera'?'el centro de cámara':'la punta de barra'}, manteniendo su orientación. Desplazamiento en ${frame==='tool'?'la herramienta':frame==='camera'?'la cámara':'la base'}: ${delta.map(v=>fmt(v*100,2)).join(', ')} cm.`);
  if(animate)generate(true);else status('Objetivo relativo fijado. Pulsa “Generar A → B” para comprobar y dibujar la recta.');
 }
 $('relativegoal').onclick=()=>relative(false);$('relativemove').onclick=()=>relative(true);
 $('moveframe').onchange=()=>{
  const frame=$('moveframe').value,suffix=frame==='tool'?'ₜ':frame==='camera'?'꜀':'';['dx','dy','dz'].forEach((id,i)=>document.querySelector(`label[for="${id}"]`).textContent=`Δ${'XYZ'[i]}${suffix} · cm`);
  document.querySelectorAll('[data-jog]').forEach(b=>{const [axis,sign]=b.dataset.jog.split(',').map(Number);b.textContent=`${sign<0?'−':'+'}${'XYZ'[axis]}${suffix}`;});
  $('relativehint').textContent=`Se mueve el punto controlado en los ejes ${frame==='tool'?'de la herramienta':frame==='camera'?'de la cámara (montaje nominal)':'fijos de la base'}. La orientación se mantiene. Cambiar los ejes no cambia qué punto se controla.`;
 };
 document.querySelectorAll('[data-jog]').forEach(b=>b.onclick=()=>{if(!valid('jogstep')){status('Introduce un paso de 0,01 a 30 cm.',true);return;}const [axis,sign]=b.dataset.jog.split(',').map(Number),delta=[0,0,0];delta[axis]=sign*Number($('jogstep').value)/100;relative(true,delta);});
 function kindHint(){const cart=$('pathtype').value==='cartesian';$('pathkindhint').textContent=cart?'El punto controlado sigue una recta. En pose completa, la orientación cambia por la rotación más corta entre A y B.':'Los seis ángulos se interpolan con un perfil quíntico. El punto controlado puede describir una curva entre A y B.';}
 $('pathtype').onchange=()=>{invalidate();kindHint();draw();};$('duration').onchange=()=>{invalidate();draw();};
 function generate(autoplay=false){
  if(!readTarget()){status('Revisa los campos del objetivo B antes de generar la trayectoria.',true);return;}
  if(!valid('duration')){status('La duración debe estar entre 0,2 y 60 segundos.',true);return;}
  clearSolutions();const id=generation,payload={startQ:q.slice(),target:{p:target.p.slice(),R:target.R.map(r=>r.slice())},g:{...g},limits:limits.map(r=>r.slice()),duration:Number($('duration').value),type:$('pathtype').value,orientation:mode==='pose'};
  status('Calculando el recorrido y comprobando la continuidad de los ángulos…');$('generatepath').disabled=true;$('generatepath').textContent='Calculando…';draw();
  const receive=result=>{
   if(id!==generation)return;if(worker){worker.terminate();worker=null;}$('generatepath').disabled=false;$('generatepath').textContent='Generar A → B';
   if(!result.ok){const where=result.fraction?` Fallo cerca del ${fmt(result.fraction*100,1)} % del recorrido.`:'';status(result.message+where+' La postura del brazo se conserva.',true);return;}
   plan=result;elapsed=0;plotCache=null;$('pathplayback').hidden=false;$('pathtime').max=plan.duration;$('pathtime').value=0;
   const description=plan.type==='joint'?'Interpolación articular lista. La curva violeta muestra el recorrido del punto controlado.':'Recta del punto controlado verificada en las muestras y en tres puntos interiores de cada intervalo.';
   status(description+(plan.singular?' Se atravesaron configuraciones singulares muestreadas.':''));
   $('pathmetrics').innerHTML=[['Duración',fmt(plan.duration,2)+' s'],['Recorrido activo',fmt(plan.pathLength*100,2)+' cm'],['Vel. articular máx.',fmt(plan.maxJointSpeed*K.deg,1)+' °/s'],['Muestras',String(plan.samples.length)],['Error cartesiano',plan.type==='cartesian'?fmt(plan.maxPositionError*1000,4)+' mm':'No exige recta'],['Error de orientación',plan.type==='cartesian'&&plan.orientation?fmt(plan.maxOrientationError*K.deg,4)+'°':'Libre entre puntos']].map(([a,b])=>`<div><span>${a}</span><strong>${b}</strong></div>`).join('');
   buildPlot();update();if(autoplay)play();
  };
  if(location.protocol==='file:'||typeof Worker==='undefined')setTimeout(()=>{if(id!==generation)return;try{receive(M.planMotion(payload));}catch(e){receive({ok:false,message:e.message});}},0);
  else{
   worker=new Worker('motion-worker.js');worker.onmessage=e=>receive(e.data);worker.onerror=()=>receive({ok:false,message:'No se pudo iniciar el cálculo en segundo plano. Recarga la página e inténtalo de nuevo.'});worker.postMessage(payload);
  }
 }
 $('generatepath').onclick=()=>generate(false);
 function seek(t){if(!plan)return;elapsed=K.clamp(t,0,plan.duration);q=M.sampleQ(plan,elapsed);update();}
 function tick(now){
  if(!playing||!plan)return;const dt=(now-lastFrame)/1000;lastFrame=now;seek(elapsed+dt);
  if(elapsed>=plan.duration){pause();status('Recorrido completado. Puedes usar esta postura como A para el siguiente movimiento.');return;}
  raf=requestAnimationFrame(tick);
 }
 function play(){if(!plan)return;if(playing)return;if(elapsed>=plan.duration)seek(0);playing=true;lastFrame=performance.now();$('playpath').textContent='Reproduciendo';raf=requestAnimationFrame(tick);}
 $('playpath').onclick=play;$('pausepath').onclick=()=>{pause();if(plan)status('Reproducción en pausa. Puedes continuar o inspeccionar cualquier instante.');};
 $('resetpath').onclick=()=>{pause();seek(0);};$('pathtime').oninput=e=>{pause();seek(Number(e.target.value));};
 document.addEventListener('visibilitychange',()=>{if(document.hidden&&playing){pause();status('Reproducción pausada al salir de esta pestaña. Pulsa “Continuar” para reanudar.');}});
 $('exportpath').onclick=()=>{if(!plan)return;const blob=new Blob([M.csv(plan)],{type:'text/csv;charset=utf-8'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=`trayectoria_${plan.geometry.control==='camera'?'camara':'herramienta'}_${plan.type==='joint'?'articular':'cartesiana'}.csv`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
 function drawPath(){
  if(!plan)return;const samples=plan.samples,other=plan.geometry.control==='camera'?'tool':'camera';
  for(let i=1;i<samples.length;i++){line(samples[i-1][other].p,samples[i][other].p,other==='camera'?'#aab8cb':'#b4a94b',1,[3,4]);line(samples[i-1].p,samples[i].p,samples[i].t<=elapsed?'#69e6d6':'#ad9bff',2.5);}
  label(samples[0].p,'A','#c4b6ff',-19,22);
 }
 function buildPlot(){
  if(!plan)return;const rect=plot.getBoundingClientRect();if(!rect.width||!rect.height)return;
  const w=rect.width,h=rect.height,dpr=Math.min(devicePixelRatio||1,2);plot.width=Math.round(w*dpr);plot.height=Math.round(h*dpr);plotctx.setTransform(dpr,0,0,dpr,0,0);
  const c=document.createElement('canvas');c.width=plot.width;c.height=plot.height;const cx=c.getContext('2d');cx.setTransform(dpr,0,0,dpr,0,0);
  let lo=Infinity,hi=-Infinity;for(const s of plan.samples)for(const v of s.q){lo=Math.min(lo,v*K.deg);hi=Math.max(hi,v*K.deg);}lo=Math.floor((lo-5)/10)*10;hi=Math.ceil((hi+5)/10)*10;
  const left=46,right=w-12,top=16,bottom=h-27,px=t=>left+t/plan.duration*(right-left),py=a=>bottom-(a-lo)/(hi-lo)*(bottom-top);
  cx.font='11px ui-monospace,monospace';cx.lineWidth=1;
  for(let i=0;i<=4;i++){const v=lo+(hi-lo)*i/4,y=py(v);cx.strokeStyle='#30405a';cx.beginPath();cx.moveTo(left,y);cx.lineTo(right,y);cx.stroke();cx.fillStyle='#b0bfd4';cx.fillText(fmt(v,0)+'°',1,y+4);}
  for(let i=0;i<=4;i++){const t=plan.duration*i/4,x=px(t);cx.fillStyle='#a4b7d1';cx.fillText(fmt(t,1),x-8,h-6);}
  for(let j=0;j<6;j++){cx.beginPath();plan.samples.forEach((s,i)=>{const x=px(s.t),y=py(s.q[j]*K.deg);if(i)cx.lineTo(x,y);else cx.moveTo(x,y);});cx.strokeStyle=colors[j];cx.lineWidth=1.7;cx.stroke();}
  plotCache={canvas:c,w,h,left,right,top,bottom};renderPlot();
 }
 function renderPlot(){if(!plan||!plotCache)return;const c=plotCache;plotctx.clearRect(0,0,c.w,c.h);plotctx.drawImage(c.canvas,0,0,c.w,c.h);const x=c.left+elapsed/plan.duration*(c.right-c.left);plotctx.strokeStyle='#fff';plotctx.lineWidth=1;plotctx.setLineDash([3,4]);plotctx.beginPath();plotctx.moveTo(x,c.top);plotctx.lineTo(x,c.bottom);plotctx.stroke();plotctx.setLineDash([]);}
 function updateMotion(){if(!plan)return;$('pathtime').value=elapsed;$('timevalue').textContent=`${fmt(elapsed,2)} / ${fmt(plan.duration,2)} s`;renderPlot();}
 $('plotlegend').innerHTML=colors.map((c,i)=>`<span style="color:${c}">━ J${i+1}</span>`).join('');
 new ResizeObserver(buildPlot).observe(plot);
 window.MotionUI={invalidate,drawPath,update:updateMotion};kindHint();
})();
