/* Motion planning for K: metres, radians, seconds. No hardware commands. */
(function(root){
'use strict';
const K=typeof module!=='undefined'&&module.exports?require('./kinematics.js'):root.K;
const {add,sub,scale,norm,mm,tr,mv,clamp,fk,errors}=K;
const mix=(a,b,s)=>a.map((v,i)=>v+(b[i]-v)*s);
const smooth=u=>{u=clamp(u,0,1);return u*u*u*(10+u*(-15+6*u));};
function quat(R){
 let w,x,y,z;const trace=R[0][0]+R[1][1]+R[2][2];
 if(trace>0){let s=2*Math.sqrt(trace+1);w=s/4;x=(R[2][1]-R[1][2])/s;y=(R[0][2]-R[2][0])/s;z=(R[1][0]-R[0][1])/s;}
 else if(R[0][0]>R[1][1]&&R[0][0]>R[2][2]){let s=2*Math.sqrt(1+R[0][0]-R[1][1]-R[2][2]);w=(R[2][1]-R[1][2])/s;x=s/4;y=(R[0][1]+R[1][0])/s;z=(R[0][2]+R[2][0])/s;}
 else if(R[1][1]>R[2][2]){let s=2*Math.sqrt(1+R[1][1]-R[0][0]-R[2][2]);w=(R[0][2]-R[2][0])/s;x=(R[0][1]+R[1][0])/s;y=s/4;z=(R[1][2]+R[2][1])/s;}
 else{let s=2*Math.sqrt(1+R[2][2]-R[0][0]-R[1][1]);w=(R[1][0]-R[0][1])/s;x=(R[0][2]+R[2][0])/s;y=(R[1][2]+R[2][1])/s;z=s/4;}
 const a=[w,x,y,z];return scale(a,1/norm(a));
}
function rotation(q){let [w,x,y,z]=scale(q,1/norm(q));return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]];}
function slerp(R0,R1,s){
 let a=quat(R0),b=quat(R1),c=K.dot(a,b);if(c<0){b=scale(b,-1);c=-c;}
 if(c>.9995)return rotation(mix(a,b,s));
 const angle=Math.acos(clamp(c,-1,1));return rotation(add(scale(a,Math.sin((1-s)*angle)/Math.sin(angle)),scale(b,Math.sin(s*angle)/Math.sin(angle))));
}
function relativeTarget(f,delta,frame='tool'){
 const basis=frame==='camera'?(f.camera?.R||f.R):(f.tool?.R||f.R);
 return {p:add(f.p,frame==='base'?delta:mv(basis,delta)),R:f.R.map(r=>r.slice())};
}
function poseAt(start,end,t,duration){const s=smooth(t/duration);return {p:mix(start.p,end.p,s),R:slerp(start.R,end.R,s)};}
function solve(target,g,limits,seed,orientation){
 if(orientation){const r=K.ikPose(target,g,limits,seed,{nearOnly:true});return r.solutions.length?{q:r.solutions[0].q,singular:r.sampled}:null;}
 const r=K.ikPosition(target.p,g,limits,seed);return r.converged?{q:r.q,singular:false}:null;
}
function sampleQ(plan,t){
 t=clamp(t,0,plan.duration);
 if(plan.type==='joint')return mix(plan.startQ,plan.endQ,smooth(t/plan.duration));
 let lo=0,hi=plan.samples.length-1;
 while(hi-lo>1){const m=(lo+hi)>>1;if(plan.samples[m].t<=t)lo=m;else hi=m;}
 const a=plan.samples[lo],b=plan.samples[hi];return mix(a.q,b.q,(t-a.t)/(b.t-a.t));
}
function planMotion({startQ,target,g,limits,duration=3,type='cartesian',orientation=true}){
 if(!Number.isFinite(duration)||duration<.2||duration>60)return {ok:false,message:'La duración debe estar entre 0,2 y 60 segundos.'};
 if(!['joint','cartesian'].includes(type)||startQ.length!==6||startQ.some((v,i)=>!Number.isFinite(v)||v<limits[i][0]-1e-9||v>limits[i][1]+1e-9))return {ok:false,message:'La postura inicial no cumple los límites articulares.'};
 const start=fk(startQ,g),end=solve(target,g,limits,startQ,orientation);
 if(!end)return {ok:false,message:'No se encontró una configuración válida para el punto B. Revisa el objetivo y los límites.'};
 const plan={ok:true,type,duration,orientation,startQ:startQ.slice(),endQ:end.q.slice(),target,start:{p:start.p,R:start.R},samples:[],geometry:{...g},maxPositionError:0,maxOrientationError:0,singular:end.singular};
 const make=(t,q)=>{const f=fk(q,g);return {t,q:q.slice(),p:f.p,R:f.R,tool:f.tool,camera:f.camera};};
 plan.samples.push(make(0,startQ));
 const n=Math.max(80,Math.ceil(duration*50)),maxStep=2*K.rad,posTolerance=0.00005,rotTolerance=0.02*K.rad;
 let calls=0;
 if(type==='joint'){
  for(let i=1;i<=n;i++){const t=i*duration/n;plan.samples.push(make(t,mix(startQ,end.q,smooth(t/duration))));}
 }else{
  function appendInterval(a,t,depth=0){
   if(++calls>16000||plan.samples.length>6000)throw {t,message:'Se alcanzó el límite de refinamiento. Reduce el desplazamiento o evita una singularidad.'};
   const desired=poseAt(start,target,t,duration),answer=solve(desired,g,limits,a.q,orientation);
   if(!answer)throw {t,message:'No se encontró inversa en un punto intermedio. No se generó una trayectoria parcial.'};
   const b=make(t,answer.q),jump=Math.max(...sub(b.q,a.q).map(Math.abs));
   let pe=0,re=0;
   // Verify quarter, midpoint and three-quarter of the actual joint interpolation.
   for(const u of [.25,.5,.75]){
    const tm=a.t+(t-a.t)*u,interpolated=fk(mix(a.q,b.q,u),g),e=errors(interpolated,poseAt(start,target,tm,duration));
    pe=Math.max(pe,e.position);if(orientation)re=Math.max(re,e.orientation);
   }
   if(jump>maxStep||pe>posTolerance||re>rotTolerance){
    if(depth>=12||t-a.t<1e-7)throw {t,message:'La rama no pudo seguirse de forma continua dentro de la tolerancia. Prueba una postura inicial distinta o una trayectoria articular.'};
    appendInterval(a,(a.t+t)/2,depth+1);appendInterval(plan.samples[plan.samples.length-1],t,depth+1);return;
   }
   const e=errors(b,desired);pe=Math.max(pe,e.position);if(orientation)re=Math.max(re,e.orientation);
   if(pe>posTolerance||re>rotTolerance)throw {t,message:'El error de la inversa supera la tolerancia del recorrido.'};
   plan.maxPositionError=Math.max(plan.maxPositionError,pe);plan.maxOrientationError=Math.max(plan.maxOrientationError,re);plan.singular ||= answer.singular;plan.samples.push(b);
  }
  try{for(let i=1;i<=n;i++)appendInterval(plan.samples[plan.samples.length-1],i*duration/n);}
  catch(e){return {ok:false,message:e.message||'No se pudo generar la trayectoria.',fraction:(e.t||0)/duration};}
  plan.endQ=plan.samples[plan.samples.length-1].q.slice();
 }
 plan.pathLength=0;plan.maxJointSpeed=0;plan.maxJointStep=0;
 for(let i=1;i<plan.samples.length;i++){
  const a=plan.samples[i-1],b=plan.samples[i],delta=Math.max(...sub(b.q,a.q).map(Math.abs));plan.pathLength+=norm(sub(b.p,a.p));plan.maxJointStep=Math.max(plan.maxJointStep,delta);plan.maxJointSpeed=Math.max(plan.maxJointSpeed,delta/(b.t-a.t));
 }
 if(type==='joint')plan.maxJointSpeed=Math.max(...sub(plan.endQ,startQ).map(Math.abs))*1.875/duration;
 return plan;
}
function csv(plan){
 const header=['time_s',...Array.from({length:6},(_,i)=>`q${i+1}_deg`),'active_x_m','active_y_m','active_z_m',...Array.from({length:9},(_,i)=>`active_R${Math.floor(i/3)+1}${i%3+1}`),'tool_x_m','tool_y_m','tool_z_m','camera_x_m','camera_y_m','camera_z_m',...Array.from({length:9},(_,i)=>`camera_R${Math.floor(i/3)+1}${i%3+1}`)];
 return [header.join(','),...plan.samples.map(s=>[s.t,...s.q.map(v=>v*K.deg),...s.p,...s.R.flat(),...s.tool.p,...s.camera.p,...s.camera.R.flat()].map(v=>v.toFixed(10)).join(','))].join('\n');
}
const api={smooth,quat,rotation,slerp,relativeTarget,poseAt,sampleQ,planMotion,csv};
if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.Motion=api;
})(typeof window!=='undefined'?window:globalThis);
