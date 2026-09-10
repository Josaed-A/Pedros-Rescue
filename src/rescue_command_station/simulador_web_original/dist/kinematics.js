/* Brazo 6R. Metres, radians. Column vectors; rotations about local axes. */
(function(root){
'use strict';
const PI=Math.PI, rad=PI/180, deg=180/PI;
const add=(a,b)=>a.map((v,i)=>v+b[i]), sub=(a,b)=>a.map((v,i)=>v-b[i]), scale=(a,s)=>a.map(v=>v*s);
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0), norm=a=>Math.hypot(...a), cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const I=()=>[[1,0,0],[0,1,0],[0,0,1]], tr=A=>A[0].map((_,i)=>A.map(r=>r[i]));
const mv=(A,v)=>A.map(r=>dot(r,v)), mm=(A,B)=>A.map(r=>tr(B).map(c=>dot(r,c)));
const rx=a=>{let c=Math.cos(a),s=Math.sin(a);return [[1,0,0],[0,c,-s],[0,s,c]];};
const ry=a=>{let c=Math.cos(a),s=Math.sin(a);return [[c,0,s],[0,1,0],[-s,0,c]];};
const rz=a=>{let c=Math.cos(a),s=Math.sin(a);return [[c,-s,0],[s,c,0],[0,0,1]];};
const wrap=a=>((a+PI)%(2*PI)+2*PI)%(2*PI)-PI;
const clamp=(v,a,b)=>Math.min(b,Math.max(a,v));
function mounting(g,frame=g.control||'tool'){
  return frame==='camera'
    ? {offset:[(g.e56||0)+(g.cx||0),g.cy||0,g.cz||0],R:rpyMatrix(g.cr||0,g.cp||0,g.cw||0)}
    : {offset:[(g.e56||0)+g.d,0,0],R:I()};
}
function homogeneous(R,p){return [...R.map((r,i)=>[...r,p[i]]),[0,0,0,1]];}
function fk(q,g){
  let R=rz(q[0]),p=[0,0,g.h],points=[[0,0,0],p.slice()],origins=[[0,0,0]],axes=[[0,0,1]];
  [g.l1,g.l2,g.l3].forEach((l,i)=>{
    origins.push(p.slice());axes.push(mv(R,[0,-1,0]));
    R=mm(R,ry(-q[i+1]));p=add(p,mv(R,[0,0,l]));points.push(p.slice());
  });
  const wrist=p.slice();
  origins.push(p.slice());axes.push(mv(R,[0,0,1]));R=mm(R,rz(q[4]));
  p=add(p,mv(R,[g.e56||0,0,0]));
  origins.push(p.slice());axes.push(mv(R,[1,0,0]));R=mm(R,rx(q[5]));
  const flange={p:p.slice(),R:R.map(r=>r.slice()),T:homogeneous(R,p)};
  const toolP=add(p,mv(R,[g.d,0,0])),tool={p:toolP,R:R.map(r=>r.slice()),T:homogeneous(R,toolP)};
  const camP=add(p,mv(R,[g.cx||0,g.cy||0,g.cz||0])),camR=mm(R,rpyMatrix(g.cr||0,g.cp||0,g.cw||0));
  const camera={p:camP,R:camR,T:homogeneous(camR,camP)};
  points.push(toolP.slice());
  const active=g.control==='camera'?camera:tool;
  return {...active,wrist,points,origins,axes,flange,tool,camera};
}
function rpyMatrix(roll,pitch,yaw){return mm(mm(rz(yaw),ry(pitch)),rx(roll));}
function matrixRPY(R){
  const p=Math.asin(clamp(-R[2][0],-1,1));
  return Math.abs(Math.cos(p))>1e-8 ? [Math.atan2(R[2][1],R[2][2]),p,Math.atan2(R[1][0],R[0][0])] : [0,p,Math.atan2(-R[0][1],R[1][1])];
}
function errors(f,target){
  const E=mm(target.R,tr(f.R));
  const s=norm([E[2][1]-E[1][2],E[0][2]-E[2][0],E[1][0]-E[0][1]])/2;
  return {position:norm(sub(f.p,target.p)),orientation:Math.atan2(s,clamp((E[0][0]+E[1][1]+E[2][2]-1)/2,-1,1))};
}
function fitAngle(a,lim,seed){
  const kmin=Math.ceil((lim[0]-a-1e-10)/(2*PI)),kmax=Math.floor((lim[1]-a+1e-10)/(2*PI));
  if(kmin>kmax)return null;
  return clamp(a+2*PI*clamp(Math.round((seed-a)/(2*PI)),kmin,kmax),...lim);
}
function ikPose(target,g,limits,seed,options={}){
  const mount=mounting(g),R6=mm(target.R,tr(mount.R));
  const w=sub(target.p,mv(R6,mount.offset)),rho=Math.hypot(w[0],w[1]);
  const baseSingular=rho<1e-9,solutions=[];let wristSingular=false,unlimited=0;
  const verticalBase=Math.atan2(R6[1][0],R6[0][0]);
  const bases=baseSingular ? [seed[0],Math.atan2(w[1],w[0]),verticalBase,verticalBase+PI,...Array.from({length:180},(_,i)=>(i*2-180)*rad)] : [Math.atan2(w[1],w[0]),Math.atan2(w[1],w[0])+PI];
  for(const q1 of bases){
    const B=mm(rz(-q1),R6),c5=Math.hypot(B[0][0],B[2][0]);
    let orientations=[];
    if(c5>1e-9){
      const phi=Math.atan2(B[2][0],B[0][0])+PI/2,q5=Math.atan2(B[1][0],c5),q6=Math.atan2(-B[1][2],B[1][1]);
      orientations=[[phi,q5,q6],[phi+PI,PI-q5,q6+PI]];
    }else{
      wristSingular=true;
      const q5=Math.sign(B[1][0])*PI/2;
      const planarR=Math.cos(q1)*w[0]+Math.sin(q1)*w[1],planarZ=w[2]-g.h;
      const theta=Math.atan2(planarZ,planarR),dist=Math.hypot(planarR,planarZ),boundary=[];
      if(dist*g.l3>1e-12)for(const radius of [g.l1+g.l2,Math.abs(g.l1-g.l2)]){
        const c=(dist*dist+g.l3*g.l3-radius*radius)/(2*dist*g.l3);
        if(Math.abs(c)<=1+1e-9){const a=Math.acos(clamp(c,-1,1));boundary.push(theta-a,theta+a);}
      }
      const phis=[PI/2+seed[1]+seed[2]+seed[3],theta,...boundary,...Array.from({length:180},(_,i)=>(i*2-180)*rad)];
      orientations=phis.map(phi=>{const C=mm(tr(mm(ry(-(phi-PI/2)),rz(q5))),B);return [phi,q5,Math.atan2(C[2][1],C[1][1])];});
    }
    const r=Math.cos(q1)*w[0]+Math.sin(q1)*w[1],z=w[2]-g.h;
    for(const [phi,q5,q6] of orientations){
      const u=r-g.l3*Math.cos(phi),v=z-g.l3*Math.sin(phi);
      const D=(u*u+v*v-g.l1*g.l1-g.l2*g.l2)/(2*g.l1*g.l2);
      if(Math.abs(D)>1+1e-9)continue;
      for(const sign of [1,-1]){
        const q3=sign*Math.acos(clamp(D,-1,1)),q2=Math.atan2(v,u)-Math.atan2(g.l2*Math.sin(q3),g.l1+g.l2*Math.cos(q3));
        unlimited++;
        let q=[q1,q2-PI/2,q3,phi-q2-q3,q5,q6].map((v,i)=>fitAngle(v,limits[i],seed[i]));
        if(q.some(v=>v===null))continue;
        const e=errors(fk(q,g),target);
        if(e.position>1e-6||e.orientation>1e-6)continue;
        if(solutions.some(s=>norm(sub(s.q,q))<1e-5))continue;
        solutions.push({q,...e,score:norm(sub(q,seed)),label:`Base ${Math.cos(q1)*w[0]+Math.sin(q1)*w[1]>=0?'frontal':'opuesta'} · codo ${q3>=0?'+':'−'}`});
      }
    }
    if(options.nearOnly&&baseSingular&&solutions.length)break;
  }
  solutions.sort((a,b)=>a.score-b.score);
  return {solutions,baseSingular,wristSingular,sampled:baseSingular||wristSingular,unlimited};
}
function solveLinear(A,b){
  const M=A.map((r,i)=>[...r,b[i]]),n=b.length;
  for(let k=0;k<n;k++){
    let pivot=k;for(let i=k+1;i<n;i++)if(Math.abs(M[i][k])>Math.abs(M[pivot][k]))pivot=i;
    if(Math.abs(M[pivot][k])<1e-15)return null;
    [M[k],M[pivot]]=[M[pivot],M[k]];const a=M[k][k];for(let j=k;j<=n;j++)M[k][j]/=a;
    for(let i=0;i<n;i++)if(i!==k){const v=M[i][k];for(let j=k;j<=n;j++)M[i][j]-=v*M[k][j];}
  }return M.map(r=>r[n]);
}
function halton(i,b){let f=1,r=0;while(i>0){f/=b;r+=f*(i%b);i=Math.floor(i/b);}return r;}
function ikPosition(p,g,limits,seed){
  let best={q:seed.slice(),position:Infinity};const bases=[2,3,5,7,11,13];
  for(let attempt=0;attempt<24;attempt++){
    let q=attempt===0?seed.slice():limits.map(([lo,hi],i)=>lo+(hi-lo)*halton(attempt,bases[i]));
    q=q.map((v,i)=>clamp(v,...limits[i]));
    for(let k=0;k<240;k++){
      const f=fk(q,g),e=sub(p,f.p),err=norm(e);
      if(err<best.position)best={q:q.slice(),position:err};
      if(err<1e-5)return {...best,converged:true,attempts:attempt+1};
      const J=tr(f.axes.map((a,i)=>cross(a,sub(f.p,f.origins[i])))),JT=tr(J);
      const lambda=0.003, A=mm(J,JT).map((r,i)=>r.map((v,j)=>v+(i===j?lambda*lambda:0))),v=solveLinear(A,e);
      if(!v)break;let dq=mv(JT,v),m=Math.max(...dq.map(Math.abs));if(m>0.25)dq=scale(dq,0.25/m);
      let improved=false;
      for(const alpha of [1,0.5,0.25,0.1]){
        const next=q.map((x,i)=>clamp(x+alpha*dq[i],...limits[i]));
        if(norm(sub(p,fk(next,g).p))<err-1e-12){q=next;improved=true;break;}
      }if(!improved)break;
    }
  }return {...best,converged:false,attempts:24};
}
const api={rad,deg,add,sub,scale,dot,norm,cross,I,tr,mv,mm,rx,ry,rz,wrap,clamp,mounting,homogeneous,fk,rpyMatrix,matrixRPY,errors,ikPose,ikPosition};
if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.K=api;
})(typeof window!=='undefined'?window:globalThis);
