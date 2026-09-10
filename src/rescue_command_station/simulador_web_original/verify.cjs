'use strict';
const assert=require('node:assert/strict'),K=require('./dist/kinematics.js');
const limits=Array.from({length:6},()=>[-Math.PI,Math.PI]);let seed=43891;
const rand=()=>{seed=(1664525*seed+1013904223)>>>0;return seed/4294967296;};
// Independent 4x4 implementation, without using the core matrix operations.
const mul=(a,b)=>a.map(row=>b[0].map((_,j)=>row.reduce((s,v,k)=>s+v*b[k][j],0)));
function H(axis,a,translation=false){
 const M=Array.from({length:4},(_,i)=>Array.from({length:4},(_,j)=>+(i===j)));
 if(translation){M[axis][3]=a;return M;}
 const i=(axis+1)%3,j=(axis+2)%3;M[i][i]=M[j][j]=Math.cos(a);M[i][j]=-Math.sin(a);M[j][i]=Math.sin(a);return M;
}
function independent(q,g){
 const chain=[H(2,q[0]),H(2,g.h,true),H(1,-q[1]),H(2,g.l1,true),H(1,-q[2]),H(2,g.l2,true),H(1,-q[3]),H(2,g.l3,true),H(2,q[4]),H(0,g.e56||0,true),H(0,q[5])];
 if(g.control==='camera')chain.push(H(0,g.cx||0,true),H(1,g.cy||0,true),H(2,g.cz||0,true),H(2,g.cw||0),H(1,g.cp||0),H(0,g.cr||0));else chain.push(H(0,g.d,true));
 return chain.reduce(mul);
}
let maxP=0,maxR=0,maxMatrix=0;
for(let i=0;i<1200;i++){
 const g={l1:.15+.3*rand(),l2:.15+.3*rand(),l3:.03+.12*rand(),h:.15*rand(),d:.15*rand(),e56:.05*rand(),cx:.08*rand(),cy:.03*rand(),cz:.05*rand(),cr:rand(),cp:rand(),cw:rand(),control:i%2?'camera':'tool'},q=Array.from({length:6},()=>6*rand()-3),f=K.fk(q,g);
 const reference=independent(q,g),diff=Math.max(...reference.flat().map((v,i)=>Math.abs(v-f.T.flat()[i])));maxMatrix=Math.max(maxMatrix,diff);assert.ok(diff<1e-12,'Independent homogeneous product');
 const result=K.ikPose(f,g,limits,[0,0,0,0,0,0]);assert.ok(result.solutions.length,'Regular IK must reconstruct a reachable pose');
 for(const s of result.solutions){maxP=Math.max(maxP,s.position);maxR=Math.max(maxR,s.orientation);assert.ok(s.position<1e-7&&s.orientation<1e-7);}
 const changed=q.slice();changed[5]+=.83;assert.ok(K.norm(K.sub(f.tool.p,K.fk(changed,g).tool.p))<1e-12,'J6 leaves the on-axis tip fixed');
 const localCamera=K.mv(K.tr(f.flange.R),K.sub(f.camera.p,f.flange.p));assert.ok(K.norm(K.sub(localCamera,[g.cx,g.cy,g.cz]))<1e-12,'Rigid camera position relative to J6');
}
const g={l1:.3,l2:.3,l3:.1,h:0,d:.12,e56:.02,cx:.06,cy:.015,cz:.04,cr:.2,cp:-.3,cw:.4};
const special=[[0,0,0,0,0,0],[.5,.6,-.9,.3,Math.PI/2,1.1],[.5,.6,-.9,.3,-Math.PI/2,1.1],[.2,Math.PI/2,0,0,.3,1],[0,0,Math.PI,0,.4,.6],[.34,.677,0,0,Math.PI/2,.543],[.2,Math.PI/2,0,0,Math.PI/2,.543]];
const singularCounts=special.map(q=>{const result=K.ikPose(K.fk(q,g),g,limits,[0,0,0,0,0,0]);assert.ok(result.solutions.length,'Singular reachable example');return result.solutions.length;});
let maxPosition=0;
for(let i=0;i<100;i++){
 const geom={...g,control:i%2?'camera':'tool'},q=Array.from({length:6},()=>5*rand()-2.5),p=K.fk(q,geom).p,result=K.ikPosition(p,geom,limits,[0,0,0,0,0,0]);assert.ok(result.converged,'Position solver reconstruction');maxPosition=Math.max(maxPosition,result.position);
}
const unreachable={p:[10,0,0],R:K.I()};assert.equal(K.ikPose(unreachable,g,limits,[0,0,0,0,0,0]).solutions.length,0);assert.equal(K.ikPosition(unreachable.p,g,limits,[0,0,0,0,0,0]).converged,false);
const limited=Array.from({length:6},()=>[-.1,.1]);assert.equal(K.ikPose(K.fk([1,.5,.8,.3,.4,.2],g),g,limited,[0,0,0,0,0,0]).solutions.length,0,'Limit rejection');
const qj=[.2,.3,.4,-.6,.5,.7],epsilon=1e-7;
for(const control of ['tool','camera']){const geom={...g,control},fj=K.fk(qj,geom);for(let i=0;i<6;i++){const q2=qj.slice();q2[i]+=epsilon;const numeric=K.scale(K.sub(K.fk(q2,geom).p,fj.p),1/epsilon),analytic=K.cross(fj.axes[i],K.sub(fj.p,fj.origins[i]));assert.ok(K.norm(K.sub(numeric,analytic))<1e-6,'Tool/camera geometric Jacobian finite difference');}}
const zero=K.fk([0,0,0,0,0,0],g),roll=K.fk([0,0,0,0,0,Math.PI/2],g);
assert.deepEqual(zero.axes[4],[0,0,1]);assert.deepEqual(zero.axes[5],[1,0,0]);assert.ok(K.norm(K.sub(zero.wrist,[0,0,.7]))<1e-12);
assert.ok(K.norm(K.sub(roll.camera.p,[g.e56+g.cx,-g.cz,.7+g.cy]))<1e-12,'Camera follows J6 orbit');
assert.ok(K.norm(K.sub(zero.tool.p,roll.tool.p))<1e-12,'Bar endpoint on J6 axis');
for(const pitch of [-Math.PI/2,-.4,Math.PI/2]){const R=K.rpyMatrix(.7,pitch,-1.2),again=K.rpyMatrix(...K.matrixRPY(R));assert.ok(K.errors({p:[0,0,0],R},{p:[0,0,0],R:again}).orientation<1e-7);}
process.stdout.write(JSON.stringify({passed:true,poseTrials:1200,independentDirectTrials:1200,maxPositionErrorMetres:maxP,maxOrientationErrorRadians:maxR,maxMatrixDifference:maxMatrix,positionTrials:100,maxPositionOnlyErrorMetres:maxPosition,singularCounts,limitsAndUnreachable:'passed',jacobianAndEuler:'passed'},null,2)+'\n');
