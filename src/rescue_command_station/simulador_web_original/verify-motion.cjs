'use strict';
const assert=require('node:assert/strict'),K=require('./dist/kinematics.js'),M=require('./dist/motion.js');
const limits=Array.from({length:6},()=>[-Math.PI,Math.PI]),q=[20,-50,40,10,25,15].map(v=>v*K.rad);
let count=0,worstP=0,worstR=0;
for(const control of ['tool','camera']){
 const g={l1:.3,l2:.3,l3:.1,h:.08,d:.12,e56:.02,cx:.06,cy:.015,cz:.04,cr:.2,cp:-.3,cw:.4,control},start=K.fk(q,g);
 for(let axis=0;axis<3;axis++)for(const sign of [-1,1]){
  const delta=[0,0,0];delta[axis]=sign*.01;
  const target=M.relativeTarget(start,delta,control),expected=start.p.map((v,i)=>v+start.R[i][axis]*sign*.01);
  assert.ok(K.norm(K.sub(target.p,expected))<1e-12,'Local displacement uses R, not R transpose or translated point');
  const p=M.planMotion({startQ:q,target,g,limits,duration:2});assert.ok(p.ok,p.message);
  assert.ok(K.norm(K.sub(M.sampleQ(p,0),q))<1e-12,'Plan starts at exact input angles');
  for(let i=0;i<=501;i++){
   const t=p.duration*i/501,f=K.fk(M.sampleQ(p,t),g),ideal=M.poseAt(start,target,t,p.duration),e=K.errors(f,ideal);
   worstP=Math.max(worstP,e.position);worstR=Math.max(worstR,e.orientation);
   assert.ok(e.position<.00005&&e.orientation<.02*K.rad,'Dense independent timing check');
   M.sampleQ(p,t).forEach((v,j)=>assert.ok(v>=limits[j][0]-1e-10&&v<=limits[j][1]+1e-10));
  }
  assert.ok(p.maxJointStep<=2*K.rad+1e-10,'No discontinuous IK branch jumps');
  const rows=M.csv(p).split('\n');assert.equal(rows.length,p.samples.length+1);assert.equal(rows[0].split(',').length,34);count++;
 }
 const base=M.relativeTarget(start,[.01,-.02,.03],'base');assert.ok(K.norm(K.sub(base.p,K.add(start.p,[.01,-.02,.03])))<1e-12);
 const q6=q.slice();q6[5]+=Math.PI/2;const rotated=K.fk(q6,g);assert.ok(K.norm(K.sub(start.tool.p,rotated.tool.p))<1e-12);
 const dy=M.relativeTarget(rotated,[0,.01,0]);assert.ok(K.norm(K.sub(K.sub(dy.p,rotated.p),K.scale(start.tool.R.map(r=>r[2]),.01)))<1e-12,'J6 rotates tool Y toward previous tool Z, independently of selected point');
 const goalQ=q.map((v,i)=>v+[.1,-.08,.12,.04,-.1,.14][i]),goal=K.fk(goalQ,g);
 for(const type of ['cartesian','joint']){
  const p=M.planMotion({startQ:q,target:goal,g,limits,duration:3,type});assert.ok(p.ok,p.message);
  const final=K.errors(K.fk(M.sampleQ(p,3),g),goal);assert.ok(final.position<1e-6&&final.orientation<1e-6);
  if(type==='joint')assert.ok(K.norm(K.sub(M.sampleQ(p,1.5),p.startQ.map((v,i)=>(v+p.endQ[i])/2)))<1e-10);
  count++;
 }
 const position=M.planMotion({startQ:q,target:{p:K.add(start.p,[.015,0,0]),R:K.I()},g,limits,duration:1.5,orientation:false});assert.ok(position.ok,position.message);count++;
 assert.equal(M.planMotion({startQ:q,target:{p:[8,0,0],R:K.I()},g,limits}).ok,false);
}
for(const axis of [K.rx,K.ry,K.rz]){const R=axis(Math.PI),mid=M.slerp(K.I(),R,.5);assert.ok(Math.abs(K.errors({p:[0,0,0],R:mid},{p:[0,0,0],R:K.I()}).orientation-Math.PI/2)<1e-10);}
const short=M.slerp(K.rz(179*K.rad),K.rz(-179*K.rad),.5);assert.ok(K.errors({p:[0,0,0],R:short},{p:[0,0,0],R:K.rz(Math.PI)}).orientation<1e-10,'Shortest rotation across Euler wrap');
const holeG={l1:.3,l2:.1,l3:0,h:0,d:0},a={p:[.35,0,0],R:K.I()},b={p:[-.35,0,0],R:K.I()},qa=K.ikPose(a,holeG,limits,[0,0,0,0,0,0]).solutions[0].q;
assert.ok(K.ikPose(b,holeG,limits,qa).solutions.length,'Endpoint B reachable');
const blocked=M.planMotion({startQ:qa,target:b,g:holeG,limits,type:'cartesian'});assert.equal(blocked.ok,false,'Reachable endpoints do not imply reachable straight segment');assert.equal(blocked.samples,undefined,'Do not return a playable partial path');
const g={l1:.3,l2:.3,l3:.1,h:0,d:0},wrapStart=q.slice();wrapStart[5]=179*K.rad;const wrapEnd=q.slice();wrapEnd[5]=-179*K.rad;
assert.equal(M.planMotion({startQ:wrapStart,target:K.fk(wrapEnd,g),g,limits,type:'cartesian'}).ok,false,'Reject a discontinuous 360-degree branch wrap at hard limits');
assert.equal(M.smooth(0),0);assert.equal(M.smooth(1),1);assert.equal(M.smooth(.5),.5);
const initialG={l1:.3,l2:.3,l3:.1,h:0,d:.12,e56:0,cx:.06,cy:0,cz:.04,cr:0,cp:0,cw:0,control:'tool'},zero=[0,0,0,0,0,0],zStart=K.fk(zero,initialG);
const rollOnly=M.planMotion({startQ:zero,target:K.fk([0,0,0,0,0,Math.PI/3],initialG),g:initialG,limits,type:'cartesian'});
assert.ok(rollOnly.ok,rollOnly.message);
assert.ok(rollOnly.samples.every(s=>K.norm(K.sub(s.tool.p,zStart.tool.p))<1e-8),'Stationary bar tip through J6-only rotation');
assert.ok(K.norm(K.sub(rollOnly.samples.at(-1).camera.p,zStart.camera.p))>.03,'Camera moves during J6-only trajectory');
const descend=M.planMotion({startQ:zero,target:M.relativeTarget(zStart,[0,0,-.01],'base'),g:initialG,limits,type:'cartesian'});assert.ok(descend.ok,descend.message);
const lateral=M.planMotion({startQ:zero,target:M.relativeTarget(zStart,[.01,0,0],'base'),g:initialG,limits,type:'cartesian'});assert.equal(lateral.ok,false,'Fully extended vertical arm cannot translate sideways at fixed height and orientation');
count+=2;
console.log(JSON.stringify({passed:true,acceptedTrajectories:count,densePositionErrorMetres:worstP,denseOrientationErrorRadians:worstR,framesAndJ6:'passed',rotationInterpolation:'passed',unreachableIntermediateAndBranchJump:'passed',csv:'passed'},null,2));
