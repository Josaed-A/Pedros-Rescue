importScripts('kinematics.js','motion.js');
self.onmessage=function(event){
 try{self.postMessage(Motion.planMotion(event.data));}
 catch(e){self.postMessage({ok:false,message:'No se pudo calcular el recorrido: '+e.message});}
};
