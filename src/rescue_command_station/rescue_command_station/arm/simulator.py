"""Offline desktop simulator; calls the SAME Python core as ROS. No publishers."""
import csv
import time
import tkinter as tk
from tkinter import ttk, filedialog
from dataclasses import asdict
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from .configuration import configured_arm
from .kinematics import make_T, rotx, roty, rotz, rot_to_rpy
from .motion import plan_segment


class Simulator:
    def __init__(self, root):
        self.root=root; root.title('Pedro · Simulador 6R · sin hardware')
        root.geometry('1280x850')
        self.arm,_=configured_arm(); self.q=np.zeros(6); self.plan=None
        self.playing=False; self.guard=False
        left=ttk.Frame(root,padding=10); left.pack(side='left',fill='y')
        tabs=ttk.Notebook(left); tabs.pack(fill='both',expand=True)
        controls=ttk.Frame(tabs,padding=8); parameters=ttk.Frame(tabs,padding=8)
        tabs.add(controls,text='Movimiento'); tabs.add(parameters,text='Geometría')
        ttk.Label(controls,text='Ángulos del MODELO (°), no ticks de servo').pack()
        self.sliders=[]
        for i in range(6):
            row=ttk.Frame(controls); row.pack(fill='x')
            ttk.Label(row,text=f'J{i+1}',width=3).pack(side='left')
            v=tk.DoubleVar(value=0); self.sliders.append(v)
            ttk.Scale(row,from_=np.degrees(self.arm.limits[i,0]),to=np.degrees(self.arm.limits[i,1]),
                      variable=v,command=lambda _,j=i:self.manual()).pack(side='left',fill='x',expand=True)
            ttk.Label(row,textvariable=v,width=8).pack(side='left')
        for label,q in [('Cero vertical',np.zeros(6)),('Postura de prueba',np.radians([20,-50,40,10,25,15]))]:
            ttk.Button(controls,text=label,command=lambda q=q:self.set_q(q)).pack(fill='x',pady=2)
        ttk.Label(controls,text='Objetivo absoluto: X Y Z (m), roll pitch yaw (°)').pack(pady=(10,2))
        self.target=[]
        for label in ['X','Y','Z','Roll','Pitch','Yaw']:
            row=ttk.Frame(controls); row.pack(fill='x')
            ttk.Label(row,text=label,width=7).pack(side='left')
            v=tk.StringVar(); self.target.append(v); ttk.Entry(row,textvariable=v,width=18).pack(side='left')
        ttk.Button(controls,text='Copiar pose actual',command=self.copy_pose).pack(fill='x')
        ttk.Button(controls,text='Resolver IK → vista previa',command=lambda:self.action(self.solve)).pack(fill='x')
        ttk.Label(controls,text='Desplazamiento relativo (m), orientación fija').pack(pady=(10,2))
        self.frame=tk.StringVar(value='tool')
        ttk.Combobox(controls,textvariable=self.frame,values=['base','tool','camera'],state='readonly').pack(fill='x')
        self.delta=[]
        row=ttk.Frame(controls); row.pack(fill='x')
        for label in 'XYZ':
            ttk.Label(row,text=label).pack(side='left')
            v=tk.StringVar(value='0'); self.delta.append(v)
            ttk.Entry(row,textvariable=v,width=7).pack(side='left')
        ttk.Button(controls,text='Usar desplazamiento como objetivo',command=lambda:self.action(self.relative)).pack(fill='x')
        row=ttk.Frame(controls); row.pack(fill='x',pady=6)
        self.kind=tk.StringVar(value='cartesian'); self.duration=tk.StringVar(value='3')
        ttk.Combobox(row,textvariable=self.kind,values=['cartesian','joint'],state='readonly',width=12).pack(side='left')
        ttk.Entry(row,textvariable=self.duration,width=5).pack(side='left'); ttk.Label(row,text='segundos').pack(side='left')
        ttk.Button(controls,text='Planificar trayectoria completa',command=lambda:self.action(self.generate)).pack(fill='x')
        ttk.Button(controls,text='Reproducir / pausa',command=self.play).pack(fill='x')
        self.scrub=tk.DoubleVar(value=0)
        self.timeline=ttk.Scale(controls,from_=0,to=1,variable=self.scrub,command=self.seek); self.timeline.pack(fill='x')
        ttk.Button(controls,text='Exportar trayectoria CSV',command=lambda:self.action(self.export)).pack(fill='x')
        self.fields={}
        for name,value in asdict(self.arm.p).items():
            ttk.Label(parameters,text=name).pack(anchor='w')
            text=','.join(map(str,value)) if isinstance(value,(tuple,list)) else str(value)
            v=tk.StringVar(value=text); self.fields[name]=v
            ttk.Entry(parameters,textvariable=v,width=30).pack(fill='x')
        ttk.Label(parameters,text='Metros y radianes. control_frame: tool o camera.\nCambios solo de sesión; no alteran el robot.\nMontaje de cámara provisional: medir/calibrar.',wraplength=300).pack(pady=8)
        ttk.Button(parameters,text='Aplicar a esta simulación',command=lambda:self.action(self.configure)).pack(fill='x')
        right=ttk.Frame(root); right.pack(side='left',fill='both',expand=True)
        fig=Figure(figsize=(8,7)); self.ax=fig.add_subplot(211,projection='3d'); self.graph=fig.add_subplot(212)
        self.canvas=FigureCanvasTkAgg(fig,master=right); self.canvas.get_tk_widget().pack(fill='both',expand=True)
        self.pose=tk.StringVar(); ttk.Label(right,textvariable=self.pose,font=('Courier',10)).pack(fill='x')
        self.message=tk.StringVar(value='Simulación exclusivamente. No verifica colisiones ni límites dinámicos.')
        ttk.Label(root,textvariable=self.message,wraplength=250).pack(side='right',fill='y')
        self.copy_pose(); self.draw()

    def action(self, fn):
        try: fn()
        except Exception as exc:
            self.message.set(str(exc)); self.playing=False

    def invalidate(self):
        self.playing=False; self.plan=None

    def manual(self):
        if self.guard: return
        self.invalidate(); self.q=np.radians([v.get() for v in self.sliders]); self.draw()

    def set_q(self,q,reset=True):
        if reset: self.invalidate()
        self.q=np.array(q).copy(); self.guard=True
        for v,a in zip(self.sliders,np.degrees(self.q)): v.set(round(float(a),4))
        self.guard=False; self.draw()

    def configure(self):
        overrides={}
        for k,v in self.fields.items():
            if k in ('camera_xyz','camera_rpy'): overrides[k]=[float(x) for x in v.get().split(',')]
            elif k=='control_frame': overrides[k]=v.get()
            else: overrides[k]=float(v.get())
        arm,_=configured_arm(overrides=overrides)
        self.arm=arm; self.invalidate(); self.copy_pose(); self.draw()

    def target_T(self):
        v=[float(x.get()) for x in self.target]; r,p,y=np.radians(v[3:])
        return make_T((rotz(y)@roty(p)@rotx(r))[:3,:3],v[:3])

    def fill_target(self,T):
        for v,x in zip(self.target,[*T[:3,3],*rot_to_rpy(T[:3,:3])]): v.set(f'{x:.10g}')

    def copy_pose(self): self.fill_target(self.arm.fk(self.q)['T06'])

    def solve(self):
        target=self.target_T(); q=self.arm.ik(target,q_init=self.q)
        e=self.arm.pose_error(q,target[:3,:3],target[:3,3]); self.set_q(q)
        self.message.set(f'IK exacta: error {np.linalg.norm(e[:3]):.3g} m / {np.linalg.norm(e[3:]):.3g} rad')

    def relative(self):
        self.fill_target(self.arm.relative_target(self.q,[float(x.get()) for x in self.delta],self.frame.get()))
        self.message.set('Objetivo relativo preparado; planifique para validar todo el recorrido.')

    def generate(self):
        self.invalidate()
        plan=plan_segment(self.arm,self.q,self.target_T(),float(self.duration.get()),self.kind.get())
        self.plan=plan; self.timeline.configure(to=float(plan.t[-1])); self.scrub.set(0)
        self.message.set(f'Plan válido: {len(plan.q)} muestras. No se enviaron órdenes a hardware.'); self.draw()

    def seek(self,_):
        if self.plan is not None: self.set_q(self.plan.sample(self.scrub.get()),reset=False)

    def play(self):
        if self.plan is None: return
        self.playing=not self.playing
        if self.playing:
            if self.scrub.get()>=self.plan.t[-1]: self.scrub.set(0)
            self.epoch=time.monotonic()-self.scrub.get(); self.tick()

    def tick(self):
        if not self.playing or self.plan is None: return
        t=min(time.monotonic()-self.epoch,self.plan.t[-1]); self.scrub.set(t); self.seek(None)
        if t>=self.plan.t[-1]: self.playing=False
        else: self.root.after(30,self.tick)

    def export(self):
        if self.plan is None: raise ValueError('Planifique primero')
        path=filedialog.asksaveasfilename(defaultextension='.csv',filetypes=[('CSV','*.csv')])
        if not path: return
        with open(path,'w',newline='',encoding='utf-8') as stream:
            writer=csv.writer(stream)
            writer.writerow(['time_s',*[f'q{i}_model_rad' for i in range(1,7)],
                             *[f'{frame}_{axis}_m' for frame in ('tool','camera') for axis in 'xyz']])
            for t,q in zip(self.plan.t,self.plan.q):
                f=self.arm.fk(q); writer.writerow([t,*q,*f['tool'][:3,3],*f['camera'][:3,3]])
        self.message.set('CSV exportado: ángulos del modelo, no comandos de servo.')

    def draw(self):
        f=self.arm.fk(self.q); ax=self.ax; ax.clear()
        points=f['points']; ax.plot(*points.T,'o-',linewidth=4,color='#287bb5')
        ax.plot(*np.vstack((f['flange'][:3,3],f['tool'][:3,3])).T,color='#d7aa18',linewidth=8)
        ax.plot(*np.vstack((f['flange'][:3,3],f['camera'][:3,3])).T,color='gray')
        ax.scatter(*f['camera'][:3,3],s=110,marker='s',c='white',edgecolors='black')
        def box(T,center,size,color):
            vertices=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*np.array(size)/2+center
            vertices=(T[:3,:3]@vertices.T).T+T[:3,3]
            faces=[[0,1,3,2],[4,5,7,6],[0,1,5,4],[2,3,7,6],[0,2,6,4],[1,3,7,5]]
            ax.add_collection3d(Poly3DCollection([vertices[i] for i in faces],facecolors=color,edgecolors='gray',alpha=.85))
        box(f['flange'],[self.arm.p.tool_length/2,0,0],[max(.005,abs(self.arm.p.tool_length)),.025,.012],'gold')
        box(f['camera'],[0,0,0],[.035,.05,.03],'white')
        for frame in ('tool','camera'):
            T=f[frame]; ax.text(*T[:3,3],frame)
            for i,color in enumerate(('r','g','b')): ax.quiver(*T[:3,3],*T[:3,i],length=.07,color=color)
        reach=self.arm.max_reach(); ax.set(xlim=(-reach,reach),ylim=(-reach,reach),zlim=(0,reach),xlabel='X',ylabel='Y',zlabel='Z')
        ax.set_box_aspect((2,2,1)); ax.set_title('J5: Z local · J6: X local · cámara y barra solidarias')
        self.graph.clear(); self.graph.set(xlabel='Tiempo (s)',ylabel='Modelo (°)')
        if self.plan is not None:
            path=np.array([self.arm.fk(q)['T06'][:3,3] for q in self.plan.q])
            ax.plot(*path.T,'--',color='#e67e22')
            self.graph.plot(self.plan.t,np.degrees(self.plan.q)); self.graph.legend([f'J{i}' for i in range(1,7)],ncol=6)
        self.pose.set(f"TCP activo: {self.arm.p.control_frame}\nTool xyz: {np.round(f['tool'][:3,3],5)}\nCamera xyz: {np.round(f['camera'][:3,3],5)}\nT activo:\n{np.array2string(f['T06'],precision=4)}")
        self.canvas.draw_idle()


def main():
    root=tk.Tk(); Simulator(root); root.mainloop()


if __name__=='__main__': main()
