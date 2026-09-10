"""Original web Canvas in the existing Tk panel, with Python as its only model.

Chromium renders offscreen on a dedicated thread. Tk receives PNG frames and
forwards view gestures, never robot commands. No HTTP server, JS solver, browser
network access or hardware transport is involved. Missing Chromium is explicit.
"""
from pathlib import Path
import queue
import threading
import tkinter as tk
import numpy as np


def pose_payload(T):
    return {'p': T[:3, 3].tolist(), 'R': T[:3, :3].tolist(), 'T': T.tolist()}


def fk_payload(arm, q_model):
    """Serialize the supplied FK, respecting active TCP versus geometric flange."""
    f = arm.fk(q_model)
    return dict(
        **pose_payload(f['T06']),
        **{name: pose_payload(f[name]) for name in ('flange', 'tool', 'camera')},
        **{name: f[name].tolist() for name in ('points', 'origins', 'axes')},
        wrist=f['origins'][4].tolist(),
    )


def scene_payload(arm, q_model, preview_model=None, status='Vista previa · sin hardware'):
    p = arm.p
    g = dict(l1=p.L1, l2=p.L2, l3=p.L3, h=p.base_height, d=p.tool_length,
             e56=p.e56, cx=p.camera_xyz[0], cy=p.camera_xyz[1], cz=p.camera_xyz[2],
             cr=p.camera_rpy[0], cp=p.camera_rpy[1], cw=p.camera_rpy[2], control=p.control_frame)
    return {'geometry': g, 'fk': fk_payload(arm, q_model), 'status': status,
            'preview': None if preview_model is None else fk_payload(arm, preview_model)}


class CanvasRenderer:
    """Synchronous browser wrapper; create/use/close on the same worker thread."""
    def __enter__(self):
        from playwright.sync_api import sync_playwright
        self.runtime = sync_playwright().start()
        try:
            self.browser = self.runtime.chromium.launch(headless=True)
            self.page = self.browser.new_page(viewport={'width': 760, 'height': 520})
            self.page.set_default_timeout(5000)
            # Assets only. An integrated view cannot reach a remote service.
            self.page.route('http://**/*', lambda route: route.abort())
            self.page.route('https://**/*', lambda route: route.abort())
            self.page.goto((Path(__file__).parent / 'web_assets/index.html').as_uri())
            self.size = (760, 520)
            return self
        except Exception:
            self.__exit__(None, None, None)
            raise

    def render(self, snapshot, size=None, events=()):
        if size and size != self.size:
            self.page.set_viewport_size({'width': size[0], 'height': size[1]})
            self.size = size
        self.page.evaluate('data => window.setArmSnapshot(data)', snapshot)
        for kind, value in events:
            if kind == 'pointer':
                self.page.evaluate('e => window.armPointer(e)', value)
            elif kind == 'wheel':
                self.page.evaluate('delta => window.armWheel(delta)', value)
            elif kind == 'key':
                self.page.evaluate('key => window.armKey(key)', value)
            elif kind == 'view':
                self.page.evaluate('view => window.armView(view)', value)
        return self.page.screenshot(type='png')

    def __exit__(self, *args):
        try:
            if hasattr(self, 'browser'):
                self.browser.close()
        finally:
            self.runtime.stop()


class ArmCanvas(tk.Frame):
    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg='#0e1725', **kwargs)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._snapshot = None
        self._size = (760, 520)
        self._events = []
        self._result = queue.Queue(maxsize=1)
        self._photo = None
        self._closed = False
        toolbar = tk.Frame(self, bg='#0e1725')
        toolbar.pack(fill='x')
        for label, view in [('3D', 'iso'), ('Frontal', 'front'), ('Superior', 'top')]:
            tk.Button(toolbar, text=label, command=lambda v=view: self._event('view', v)).pack(side='left')
        tk.Label(toolbar, text='Arrastra para girar · rueda para zoom', bg='#0e1725', fg='#cad7e9').pack(side='left', padx=10)
        self.surface = tk.Label(self, text='Cargando Canvas original…', bg='#0e1725', fg='#cad7e9', anchor='center', borderwidth=0, highlightthickness=0)
        self.surface.pack(fill='both', expand=True)
        self.surface.bind('<Configure>', self._resize)
        for binding, kind in [('<ButtonPress-1>', 'pointerdown'), ('<B1-Motion>', 'pointermove'), ('<ButtonRelease-1>', 'pointerup')]:
            self.surface.bind(binding, lambda e, k=kind: self._pointer(e, k))
        self.surface.bind('<MouseWheel>', lambda e: self._event('wheel', -e.delta))
        self.surface.bind('<Button-4>', lambda e: self._event('wheel', -120))
        self.surface.bind('<Button-5>', lambda e: self._event('wheel', 120))
        self.surface.bind('<KeyPress>', lambda e: self._event('key', {'Left':'ArrowLeft','Right':'ArrowRight','Up':'ArrowUp','Down':'ArrowDown','plus':'+','minus':'-'}.get(e.keysym, e.char)))
        self.bind('<Destroy>', lambda e: self.close() if e.widget is self else None)
        self._worker = threading.Thread(target=self._run, daemon=True, name='arm-canvas')
        self._worker.start()
        self._poll_id = self.after(50, self._poll)

    def _resize(self, event):
        with self._lock:
            self._size = (max(100, event.width), max(100, event.height))

    def _pointer(self, event, kind):
        self.surface.focus_set()
        self._event('pointer', {'type': kind, 'clientX': event.x, 'clientY': event.y})

    def _event(self, kind, value):
        with self._lock:
            # Coalesce mouse motion to keep a slow renderer from accumulating lag.
            if kind == 'pointer' and value['type'] == 'pointermove' and self._events and self._events[-1][0] == kind and self._events[-1][1]['type'] == 'pointermove':
                self._events[-1] = (kind, value)
            else:
                self._events.append((kind, value))

    def submit(self, snapshot):
        with self._lock:
            self._snapshot = snapshot

    def _put_result(self, value):
        try:
            self._result.get_nowait()
        except queue.Empty:
            pass
        self._result.put_nowait(value)

    def _run(self):
        try:
            with CanvasRenderer() as renderer:
                previous, previous_size = None, None
                while not self._stop.wait(.10):
                    with self._lock:
                        snapshot, size = self._snapshot, self._size
                        events, self._events = self._events, []
                    if snapshot is not None and (snapshot != previous or size != previous_size or events):
                        self._put_result(('png', renderer.render(snapshot, size, events)))
                        previous, previous_size = snapshot, size
        except Exception as exc:
            self._put_result(('error', 'Canvas no disponible. Instala playwright y ejecuta\n'
                              'python -m playwright install chromium\n'
                              'Consulta ARM_6R.md.\n' + str(exc)))

    def _poll(self):
        if self._closed:
            return
        try:
            kind, value = self._result.get_nowait()
            if kind == 'png':
                self._photo = tk.PhotoImage(master=self, data=value, format='png')
                self.surface.configure(image=self._photo, text='')
            else:
                self.surface.configure(image='', text=value, wraplength=500)
        except queue.Empty:
            pass
        self._poll_id = self.after(50, self._poll)

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        if hasattr(self, '_poll_id'):
            self.after_cancel(self._poll_id)


def main():
    """Smoke/preview window without ROS, using the installed physical config."""
    from .configuration import configured_arm
    arm, _ = configured_arm()
    root = tk.Tk()
    root.title('Canvas original 6R · sin ROS ni hardware')
    root.geometry('1000x700')
    view = ArmCanvas(root)
    view.pack(fill='both', expand=True)
    view.submit(scene_payload(arm, np.radians([20, -50, 40, 10, 25, 15])))
    try:
        root.mainloop()
    finally:
        view.close()
        view._worker.join(timeout=8)


if __name__ == '__main__':
    main()
