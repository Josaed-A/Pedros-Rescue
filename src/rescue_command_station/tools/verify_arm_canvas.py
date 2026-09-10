"""Optional Chromium/Tk integration smoke; no ROS or hardware is imported.

PYTHONPATH=src/rescue_command_station python src/rescue_command_station/tools/verify_arm_canvas.py --tk
Use --output-dir PATH to retain screenshots for visual review.
"""
import argparse
from dataclasses import replace
from pathlib import Path
import time
import numpy as np
from rescue_command_station.arm.configuration import configured_arm
from rescue_command_station.arm.kinematics import Arm6DOF
from rescue_command_station.arm.web_view import CanvasRenderer, scene_payload


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--tk', action='store_true')
    args = parser.parse_args()
    arm, _ = configured_arm()
    # Session-only delivered geometry, to inspect the yellow bar and third link
    # even when the user's configuration has zero length for those components.
    example = Arm6DOF(replace(arm.p, L1=.3, L2=.3, L3=.1, tool_length=.1), arm.limits)
    q = np.radians([20, -50, 40, 10, 25, 15])
    original = scene_payload(example, q)
    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=True)
    with CanvasRenderer() as renderer:
        errors = []
        renderer.page.on('pageerror', lambda error: errors.append(str(error)))
        first = renderer.render(original, (1000, 700))
        assert renderer.page.evaluate('typeof K.fk') == 'undefined', 'No JS solver allowed'
        assert renderer.page.evaluate('typeof window.M') == 'undefined', 'No JS planner allowed'
        assert renderer.page.evaluate('document.querySelectorAll("script").length') == 1
        assert renderer.page.evaluate('snapshot.fk.points.length') == 6
        renderer.render(original, (1000, 700), [('wheel', -120), ('view', 'front')])
        assert renderer.page.evaluate('camera.el') == 0
        moved = renderer.render(original, events=[
            ('pointer', {'type':'pointerdown','clientX':100,'clientY':100}),
            ('pointer', {'type':'pointermove','clientX':140,'clientY':120}),
            ('pointer', {'type':'pointerup','clientX':140,'clientY':120}),
        ])
        assert renderer.page.evaluate('camera.el') != 0
        assert first != moved
        turned_q = q.copy(); turned_q[5] += .7
        turned = renderer.render(scene_payload(example, turned_q, q), events=[('view', 'iso')])
        local = renderer.render(scene_payload(arm, q), (1000, 700))
        for frame in ('tool', 'camera'):
            model = Arm6DOF(replace(example.p, control_frame=frame), example.limits)
            renderer.render(scene_payload(model, q), (600, 400))
            np.testing.assert_allclose(renderer.page.evaluate('snapshot.fk.T'), model.fk(q)['T06'])
        assert not errors, errors
        if args.output_dir:
            for name, png in [('canvas_entrega.png', first), ('canvas_j6_preview.png', turned), ('canvas_config_local.png', local)]:
                (args.output_dir / name).write_bytes(png)
        print('PASS Chromium: original Canvas, Python-only FK, both TCPs, preview, J6, resize, drag and views')

    if args.tk:
        import tkinter as tk
        from rescue_command_station.arm.web_view import ArmCanvas
        root = tk.Tk(); root.withdraw()
        view = ArmCanvas(root)
        view.submit(original)
        deadline = time.monotonic() + 20
        try:
            while view._photo is None and time.monotonic() < deadline:
                root.update()
                time.sleep(.05)
            assert view._photo is not None, view.surface.cget('text')
        finally:
            view.close(); root.destroy(); view._worker.join(8)
        assert not view._worker.is_alive(), 'Chromium worker did not terminate'
        print('PASS Tk: received PNG in main thread and closed renderer cleanly (window hidden)')


if __name__ == '__main__':
    main()
