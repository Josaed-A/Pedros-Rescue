# Brazo 6R — simulador paramétrico

Sitio estático, sin dependencias ni solicitudes a servicios externos. Abrir `dist/index.html` en un navegador moderno. Toda la matemática se ejecuta localmente en JavaScript; las medidas internas son metros y radianes.

Montaje corregido: T06 = Rz(q1) Tz(h) Ry(-q2) Tz(L1) Ry(-q3) Tz(L2) Ry(-q4) Tz(L3) Rz(q5) Tx(e56) Rx(q6). Todos los ángulos cero corresponden al brazo vertical y barra hacia +X. J5 apunta a Z global y J6 a X global en ese cero. La punta es T06 Tx(d); la cámara es T06 T6c. Ambas piezas giran con J6. Las traslaciones y la orientación de T6c son editables, no calibradas.

L1=0.30 m, L2=0.30 m, L3=0.10 m son las medidas proporcionadas. h=e56=0, barra d=0.12 m y cámara c=(0.06,0,0.04) m, con R6c=I, son valores iniciales provisionales. Las cajas dibujadas son esquemáticas, sin cálculo de colisiones. La inversa y las trayectorias permiten seleccionar punta de barra o centro de cámara como punto controlado; los desplazamientos pueden expresarse en herramienta, cámara o base. El cero vertical es singular; existe una postura de prueba flexionada.

- `dist/kinematics.js`: directa, inversa analítica para pose completa e inversa numérica amortiguada para posición.
- `dist/app.js`: controles y visualización 3D ortográfica mediante Canvas; órbita y zoom.
- `dist/motion.js`: desplazamientos locales/base, perfil quíntico, SLERP, planificación cartesiana adaptativa y articular, muestreo temporal y CSV.
- `dist/motion-ui.js` y `dist/motion-worker.js`: generación en segundo plano, reproducción, pausa, inspección temporal, curva de punta y gráfico articular. En archivos locales se utiliza un cálculo directo como alternativa al Worker.
- `dist/modelo.html`: derivación, ramas, singularidades, unidades y uso.
- `verify.cjs`: verificación matemática independiente; ejecutar `node verify.cjs`.
- `verify-motion.cjs`: comprueba marcos locales, trayectorias, límites, cambios de rama, orientación, errores intermedios y CSV; ejecutar `node verify-motion.cjs`.

Las familias singulares se representan por candidatos y muestreo; no se certifica imposibilidad cuando este muestreo falla. La solución numérica de posición no garantiza convergencia global. Las trayectorias cartesianas se comprueban discretamente, se subdividen y se rechazan si no se puede seguir una rama continua. No se evalúan colisiones, dinámica, límites de velocidad/aceleración/torque ni comandos de hardware.
