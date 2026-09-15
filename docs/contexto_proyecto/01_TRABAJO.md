# Trabajo realizado, objetivo y pendientes

Estado al cierre 2026-09-12: **auditoría estática general entregada; sin correcciones de implementación**. Se completó la documentación de continuidad. Las pruebas de integración/hardware siguen pendientes y están explícitamente delimitadas; no se presentan como aprobadas.

## Objetivo autorizado

Revisar a fondo división de carpetas, contenido, basura/residuos, arquitectura, contratos y fuentes de verdad. Crear contexto persistente y actualizarlo conforme avance el trabajo. No confiar en la documentación previa ni arreglar todavía los problemas.

## Avance completado

- [x] Registrar commit, cambios preexistentes y SHA256 iniciales.
- [x] Consolidar la documentación de Codex en docs/contexto_proyecto y eliminar los tres MD del contexto anterior, por petición del usuario.
- [x] Inventariar archivos versionados y carpetas locales/ignoradas.
- [x] Revisar siete paquetes, setup/manifest, interfaces y quince launch.
- [x] Seguir conducción, patas, brazo, cámara, detección, SLAM, misión y exportadores.
- [x] Contrastar configuraciones, scripts, Docker, requirements y documentación con consumidores reales.
- [x] Clasificar derivados/caches y comprobar consumidor/generador de dist y Canvas.
- [x] Ejecutar comprobaciones estáticas y pruebas aisladas disponibles.
- [x] Documentar 29 hallazgos con evidencia, impacto y límites.
- [x] Redactar mapa por carpeta y matriz de fuentes de verdad.
- [x] Verificar preservación de los 167 archivos versionados respecto al comienzo.

## Hitos registrados durante la revisión

1. Contexto inicial y primer inventario persistidos antes de revisar lógica en profundidad.
2. H01–H07 registrados tras launch/control; se corrigió la interpretación del entorno al encontrar ROS en /opt, inaccesible al Python activo.
3. Inventario, contratos, percepción/exports y resultados de pruebas persistidos.
4. Cinco reproducciones aisladas confirmadas; canvas derivado coincide con generador.
5. Informe consolidado, límites explícitos y hashes finales sin alteración de fuente.

## Cómo retomar

Leer primero 00_LEER_PRIMERO.md, luego 02_HALLAZGOS.md y 03_MAPA_Y_FUENTES_DE_VERDAD.md. Usar inventario/contratos/launch para localizar código. Verificar git status y commit antes de aplicar estas conclusiones a otro checkout. La evidencia describe archivos locales modificados del usuario, no solo el commit base.

Mantener IDs H01–H29 al añadir evidencia o cambiar severidad; explicar toda rectificación en lugar de borrar silenciosamente conclusiones. Tras cualquier futuro cambio de código, registrar qué hallazgos se resuelven y qué pruebas se repiten.

## Pendientes, sin autorización para corregirlos en esta tarea

Prioridad: arranque duplicado/flags (H01–H03,H29), pérdida de mando/feedback y límites de driver (H04–H05,H23); luego coherencia RGBD y misión (H08–H17); despliegue/credencial y dependencias (H18–H21); modularización, unidades, calibración y modelos (H22–H26). Limpieza al final (H27). Validación real y cobertura adicional en H28 y 06_COMPROBACIONES_Y_LIMITES.md.

No confundir «pendiente» con solicitud de seguir implementando: el usuario pidió solo revisión. No borrar dist, modelos, build/install ni cambios locales. La eliminación del contexto anterior fue una petición posterior explícita y ya está realizada. El siguiente trabajo de reparación deberá partir de una petición del usuario que autorice ese alcance.

## Cambios hechos por esta tarea

La auditoría creó documentos y evidencias, más un venv temporal fuera del repositorio. Después, por petición del usuario, se eliminaron los tres MD del contexto anterior y se trasladó toda la documentación de Codex a docs/contexto_proyecto; se añadió docs/README.md como entrada y se actualizaron referencias. No commits, push, despliegues ni cambios de configuración/código. requirements_pc.txt y config/arm.yaml ya tenían modificaciones del usuario y se preservaron byte por byte respecto al inicio.

## Última tarea — consolidación del contexto (2026-09-12)

Objetivo: quitar el contexto de autoría desconocida y dejar el de Codex como contexto general. Estado: completado. Una sola carpeta vigente: docs/contexto_proyecto. Se preservan los 29 hallazgos, evidencias, inventarios y pendientes de implementación; no se corrigió código del robot.

## Tarea siguiente — dependencias cruzadas Pi/PC (2026-09-12)

Objetivo pedido por el usuario: registrar en un documento dentro de este contexto todo lo encontrado sobre dependencias cruzadas entre el código de la Pi/robot y el del PC/estación. Estado: completado sin correcciones. Se creó [08_DEPENDENCIAS_CRUZADAS.md](08_DEPENDENCIAS_CRUZADAS.md) (D01–D08) a partir de `package.xml`, `setup.py`, imports Python y `get_package_share_directory` reales, sin ejecutar colcon/rosdep. Complementa H09/H19/H20/H25/H29, no los sustituye. Esta primera versión fue revisada en la ampliación siguiente: se retiró la afirmación de Dockerfile PC huérfano y no se recomienda declarar station→bringup porque formaría un ciclo. Las conclusiones vigentes son D01–D10 del documento 08; este párrafo registra el hito histórico, no recomendaciones pendientes de la versión anterior.

## Revisión ampliada — independencia PC/robot (2026-09-12)

Estado: COMPLETADO, sin implementación. Objetivo: contrastar D01–D08 con código y distinguir independencia de instalación, ejecución y contratos compartidos. Solo documentación. Se contrastaron configuración/simulación, imports, selección de imágenes y ciclo de recursos. D08 fue rectificado: run_slam_container.sh sí construye Dockerfile PC. Compartir interfaces no implica depender de la implementación de la otra máquina. El documento 08 contiene conclusiones, arquitectura propuesta y criterios de aceptación.

Checkpoint intermedio, posteriormente cerrado: grafo AST/manifiestos guardado en dependencias_cruzadas_evidencia.json; 08 actualizado con D01–D08 rectificados y D09–D10 nuevos. Confirmado: no hay imports Python de implementación entre station/core; sí dependencia por paquete, YAML, launch y topología de buses. D08 «Dockerfile PC huérfano» retirado por evidencia de scripts. D05 distinguió 6 literales ejecutables de 3 ejemplos. Cierre: índices/contexto reconciliados; fuente preservada. La reproducción aislada adicional confirma que resolver core precede al override de configuración.

### Resultado y pendientes actuales

La independencia de instalación y drivers PC/robot es un objetivo arquitectónico razonable. Contratos ROS y recursos compartidos deben tener una autoridad compatible, sin duplicarlos por máquina. No hay imports Python de producción station↔core; el acoplamiento se concentra en manifiestos, configuración, simulador, recursos y topología de buses. Se añadieron D09 (exclusiones de build no desacoplan manifiestos) y D10 (contrato local de desconexión).

Pendiente si se autoriza implementar: separar perfiles de arranque/instalación, extraer recursos/simulación realmente compartidos, eliminar resolución obligatoria de core en PC y revisar enrutamiento lógico de articulaciones. Primero resolver consumidores, luego ajustar dependencias. Validación futura: instalaciones limpias independientes y pruebas de comunicación/desconexión; no dar por resueltas al cambiar carpetas.
