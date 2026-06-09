# Pedros

Rescue Robot Platform for RoboCup Rescue League.

## Control actual

El robot usa ROS 2 y esta dividido en dos paquetes principales:

- `rescue_pc_brain`: lee el control PS4, calcula el comando de manejo y publica `/cmd_vel`.
- `rescue_raspberry_brain`: escucha `/cmd_vel`, calcula potencia para cada oruga y controla los drivers BTS7960 por GPIO.

### Manejo tipo tanque

Todo el movimiento sale del joystick izquierdo:

- Joystick hacia adelante: ambas orugas avanzan a la misma velocidad.
- Joystick hacia atras: ambas orugas retroceden a la misma velocidad.
- Joystick solo hacia un lado: las orugas giran en sentidos opuestos.
- Joystick diagonal: una oruga va mas rapido que la otra para girar avanzando o retrocediendo.

No se usa acelerador con R2 y no hay habilitacion por Share.

### Cajas

Hay 5 cajas positivas:

- Caja 1: 20%
- Caja 2: 40%
- Caja 3: 60%
- Caja 4: 80%
- Caja 5: 100%

Controles:

- `R1`: subir caja.
- `L1`: bajar caja.

