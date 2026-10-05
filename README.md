# Taller Segundo Corte — Real-to-Sim con ESP32-S3 y PyBullet

Este repositorio desarrolla la práctica **real-to-sim**: un dispositivo real (una **ESP32-S3** con un **joystick analógico KY-023**) controla robots simulados en **PyBullet**.

| Punto | Qué se hace | Robot / entorno | Repositorio base |
|---|---|---|---|
| **A** | Mover drones de un punto A a un punto B y a un punto C, con la misión gestionada desde la ESP32-S3 | Formación de 3 drones Crazyflie | [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones) |
| **B** | Consola de mandos para mover los brazos de Baxter, posicionarlo y coger y mover un objeto | Baxter (dos brazos de 7 GDL) | [pybullet_robots](https://github.com/erwincoumans/pybullet_robots) (`baxter_ik_demo.py`) |
| **C** | Consola de mandos para dar movilidad al robot humanoide del laboratorio de Bullet | Atlas (Boston Dynamics) en el `botlab` | [pybullet_robots](https://github.com/erwincoumans/pybullet_robots/tree/master) |

> **Nota sobre el punto C.** El enunciado dice "robot Baxter", pero la imagen y el enlace del punto C muestran al **Atlas** de Boston Dynamics dentro del laboratorio del *Bullet ExampleBrowser*. Por eso el punto C se desarrolló con Atlas, y el punto B con Baxter.

---

## Contenido

1. [Arquitectura general](#1-arquitectura-general)
2. [Hardware y conexiones](#2-hardware-y-conexiones)
3. [Organización del repositorio](#3-organización-del-repositorio)
4. [Preparación del computador](#4-preparación-del-computador)
5. [Punto A — Drones A → B → C](#5-punto-a--drones-a--b--c)
6. [Punto B — Consola para Baxter](#6-punto-b--consola-para-baxter)
7. [Punto C — Consola para Atlas](#7-punto-c--consola-para-atlas)
8. [Solución de problemas](#8-solución-de-problemas)
9. [Conclusiones](#9-conclusiones)

---

## 1. Arquitectura general

```text
   MUNDO REAL                                   MUNDO SIMULADO
 ┌──────────────────────┐   USB serie    ┌──────────────────────────────┐
 │  Joystick KY-023     │   115200 bps   │  Computador (Python)         │
 │   VRx, VRy, SW       │                │                              │
 │        │             │  50 tramas/s   │  Hilo lector serie           │
 │        ▼             │ ─────────────► │        │                     │
 │  ESP32-S3            │                │        ▼                     │
 │  (MicroPython)       │                │  Control: PID / IK / marcha  │
 │  - ADC + filtro      │                │        │                     │
 │  - zona muerta       │                │        ▼                     │
 │  - modos y botones   │                │  PyBullet (física + GUI)     │
 │  - trayectoria (A)   │                │  drones / Baxter / Atlas     │
 └──────────────────────┘                └──────────────────────────────┘
```

**Reparto de tareas:**

- **ESP32-S3:** lee el joystick con el ADC, calibra el centro al arrancar, aplica zona muerta, curva *expo* y filtro, y distingue la pulsación corta de la larga. También lleva el estado de la consola (modo actual). En el punto A, además, **planifica la misión completa** (A → B → C) y genera la trayectoria.
- **Computador:** recibe las tramas por USB en un hilo aparte (así la simulación nunca se congela), convierte los comandos en movimientos (PID para los drones, cinemática inversa para Baxter, marcha y articulaciones para Atlas) y ejecuta la física en tiempo real.

### Protocolo serie

| Punto | Trama que envía la ESP32-S3 | Significado |
|---|---|---|
| A | `SP,x,y,z,destino,estado,modo` | Referencia de la formación (m), punto destino, estado (`HACIA_B`, `EN_C`...) y modo (`AUTO`/`MANUAL`) |
| A | `WP,nombre,x,y,z` | Definición de los puntos A, B y C (cada segundo) |
| B y C | `JOY,x,y,modo,acciones` | Joystick en % (−100 a 100), modo actual y contador de pulsaciones largas |
| Todos | `INFO,texto` | Mensajes informativos (se muestran en la terminal del PC) |

Las pulsaciones largas se envían como un **contador**. El PC detecta cuándo aumenta, así que no se pierde ninguna aunque se pierda una línea por el camino.

---

## 2. Hardware y conexiones

**Materiales:** 1 ESP32-S3 DevKit con MicroPython, 1 joystick KY-023, 5 cables Dupont hembra-hembra (o protoboard) y 1 cable USB de datos.

### Tabla de pines (es la misma para los tres puntos)

| KY-023 | ESP32-S3 | Función |
|---|---|---|
| GND | GND | Referencia |
| +5V | **3V3** | Alimentación del joystick |
| VRx | GPIO 1 (ADC1_CH0) | Eje X analógico |
| VRy | GPIO 2 (ADC1_CH1) | Eje Y analógico |
| SW | GPIO 4 | Pulsador (pull-up interno) |

> ⚠️ **Alimenta el KY-023 con 3V3, no con 5V.** Los potenciómetros del joystick entregan una tensión entre 0 V y su alimentación. Con 5 V, el ADC de la ESP32-S3 (máximo 3,3 V) recibiría más de lo que soporta.

### Esquema

```text
          KY-023                              ESP32-S3 DevKit
   ┌──────────────────┐                   ┌─────────────────────┐
   │  GND  ●──────────┼───────────────────┼──● GND              │
   │  +5V  ●──────────┼───────────────────┼──● 3V3              │
   │  VRx  ●──────────┼───────────────────┼──● GPIO1  (ADC)     │
   │  VRy  ●──────────┼───────────────────┼──● GPIO2  (ADC)     │
   │  SW   ●──────────┼───────────────────┼──● GPIO4            │
   │      ( ◉ )       │                   │                     │
   │     joystick     │                   │   USB ════════ PC   │
   └──────────────────┘                   └─────────────────────┘
```

**Por qué estos pines:**

- GPIO1 y GPIO2 pertenecen al **ADC1**, que funciona aunque se use Wi-Fi.
- GPIO4 no es pin de arranque (*strapping*), así que el pulsador no interfiere al encender.
- Se evitan GPIO0, 3, 45 y 46 (arranque) y GPIO19/20 (USB nativo).

**Convención de ejes:** empujar el joystick hacia arriba debe ser **adelante** y hacia la derecha, **derecha**. Como depende de cómo se sostenga el módulo, si un eje sale al revés cambia `INVERTIR_X`, `INVERTIR_Y` o `INTERCAMBIAR_XY` al inicio de `main.py` y vuelve a cargarlo.

**Calibración:** al encender o reiniciar la placa, **no toques el joystick durante 1 segundo**. En ese tiempo la ESP32 mide el centro de cada eje.

---

## 3. Organización del repositorio

```text
Taller Segundo Corte/
├── README.md
├── requisitos.txt                 ← todas las librerías del taller
├── .gitignore
├── evidencias/                    ← capturas y videos
│
├── Punto_A_Drones/
│   ├── esp32/
│   │   ├── main.py                ← lectura del joystick + envío de la referencia
│   │   └── trayectoria.py         ← planificador A→B→C (corre en la ESP32)
│   └── pc/
│       ├── drones_esp32.py        ← simulación + control PID de los drones
│       └── requisitos.txt
│
├── Punto_B_Baxter/
│   ├── esp32/
│   │   └── main.py                ← consola: joystick, modos y botón
│   └── pc/
│       ├── baxter_consola.py      ← Baxter: IK, pinzas, agarre, base móvil
│       └── requisitos.txt
│
├── Punto_C_Atlas/
│   ├── esp32/
│   │   └── main.py                ← consola: joystick, modos y botón
│   └── pc/
│       ├── atlas_consola.py       ← Atlas: marcha, brazos, torso, gestos, cámara
│       └── requisitos.txt
│
├── gym-pybullet-drones/           ← se clona (punto A), no se sube al repositorio
└── pybullet_robots/               ← se clona (puntos B y C), no se sube al repositorio
```

---

## 4. Preparación del computador

Todos los comandos se escriben en la terminal de **Visual Studio Code (PowerShell)**, desde la carpeta raíz del repositorio.

### 4.1 Librerías de Python

```powershell
python -m pip install --upgrade pip
python -m pip install -r requisitos.txt
```

> Si `pybullet` no instala en Windows, instala las **Microsoft C++ Build Tools** (Visual Studio Installer → "Desarrollo para el escritorio con C++") y repite el comando.

### 4.2 Repositorios base

```powershell
git clone https://github.com/utiasDSL/gym-pybullet-drones.git
git clone https://github.com/erwincoumans/pybullet_robots.git
```

Si no tienes `git`, descarga cada repositorio como ZIP desde GitHub ("Code → Download ZIP"), descomprímelo en la raíz y renombra la carpeta para quitarle `-main` o `-master`.

> **No hace falta instalar `gym-pybullet-drones` con `pip install -e .`** (su última versión pide Python 3.12 y PyTorch). El script del punto A agrega la carpeta clonada a la ruta de Python. Para eso solo necesita `scipy`, `gymnasium`, `pillow` y `transforms3d`, que ya están en `requisitos.txt`. Así funciona también con Python 3.10 y 3.11.

### 4.3 MicroPython en la ESP32-S3 (solo si la placa no lo tiene)

Descarga el firmware **ESP32_GENERIC_S3** (`.bin`) desde [micropython.org/download/ESP32_GENERIC_S3](https://micropython.org/download/ESP32_GENERIC_S3/) y ejecuta (cambia `COM7` y el nombre del archivo):

```powershell
python -m esptool --chip esp32s3 --port COM7 erase_flash
python -m esptool --chip esp32s3 --port COM7 --baud 460800 write_flash -z 0x0 ESP32_GENERIC_S3-xxxx.bin
```

### 4.4 Identificar el puerto

```powershell
python -m mpremote connect list
```

En los ejemplos se usa **COM7**. Reemplázalo por el puerto de tu placa.

> Se usa **una sola ESP32-S3** para los tres puntos. Para pasar de un punto a otro solo se carga el `main.py` correspondiente.

---

## 5. Punto A — Drones A → B → C

![Drones del punto A](evidencias/punto_a_drones.png)

### 5.1 Análisis

El enunciado pide que **el control se gestione desde la ESP32**. Por eso la placa no solo envía el joystick: es la que **decide y genera la misión**.

- `trayectoria.py` (corre **dentro** de la ESP32-S3) contiene los puntos A, B y C y una máquina de estados: `DESPEGUE → EN_A → HACIA_B → EN_B → HACIA_C → EN_C → FIN_MISION`.
- Cada tramo usa un **perfil de quinto orden**: `s(τ) = 10τ³ − 15τ⁴ + 6τ⁵`. Arranca y frena con velocidad y aceleración cero, así los drones se mueven de forma fluida y sin sacudidas. La duración se calcula para no superar `V_MAX = 0,35 m/s`.
- La ESP32 envía 50 veces por segundo la **referencia** (setpoint) de la formación.
- El PC ejecuta el control de bajo nivel. Usa `DSLPIDControl` del repositorio, un PID de posición y actitud, para que cada dron siga su lugar en la formación: tres drones en un círculo de 0,3 m alrededor de la referencia.
- Si se pierde la señal de la ESP32 durante más de 1 s, los drones **mantienen la posición** (modo a prueba de fallos).

| Punto | x (m) | y (m) | z (m) |
|---|---|---|---|
| A | 0,0 | 0,0 | 0,6 |
| B | 1,2 | 0,8 | 1,0 |
| C | −1,0 | 1,0 | 0,7 |

Los puntos se eligieron de modo que la ruta no choque con la esfera ni con los demás obstáculos del entorno (`obstacles=True`).

### 5.2 Controles

| Acción en el joystick | Resultado |
|---|---|
| Al encender | Despegue automático y misión **AUTO**: A → B → C, con 3 s de espera en cada punto |
| Pulsación **larga** (> 0,7 s) | Reinicia la misión AUTO desde A |
| Pulsación **corta** | Modo **MANUAL**: ir al siguiente punto (A → B → C → A) |
| Mover el joystick (en un punto) | Ajuste fino de la formación en X/Y (±0,6 m) |

### 5.3 Cargar y ejecutar

```powershell
# 1. Cargar el firmware (los DOS archivos)
python -m mpremote connect COM7 fs cp .\Punto_A_Drones\esp32\trayectoria.py :trayectoria.py
python -m mpremote connect COM7 fs cp .\Punto_A_Drones\esp32\main.py :main.py
python -m mpremote connect COM7 reset

# 2. (Opcional) Ver lo que envía la placa. Salir con Ctrl + ]
python -m mpremote connect COM7 repl

# 3. Ejecutar la simulación
python .\Punto_A_Drones\pc\drones_esp32.py --puerto COM7
```

Prueba sin la placa (usa el mismo `trayectoria.py`; flechas = joystick, **n** = pulsación corta, **m** = pulsación larga):

```powershell
python .\Punto_A_Drones\pc\drones_esp32.py --sin-esp
```

Otras opciones: `--drones 1` a `6` y `--repo RUTA` (si `gym-pybullet-drones` está en otra carpeta).

### 5.4 Partes clave del código

- **`esp32/trayectoria.py` → `Planificador.actualizar()`**: avanza el tramo con el perfil de 5.º orden y aplica el ajuste manual. En modo AUTO, pasa al siguiente punto al cumplirse la espera.
- **`esp32/main.py`**: bucle de 50 Hz que lee los botones (`Boton.evento()` distingue pulsación corta y larga), lee el joystick (`Joystick.leer()`) y envía `SP,...`.
- **`pc/drones_esp32.py`**: `LectorSerial` recibe las tramas en un hilo aparte. En el bucle principal, `computeControlFromState(...)` calcula las RPM de cada dron hacia `referencia + OFFSETS[j]`. `sync()` mantiene la simulación en tiempo real, igual que el reloj de la ESP32.

---

## 6. Punto B — Consola para Baxter

![Baxter del punto B](evidencias/punto_b_baxter.png)

### 6.1 Análisis

Se parte de `baxter_ik_demo.py` del repositorio. En lugar de deslizadores en pantalla, el objetivo de cada pinza lo mueve el joystick de la ESP32-S3.

- **Control por velocidad:** el joystick no fija la posición, sino la **velocidad** del objetivo (máximo 0,25 m/s en XY y 0,18 m/s en Z). Soltar el joystick deja el brazo quieto. Esto, junto con el filtro y la curva *expo* de la ESP32, da un movimiento fluido y preciso.
- **Cinemática inversa:** cada 1/60 s se resuelve la IK de los dos brazos con `calculateInverseKinematics`, con límites articulares y la postura actual como *rest pose*. Esto evita saltos de configuración. La orientación de la pinza se mantiene **hacia abajo** y su giro se controla con la muñeca.
- **Motores:** los 7 motores de cada brazo usan `POSITION_CONTROL` con velocidad máxima limitada (1,5 rad/s).
- **Espacio de trabajo:** el objetivo se limita a una caja frente al robot y a la esfera de alcance del brazo (1,05 m desde el hombro). Así nunca se pide una posición imposible.
- **Agarre:** al cerrar la pinza, los dedos se cierran y, si hay un cubo a menos de 6 cm, se crea una **unión fija** (`createConstraint`, `JOINT_FIXED`) con la posición relativa de ese instante. Al abrir, la unión se elimina y el cubo cae por gravedad. Es la técnica habitual en simulación porque el agarre por fricción en PyBullet es inestable.
- **Posicionamiento:** en modo BASE se mueve y gira todo el robot sobre el piso. Los objetivos de los brazos están en el marco del robot, así que acompañan a la base.
- **Escena:** mesa, dos cubos (rojo y azul) y una bandeja para dejarlos.

### 6.2 Controles

| Modo (pulsación corta para cambiar) | Joystick Y (arriba/abajo) | Joystick X (izq./der.) |
|---|---|---|
| 0 · BRAZO IZQ: plano XY | Alejar / acercar la pinza | Mover a la izquierda / derecha |
| 1 · BRAZO IZQ: altura Z / muñeca | Subir / bajar | Girar la muñeca |
| 2 · BRAZO DER: plano XY | Alejar / acercar | Izquierda / derecha |
| 3 · BRAZO DER: altura Z / muñeca | Subir / bajar | Girar la muñeca |
| 4 · BASE | Avanzar / retroceder el robot | Girar el robot |

**Pulsación larga:** abre o cierra la pinza del brazo activo. En modo BASE, reinicia la escena.

**Secuencia para coger y mover un cubo:**

1. Modo 0: lleva la esfera verde sobre el cubo rojo.
2. Modo 1: baja hasta tocarlo.
3. Pulsación larga para cerrar la pinza.
4. Modo 1: sube.
5. Modo 0: llévalo sobre la bandeja.
6. Modo 1: baja un poco y suéltalo con otra pulsación larga.

También se puede pasar el cubo al brazo derecho (modos 2 y 3).

### 6.3 Cargar y ejecutar

```powershell
python -m mpremote connect COM7 fs cp .\Punto_B_Baxter\esp32\main.py :main.py
python -m mpremote connect COM7 reset

python .\Punto_B_Baxter\pc\baxter_consola.py --puerto COM7
```

Prueba sin la placa (flechas, **m** = cambiar modo, **espacio** = pulsación larga):

```powershell
python .\Punto_B_Baxter\pc\baxter_consola.py --sin-esp
```

Si `pybullet_robots` está en otra carpeta, agrega `--datos RUTA\pybullet_robots\data`.

### 6.4 Partes clave del código

- **`SimBaxter._ik()`**: IK con orientación de la pinza hacia abajo y la postura actual como referencia.
- **`SimBaxter.mover_objetivo()`**: integra la velocidad del joystick y recorta al espacio de trabajo y al alcance del brazo.
- **`SimBaxter.accion_pinza()`**: cierra y abre los dedos, y crea o elimina la unión fija con el cubo más cercano.
- **`SimBaxter.mover_base()`**: reposiciona el robot. Como los objetivos están en el marco del robot, los brazos lo acompañan.

---

## 7. Punto C — Consola para Atlas

![Atlas del punto C](evidencias/punto_c_atlas.png)

### 7.1 Análisis

Se usa el humanoide **Atlas** dentro del laboratorio `botlab` de `pybullet_robots`, sobre la caja azul, como en la imagen del taller. Hacer caminar a un humanoide con equilibrio dinámico real es un problema de control avanzado. Para lograr una **movilidad real y fluida** desde la consola se usa un enfoque cinemático:

- **Pelvis ubicada por software** (base fija que se reposiciona en cada paso). Las articulaciones sí son motores físicos con su fuerza máxima del URDF, así que los brazos empujan y chocan con el entorno. Las piernas no colisionan con el piso para no "pelear" con la ubicación de la pelvis.
- **Marcha procedural:** al avanzar o girar, las piernas siguen un ciclo senoidal con desfase de 180°. La cadera oscila, la rodilla se levanta en la fase de vuelo y el tobillo se calcula como `aky = −(hpy + kny)` para que el **pie quede paralelo al piso**. La frecuencia y la amplitud dependen de la velocidad, y los brazos se balancean en contrafase.
- **Seguimiento del piso:** un rayo vertical bajo la pelvis mide la altura del suelo. La altura de la pelvis se calcula con la cinemática de la pierna (`0,066 + 0,374·cos(hpy) + 0,422·cos(hpy+kny) + 0,075`), así el pie de apoyo siempre toca el piso. Si el escalón es mayor que 0,20 m, el robot se detiene (obstáculo). Si es un desnivel hacia abajo, baja suavemente (por ejemplo, al bajar de la caja).
- **Velocidad suavizada:** arranques y frenadas exponenciales, sin saltos.
- **Cámara de la cabeza:** cada pocos pasos se toma una imagen desde la cabeza de Atlas. Se ve en los paneles **RGB, Depth y Segmentation Mask** de PyBullet, como en la imagen del enunciado. La cámara principal sigue al robot y puedes girarla con el mouse.
- **Gestos predefinidos:** reposo, saludo (animado), brazos arriba y pose T.

![Gesto brazos arriba](evidencias/punto_c_atlas_gesto.png)

### 7.2 Controles

| Modo (pulsación corta para cambiar) | Joystick Y | Joystick X |
|---|---|---|
| 0 · CAMINAR | Avanzar / retroceder | Girar |
| 1 · BRAZO DER: hombro | Subir / bajar el brazo | Adelante / atrás |
| 2 · BRAZO DER: codo | Doblar / estirar el codo | Girar el brazo |
| 3 · BRAZO IZQ: hombro | Subir / bajar | Adelante / atrás |
| 4 · BRAZO IZQ: codo | Doblar / estirar | Girar |
| 5 · TORSO Y CABEZA | Mover la cabeza arriba / abajo | Girar el torso |
| 6 · AGACHARSE / INCLINARSE | Pararse / agacharse | Inclinarse a un lado |

**Pulsación larga:** siguiente gesto (REPOSO → SALUDO → BRAZOS ARRIBA → POSE T).

### 7.3 Cargar y ejecutar

```powershell
python -m mpremote connect COM7 fs cp .\Punto_C_Atlas\esp32\main.py :main.py
python -m mpremote connect COM7 reset

python .\Punto_C_Atlas\pc\atlas_consola.py --puerto COM7
```

En un PC lento, usa la escena simple (piso liso con cajas):

```powershell
python .\Punto_C_Atlas\pc\atlas_consola.py --puerto COM7 --escena simple
```

Prueba sin la placa: `python .\Punto_C_Atlas\pc\atlas_consola.py --sin-esp`

### 7.4 Partes clave del código

- **`SimAtlas.caminar()`**: velocidad suavizada, rayo de piso, detección de obstáculos y avance de la fase de la marcha.
- **`SimAtlas._calcular_objetivos()`**: convierte el estado de la consola (marcha, brazos, torso, agachado, saludo) en los ángulos de las 30 articulaciones, recortados a los límites del URDF.
- **`SimAtlas._ubicar_pelvis()`**: altura de la pelvis según la pierna de apoyo, con subida y bajada limitadas.
- **`SimAtlas.camara_cabeza()`**: cámara sintética desde la cabeza para los paneles RGB, Depth y Segmentation.

---

## 8. Solución de problemas

| Problema | Causa probable | Solución |
|---|---|---|
| `could not enter raw repl` | La placa no tiene MicroPython o el puerto está ocupado | Instalar MicroPython (sección 4.3) y cerrar otras terminales |
| `PermissionError` / puerto ocupado al ejecutar la simulación | Hay un `mpremote repl` o monitor serie abierto | Cerrarlo (Ctrl + ]) antes de correr el script |
| "SIN SEÑAL DE LA ESP32" en pantalla | Puerto equivocado, placa reiniciándose o cable solo de carga | Revisar el COM con `mpremote connect list` y probar otro cable |
| El robot se mueve solo, sin tocar el joystick | Se tocó el joystick durante la calibración | Soltarlo y pulsar **RST** en la placa |
| Un eje responde al revés | Orientación del módulo | Cambiar `INVERTIR_X` / `INVERTIR_Y` / `INTERCAMBIAR_XY` en `main.py` |
| `ImportError: trayectoria` (punto A) | Solo se copió `main.py` | Copiar también `trayectoria.py` a la placa |
| `No module named gym_pybullet_drones` | No se clonó el repositorio | Clonarlo en la raíz o usar `--repo RUTA` |
| "No se encontró el modelo de Baxter/Atlas" | No se clonó `pybullet_robots` | Clonarlo en la raíz o usar `--datos RUTA\data` |
| La simulación va lenta | PC con poca GPU | Punto C con `--escena simple`; punto A con `--drones 1` |

---

## 9. Conclusiones

- La ESP32-S3 funcionó como una interfaz física completa: adquisición analógica, acondicionamiento de la señal (calibración, zona muerta, curva expo y filtro), interfaz de usuario con modos y pulsaciones corta y larga, y comunicación serie con un protocolo de texto simple y robusto.
- En el **punto A**, la misión entera se gestiona en el microcontrolador. La ESP32 genera trayectorias suaves de quinto orden entre A, B y C, y el PC solo cierra el lazo de control PID de cada dron. Esto muestra la separación típica entre **planificación** y **control de bajo nivel**.
- En el **punto B**, el control por velocidad con cinemática inversa permitió mover los dos brazos de Baxter con precisión, coger un objeto, trasladarlo y entregarlo de un brazo al otro o a la bandeja, además de reposicionar el robot.
- En el **punto C**, un enfoque cinemático (marcha procedural, pie paralelo al piso y seguimiento del terreno por rayos) dio a Atlas una movilidad fluida y creíble, controlada solo con un joystick de dos ejes y un botón.
- El paso *real-to-sim* depende de un buen manejo del tiempo: lectura serie en un hilo independiente, simulación sincronizada con el reloj real y comportamiento seguro ante la pérdida de señal.

### Evidencias en video

- [▶️ Video Punto A](https://drive.google.com/file/d/1Msv3wb7_htifA8v03c4k0LNmRBjU9H5o/view?usp=sharing))
- [▶️ Video Punto B](evidencias/punto_b.mp4)](https://drive.google.com/file/d/1CiD93sSjSVOV0lZsmvCNVxhSWjz4e5Q5/view?usp=sharing)
- [▶️ Video Punto C](https://drive.google.com/file/d/1HjAXQVlA3X45z_lQovYYwzrP-ScdmWm5/view?usp=sharing)
