# can_protocol

Aquí se describe el protocolo de comunicación CAN entre todas las placas que
conforman el sistema modular de la aviónica.

Cada placa tiene su propio repositorio y todas incluyen este como **git
submodule**, de modo que el protocolo solo se define en un sitio.

## Contenido

| Archivo | Qué es |
|---------|--------|
| [`ICD.md`](ICD.md) | Especificación del protocolo: capa física, codificación, nodos, IDs y mensajes. **Es la referencia.** |
| `include/can_protocol/can_protocol_version.h` | Versión del protocolo (MAJOR.MINOR) |
| `include/can_protocol/can_bus_config.h` | Parámetros comunes del bus (velocidad, punto de muestreo...) |
| `include/can_protocol/can_ids.h` | Nodos e identificadores CAN |
| `include/can_protocol/can_pack.h` | Lectura y escritura de campos en little-endian |
| `CMakeLists.txt` | Librería CMake `can_protocol` para integrarlo en los proyectos |

## Uso en una placa

TODO: añadir como submodule.

En el `CMakeLists.txt` de la placa (requiere CMake ≥ 3.22 y C11):

```cmake
add_subdirectory(ruta/al/submodule/can_protocol)
target_link_libraries(${PROJECT_NAME} can_protocol)
```

En el código, las cabeceras se incluyen con su carpeta:

```c
#include "can_protocol/can_ids.h"
```

## Flujo de trabajo

- `main`: versiones estables, las que usan las placas en ensayos y vuelo.
- `develop`: integración del trabajo en curso.
- Una rama por issue (`N-descripcion`), creada desde `develop` y integrada
  con un pull request a `develop`.

## Reglas para modificar el protocolo

1. Todo cambio va por **pull request a `develop`**.
2. El cambio en el código y en el [`ICD.md`](ICD.md) van **en el mismo PR**.
   Si no coinciden, manda el ICD.
3. Si el cambio afecta a las tramas, se **sube la versión** (ver ICD, sección 1)
   y se anota en el historial del ICD.
4. **Cada ID tiene un único nodo transmisor.**
5. Los datos siguen las reglas de codificación del ICD (sección 3):
   little-endian, tipos de tamaño fijo, `float32` IEEE-754 y serialización
   campo a campo, nunca con `memcpy` de structs.
6. Las placas no deben editar el protocolo dentro de su submodule: los cambios
   se hacen en un clon de este repositorio y luego se actualiza el submodule.
