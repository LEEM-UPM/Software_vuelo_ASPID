/*
 * Banco PIL del GNC v1.7: inyecta en los modelos los vectores de la entrega
 * (GNC_V1_7/pil) y comprueba que la placa saca lo mismo que sacó el PC al
 * generarlos.
 *
 * Son 75805 filas a 416 Hz (182,2 s: el vuelo entero, del despegue al suelo,
 * con drogue y main): nav y máquina de estados en cada fila, y el guiado una
 * de cada 8 (9476 filas a 52 Hz), intercalados como en vuelo. Es la única forma de ejercitar aquí la
 * máquina de estados y el guiado: en el banco el cohete nunca ve una
 * aceleración de lanzamiento.
 *
 * Los datos van en la SD1, en binario de registro fijo. Se preparan con
 * tools/pil_csv_to_bin.py (que saca los campos de PIL_manifest.txt) y se
 * copian a la raíz de la tarjeta:
 *
 *   PILNAVI.BIN / PILNAVE.BIN    entradas y salidas esperadas de nav
 *   PILSMI.BIN  / PILSME.BIN     idem state_machine
 *   PILGCI.BIN  / PILGCE.BIN     idem gc
 *
 * y la prueba deja en la misma tarjeta:
 *
 *   PILNAVO.CSV (una fila de cada 4), PILSMO.CSV, PILGCO.CSV   lo que ha
 *                                         sacado la placa
 *   PILRES.TXT                            el informe, igual que por USB
 *
 * IMPORTANTE. Es una prueba secuencial: un *_initialize() y un *_step() por
 * fila, en orden. Los modelos tienen estado interno, así que saltarse o
 * reordenar filas invalida todo lo que venga detrás. Por eso el informe da
 * la PRIMERA discrepancia de cada modelo: las siguientes son arrastre.
 * Tolerancias del manifiesto: reales |a-b| <= 1e-3 + 1e-4*|b|, enteros
 * exactos.
 *
 * Los vectores ya vienen en los ejes del modelo (X morro, Y derecha, Z abajo),
 * así que aquí NO se aplica el giro de montaje de gnc.c: se escribe
 * directamente sobre las estructuras de entrada de los modelos, con los
 * ajustables de fábrica.
 *
 * Al terminar se vuelve a llamar a gnc_init(), porque la prueba deja los tres
 * modelos con el estado del final del vuelo simulado.
 */
#ifndef PIL_TEST_H
#define PIL_TEST_H

#include <stdbool.h>
#include <stdint.h>

/* A 1, flight_software lanza la prueba al arrancar, después de montar la SD.
 * Bloquea el bucle principal mientras dura. Dejar a 0 para volar. */
#ifndef PIL_TEST_ENABLE
#define PIL_TEST_ENABLE 0
#endif

typedef struct {
  bool ran;            /* se llegó a ejecutar (archivos presentes) */
  bool passed;         /* los tres modelos dentro de tolerancia */
  uint32_t steps;      /* pasos ejecutados */
  uint32_t mismatches; /* señales fuera de tolerancia, sumando los 3 modelos */
  uint32_t duration_ms;
  uint32_t max_step_us; /* el paso más lento de los tres modelos juntos */
} pil_test_result_t;

/*
 * Ejecuta la prueba entera. Bloquea hasta terminar, informando del avance por
 * la consola USB. Si falta algún archivo en la tarjeta lo dice y no hace nada.
 */
void pil_test_run(void);

/* Resultado de la última ejecución (todo a cero si no se ha ejecutado). */
const pil_test_result_t *pil_test_get_result(void);

#endif /* PIL_TEST_H */
