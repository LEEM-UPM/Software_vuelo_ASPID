/*
 * Banco de velocidad de la SD1: cuánto se puede escribir, a qué ritmo y,
 * sobre todo, CUÁNTO BLOQUEA cada escritura.
 *
 * Lo segundo es lo que importa para el registro de vuelo: la SD se escribe
 * desde el mismo bucle que el GNC a 400 Hz (2,5 ms por tick) y todo el
 * acceso es bloqueante. Una escritura que tarde 30 ms son 12 ticks del GNC
 * parados, y el GNC solo recupera hasta 8 (GNC_MAX_RETRASO_TICKS).
 *
 * Qué hace, en este orden (escribe SDBENCH.BIN y lo borra al final):
 *
 *  1. Barrido de velocidades del bus: 625 kHz (la de hoy), 1,25, 2,5, 5, 10
 *     y 20 MHz. En cada una escribe a trozos de 512 B y de 16 KB, mide la
 *     velocidad y lo que tarda cada llamada, RELEE Y COMPARA todo (a más
 *     velocidad, más fácil que el cableado corrompa bits). Se para en la
 *     primera velocidad que falle.
 *  2. Escritura sostenida en la más rápida que pasó: 8 MB a trozos de 16 KB,
 *     con un fx_media_flush cada 1 MB, con y sin reservar el archivo antes
 *     (fx_file_allocate). Saca un histograma de bloqueos: las tarjetas
 *     tienen picos de cientos de ms cuando reorganizan por dentro, y solo
 *     se ven escribiendo bastante.
 *  3. Simulación del registro: 20 s a 400 Hz con un registro de 256 B por
 *     tick (100 KB/s, "todo tipo de datos"), escribiendo en bloques de 16 KB
 *     y con un flush por segundo. Cuenta cuántos ticks llegarían tarde y
 *     por cuánto.
 *
 * Al terminar deja el bus como estaba y el informe en SDBENCH.TXT (y por la
 * consola USB mientras se ejecuta). Bloquea el arranque 1-2 minutos.
 */
#ifndef SD_BENCH_H
#define SD_BENCH_H

/* A 1, flight_software lanza el banco al arrancar, después de montar la SD.
 * Dejar a 0 para volar. */
#ifndef SD_BENCH_ENABLE
#define SD_BENCH_ENABLE 0
#endif

void sd_bench_run(void);

#endif /* SD_BENCH_H */
