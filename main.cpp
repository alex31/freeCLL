#include <ch.h>
#include <hal.h>
#include "globalVar.h"
#include "stdutil.h"
#include "hal_stm32_dma.h"
#include "ttyConsole.hpp"
#include "serial_link.hpp"
#include "castel_link.hpp"

/*

  ° connecter PA08 et PA05 ensemble -> entrée PWM du controller castellink

 */


/*
  ° architecture :

  * un (mailBox + memory chunk manager) pour envoyer les 11 int16_t (contiennent des longueurs en µs ou .1µs)
  * la cb de l'ICU recupère les valeurs et les envoie dans la mailbox

  * un fichier castellink_pwm
    ° conf ICU et PWM
    ° callback, etc
    ° fonction start
    ° fonction setDutyLen

    * un fichier serialLink
      ° gestion I/O

    * utilisation de  simpleSerialMessage (ou une version c++ parametrisable)
      ° 115200 bauds 8 bits sans parité, 1 bit de stop
      ° sortie : payload -> 12 floats (53 octets à 50Hz) 2650 octets/seconde sur 11500 possible : 25%
      ° entree : 1 octet msgId, 2 octets valeur 
	+ msgId 1 à 4 : valeur longueur du creneau PWM en µs
      ° classification des
    

 */

static constexpr DMAConfig dmaConfig = {
  stream : STM32_ICU2_CH1_DMA_STREAM,
  {request : STM32_ICU2_CH1_DMA_REQUEST},
  inc_peripheral_addr : false,
  inc_memory_addr : true,
  circular : false,
  end_cb : nullptr,
  error_cb : nullptr,
  direction : DMA_DIR_P2M,
  dma_priority : STM32_ICU2_CH1_DMA_PRIORITY,
  irq_priority : STM32_ICU2_CH1_DMA_IRQ_PRIORITY,
  psize : 4,
  msize : 4,
};


static THD_WORKING_AREA(waBlinker, 384);
static void blinker (void *arg)
{
  (void)arg;
  chRegSetThreadName("blinker");
  palToggleLine(LINE_B03_LED_GREEN);
  chThdSleepMilliseconds(500);
}


void _init_chibios() __attribute__ ((constructor(101)));
void _init_chibios() {
  halInit();
  chSysInit();
  initHeap();
}


int main(void) {

    /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */

  
  consoleInit();
  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, &blinker, NULL);

  consoleLaunch();  
  
  // main thread does nothing
  chThdSleep (TIME_INFINITE);
}


