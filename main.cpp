#include <ch.h>
#include <hal.h>
#include "globalVar.h"
#include "stdutil.h"
#include "config.hpp"
#include "castel_link.hpp"
#include "ttyConsole.hpp"


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
      ° sortie : payload -> 11 floats + 1 short + protocole  :
      (50 octets à 5Hz) 250 octets/seconde sur 11500 possible : 3%
      ° entree : 1 octet msgId, 2 octets valeur 
	+ msgId 1 à 4 : valeur longueur du creneau PWM en µs
    

 */

static THD_WORKING_AREA(waBlinker, 384);
static void blinker (void *arg)
{
  (void)arg;
  chRegSetThreadName("blinker");
  palToggleLine(LINE_LED_GREEN);
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

  if constexpr (USE_SHELL) {
      consoleInit();
    } else {
#ifdef TRACE
    chSysHalt("-DTRACE involve USE_SHELL = true in config.hpp");
#endif
  }
  
  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, &blinker, NULL);
  castelLinkStart();
  
  if constexpr (USE_SHELL) {
      consoleLaunch();  
    }

  
  // main thread does nothing
  chThdSleep (TIME_INFINITE);
}


