#include <ch.h>
#include <hal.h>
#include "stdutil.h"
#include "led_blink.hpp"
#include <stdnoreturn.h>
#ifdef TRACE
#include <strings.h>
#endif

LedBlink ledBlink(LINE_LED_GREEN, 1000, 0, 200);

uint8_t LedBlink::indexer = 0;

void  LedBlink::launchThread(void)
{
  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO,
		    [] (void *arg) {
		      const LedBlink *led = static_cast<LedBlink *>(arg);
		      led->thdBlinkLed();
		    },
		    (void *) this);
}


_Noreturn void LedBlink::thdBlinkLed (void) const
{
  //  const LedBlink *ledBlink = (LedBlink *) arg;

  // give uniq name to threads if several leds are used
#ifdef TRACE
  *(index(name, '#')) = indexer++;
  chRegSetThreadName (name);
#endif
  
  while (true) {
    for (uint32_t i=0; i< seq1Flashes; i++) {
      palToggleLine(ledLine);
      chThdSleepMilliseconds(msBetweenFlashes);
    }
    palClearLine(ledLine);
    chThdSleepMilliseconds(msBetweenSeq1Seq2);

    for (uint32_t i=0; i< seq2Flashes; i++) {
      palToggleLine(ledLine);
      chThdSleepMilliseconds(msBetweenFlashes);
    }
    palClearLine(ledLine);
    chThdSleepMilliseconds(msBetweenSeqs);
  }
  
}
