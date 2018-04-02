#pragma once
#include <ch.h>
#include <hal.h>


class LedBlink {
public:
  LedBlink(ioline_t _ledLine) : LedBlink(_ledLine, 2000, 500, 200)
  {};

  LedBlink(ioline_t _ledLine, uint16_t _msBetweenSeqs, uint16_t _msBetweenSeq1Seq2,
	   uint16_t _msBetweenFlashes)
    : ledLine(_ledLine),
      msBetweenSeqs{_msBetweenSeqs}, msBetweenSeq1Seq2{_msBetweenSeq1Seq2}, msBetweenFlashes{_msBetweenFlashes},
      seq1Flashes{1}, seq2Flashes{0}
  {launchThread();};
  
  void setFlashes(uint8_t _seq1Flashes, uint8_t _seq2Flashes)
  {seq1Flashes = _seq1Flashes; seq2Flashes = _seq2Flashes; };

private:
  void launchThread(void);
  THD_WORKING_AREA(waBlinker, 384);
  
  ioline_t ledLine;
  const uint16_t msBetweenSeqs;
  const uint16_t msBetweenSeq1Seq2;
  const uint16_t msBetweenFlashes;
  volatile uint8_t seq1Flashes;
  volatile uint8_t seq2Flashes;
#ifdef TRACE
  char name[11] = "blinkLed_#";
#endif
  void thdBlinkLed  (void) const;
  static uint8_t indexer;
};

extern LedBlink ledBlink;
