/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef GAS_INDEX_H
#define GAS_INDEX_H

// Sampling interval fed to the Sensirion Gas Index Algorithm.
// MAX wakes every ~3 minutes, so the algorithm is processed once per cycle.
// NOTE: Not the ideal 1s/10s interval; server does post-processing.
#define GAS_INDEX_SAMPLING_INTERVAL_SEC 180.0f

class GasIndex {
public:
  GasIndex() {}
  ~GasIndex() {}

  /**
   * Initialize the VOC and NOx algorithm states.
   * On first boot, states are (re)initialized. On wake from deep sleep the
   * states are already restored from RTC memory, so this is a no-op.
   */
  void init(bool firstBoot);

  /**
   * Convert raw SGP41 signals to VOC/NOx index values.
   * Invalid raw inputs (DEFAULT_INVALID_*) are skipped to avoid poisoning the
   * estimator, and the corresponding index output is set to invalid.
   */
  void process(int tvocRaw, int noxRaw, int *tvocIndex, int *noxIndex);

private:
  const char *const TAG = "GasIndex";
};

#endif // !GAS_INDEX_H
