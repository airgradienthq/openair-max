#include "AirgradientSerial.h"

AirgradientSerial::AirgradientSerial() {}

AirgradientSerial::~AirgradientSerial() {}

bool AirgradientSerial::begin(int baud) { return false; }

bool AirgradientSerial::begin(int baud, int iicResetIO) { return false; }

bool AirgradientSerial::begin(int port, int baud, int rx, int tx) { return false; }

void AirgradientSerial::end() {}

int AirgradientSerial::available() { return 0; }

void AirgradientSerial::print(const char *str) {}

int AirgradientSerial::write(const uint8_t *data, int len) { return 0; }

int AirgradientSerial::read() { return -1; }

void AirgradientSerial::setDebug(bool debug) { isDebug = debug; }
