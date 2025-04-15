#include "AirgradientSerial.h"

AirgradientSerial::AirgradientSerial() {}

AirgradientSerial::~AirgradientSerial() {}

bool AirgradientSerial::open(int port, int baud, int rx, int tx) { return false; }

void AirgradientSerial::close() {}

int AirgradientSerial::available() { return 0; }

void AirgradientSerial::print(const char *str) {}

int AirgradientSerial::write(const uint8_t *data, int len) { return 0; }

uint8_t AirgradientSerial::read() { return 0; }
