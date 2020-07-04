#ifndef SOCHELPER_H_
#define SOCHELPER_H_

static float defaultOcvTable[] = { 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0, 4.1, 4.2 };
static float defaultSocTable[] = { 1, 7, 10, 15, 18, 25, 30, 40, 51, 60, 74, 84, 95, 98, 100};
static const int defaultTableSize = 15;

float lookupSOCByOCV(float restedOcvInput, float* ocvTable, int tableSize, float* socTable);

#endif /* SOCHELPER_H_ */
