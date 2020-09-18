#ifndef POWER_H_
#define POWER_H_

///TODO preetham check what is the internal impedence of your panasonic cell and is that here
static const float ir_samsung25R = 20.0; // mohms
static const float uv_samsung25R = 2.8; //V
static const float peakCellCurr_samsung25R = 100; //A
static const float maxConstCellCurr_samsung25R = 20; //A

float computePower(float cellV);


#endif
