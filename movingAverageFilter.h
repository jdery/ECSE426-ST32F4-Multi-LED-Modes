#ifndef MOVINGAVERAGEFILTER_H
#define MOVINGAVERAGEFILTER_H

void addToXBuffer(int angle);
int getXValue();
void addToYBuffer(int angle);
int getYValue();
void addToZBuffer(int angle);
int getZValue();
void addToTemperatureBuffer(uint32_t temperatureInCelcius);
uint32_t getTemperatureAverage();
void initializeFilter();

#endif