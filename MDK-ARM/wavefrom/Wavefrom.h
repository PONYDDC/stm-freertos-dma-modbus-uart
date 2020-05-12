#ifndef _wavefrom_H
#define  _wavefrom_H
#include "main.h"
#include "cmsis_os.h"

void Tichabuxie_bufa(uint8_t num,uint8_t T,uint16_t Power,uint8_t Chip_choice);
void Tichabuxie_xiefa(uint8_t num,uint8_t T,uint16_t Power,uint8_t Chip_choice);
void nianzhuanbuxie_bufa(uint8_t num,uint8_t T,uint16_t Power,uint8_t Chip_choice);
void nianzhuanbuxie_xiefa(uint8_t num,uint8_t T,uint16_t Power,uint8_t Chip_choice);
void pingbupingxie(uint8_t num,uint8_t T,uint16_t Power,uint8_t Chip_choice);
void xujibuxie_bufa(uint8_t num,uint8_t T, uint16_t  Power,uint8_t Chip_choice);
void xujibuxie_xiefa(uint8_t num,uint8_t T, uint16_t  Power,uint8_t Chip_choice);

void wavefrom_choice(uint16_t  num);
void square_wave2(uint8_t T,uint8_t i,uint8_t dianjiA,uint8_t dianjiB);
void tiaozhi_wavefrom(void );
#endif


