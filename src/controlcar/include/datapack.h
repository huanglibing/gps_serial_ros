/*
 * @Description: 
 * @Version: 
 * @Autor: Zeng Tianhao
 * @Date: 2021-09-03 09:14:51
 * @LastEditors: Zeng Tianhao
 * @LastEditTime: 2021-09-18 09:27:59
 */
#ifndef _DATAPACK_H_
#define _DATAPACK_H_

#define ROBOT_ID                0x00

#define CMD_A1SIZE              32
#define CMD_MOVECTRL            0xA1

void ControlDataPack(char* data, short X, short Y, short Z);
void A1DataPack(unsigned char* data, int move);

#endif