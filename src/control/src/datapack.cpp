/*
 * @Description: 
 * @Version: 
 * @Autor: Zeng Tianhao
 * @Date: 2021-09-03 09:14:08
 * @LastEditors: Zeng Tianhao
 * @LastEditTime: 2021-09-03 09:36:35
 */
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include "datapack.h"


static unsigned char Checksum(unsigned char* buf, int length){
    unsigned char index;
    unsigned short int tempResult = 0x0000;
    unsigned char checksum = 0;
    for (index = 0; index < length; index++){
        tempResult += buf[index];
    }
    checksum = (tempResult & 0xff);
    return checksum;
}

void A1DataPack(unsigned char* data, int move){
    data[0] = 0x55;
    data[1] = 0xAA;
    data[2] = ROBOT_ID;

    data[3] = CMD_A1SIZE - 5;
    data[4] = CMD_MOVECTRL;

    data[5] = move;

    data[CMD_A1SIZE - 1] = Checksum(&data[4], data[3]);
}

