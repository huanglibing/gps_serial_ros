/*
 * @Description: 
 * @Version: 
 * @Autor: Zeng Tianhao
 * @Date: 2021-09-03 09:14:08
 * @LastEditors: Zeng Tianhao
 * @LastEditTime: 2021-09-18 09:47:17
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

/*
    X forward
    Z left right
*/
void ControlDataPack(char *data, short X, short Y, short Z){
    data[0] = 0x7B;
    data[1] = 0;
    data[2] = 0;

    data[3] = X << 8;
    data[4] = X;

    data[5] = Y << 8;
    data[6] = Y;

    data[7] = Z << 8;
    data[8] = Z;

    data[9] = Checksum((unsigned char*)&data[0], 9);

    data[10] = 0x7D;
}

