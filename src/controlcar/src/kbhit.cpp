/*
 * @Description: 
 * @Version: 
 * @Autor: Zeng Tianhao
 * @Date: 2021-09-17 16:06:28
 * @LastEditors: Zeng Tianhao
 * @LastEditTime: 2021-09-17 17:46:19
 */

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include "../include/kbhit.h"

int scanKeyboard(void){
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0, &stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);

    in = getchar();

    tcsetattr(0, TCSANOW, &stored_settings);
    return in;
}

//这个方法就可以，返回值是该键的ASCII码值，不需要回车的，
// int main(){
//     init_keyboard();
//     while (1)	{
//         kbhit();
//         printf("\n%d\n", readch());
//     }
//     close_keyboard();
//     return 0;
// }