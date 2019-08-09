#ifndef COMP
#define COMP

#include<stdio.h>
#include<unistd.h>
#include<stdint.h>

uint8_t Compress(uint16_t x){
    uint8_t sign;
    uint16_t linear;
    if (x & 0x8000){
        sign = 0x80;
        linear = ~x+1;
    }else{
        sign = 0;
        linear = x;
    }

    uint16_t temp = 0x4000;
    int i;
    for (i=0;i<7;i++){
        if ((temp>>i) & linear)
            break;
    }
    uint8_t high = (7-i) << 4;
    uint8_t low;
    if (i == 7)
        low = 0x0F & (linear >> 4);
    else
        low = 0x0F & (linear >> 10-i);;

    return (sign | high | low);
}


uint16_t Decompress(uint8_t x){
    uint16_t sign = x & 0x80;
    uint16_t high = (x>>4) & 0x07;
    uint16_t low = x & 0x0F;

    uint16_t linear;
    if (!high){
        linear = low << 4;
    }else{
        linear = low << (4+high-1);
        linear |= 0x0100 << (high-1);
    }

    uint16_t temp;
    if (sign){
        temp = ~linear+1;
    }else
        temp = linear;
    return ((temp & 0xFFFF)|(sign << 8));
}

#endif // COMP

