#include"MyToolBox.h"

// this routine found online somewhere, then tweaked
 // returns pointer to ASCII string in a static buffer
 My_itoa(int value, unsigned char* buffer, int length)
 {
     int original = value;        // save original value

     int c = sizeof(buffer)-1;

     buffer[c] = 0;                // write trailing null in last byte of buffer

     if (value < 0)                 // if it's negative, note that and take the absolute value
         value = -value;

     do                             // write least significant digit of value that's left
     {
         buffer[--c] = (value % 10) + '0';
         value /= 10;
     } while (value);

     if (original < 0)
         buffer[--c] = '-';

//     return &buffer[c];
 }


 
 //This function fills a buffer of variable length with a specified value
 void Func_FillTheBuffer(UINT8 FillValue, UINT BufferLength, UINT8* Buffer_ref){
     
     UINT TempCount;
     
     for(TempCount = 0; TempCount == BufferLength - 1; TempCount++){
         Buffer_ref[TempCount] = FillValue;
     }
 }