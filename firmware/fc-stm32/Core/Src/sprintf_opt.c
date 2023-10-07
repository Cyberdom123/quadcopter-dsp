#include "sprintf_opt.h"

/** 
    @brief normalize function to form 0.XXX * 10 ^ exponent
    exponent =  (int) (log(val) + 1)
 */
static int normalize(double *val) {
    int exponent = 0;
    double value = *val;

    while (value >= 1.0) {
        value /= 10.0;
        ++exponent;
    }

    while (value < 0.1) {
        value *= 10.0;
        --exponent;
    }
    *val = value;
    return exponent;
}


static void ftoa_fixed(char *buffer, double value) {  
    /* carry out a fixed conversion of a double value to a string, with a precision of 5 decimal digits. 
     * Values with absolute values less than 0.000001 are rounded to 0.0
     * Note: this blindly assumes that the buffer will be large enough to hold the largest possible result.
     * The largest value we expect is an IEEE 754 double precision real, with maximum magnitude of approximately
     * e+308. The C standard requires an implementation to allow a single conversion to produce up to 512 
     * characters, so that's what we really expect as the buffer size.     
     */

    int exponent = 0;
    int places = 0;
    static const int width = 4;

    if (value == 0.0) {
        buffer[0] = '0';
        buffer[1] = '\0';
        return;
    }         

    if (value < 0.0) {
        *buffer++ = '-';
        value = -value;
    } else {
        *buffer++ = '+';
    }

    exponent = normalize(&value);

    while (exponent > 0) {
        int digit = value * 10;
        //convert to char representing the digit
        *buffer++ = digit + '0';
        value = value * 10 - digit; //0.137
        ++places;
        --exponent;
    }

    if (places == 0)
        *buffer++ = '0';

    *buffer++ = '.';
    
    while (exponent < 0 && places < width) {
        *buffer++ = '0';
        --exponent;
        ++places;
    }

    while (places < width) {
        int digit = value * 10.0;
        *buffer++ = digit + '0';
        value = value * 10.0 - digit;
        ++places;
    }
    *buffer = '\0';
}


void sprintf_opt(char* buff, char const *fmt, ...){
    va_list args;
    va_start(args, fmt);


    double double_temp;
    char buff_temp[512];
    volatile char ch;
    int length = 0;

    char* from = buff_temp;
    char* to = buff;

    while ( (ch = *fmt++) != '\0')
    {
        if(ch == '%'){
            switch (ch = *fmt)
            {
            case '%':
                strcat(buff, "%");
                length++;
                break;
            case 'f':
                double_temp = va_arg(args, double);
                ftoa_fixed(buff_temp, double_temp);
                from = buff_temp;
                while( (*to++ = *from++) != '\0')
                    ;
                to--;
                fmt++;
                break;
            case 'd':
                int int_temp = va_arg(args, int);

                itoa(int_temp, buff_temp, 10);

                from = buff_temp;
                while( (*to++ = *from++) != '\0')
                    ;
                to--;
                fmt++;
                break;
            default:
                break;
            }
        } else {
            *to++ = ch;
        }
    }
    
    va_end(args);
}