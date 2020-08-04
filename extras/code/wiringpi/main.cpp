#include <stdio.h>
#include <CrossPlatformI2C_Core.h>

int main(int argc, char ** argv)
{
    uint8_t i2c = cpi2c_open(0x57, 1);

    printf("%d\n", i2c);

    return 0;
}
