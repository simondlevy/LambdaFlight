#include <pmw3901.hpp>
void PMW3901::spi_begin_transaction(void)
{
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
}

void PMW3901::spi_write_byte(const uint8_t byte)
{
    uint8_t b = byte;

    SPI.transfer(&b, 1);
}

uint8_t PMW3901::spi_read_byte(void)
{
    uint8_t byte = 0;
    
    SPI.transfer(&byte, 1);

    return byte;
}

void PMW3901::spi_end_transaction(void)
{
    SPI.endTransaction();
}


