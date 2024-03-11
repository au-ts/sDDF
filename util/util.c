#include <microkit.h>
#include <sddf/util/printf.h>

void _putchar(char character)
{
    microkit_dbg_putc(character);
}