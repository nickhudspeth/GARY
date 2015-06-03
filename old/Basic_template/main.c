#include <avr/io.h>
#include <util/delay.h>
#define V_MIN 0
#define V_MAX

uint16_t midi_note_numbers[127];

/*************************************************************************
Function: init()
Purpose:  Performs all MCU initialization routines on startup
Input:    void
Returns:  void
**************************************************************************/
void init(void)
{
    for (int i = 0; i < 128;  i++)
    {
        midi_note_numbers[i] = 4096 / 128 * i;
    }
}

int main(void)
{
    init();
    for (;;)
    {
        _delay_ms(500);
    }
    return -1;
}