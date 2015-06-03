/************************************************************************
Title:    GARY Synthesizer - Backplane Firmware
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     main.c
Software: AVR-GCC 4.1, AVR Libc 1.4, libxnormidi
Hardware: Atmel ATMega328p
License:  GNU General Public License
Usage:
NOTES:
    Polyphony management functions are currently written to handle only two notes.
    MIDI note-on/note-off callbacks and mcp429x send function will have to be
    rewritten to support POLY > 2.
LICENSE:
    Copyright (C) 2015 Nicholas Morrow

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

************************************************************************/

/**********************    INCLUDE DIRECTIVES    ***********************/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "xnormidi/serial_midi.h"

/*********************    CONSTANTS AND MACROS    **********************/

#define V_MIN 0
#define V_MAX
#define POLY 2
#define PITCH_BEND_RANGE 1 // Pitch bend range, in steps //
#define RETRIG 1 // Retrigger gate on each new note?


#define SPI_DOUBLE_MODE 0

/*MCP492x CONFIGURATION PARAMETERS */
#define MCP_INPUT_BUFFER 0  // If active, VOUT_MAX = VDD - 0.040V.
#define MCP_2X_GAIN      0

/* PIN DEFINITIONS */
#define MIDI_RX_LED_PORT PORTB
#define MIDI_RX_LED_PIN  PB0
#define GATE_0_PORT PORTC
#define GATE_0_PIN  PC4
#define GATE_1_PORT PORTC
#define GATE_1_PIN  PC5
/* MIDI-RELATED DEFINITIONS */
#define USART_BAUDRATE 31250
#define MIDI_CLOCK_SCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define MIDI_RCV_CHAN 0



/***********************    GLOBAL VARIABLES    ************************/

/* Data structure to assist with polyphony management */
struct active_note {
    unsigned char val;  // MIDI note value
    //uint16_t conv;      // MCP device PORT|PIN, PORT on high byte
    unsigned char adc;  // 0 to select ADC a, 1 to select ADC b
};

/* Circular buffer to manage note polyphony */
struct active_note poly_buf[POLY];

/* Lookup table for MIDI note / MCP492x command translation */
uint16_t midi_note_table[128];

/* Counter to keep track of how many notes are being played */
uint8_t poly_count = 0;

MidiDevice *midi_device;


/*******************    FUNCTION IMPLEMENTATIONS    ********************/

/*************************************************************************
 * Function :    set_gate()
 * Purpose  :    Turns the gate on/off
 * Input    :    unsigned char gate - Which gate to set
 *               unsigned char state - Boolean value (0 = off, 1 = on)
 * Returns  :    void
 *************************************************************************/
void set_gate(unsigned char gate, unsigned char state)
{
    if (!gate) {
        GATE_0_PORT &= ~_BV(GATE_0_PIN);
        /* Toggle pin to reset if state should be high */
        if (state) {GATE_0_PORT |= _BV(GATE_0_PIN);}
    } else {
        GATE_1_PORT &= ~_BV(GATE_1_PIN);
        /* Toggle pin to reset if state should be high */
        if (state) {GATE_1_PORT |= _BV(GATE_1_PIN);}
    }
}

/*************************************************************************
 * Function :   mcp492x_write_command()
 * Purpose  :   What does this function do?
 * Input    :   uint16_t ss       : MCP converter Slave select PORT|PIN, PORT on high byte
 *              unsigned char adc : 0 to select ADC a, 1 to select ADC b
 *              uint16_t data     : Value to be written to MCP
 * Returns  :   void
 *************************************************************************/
void mcp492x_write_command(uint16_t ss, unsigned char adc, uint16_t data)
{
    /* Make sure variable is initialized to zero */
    uint16_t command = 0;
    /* Zero out high four bits of data byte and combine with command byte */
    command |= (data & 0xFFF);
    /* Set MCP control values*/
    command = (MCP_INPUT_BUFFER == 1) ? (command | _BV(14)) : (command & ~_BV(14));
    command = (MCP_2X_GAIN == 1) ? (command | _BV(13)) : (command & ~_BV(13));
    command = (adc = 1) ? (command | _BV(15)) : (command & ~_BV(15));
    /* Activate MCP492x output amplifiers */
    command |= _BV(12);
    /* Load high byte into SPI output buffer */
    SPDR = (command >> 8);
    /* Wait until transmission is complete */
    while (!(SPSR & _BV(SPIF)));
    /* Load low byte into SPI output buffer */
    SPDR = (command & 0xFF);
    /* Wait until transmission is complete */
    while (!(SPSR & _BV(SPIF)));

    return;
}

ISR(USART_RX_vect)
{
    uint8_t inByte = UDR0;
    midi_device_input(midi_device, 1, &inByte);
}

/*************************************************************************
 * Function :   noteon_callback()
 * Purpose  :   Handles MIDI note-on events
 * Input    :   MidiDevice* device,
*               uint8_t chan - MIDI channel,
*               uint8_t num  - MIDI note number,
*               uint8_t amt  - Note velocity
 * Returns  :   void
 *************************************************************************/
void noteon_callback(MidiDevice *device, uint8_t chan, uint8_t num,
                     uint8_t vel)
{
    /* Reset Gate_0 to active */
    if (((poly_count == 0) || RETRIG)) {set_gate(0, 1);}
    if (poly_buf[1].val != 0) {
        /*** TWO NOTES CURRENTLY PLAYING ***/

        /* Copy information from oldest note */
        unsigned char tmpadc = poly_buf[1].adc;
        //uint16_t tmpconv = poly_buf[1].conv;

        /* If both slots are used, turn off oldest note */
        /* mcp492x_write_command(poly_buf[1].conv, poly_buf[1].adc, 0); */
        mcp492x_write_command(0, poly_buf[1].adc, 0);
        /* Steal the adc used by the oldest note for the newest note & play new note*/
        mcp492x_write_command(0, poly_buf[1].adc, midi_note_table[num]);
        /* Copy data from second-oldest note to oldest-note slot */
        poly_buf[1] = poly_buf[0];
        /* Write new note data into second-oldest note slot */
        poly_buf[0].val = midi_note_table[num];
        poly_buf[0].adc = tmpadc;
        //poly_buf[0].conv = tmpconv;
    } else {
        if (poly_buf[0].val != 0) {
            /*** ONE NOTE ALREADY PLAYING ***/

            /* Play the new note on the free adc */
            mcp492x_write_command(0, !(poly_buf[0].adc), midi_note_table[num]);
            /* Copy data from second-oldest note to oldest-note slot */
            poly_buf[1] = poly_buf[0];
            /* Write new note data to second-oldest note slot */
            poly_buf[0].val = num;
            poly_buf[0].adc = !(poly_buf[1].adc);
            //poly_buf[0].conv = ??
            poly_count ++;
        } else {
            /*** NO NOTES CURRENTLY PLAYING ***/

            /* Play note on ADC a */
            mcp492x_write_command(0, 0, midi_note_table[num]);
            /* Write new note data to second-oldest note slot */
            poly_buf[0].val = num;
            poly_buf[0].adc = 0;
            //poly_buf[0].conv = ??
            poly_count++;

        }
    }
    /* Toggle MIDI RX LED */
    MIDI_RX_LED_PORT ^= _BV(MIDI_RX_LED_PIN);
}

/*************************************************************************
* Function :   noteoff_callback()
* Purpose  :   Handles MIDI note-off events
* Input    :   MidiDevice* device,
*              uint8_t chan - MIDI channel,
*              uint8_t num  - MIDI note number,
*              uint8_t amt  - Aftertouch amount
* Returns  :   void
*************************************************************************/
void noteoff_callback(MidiDevice *device, uint8_t chan, uint8_t num,
                      uint8_t amt)
{
    uint8_t i, match = 0;
    for (i = 0 ; i < POLY; i++ ) {
        if (poly_buf[i].val == num) { break; }
    }
    if (!match) { return; } // Note not found in buffer. Erroneous noteoff msg?
    else {
        /* Turn off the requested note and clear*/
        mcp492x_write_command(0, poly_buf[i].adc, 0);
        /* Update the note ordering in the poly buffer */
        if (i == 0) {
            poly_buf[0] = poly_buf[1];
            poly_buf[1].val = 0;
            poly_buf[1].adc = 0;
            /*poly_buf[1].conv = 0;*/
        } else {
            poly_buf[1].val = 0;
            poly_buf[1].adc = 0;
            //poly_buf[1].conv = 0;
            /*Reordering not necessary in this case for POLY = 2*/
        }
    }
    /* Toggle MIDI RX LED */
    MIDI_RX_LED_PORT ^= _BV(MIDI_RX_LED_PIN);
    poly_count = (poly_count > 0) ? (poly_count - 1) : 0;

    /* Turn off Gate 0 if no notes are being played */
    if (poly_count == 0) {set_gate(0, 0);}
}

/*************************************************************************
 * Function :    pitchbend_callback()
 * Purpose  :    Handles MIDI pitch bend events
 * Input    :    MidiDevice* device,
 *               uint8_t chan - MIDI channel,
 *               int16_t amt  - pitch bend amount [-8192,8192]
 * Returns  :    void
 *************************************************************************/
void pitchbend_callback(MidiDevice *device, uint8_t chan, int16_t amt)
{
    uint8_t i; int16_t adj;

    /* Calculate pitch bend amount and add to value for all currently playing notes
        Formula to calculate pitch bend = PITCH_BEND_RANGE * (4096/128)*(amt/8192)*/
    for (i = 0; i < POLY; i++) {
        adj = (midi_note_table[poly_buf[i].val] + (PITCH_BEND_RANGE * (amt / 256)));
        mcp492x_write_command(0 /*poly_buf[i].conv */, poly_buf[i].adc, adj);
    }
}

/*************************************************************************
 * Function :    init_gpio()
 * Purpose  :    Initializes all GPIO registers
 * Input    :    void
 * Returns  :    void
 *************************************************************************/
void init_gpio(void)
{
    /* Configure MIDI_RX_LED, SS', SCK, MOSI as outputs */
    DDRB |= (_BV(PB0) | _BV(PB2) | _BV(PB3) | _BV( PB5));
    /* Configure GPIO[0,3] , GATE[0,1] as outputs */
    DDRC |= (_BV(PC0) | _BV(PC1) | _BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5));
    /* Configure PWM[0,1] CV_1_CS as outputs */
    DDRD |= (_BV(PD5) | _BV(PD6) | _BV(PD7));
}

/*************************************************************************
 * Function :    init_spi()
 * Purpose  :    Initializes SPI peripheral
 * Input    :    void
 * Returns  :    void
 *************************************************************************/
void init_spi(void)
{
    /* Initialize SPI */

    /*  Clock Polarity = Active High, Clock Phase = Sample on first rising edge,
        Data Order = MSB first */
    SPCR &= ~(_BV(CPOL) | _BV(CPHA) | _BV( DORD));
    /*  Enable SPI Interrupts,Enable SPI Peripheral, SPI in Master Mode
        SCK Freq = FOSC/4 */
    SPCR |= (_BV(SPIE) | _BV(SPE) | _BV(MSTR) | _BV(SPR0) | \
             _BV(SPR1));
    /* Set SPI2X bit in SPSR if SPI_DOUBLE_MODE defined */
    SPSR = (SPI_DOUBLE_MODE == 1) ? (SPSR | _BV(SPI2X)) : (SPSR & ~_BV(SPI2X));
    return;
}

int main(void)
{
    cli();
    init_gpio();
    init_spi();

    /* Setup MIDI device and register callbacks */
    midi_device = serial_midi_init_usart0(MIDI_CLOCK_SCALE, 1, 1);
    midi_register_noteon_callback(midi_device, noteon_callback);
    midi_register_noteoff_callback(midi_device, noteoff_callback);
    midi_register_pitchbend_callback(midi_device, pitchbend_callback);

    /* Populate lookup table for MIDI note / MCP492x command translation */
    for (int i = 0; i < 128;  i++) {midi_note_table[i] = 4096 / 128 * i;}

    sei();
    /* MAIN LOOP */
    while (1) {
        midi_device_process(midi_device);
    }
    return -1;
}