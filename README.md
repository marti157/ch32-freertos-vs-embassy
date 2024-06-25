### RTOS vs Embassy test

This is a simple experiment to test the duration of a sequence involving reading a byte through I2C in concurrent tasks. Both Embassy in Rust and FreeRTOS in C are used on the CH32V line of chips. The I2C interrupt mechanism is used for context switching.
