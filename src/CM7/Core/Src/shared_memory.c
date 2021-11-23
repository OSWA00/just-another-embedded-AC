#include "shared_memory.h"

void send_to_M7(uint16_t buffer, volatile struct shared_data * const xfr_ptr)
{
    if (xfr_ptr->sts_4to7 == 0) // if M4 to M7 buffer is empty
	{
		xfr_ptr->sts_4to7 = 2; // lock the M4 to M7 buffer

		xfr_ptr->M4toM7 = buffer; // transfer data
	}
	xfr_ptr->sts_4to7 = 1; // M7 to M4 buffer has data
}

uint16_t *get_from_M4(volatile struct shared_data * const xfr_ptr)
{
    static uint16_t buffer; // buffer to receive data
	if (xfr_ptr->sts_4to7 == 1) // if M4 to M7 buffer has data
	{
		xfr_ptr->sts_4to7 = 2; // lock the M4 to M7 buffer
		for(int n = 0; n < 64; n++)
		{
			buffer = xfr_ptr->M4toM7; // transfer data
			xfr_ptr->M4toM7 = 0; // clear M4 to M7 buffer
		}
		xfr_ptr->sts_4to7 = 0; // M4 to M7 buffer is empty
	}
	return buffer; // return the buffer (pointer)
}