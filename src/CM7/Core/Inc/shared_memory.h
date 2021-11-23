#include <stdint.h>

// inter-core buffers
struct shared_data
{
	uint8_t sts_4to7; // status: 0 = empty, 1 = has data, 2 = locked (CM4-CM7)
	uint16_t M4toM7; // 256 bytes from CM4 to CM7
};

/**
 * @brief Sends data from M4 to M7
 * 
 * @param buffer 256 bytes buffer
 */
void send_to_M7(uint16_t buffer, volatile struct shared_data * const xfr_ptr);

/**
 * @brief Reads data from shared memory buffer
 * 
 */
uint16_t *get_from_M4(volatile struct shared_data * const xfr_ptr);