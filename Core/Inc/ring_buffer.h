/*
 * ring_buffer.h
 *
 *  Created on: Nov 18, 2024
 *      Author: klim405
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#include <stddef.h>
#include <stdbool.h>

/**
 * @brief Определение кольцевого буфера.
 */
typedef struct __RingBuffer {
	char* start;
	char* stop;
	volatile char* read_ptr;
	volatile char* write_ptr;
	volatile bool isFull;
} RingBuffer;

void RingBuffer_Init(RingBuffer* rBuff, char* start, size_t size);

void RingBuffer_Clear(RingBuffer* rBuff);

size_t RingBuffer_Write(RingBuffer* rBuff, char* buff, size_t size);

size_t RingBuffer_Read(RingBuffer* rBuff, char* buff, size_t size);

bool RingBuffer_IsEmpty(RingBuffer* rBuff);

#endif /* INC_RING_BUFFER_H_ */
