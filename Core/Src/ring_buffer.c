/*
 * ring_buffer.c
 *
 *  Created on: Nov 18, 2024
 *      Author: klim405
 */
#include "ring_buffer.h"

/**
 * @brief Подготавливает сруктуру RingBuffer к работе
 * @param rBuff Указатель на структуру RingBuffer
 * @param start Указатель на начало массива, который будет использоватся для хранения значений буфера.
 * @param size Размер переданного масива в start, и соответсвенно размер буфера.
 */
void RingBuffer_Init(RingBuffer* rBuff, char* start, size_t size) {
	rBuff->start = start;
	rBuff->stop = start + size;
	RingBuffer_Clear(rBuff);
}

/**
 * @brief Очищает буфер
 * @param rBuff Указатель на структуру RingBuffer
 */
void RingBuffer_Clear(RingBuffer* rBuff) {
	rBuff->read_ptr = rBuff->start;
	rBuff->write_ptr = rBuff->start;
	rBuff->isFull = false;
}

/**
 * @brief Записывает данные в RingBuffer
 * @param rBuff Указатель на структуру RingBuffer
 * @param buff Указатель на буфер с данными, которые необходимо записать.
 * @param size Размер буфера с данными, которые необходимо записать.
 * @return Количество записанных данных в кольцевой буфер (RingBuffer).
 */
size_t RingBuffer_Write(RingBuffer* rBuff, char* buff, size_t size) {
	size_t i = 0; // Количество записанных байтов
	while (i < size) {
		if (rBuff->isFull) { // Заполнен ли буфер
			break;
		}
		if (++rBuff->write_ptr > rBuff->stop) { // Инкрементирум указатель и проверяем не дошли ли до конца массива
			rBuff->write_ptr = rBuff->start;    // Если дошли переходим на начало
		}
		if (rBuff->write_ptr == rBuff->read_ptr) { // Полностью заполнили буфер
			rBuff->isFull = true;
		}
		*(rBuff->write_ptr) = buff[i];
		i++;
	}
	return i;
}

/**
 * @brief Читает нужное количество байт из RingBuffer
 * @note После прочетения, заново прочитать эти данные нельзя. 
 *       Если данных в RingBuffer меньше чем size, то будут прочитаны все что есть.
 * @param rBuff Указатель на структуру RingBuffer
 * @param buff Указатель на буфер, в который будут записаны прочитанные данные.
 * @param size Количество байт, которое необходимо прочитать. Не должно быть больше размера buff.
 * @return Количество записанных данных в кольцевой буфер (RingBuffer).
 */
size_t RingBuffer_Read(RingBuffer* rBuff, char* buff, size_t size) {
	size_t i = 0; // Количество записанных байтов
	while (i < size) {
		if (!rBuff->isFull && rBuff->write_ptr == rBuff->read_ptr) { // Проверяем есть ли что читать
			break;
		}
		if (++rBuff->read_ptr > rBuff->stop) { // Инкрементирум указатель и проверяем не дошли ли до конца массива
			rBuff->read_ptr = rBuff->start;     // Если дошли переходим на начало
		}
		buff[i] = *(rBuff->read_ptr);
		++i;
		if (rBuff->isFull) {        // if для уменьшения блокировок переменной ?
			rBuff->isFull = false;
		}
	}
	return i;
}

/**
 * @brief Проверяет есть ли данные для чтения в RingBuffer.
 * @param rBuff Указатель на структуру RingBuffer
 * @return Есть ли данные для чтения.
 */
bool RingBuffer_IsEmpty(RingBuffer* rBuff) {
	return !rBuff->isFull && rBuff->write_ptr == rBuff->read_ptr;
}
