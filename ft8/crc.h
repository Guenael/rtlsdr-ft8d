/*
 * MIT License
 *
 * Copyright (c) 2018 Kārlis Goba
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef _INCLUDE_CRC_H_
#define _INCLUDE_CRC_H_

#include <stdint.h>
#include <stdbool.h>

// Compute 14-bit CRC for a sequence of given number of bits using FT8/FT4 CRC polynomial
// [IN] message  - byte sequence (MSB first)
// [IN] num_bits - number of bits in the sequence
uint16_t ftx_compute_crc(const uint8_t message[], int num_bits);

/// Extract the FT8/FT4 CRC of a packed message (during decoding)
/// @param[in] a91 77 bits of payload data + CRC
/// @return Extracted CRC
uint16_t ftx_extract_crc(const uint8_t a91[]);

/// Add FT8/FT4 CRC to a packed message (during encoding)
/// @param[in] payload 77 bits of payload data
/// @param[out] a91 91 bits of payload data + CRC
void ftx_add_crc(const uint8_t payload[], uint8_t a91[]);

#endif  // _INCLUDE_CRC_H_