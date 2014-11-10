/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Marco Tranzatto <marco.tranzatto@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file utilities.h
 *
 * Interface for common utility functions.
 *
 *
 * @author Marco Tranzatto <marco.tranzatto@gmail.com>
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

/** @brief Find string everywhere in buffer. */
int find_string_everywhere(const int start_index, const char *buffer, const int buffer_length, const char *str);

/** @brief Check if the strin is in the buffer starting from an exact position. */
int find_string_here(const int start_index, const char *buffer, const int buffer_length, const char *str);

/** @brief Extract double from string. */
bool d_extract_until_coma(int *index_pointer, const char *buffer, const int buffer_length, double *ret_val_pointer);

/** @brief Extract float from string. */
bool f_extract_until_coma(int *index_pointer, const char *buffer, const int buffer_length, float *ret_val_pointer);

/** @brief Extract int from string. */
bool i_extract_until_coma(int *index_pointer, const char *buffer, const int buffer_length, int *ret_val_pointer);

/** @brief Go to buffer until next ','. */
int jump_to_next_coma(const int start_index, const char *buffer, const int buffer_length);

/** @brief Jump n comas. */
int jump_to_next_n_coma(const int start_index, const char *buffer, const int buffer_length, int n);

/** @brief Print buffer on terminal */
void debug_print_nchar(const char *buffer, const int length, const int start, const int end);

/** @brief Print buffer on terminal */
void debug_print_until_char(const char *buffer, const int length, const int start, const char stop_char);

#endif /* UTILITIES_H_ */
