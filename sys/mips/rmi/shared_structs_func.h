/*-
 * Copyright (c) 2003-2009 RMI Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of RMI Corporation, nor the names of its contributors,
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * RMI_BSD */
/* DO NOT EDIT THIS FILE
 * This file has been autogenerated by ./gen_struct_offsets
 */
#ifndef _SHARED_STRUCTS_FUNC_H
#define _SHARED_STRUCTS_FUNC_H

/* struct boot1_info function prototypes */
#define boot1_info_uart_print_func(info_ptr, ...) ((void (*)(const char *, ...))(unsigned long)(info_ptr->uart_print))( __VA_ARGS__ )
#define boot1_info_led_output_func(info_ptr, ...) ((void (*)(int))(unsigned long)(info_ptr->led_output))( __VA_ARGS__ )
#define boot1_info_init_func(info_ptr, ...) ((void (*)(void))(unsigned long)(info_ptr->init))( __VA_ARGS__ )
#define boot1_info_exit_func(info_ptr, ...) ((void (*)(void))(unsigned long)(info_ptr->exit))( __VA_ARGS__ )
#define boot1_info_warm_reset_func(info_ptr, ...) ((void (*)(void))(unsigned long)(info_ptr->warm_reset))( __VA_ARGS__ )
#define boot1_info_wakeup_func(info_ptr, ...) ((int (*)(void *, void *, unsigned int))(unsigned long)(info_ptr->wakeup))( __VA_ARGS__ )
#define boot1_info_master_reentry_fn_func(info_ptr, ...) ((void (*)(void *))(unsigned long)(info_ptr->master_reentry_fn))( __VA_ARGS__ )
#define boot1_info_slave_reentry_fn_func(info_ptr, ...) ((void (*)(void *))(unsigned long)(info_ptr->slave_reentry_fn))( __VA_ARGS__ )
#define boot1_info_uart_putchar_func(info_ptr, ...) ((void (*)(char))(unsigned long)(info_ptr->uart_putchar))( __VA_ARGS__ )
#define boot1_info_uart_getchar_func(info_ptr, ...) ((char (*)(void))(unsigned long)(info_ptr->uart_getchar))( __VA_ARGS__ )
#define boot1_info_malloc_func(info_ptr, ...) ((void *(*)(size_t))(unsigned long)(info_ptr->malloc))( __VA_ARGS__ )
#define boot1_info_free_func(info_ptr, ...) ((void (*)(void *))(unsigned long)(info_ptr->free))( __VA_ARGS__ )
#define boot1_info_alloc_pbuf_func(info_ptr, ...) ((struct packet *(*)(void))(unsigned long)(info_ptr->alloc_pbuf))( __VA_ARGS__ )
#define boot1_info_free_pbuf_func(info_ptr, ...) ((void (*)(struct packet *))(unsigned long)(info_ptr->free_pbuf))( __VA_ARGS__ )
#define boot1_info_wakeup_os_func(info_ptr, ...) ((int (*)(void *, void *, unsigned int))(unsigned long)(info_ptr->wakeup_os))( __VA_ARGS__ )


#endif
