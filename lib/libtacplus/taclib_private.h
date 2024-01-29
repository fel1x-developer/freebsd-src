/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 1998, 2001, Juniper Networks, Inc.
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
 */

#ifndef TACLIB_PRIVATE_H
#define TACLIB_PRIVATE_H

#include "taclib.h"

/* Defaults */
#define PATH_TACPLUS_CONF	"/etc/tacplus.conf"
#define TACPLUS_PORT		49
#define TIMEOUT			3	/* In seconds */

/* Limits */
#define BODYSIZE	8150		/* Maximum message body size */
#define ERRSIZE		128		/* Maximum error message length */
#define MAXCONFLINE	1024		/* Maximum config file line length */
#define MAXSERVERS	10		/* Maximum number of servers to try */
#define MAXAVPAIRS      255             /* Maximum number of AV pairs */

/* Protocol constants. */
#define HDRSIZE		12		/* Size of message header */

/* Protocol version number */
#define TAC_VER_MAJOR		0xc		/* Major version number */

/* Protocol packet types */
#define TAC_AUTHEN		0x01		/* Authentication */
#define TAC_AUTHOR		0x02		/* Authorization */
#define TAC_ACCT		0x03		/* Accouting */

/* Protocol header flags */
#define TAC_UNENCRYPTED		0x01
#define TAC_SINGLE_CONNECT	0x04

struct tac_str {
	char		*data;
	size_t		 len;
};

struct tac_authen_start {
	uint8_t	action;
	uint8_t	priv_lvl;
	uint8_t	authen_type;
	uint8_t	service;
	uint8_t	user_len;
	uint8_t	port_len;
	uint8_t	rem_addr_len;
	uint8_t	data_len;
	unsigned char	rest[1];
};

struct tac_authen_reply {
	uint8_t	status;
	uint8_t	flags;
	uint16_t	msg_len;
	uint16_t	data_len;
	unsigned char	rest[1];
};

struct tac_authen_cont {
	uint16_t	user_msg_len;
	uint16_t	data_len;
	uint8_t	flags;
	unsigned char	rest[1];
};

struct tac_author_request {
	uint8_t	authen_meth;
	uint8_t	priv_lvl;
	uint8_t	authen_type;
	uint8_t	service;
	uint8_t	user_len;
	uint8_t	port_len;
	uint8_t	rem_addr_len;
	uint8_t	av_cnt;
	unsigned char	rest[1];
};

struct tac_author_response {
	uint8_t	status;
	uint8_t	av_cnt;
	uint16_t	msg_len;
	uint16_t	data_len;
	unsigned char	rest[1];
};

struct tac_acct_start {
	uint8_t	action;
	uint8_t	authen_action;
	uint8_t	priv_lvl;
	uint8_t	authen_type;
	uint8_t	authen_service;
	uint8_t	user_len;
	uint8_t	port_len;
	uint8_t	rem_addr_len;
	uint8_t	av_cnt;
	unsigned char	rest[1];
};

struct tac_acct_reply {
	uint16_t	msg_len;
	uint16_t	data_len;
	uint8_t	status;
	unsigned char	rest[1];
};

struct tac_msg {
	uint8_t	version;
	uint8_t	type;
	uint8_t	seq_no;
	uint8_t	flags;
	uint8_t	session_id[4];
	uint32_t	length;
	union {
		struct tac_authen_start authen_start;
		struct tac_authen_reply authen_reply;
		struct tac_authen_cont authen_cont;
		struct tac_author_request author_request;
		struct tac_author_response author_response;
		struct tac_acct_start acct_start;
		struct tac_acct_reply acct_reply;
		unsigned char body[BODYSIZE];
	} u;
};

struct tac_server {
	struct sockaddr_in addr;	/* Address of server */
	char		*secret;	/* Shared secret */
	int		 timeout;	/* Timeout in seconds */
	int		 flags;
	unsigned int	 navs;
	struct tac_str	 avs[MAXAVPAIRS];
};

struct tac_handle {
	int		 fd;		/* Socket file descriptor */
	struct tac_server servers[MAXSERVERS];	/* Servers to contact */
	int		 num_servers;	/* Number of valid server entries */
	int		 cur_server;	/* Server we are currently using */
	int		 single_connect;	/* Use a single connection */
	int		 last_seq_no;
	char		 errmsg[ERRSIZE];	/* Most recent error message */

	struct tac_str	 user;
	struct tac_str	 port;
	struct tac_str	 rem_addr;
	struct tac_str	 data;
	struct tac_str	 user_msg;
	struct tac_str	 avs[MAXAVPAIRS];

	struct tac_msg	 request;
	struct tac_msg	 response;

	int		 srvr_pos;	/* Scan position in response body */
	unsigned int	 srvr_navs;
	struct tac_str	 srvr_msg;
	struct tac_str	 srvr_data;
	struct tac_str	 srvr_avs[MAXAVPAIRS];
};

#define is_alpha(ch) /* alphabetical */					\
	(((ch) >= 'A' && (ch) <= 'Z') || ((ch) >= 'a' && (ch) <= 'z'))
#define is_num(ch) /* numerical */					\
	((ch) >= '0' && (ch) <= '9')
#define is_alnum(ch) /* alphanumerical */				\
	(is_alpha(ch) || is_num(ch))
#define is_arg(ch) /* valid in an argument name */			\
	(is_alnum(ch) || (ch) == '_' || (ch) == '-')

#endif
