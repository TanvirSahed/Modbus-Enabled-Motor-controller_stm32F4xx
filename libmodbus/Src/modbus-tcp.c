/*
 * Copyright © Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

// clang-format off
#if defined(_WIN32)
# define OS_WIN32
/* ws2_32.dll has getaddrinfo and freeaddrinfo on Windows XP and later.
 * minwg32 headers check WINVER before allowing the use of these */
# ifndef WINVER
#   define WINVER 0x0501
# endif
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include <string.h>
#include <errno.h>
#ifndef _MSC_VER
#include <unistd.h>
#endif
#include <signal.h>
#include <sys/types.h>

#if defined(_WIN32)
/* Already set in modbus-tcp.h but it seems order matters in VS2005 */
# include <winsock2.h>
# include <ws2tcpip.h>
# define SHUT_RDWR 2
# define close closesocket
# define strdup _strdup
#else
# include <socket.h>
#include "w5500_spi_handler.h"
//# include <sys/ioctl.h>

#if defined(__OpenBSD__) || (defined(__FreeBSD__) && __FreeBSD__ < 5)
# define OS_BSD
# include <netinet/in_systm.h>
#endif

//# include <netinet/in.h>
//# include <netinet/ip.h>
//# include <netinet/tcp.h>
//# include <arpa/inet.h>
//# include <netdb.h>
#endif

#if !defined(MSG_NOSIGNAL)
#define MSG_NOSIGNAL 0
#endif

#if defined(_AIX) && !defined(MSG_DONTWAIT)
#define MSG_DONTWAIT MSG_NONBLOCK
#endif
// clang-format on

#include "modbus-private.h"

#include "modbus-tcp-private.h"
#include "modbus-tcp.h"
#include <errno.h>


// Socket connection states for Modbus TCP
typedef enum {
    STATE_IDLE,
    STATE_LISTEN,
    STATE_ACCEPT,
    STATE_CONNECTED,
    STATE_CLOSE
} modbus_tcp_state_t;
modbus_tcp_state_t connection_state = STATE_IDLE;
uint8_t status;


#ifdef OS_WIN32
static int _modbus_tcp_init_win32(void)
{
    /* Initialise Windows Socket API */
    WSADATA wsaData;

    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        fprintf(stderr,
                "WSAStartup() returned error code %d\n",
                (unsigned int) GetLastError());
        errno = EIO;
        return -1;
    }
    return 0;
}
#endif

static int _modbus_set_slave(modbus_t *ctx, int slave) {
	int max_slave = (ctx->quirks & MODBUS_QUIRK_MAX_SLAVE) ? 255 : 247;

	/* Broadcast address is 0 (MODBUS_BROADCAST_ADDRESS) */
	if (slave >= 0 && slave <= max_slave) {
		ctx->slave = slave;
	} else if (slave == MODBUS_TCP_SLAVE) {
		/* The special value MODBUS_TCP_SLAVE (0xFF) can be used in TCP mode to
		 * restore the default value. */
		ctx->slave = slave;
	} else {
		errno = EINVAL;
		return -1;
	}

	return 0;
}

/* Builds a TCP request header */
static int _modbus_tcp_build_request_basis(modbus_t *ctx, int function,
		int addr, int nb, uint8_t *req) {
	modbus_tcp_t *ctx_tcp = ctx->backend_data;

	/* Increase transaction ID */
	if (ctx_tcp->t_id < UINT16_MAX)
		ctx_tcp->t_id++;
	else
		ctx_tcp->t_id = 0;
	req[0] = ctx_tcp->t_id >> 8;
	req[1] = ctx_tcp->t_id & 0x00ff;

	/* Protocol Modbus */
	req[2] = 0;
	req[3] = 0;

	/* Length will be defined later by set_req_length_tcp at offsets 4
	 and 5 */

	req[6] = ctx->slave;
	req[7] = function;
	req[8] = addr >> 8;
	req[9] = addr & 0x00ff;
	req[10] = nb >> 8;
	req[11] = nb & 0x00ff;

	return _MODBUS_TCP_PRESET_REQ_LENGTH;
}

/* Builds a TCP response header */
static int _modbus_tcp_build_response_basis(sft_t *sft, uint8_t *rsp) {
	/* Extract from MODBUS Messaging on TCP/IP Implementation
	 Guide V1.0b (page 23/46):
	 The transaction identifier is used to associate the future
	 response with the request. */
	rsp[0] = sft->t_id >> 8;
	rsp[1] = sft->t_id & 0x00ff;

	/* Protocol Modbus */
	rsp[2] = 0;
	rsp[3] = 0;

	/* Length will be set later by send_msg (4 and 5) */

	/* The slave ID is copied from the indication */
	rsp[6] = sft->slave;
	rsp[7] = sft->function;

	return _MODBUS_TCP_PRESET_RSP_LENGTH;
}

static int _modbus_tcp_prepare_response_tid(const uint8_t *req, int *req_length) {
	return (req[0] << 8) + req[1];
}

static int _modbus_tcp_send_msg_pre(uint8_t *req, int req_length) {
	/* Subtract the header length to the message length */
	int mbap_length = req_length - 6;

	req[4] = mbap_length >> 8;
	req[5] = mbap_length & 0x00FF;

	return req_length;
}

static ssize_t _modbus_tcp_send(modbus_t *ctx, const uint8_t *req,
		int req_length) {
	/* MSG_NOSIGNAL
	 Requests not to send SIGPIPE on errors on stream oriented
	 sockets when the other end breaks the connection.  The EPIPE
	 error is still returned. */
#if STM_HAL
	/*This function for STM32 Socket*/
	/*
	 * @param sn Socket number. It should be <b>0 ~ @ref \_WIZCHIP_SOCK_NUM_</b>.
	 * @param buf Pointer buffer containing data to be sent.
	 * @param len The byte length of data in buf.
	 */
	return send(ctx->s, (uint8_t*) req, req_length);
#else
    return send(ctx->s, (const char *) req, req_length, MSG_NOSIGNAL);
#endif
}

static int _modbus_tcp_receive(modbus_t *ctx, uint8_t *req) {
	return _modbus_receive_msg(ctx, req, MSG_INDICATION);
}

static ssize_t _modbus_tcp_recv(modbus_t *ctx, uint8_t *rsp, int rsp_length) {
#if STM_HAL
	/*This function for STM32 Socket
	 * @param sn Socket number. It should be <b>0 ~ @ref \_WIZCHIP_SOCK_NUM_</b>.
	 * @param buf Pointer buffer containing data to be sent.
	 * @param len The byte length of data in buf.
	 */
	return recv(ctx->s, (uint8_t*) rsp, rsp_length);
#else
    return recv(ctx->s, (char *) rsp, rsp_length, 0);
#endif
}

static int _modbus_tcp_check_integrity(modbus_t *ctx, uint8_t *msg,
		const int msg_length) {
	return msg_length;
}

static int _modbus_tcp_pre_check_confirmation(modbus_t *ctx, const uint8_t *req,
		const uint8_t *rsp, int rsp_length) {
	unsigned int protocol_id;
	/* Check transaction ID */
	if (req[0] != rsp[0] || req[1] != rsp[1]) {
		if (ctx->debug) {
			fprintf(stderr, "Invalid transaction ID received 0x%X (not 0x%X)\n",
					(rsp[0] << 8) + rsp[1], (req[0] << 8) + req[1]);
		}
		errno = EMBBADDATA;
		return -1;
	}

	/* Check protocol ID */
	protocol_id = (rsp[2] << 8) + rsp[3];
	if (protocol_id != 0x0) {
		if (ctx->debug) {
			fprintf(stderr, "Invalid protocol ID received 0x%X (not 0x0)\n",
					protocol_id);
		}
		errno = EMBBADDATA;
		return -1;
	}

	return 0;
}

//static int _modbus_tcp_set_ipv4_options(int s) {
//    int rc;
//    int option;
//
//    /* Set the TCP no delay flag */
//
//    /* SOL_TCP = IPPROTO_TCP */
//    option = 1;
//#if STM_HAL
//    rc = setsockopt(s,  SO_DESTIP, sizeof(int));
//#else
//    rc = setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &option, sizeof(int));
//#endif
//
//    if (rc == -1) {
//        return -1;
//    }
//
//    /* If the OS does not offer SOCK_NONBLOCK, fall back to setting FIONBIO to
//     * make sockets non-blocking */
//    /* Do not care about the return value, this is optional */
//#if !defined(SOCK_NONBLOCK) && defined(FIONBIO)
//#ifdef OS_WIN32
//    {
//        /* Setting FIONBIO expects an unsigned long according to MSDN */
//        u_long loption = 1;
//        ioctlsocket(s, FIONBIO, &loption);
//    }
//#else
//    option = 1;
//    ioctl(s, FIONBIO, &option);
//#endif
//#endif
//
//#ifndef OS_WIN32
//    /**
//     * Cygwin defines IPTOS_LOWDELAY but can't handle that flag so it's
//     * necessary to workaround that problem.
//     **/
//    /* Set the IP low delay option */
//    option = IPTOS_LOWDELAY;
//    rc = setsockopt(s, IPPROTO_IP, IP_TOS, &option, sizeof(int));
//    if (rc == -1) {
//        return -1;
//    }
//#endif

//	return 0;
//}

//I have to chgange the arguments as soclen_t is POISX parameters. It is not available
//static int _connect(int sockfd, const struct sockaddr *addr,
//		const struct timeval *ro_tv) {
//	int rc;
//#if STM_HAL
//	// Get the IP address from the sockaddr structure (assuming it's already parsed)
//	uint8_t *ip_addr = (uint8_t*) addr;
//	uint16_t port = MODBUS_TCP_DEFAULT_PORT;  // Port 502 for Modbus
//
//	// Use W5500's connect function. We don't need addrlen; instead, we pass the port directly.
//	rc = connect(sockfd, ip_addr, port);
//#else
//	int rc = connect(sockfd, addr, addrlen);
//#endif
//
//#ifdef OS_WIN32
//    int wsaError = 0;
//    if (rc == -1) {
//        wsaError = WSAGetLastError();
//    }
//
//    if (wsaError == WSAEWOULDBLOCK || wsaError == WSAEINPROGRESS) {
//#else
//	if (rc == -1 && errno == EINPROGRESS) {
//#endif
//		fd_set wset;
//		int optval;
//		// socklen_t optlen = sizeof(optval);
//		struct timeval tv = *ro_tv;
//
//		/* Wait to be available in writing */
//		FD_ZERO(&wset);
//		FD_SET(sockfd, &wset);
//		rc = select(sockfd + 1, NULL, &wset, NULL, &tv);
//		if (rc < 0) {
//			/* Fail */
//			return -1;
//		}
//
//		if (rc == 0) {
//			/* Timeout */
//			errno = ETIMEDOUT;
//			return -1;
//		}
//
//#if STM_HAL
//		// In STM32, check the socket connection status directly using getsockopt
//		uint8_t sock_status;
//		rc = getsockopt(sockfd, SO_DESTIP, &sock_status); // Use W5500-specific getsockopt
//
//		if (rc != SOCK_OK) {
//			return -1;  // Error in checking the socket status
//		}
//
//		if (sock_status == SOCK_OK) {
//			// Connection is successful
//			return 0;
//		} else {
//			// Connection failed
//			return -1;
//		}
//#else
//        /* The connection is established if SO_ERROR and optval are set to 0 */
//        rc = getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (void *) &optval, &optlen);
//#endif
//
//		if (rc == 0 && optval == 0) {
//			return 0;
//		} else {
//			errno = ECONNREFUSED;
//			return -1;
//		}
//	}
//	return rc;
//}

/* Establishes a modbus TCP connection with a Modbus server. */
//static int _modbus_tcp_connect(modbus_t *ctx)
//{
//    int rc;
//    /* Specialized version of sockaddr for Internet socket address (same size) */
//    struct sockaddr_in addr;
//    modbus_tcp_t *ctx_tcp = ctx->backend_data;
//    int flags = SOCK_STREAM;
//
//#ifdef OS_WIN32
//    if (_modbus_tcp_init_win32() == -1) {
//        return -1;
//    }
//#endif
//
//#ifdef SOCK_CLOEXEC
//    flags |= SOCK_CLOEXEC;
//#endif
//
//#ifdef SOCK_NONBLOCK
//    flags |= SOCK_NONBLOCK;
//#endif
//
//    ctx->s = socket(PF_INET, flags, 0);
//    if (ctx->s < 0) {
//        return -1;
//    }
//
//    rc = _modbus_tcp_set_ipv4_options(ctx->s);
//    if (rc == -1) {
//        close(ctx->s);
//        ctx->s = -1;
//        return -1;
//    }
//
//    if (ctx->debug) {
//        printf("Connecting to %s:%d\n", ctx_tcp->ip, ctx_tcp->port);
//    }
//
//    addr.sin_family = AF_INET;
//    addr.sin_port = htons(ctx_tcp->port);
//    rc = inet_pton(addr.sin_family, ctx_tcp->ip, &(addr.sin_addr));
//    if (rc <= 0) {
//        if (ctx->debug) {
//            fprintf(stderr, "Invalid IP address: %s\n", ctx_tcp->ip);
//        }
//        close(ctx->s);
//        ctx->s = -1;
//        return -1;
//    }
//
//    rc =
//        _connect(ctx->s, (struct sockaddr *) &addr, &ctx->response_timeout);
//    if (rc == -1) {
//        close(ctx->s);
//        ctx->s = -1;
//        return -1;
//    }
//
//    return 0;
//}
static int _modbus_tcp_connect(modbus_t *ctx) {
	int rc;
	modbus_tcp_t *ctx_tcp = ctx->backend_data;
	uint8_t dest_ip[4];    // To store the destination IP address
	uint16_t dest_port;    // Destination port
	uint8_t sn = 0;        // Use socket 0, change if needed

	// W5500 specific settings
	sn = 0; // Assign socket 0, this can be changed based on your design

	// Convert IP address string to bytes manually since we can't use inet_pton()
	if (sscanf(ctx_tcp->ip, "%hhu.%hhu.%hhu.%hhu", &dest_ip[0], &dest_ip[1],
			&dest_ip[2], &dest_ip[3]) != 4) {
		if (ctx->debug) {
			printf("Invalid IP address: %s\n", ctx_tcp->ip);
		}
		return -1;
	}

	//dest_port = htons(ctx_tcp->port); // Convert port to network byte order

	// Open the W5500 socket as a TCP socket (Sn_MR_TCP)
	rc = socket(sn, Sn_MR_TCP, ctx_tcp->port, 0); // Opens a socket on the specified port
	if (rc != sn) {
		if (ctx->debug) {
			printf("Failed to open socket %d\n", sn);
		}
		return -1;
	}

	// Set the destination IP address and port using setsockopt or socket functions
	rc = setsockopt(sn, SO_DESTIP, dest_ip);
	if (rc != SOCK_OK) {
		if (ctx->debug) {
			printf("Failed to set destination IP\n");
		}
		close(sn);  // Close socket if failure occurs
		return -1;
	}

	rc = setsockopt(sn, SO_DESTPORT, &dest_port);
	if (rc != SOCK_OK) {
		if (ctx->debug) {
			printf("Failed to set destination port\n");
		}
		close(sn);  // Close socket if failure occurs
		return -1;
	}

	// Debug print for connection info
	if (ctx->debug) {
		printf("Connecting to %s:%d\n", ctx_tcp->ip, ctx_tcp->port);
	}

	// Connect to the remote server (W5500 doesn't have timeouts for connect)
	rc = connect(sn, dest_ip, dest_port); // Should conncet over _connect-- I directly connect with socket
	if (rc != SOCK_OK) {
		if (ctx->debug) {
			printf("Connection failed\n");
		}
		close(sn);  // Close socket if connection fails
		return -1;
	}

	// Save the socket number in the context for further communication
	ctx->s = sn;

	return 0;
}

///* Establishes a modbus TCP PI connection with a Modbus server. */
//static int _modbus_tcp_pi_connect(modbus_t *ctx) {
//	int rc;
//	struct addrinfo *ai_list;
//	struct addrinfo *ai_ptr;
//	struct addrinfo ai_hints;
//	modbus_tcp_pi_t *ctx_tcp_pi = ctx->backend_data;
//
//#ifdef OS_WIN32
//    if (_modbus_tcp_init_win32() == -1) {
//        return -1;
//    }
//#endif
//
//	memset(&ai_hints, 0, sizeof(ai_hints));
//#ifdef AI_ADDRCONFIG
//    ai_hints.ai_flags |= AI_ADDRCONFIG;
//#endif
//	ai_hints.ai_family = AF_UNSPEC;
//	ai_hints.ai_socktype = SOCK_STREAM;
//	ai_hints.ai_addr = NULL;
//	ai_hints.ai_canonname = NULL;
//	ai_hints.ai_next = NULL;
//
//	ai_list = NULL;
//	rc = getaddrinfo(ctx_tcp_pi->node, ctx_tcp_pi->service, &ai_hints,
//			&ai_list);
//	if (rc != 0) {
//		if (ctx->debug) {
//#ifdef HAVE_GAI_STRERROR
//            fprintf(stderr, "Error returned by getaddrinfo: %s\n", gai_strerror(rc));
//#else
//			fprintf(stderr, "Error returned by getaddrinfo: %d\n", rc);
//#endif
//		}
//		freeaddrinfo(ai_list);
//		errno = ECONNREFUSED;
//		return -1;
//	}
//
//	for (ai_ptr = ai_list; ai_ptr != NULL; ai_ptr = ai_ptr->ai_next) {
//		int flags = ai_ptr->ai_socktype;
//		int s;
//
//#ifdef SOCK_CLOEXEC
//        flags |= SOCK_CLOEXEC;
//#endif
//
//#ifdef SOCK_NONBLOCK
//        flags |= SOCK_NONBLOCK;
//#endif
//
//		s = socket(ai_ptr->ai_family, flags, ai_ptr->ai_protocol);
//		if (s < 0)
//			continue;
//
//		if (ai_ptr->ai_family == AF_INET)
//			_modbus_tcp_set_ipv4_options(s);
//
//		if (ctx->debug) {
//			printf("Connecting to [%s]:%s\n", ctx_tcp_pi->node,
//					ctx_tcp_pi->service);
//		}
//
//		rc = _connect(s, ai_ptr->ai_addr, &ctx->response_timeout);
//		if (rc == -1) {
//			close(s);
//			continue;
//		}
//
//		ctx->s = s;
//		break;
//	}
//
//	freeaddrinfo(ai_list);
//
//	if (ctx->s < 0) {
//		return -1;
//	}
//
//	return 0;
//}

/* Custom function to convert string IP to byte array */
int string_to_ip(const char *ip_str, uint8_t *ip_array) {
	int ip1, ip2, ip3, ip4;
	if (sscanf(ip_str, "%d.%d.%d.%d", &ip1, &ip2, &ip3, &ip4) != 4) {
		return -1; // Invalid IP format
	}
	ip_array[0] = (uint8_t) ip1;
	ip_array[1] = (uint8_t) ip2;
	ip_array[2] = (uint8_t) ip3;
	ip_array[3] = (uint8_t) ip4;
	return 0; // Success
}

/* Establishes a modbus TCP PI connection with a Modbus server for STM32 with W5500. */
static int _modbus_tcp_pi_connect(modbus_t *ctx) {
	int rc;
	modbus_tcp_pi_t *ctx_tcp_pi = ctx->backend_data;
	uint8_t ip_addr[4];   // Array to hold the IPv4 address
	uint16_t port = MODBUS_TCP_DEFAULT_PORT; // Default Modbus TCP port (502)
	uint8_t sn = 0; // Socket number (adjust this based on your setup)

#ifdef STM_HAL
	// For STM32 with W5500, manually parse the IP and port from the context
	// Convert the IP address from string to a byte array.
	if (string_to_ip(ctx_tcp_pi->node, ip_addr) != 0) {
		if (ctx->debug) {
			printf("Invalid IP address: %s\n", ctx_tcp_pi->node);
		}
		return -1;
	}

	if (ctx_tcp_pi->service != NULL) {
		// Convert the service (port) from string to integer
		port = atoi(ctx_tcp_pi->service);
	}

	// Open a TCP socket using the W5500.
	ctx->s = socket(sn, Sn_MR_TCP, port, 0); // Create a TCP socket using the W5500
	if (ctx->s < 0) {
		if (ctx->debug) {
			printf("Failed to create socket\n");
		}
		return -1;
	}

	// Attempt to connect using W5500’s connect function
	rc = connect(ctx->s, ip_addr, port);  // Connect to the given IP and port
	if (rc != SOCK_OK) {
		// If connection fails, close the socket and return error
		close(ctx->s);
		ctx->s = -1;
		return -1;
	}

#else
    // Fallback for non-STM32 environments (POSIX-style)
    struct addrinfo *ai_list;
    struct addrinfo *ai_ptr;
    struct addrinfo ai_hints;

    memset(&ai_hints, 0, sizeof(ai_hints));
#ifdef AI_ADDRCONFIG
    ai_hints.ai_flags |= AI_ADDRCONFIG;
#endif
    ai_hints.ai_family = AF_UNSPEC;
    ai_hints.ai_socktype = SOCK_STREAM;
    ai_hints.ai_addr = NULL;
    ai_hints.ai_canonname = NULL;
    ai_hints.ai_next = NULL;

    ai_list = NULL;
    rc = getaddrinfo(ctx_tcp_pi->node, ctx_tcp_pi->service, &ai_hints, &ai_list);
    if (rc != 0) {
        if (ctx->debug) {
#ifdef HAVE_GAI_STRERROR
            fprintf(stderr, "Error returned by getaddrinfo: %s\n", gai_strerror(rc));
#else
            fprintf(stderr, "Error returned by getaddrinfo: %d\n", rc);
#endif
        }
        freeaddrinfo(ai_list);
        errno = ECONNREFUSED;
        return -1;
    }

    for (ai_ptr = ai_list; ai_ptr != NULL; ai_ptr = ai_ptr->ai_next) {
        int flags = ai_ptr->ai_socktype;
        int s;

#ifdef SOCK_CLOEXEC
        flags |= SOCK_CLOEXEC;
#endif

#ifdef SOCK_NONBLOCK
        flags |= SOCK_NONBLOCK;
#endif

        s = socket(ai_ptr->ai_family, flags, ai_ptr->ai_protocol);
        if (s < 0)
            continue;

        if (ai_ptr->ai_family == AF_INET)
            _modbus_tcp_set_ipv4_options(s);

        if (ctx->debug) {
            printf("Connecting to [%s]:%s\n", ctx_tcp_pi->node, ctx_tcp_pi->service);
        }

        rc = _connect(s, ai_ptr->ai_addr, &ctx->response_timeout);
        if (rc == -1) {
            close(s);
            continue;
        }

        ctx->s = s;
        break;
    }

    freeaddrinfo(ai_list);
#endif

	if (ctx->s < 0) {
		return -1;
	}

	return 0;
}

static unsigned int _modbus_tcp_is_connected(modbus_t *ctx) {
	return ctx->s >= 0;
}

#if STM_HAL
/* Closes the network connection and socket in TCP mode */
static void _modbus_tcp_close(modbus_t *ctx) {
	if (ctx->s >= 0) {
		// Close the socket using W5500's socket close function
//        socket(ctx->s, Sn_MR_TCP, 0, SF_CLOSE); // Close the socket
		closesock(ctx->s);
		ctx->s = -1; // Reset the socket descriptor
	}
}
#else
/* Closes the network connection and socket in TCP mode */
static void _modbus_tcp_close(modbus_t *ctx) {
	if (ctx->s >= 0) {
		shutdown(ctx->s, SHUT_RDWR);
		close(ctx->s);
		ctx->s = -1;
	}
}
#endif
static int _modbus_tcp_flush(modbus_t *ctx) {
	int rc;
	// Use an unsigned 16-bit integer to reduce overflow risk. The flush function
	// is not expected to handle huge amounts of data (> 2GB).
	uint16_t rc_sum = 0;

	do {
		/* Extract the garbage from the socket */
		char devnull[MODBUS_TCP_MAX_ADU_LENGTH];
#if  STM_HAL
		rc = recv(ctx->s, (void*) devnull, MODBUS_TCP_MAX_ADU_LENGTH); //void buffer to be filled
#else
        /* On Win32, it's a bit more complicated to not wait */
        fd_set rset;
        struct timeval tv;

        tv.tv_sec = 0;
        tv.tv_usec = 0;
        FD_ZERO(&rset);
        FD_SET(ctx->s, &rset);
        rc = select(ctx->s + 1, &rset, NULL, NULL, &tv);
        if (rc == -1) {
            return -1;
        }

        if (rc == 1) {
            /* There is data to flush */
            rc = recv(ctx->s, devnull, MODBUS_TCP_MAX_ADU_LENGTH, 0);
        }
#endif
		if (rc > 0) {
			// Check for overflow before adding
			if (rc_sum <= UINT16_MAX - rc) {
				rc_sum += rc;
			} else {
				// Handle overflow
				ctx->error_recovery = MODBUS_ERROR_RECOVERY_PROTOCOL;
				errno = EOVERFLOW;
				return -1;
			}
		}
	} while (rc == MODBUS_TCP_MAX_ADU_LENGTH);

	return rc_sum;
}


int modbus_tcp_listen(modbus_t *ctx, int nb_connection) {

    int8_t ret;
    uint8_t sn = SOCK_0;
   // modbus_tcp_t *ctx_tcp;

    // Validate input context
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

   // ctx_tcp = ctx->backend_data;

    // Step 1: Open a TCP socket in blocking mode
    ret = socket(sn, Sn_MR_TCP, MODBUS_TCP_DEFAULT_PORT, 0); // 0 = blocking mode
    if (ret != sn) {
        return SOCKERR_SOCKINIT;  // Socket initialization failed
    }

    // Step 2: Configure the socket to listen for incoming connections
    ret = listen(sn);
    if (ret != SOCK_OK) {
        closesock(sn);  // Clean up if listen fails
        return ret;     // Return error code from listen
    }

    // Update connection state if successful
    connection_state = STATE_LISTEN;
    status = getSn_SR(SOCK_0);

    // Socket is now listening on the Modbus TCP port
    return SOCK_OK;  // Successfully listening
}


#if 0 //TODO

int modbus_tcp_accept(modbus_t *ctx, int *s) {
    uint8_t remote_ip[4];
    uint16_t remote_port;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }



        switch (connection_state) {
            case STATE_IDLE:
                // Initialize the socket and set it to listen
                socket(SOCK_0, Sn_MR_TCP, MODBUS_TCP_DEFAULT_PORT, 0);
                listen(SOCK_0);
                connection_state = STATE_LISTEN;
                break;

            case STATE_LISTEN:
                // Check if the socket is in the listening state
//                status = getSn_SR(SOCK_0);
//                if (status == SOCK_ESTABLISHED) {
//                    connection_state = STATE_ACCEPT;  // Move to accept connection
//                } else if (status != SOCK_LISTEN) {
//                    connection_state = STATE_IDLE;  // Reinitialize if unexpected state
//                }
            	if (status == SOCK_LISTEN) {
            	        // Wait for a connection request
            	        while (getSn_SR(SOCK_0) == SOCK_LISTEN) {
            	            // Loop until connection is established
            	        }

            	        status = getSn_SR(SOCK_0);
            	    }

            	else if (status == SOCK_ESTABLISHED){
            		connection_state = STATE_ACCEPT;
            	}
            	else{
            		connection_state = STATE_IDLE;
            	}
                break;

            case STATE_ACCEPT:
                // Connection established, retrieve client's IP and port
                getSn_DIPR(SOCK_0, remote_ip);
                remote_port = getSn_PORT(SOCK_0);

                // Assign the socket and update context
                ctx->s = SOCK_0;
                *s = SOCK_0;

                if (ctx->debug) {
                    printf("Client connection accepted from %d.%d.%d.%d:%d\n",
                           remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3], remote_port);
                }

                connection_state = STATE_CONNECTED;  // Move to connected state
                break;

            case STATE_CONNECTED:
                // Check if the client is still connected
                status = getSn_SR(SOCK_0);
                if (status == SOCK_CLOSE_WAIT) {
                    connection_state = STATE_CLOSE;  // Move to close state if client is disconnecting
                } else if (status != SOCK_ESTABLISHED) {
                    connection_state = STATE_IDLE;  // Return to IDLE if disconnected unexpectedly
                }
                return ctx->s;  // Return socket in CONNECTED state

            case STATE_CLOSE:
                // Close the connection and reset to idle
                disconnect(SOCK_0);
                connection_state = STATE_IDLE;
                break;
        }



    return -1;  // Return -1 if no active connection
}

#elif 0
//for interrupt mode
int modbus_tcp_accept(modbus_t *ctx, int *s) {
    uint8_t remote_ip[4];
    uint16_t remote_port;
    int8_t ret;
    uint8_t sn = SOCK_0;

    // Check if context is valid
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    // Check if the socket is already in a listening state
    status = getSn_SR(SOCK_0);

    // If the socket is closed, reinitialize and start listening
    if (status == SOCK_CLOSED) {
        // Open a TCP socket in blocking mode on the Modbus TCP port
        ret = socket(sn, Sn_MR_TCP, MODBUS_TCP_DEFAULT_PORT, 0);
        if (ret != sn) {
            return SOCKERR_SOCKINIT;  // Socket initialization failed
        }

        // Configure the socket to listen for incoming connections
        ret = listen(sn);
        if (ret != SOCK_OK) {
            closesock(sn);  // Clean up if listen fails
            return ret;     // Return error code from listen
        }
        connection_state = STATE_LISTEN; // Update state to listening
    }

    // If a connection is established, accept it
    if (getSn_SR(SOCK_0) == SOCK_ESTABLISHED) {
        getSn_DIPR(SOCK_0, remote_ip);      // Get client's IP
        remote_port = getSn_PORT(SOCK_0);   // Get client's port

        // Set the context socket to the current socket number
        ctx->s = SOCK_0;

        if (ctx->debug) {
            printf("Client connection accepted from %d.%d.%d.%d:%d\n",
                   remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3], remote_port);
        }

        return ctx->s;  // Return the accepted socket number
    } else {
        // No connection established yet
        return -1;
    }
}

#else


int modbus_tcp_accept(modbus_t *ctx, int *s) {
    uint8_t remote_ip[4];
    uint16_t remote_port;
    int8_t ret;
    uint8_t sn = SOCK_0;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    // Check if the socket is already in a listening state
    status = getSn_SR(SOCK_0);
    //Check the status
    if(status == SOCK_CLOSED){
        // Step 1: Open a TCP socket in blocking mode
        ret = socket(sn, Sn_MR_TCP, MODBUS_TCP_DEFAULT_PORT, 0); // 0 = blocking mode
        if (ret != sn) {
            return SOCKERR_SOCKINIT;  // Socket initialization failed
        }

        // Step 2: Configure the socket to listen for incoming connections
        ret = listen(sn);
        if (ret != SOCK_OK) {
            closesock(sn);  // Clean up if listen fails
            return ret;     // Return error code from listen
        }
        // Check if the socket is already in a listening state
        status = getSn_SR(SOCK_0);
        connection_state = STATE_LISTEN;
    }


    if (status == SOCK_LISTEN) {
        // Wait for a connection request
        while (getSn_SR(SOCK_0) == SOCK_LISTEN) {
            // Loop until connection is established
        }

        status = getSn_SR(SOCK_0);
    }

    // Verify if the connection is established
    if (status == SOCK_ESTABLISHED) {
        // Retrieve the client's IP and port
        //getSn_DIPR(*s, remote_ip);  // Get destination IP (client IP)
        getSn_DIPR(SOCK_0, remote_ip);  // Get destination IP (client IP)
        remote_port = getSn_PORT(SOCK_0); // Get destination port (client port)
        printf("Remote ip: %d.%d.%d.%d \t Port: %d \n",remote_ip[0],remote_ip[1],remote_ip[2],remote_ip[3],remote_port );

        // Set the context socket to the current socket number
        ctx->s = SOCK_0;//*s;

        if (ctx->debug) {
            printf("Client connection accepted from %d.%d.%d.%d:%d\n",
                   remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3], remote_port);
        }

        return ctx->s;
    } else {
        // No connection established

        return -1;
    }
}

#endif
int modbus_tcp_pi_listen(modbus_t *ctx, int nb_connection) {
	modbus_tcp_pi_t *ctx_tcp_pi;
	int new_s;

	if (ctx == NULL) {
		errno = EINVAL;
		return -1;
	}

	ctx_tcp_pi = ctx->backend_data;

	// Use default service port if none is specified
	if (ctx_tcp_pi->service[0] == 0) {
		ctx_tcp_pi->service = "502"; // Modbus default port
	}

	// Open a socket
	new_s = socket(0, Sn_MR_TCP, atoi(ctx_tcp_pi->service), 0); // Using socket number 0
	if (new_s < 0) {
		return -1;
	}

	// Set socket options if necessary (W5500 might not require SO_REUSEADDR)
	// For W5500, it may not directly support setsockopt for SO_REUSEADDR.

	// Prepare for listening
	if (listen(new_s) < 0) {
		close(new_s);
		return -1;
	}

	return new_s; // Return the socket number on success
}





int modbus_tcp_pi_accept(modbus_t *ctx, int *s) {

#if 0
	struct sockaddr_in6 addr;
    socklen_t addrlen;

    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    addrlen = sizeof(addr);
#ifdef HAVE_ACCEPT4
    /* Inherit socket flags and use accept4 call */
    ctx->s = accept4(*s, (struct sockaddr *) &addr, &addrlen, SOCK_CLOEXEC);
#else
    ctx->s = accept(*s, (struct sockaddr *) &addr, &addrlen);
#endif

    if (ctx->s < 0) {
        return -1;
    }

    if (ctx->debug) {
        char buf[INET6_ADDRSTRLEN];
        if (inet_ntop(AF_INET6, &(addr.sin6_addr), buf, INET6_ADDRSTRLEN) == NULL) {
            fprintf(stderr, "Client connection accepted from unparsable IP.\n");
        } else {
            printf("Client connection accepted from %s.\n", buf);
        }
    }
#endif
	return ctx->s;
}
#if STM_HAL
static int _modbus_tcp_select(modbus_t *ctx, fd_set *rset, struct timeval *tv, int length_to_read) {
    uint8_t sock_num = ctx->s;  // Assuming 's' holds the socket number
    uint32_t start_time = HAL_GetTick(); // Using HAL for timing
    uint32_t timeout = (tv->tv_sec * 1000) + (tv->tv_usec / 1000); // Convert timeout to milliseconds

    while (1) {
        // Check if the socket is established
        if (getSn_SR(sock_num) == SOCK_ESTABLISHED) {
            // Check if there is data available in the RX buffer
            if (getSn_RX_RSR(sock_num) >= length_to_read) {
                return 1; // Data is ready to be read
            }
        } else if (getSn_SR(sock_num) == SOCK_CLOSE_WAIT) {
            // Connection is closed
            close(sock_num); // Close the socket
            errno = ENOTCONN; // Set "not connected" error
            return -1;
        }

        // Check if the timeout has been reached
        if ((HAL_GetTick() - start_time) >= timeout) {
            errno = ETIMEDOUT; // Set timeout error
            return -1; // Indicate a timeout occurred
        }

        // Optional: small delay to avoid busy waiting
        HAL_Delay(1); // Delay for 1 ms
    }

    // This return statement is just for safety; it should never be reached
    return -1;
}

#else
static int _modbus_tcp_select(modbus_t *ctx, fd_set *rset, struct timeval *tv,
		int length_to_read) {
	int s_rc;
	while ((s_rc = select(ctx->s + 1, rset, NULL, NULL, tv)) == -1) {
		if (errno == EINTR) {
			if (ctx->debug) {
				fprintf(stderr, "A non blocked signal was caught\n");
			}
			/* Necessary after an error */
			FD_ZERO(rset);
			FD_SET(ctx->s, rset);
		} else {
			return -1;
		}
	}

	if (s_rc == 0) {
		errno = ETIMEDOUT;
		return -1;
	}

	return s_rc;
}
#endif
static void _modbus_tcp_free(modbus_t *ctx) {
	if (ctx->backend_data) {
		free(ctx->backend_data);
	}
	free(ctx);
}

static void _modbus_tcp_pi_free(modbus_t *ctx) {
	if (ctx->backend_data) {
		modbus_tcp_pi_t *ctx_tcp_pi = ctx->backend_data;
		free(ctx_tcp_pi->node);
		free(ctx_tcp_pi->service);
		free(ctx->backend_data);
	}

	free(ctx);
}

// clang-format off
const modbus_backend_t _modbus_tcp_backend = { _MODBUS_BACKEND_TYPE_TCP,
_MODBUS_TCP_HEADER_LENGTH,
_MODBUS_TCP_CHECKSUM_LENGTH,
MODBUS_TCP_MAX_ADU_LENGTH, _modbus_set_slave, _modbus_tcp_build_request_basis,
		_modbus_tcp_build_response_basis, _modbus_tcp_prepare_response_tid,
		_modbus_tcp_send_msg_pre, _modbus_tcp_send, _modbus_tcp_receive,
		_modbus_tcp_recv, _modbus_tcp_check_integrity,
		_modbus_tcp_pre_check_confirmation, _modbus_tcp_connect,
		_modbus_tcp_is_connected, _modbus_tcp_close, _modbus_tcp_flush,
		_modbus_tcp_select, _modbus_tcp_free };

const modbus_backend_t _modbus_tcp_pi_backend = { _MODBUS_BACKEND_TYPE_TCP,
_MODBUS_TCP_HEADER_LENGTH,
_MODBUS_TCP_CHECKSUM_LENGTH,
MODBUS_TCP_MAX_ADU_LENGTH, _modbus_set_slave, _modbus_tcp_build_request_basis,
		_modbus_tcp_build_response_basis, _modbus_tcp_prepare_response_tid,
		_modbus_tcp_send_msg_pre, _modbus_tcp_send, _modbus_tcp_receive,
		_modbus_tcp_recv, _modbus_tcp_check_integrity,
		_modbus_tcp_pre_check_confirmation, _modbus_tcp_pi_connect,
		_modbus_tcp_is_connected, _modbus_tcp_close, _modbus_tcp_flush,
		_modbus_tcp_select, _modbus_tcp_pi_free };

// clang-format on

modbus_t* modbus_new_tcp(const char *ip, int port) {
	modbus_t *ctx;
	modbus_tcp_t *ctx_tcp;
	size_t dest_size;
	size_t ret_size;

#if defined(OS_BSD)
    /* MSG_NOSIGNAL is unsupported on *BSD so we install an ignore
       handler for SIGPIPE. */
    struct sigaction sa;

    sa.sa_handler = SIG_IGN;
    if (sigaction(SIGPIPE, &sa, NULL) < 0) {
        /* The debug flag can't be set here... */
        fprintf(stderr, "Could not install SIGPIPE handler.\n");
        return NULL;
    }
#endif

	ctx = (modbus_t*) malloc(sizeof(modbus_t));
	if (ctx == NULL) {
		return NULL;
	}
	_modbus_init_common(ctx);

	/* Could be changed after to reach a remote serial Modbus device */
	ctx->slave = MODBUS_TCP_SLAVE;

	ctx->backend = &_modbus_tcp_backend;

	ctx->backend_data = (modbus_tcp_t*) malloc(sizeof(modbus_tcp_t));
	if (ctx->backend_data == NULL) {
		modbus_free(ctx);
		errno = ENOMEM;
		return NULL;
	}
	ctx_tcp = (modbus_tcp_t*) ctx->backend_data;

	if (ip != NULL) {
		dest_size = sizeof(char) * 16;
		ret_size = strlcpy(ctx_tcp->ip, ip, dest_size);
		if (ret_size == 0) {
			fprintf(stderr, "The IP string is empty\n");
			modbus_free(ctx);
			errno = EINVAL;
			return NULL;
		}

		if (ret_size >= dest_size) {
			fprintf(stderr, "The IP string has been truncated\n");
			modbus_free(ctx);
			errno = EINVAL;
			return NULL;
		}
	} else {
		ctx_tcp->ip[0] = '0';
	}
	ctx_tcp->port = port;
	ctx_tcp->t_id = 0;

	return ctx;
}

modbus_t* modbus_new_tcp_pi(const char *node, const char *service) {
	modbus_t *ctx;
	modbus_tcp_pi_t *ctx_tcp_pi;

	ctx = (modbus_t*) malloc(sizeof(modbus_t));
	if (ctx == NULL) {
		return NULL;
	}
	_modbus_init_common(ctx);

	/* Could be changed after to reach a remote serial Modbus device */
	ctx->slave = MODBUS_TCP_SLAVE;

	ctx->backend = &_modbus_tcp_pi_backend;

	ctx->backend_data = (modbus_tcp_pi_t*) malloc(sizeof(modbus_tcp_pi_t));
	if (ctx->backend_data == NULL) {
		modbus_free(ctx);
		errno = ENOMEM;
		return NULL;
	}
	ctx_tcp_pi = (modbus_tcp_pi_t*) ctx->backend_data;
	ctx_tcp_pi->node = NULL;
	ctx_tcp_pi->service = NULL;

	if (node != NULL) {
		ctx_tcp_pi->node = strdup(node);
	} else {
		/* The node argument can be empty to indicate any hosts */
		ctx_tcp_pi->node = strdup("");
	}

	if (ctx_tcp_pi->node == NULL) {
		modbus_free(ctx);
		errno = ENOMEM;
		return NULL;
	}

	if (service != NULL && service[0] != '\0') {
		ctx_tcp_pi->service = strdup(service);
	} else {
		/* Default Modbus port number */
		ctx_tcp_pi->service = strdup("502");
	}

	if (ctx_tcp_pi->service == NULL) {
		modbus_free(ctx);
		errno = ENOMEM;
		return NULL;
	}

	ctx_tcp_pi->t_id = 0;

	return ctx;
}
