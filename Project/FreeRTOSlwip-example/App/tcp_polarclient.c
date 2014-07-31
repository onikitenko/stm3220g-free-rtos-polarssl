/**
  ******************************************************************************
  * @file    LwIP/LwIP_TCP_Echo_Client/Src/tcp_polarclient.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-March-2014
  * @brief   tcp polarclient application using LwIP RAW API
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "lwip/memp.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "polarssl/config.h"
#include "polarssl/net.h"
#include "polarssl/ssl.h"
#include "polarssl/entropy.h"
#include "polarssl/ctr_drbg.h"
#include "polarssl/error.h"
#include "polarssl/certs.h"
#include "../../FreeRTOS/inc/semphr.h"

#if LWIP_TCP
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

u8_t  recev_buf[50];
__IO uint32_t message_count=0;

u8_t   data[100];

struct tcp_pcb *polarclient_pcb;

/* Polar SSL Data */
unsigned char buf[1024];
const char *pers = "ssl_client1";
entropy_context entropy;
ctr_drbg_context ctr_drbg;
ssl_context ssl;
x509_crt cacert;
xQueueHandle xLwIPQueue;
#define GET_REQUEST "GET / HTTP/1.0\r\n\r\n"
#define DEBUG_LEVEL 1

struct polarclient es_my;

static void my_debug( void *ctx, int level, const char *str )
{
    if( level < DEBUG_LEVEL )
    {
    	char tmp_buf[100];
    	strcpy(tmp_buf, str);
        usart_putstr( tmp_buf );
    }
}


/* ECHO protocol states */
enum polarclient_states
{
  ES_NOT_CONNECTED = 0,
  ES_CONNECTED,
  ES_RECEIVED,
  ES_SENT,
  ES_CLOSING,
};


/* structure to be passed as argument to the tcp callbacks */
struct polarclient
{
  int num_bytes_sent;
  enum polarclient_states state; /* connection status */
  struct tcp_pcb *pcb;          /* pointer on the current tcp_pcb */
  struct pbuf *p_tx;            /* pointer on pbuf to be transmitted */
};

xSemaphoreHandle xSem_Sent;
xQueueHandle xQueue1;
unsigned char c_count = 0;

/* Private function prototypes -----------------------------------------------*/
static err_t tcp_polarclient_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_polarclient_connection_close(struct tcp_pcb *tpcb, struct polarclient * es);
static err_t tcp_polarclient_poll(void *arg, struct tcp_pcb *tpcb);
static err_t tcp_polarclient_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void tcp_polarclient_send(struct tcp_pcb *tpcb, struct polarclient * es);
static err_t tcp_polarclient_connected(void *arg, struct tcp_pcb *tpcb, err_t err);
static int tcp_wrapped_write(struct tcp_pcb *tpcb, void *ctx, const unsigned char *buf, size_t len);
static int tcp_wrapped_recv( void *ctx, unsigned char *buf, size_t len );
err_t polarssl_init(void);
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  This function implements delay.
 * @param  d_num: number of delay cycles.
 * @retval None
 */
void Delay(int d_num)
{
	unsigned int d_iter;
	unsigned int i, j;

	for(i = 0; i <= d_num; i++)
	{
		for(j = 0; j <= 500000; j++)
		{
			// Empty loop for delay.
			__NOP();
		}
	}
}


int tcp_wrapped_write(struct tcp_pcb *tpcb, void *ctx, const unsigned char *buf, size_t len)
{
	struct polarclient *es;
	struct polarclient *es_my;
	char *data = buf;
	//xSemaphoreHandle xSem_Sent;

	usart_putstr("tcp_wrapped_write - Func start\n");
	es->pcb = tpcb;

	/* allocate pbuf */
	es->p_tx = pbuf_alloc(PBUF_TRANSPORT, strlen((char*)data) , PBUF_POOL);

	if (es->p_tx)
	{
		/* copy data to pbuf */
		pbuf_take(es->p_tx, (char*)data, strlen((char*)data));

		/* pass newly allocated es structure as argument to tpcb */
		tcp_arg(tpcb, es);

		/* initialize LwIP tcp_recv callback function */
		usart_putstr("tcp_recv is called from tcp_wrapped_write\n");
		tcp_recv(tpcb, tcp_polarclient_recv);

		/* initialize LwIP tcp_sent callback function */
		tcp_sent(tpcb, tcp_polarclient_sent);

		/* initialize LwIP tcp_poll callback function */
		tcp_poll(tpcb, tcp_polarclient_poll, 1);

		/* send data */
		tcp_polarclient_send(tpcb,es);

		//TODO not only delay, but with check for es->state == ES_SENT
			xSemaphoreTake( xSem_Sent , 2000 / portTICK_RATE_MS);
	}
		usart_putstr("tcp_wrapped_write - Func end\n");

		char tmp_buf[50] = "";
		sprintf(tmp_buf, "%d",  es->num_bytes_sent);
		usart_putstr(tmp_buf);


	    return es->num_bytes_sent;
}

int tcp_wrapped_recv( void *ctx, unsigned char *buf, size_t len) {
	//TODO
	//usart_putstr("tcp_recv - Start\n");
	//tcp_recv(polarclient_pcb, callback_recv);
	//usart_putstr("tcp_recv - End\n");

	// ... some code to convert params from
	// <void *ctx, unsigned char *buf, size_t len > to
	// <struct tcp_pcb *pcb, tcp_recv_fn recv>

	// tcp_recv(struct tcp_pcb *pcb, tcp_recv_fn recv);
	int num_bytes = len;

	char tmp_buf[50];

	usart_putstr("Num bytes: ");
	sprintf(tmp_buf, "%d",  len);
	usart_putstr(tmp_buf);
	usart_putstr(" b.\n");

	return num_bytes;
}

err_t polarssl_init(void) {

	err_t ret;
    entropy_context entropy;
    ctr_drbg_context ctr_drbg;
    ssl_context ssl;
    x509_crt cacert;
    int len;
    unsigned char buf[1024];

	/*
     * 0. Initialize the RNG and the session data
     */

		vSemaphoreCreateBinary( xSem_Sent );
		//xQueueHandle xQueue1;
		unsigned char item = 0;

    memset( &ssl, 0, sizeof( ssl_context ) );
    x509_crt_init( &cacert );

    usart_putstr( "\n  . Seeding the random number generator..." );

    entropy_init( &entropy );
    usart_putstr( "\n  . Entropy done" );
    if( ( ret = ctr_drbg_init( &ctr_drbg, entropy_func, &entropy,
                               (const unsigned char *) pers,
                               strlen( pers ) ) ) != 0 )
    {
        //printf( " failed\n  ! ctr_drbg_init returned %d\n", ret );
    	usart_putstr( "failed  ! ctr_drbg_init\n" );
        goto exit;
    }

    usart_putstr( " - done after ret = ctr_drbg_init\n" );

    xLwIPQueue = xQueueCreate( 1, sizeof( unsigned long ) );
    if( xLwIPQueue == 0 )
    {
    	usart_putstr( "failed  initializing xLwIPQueue\n" );
    }

    usart_putstr( " ok\n" );

    /*
     * 0. Initialize certificates
     */
    usart_putstr( "  . Loading the CA root certificate ..." );
    getTaskName("polarssl_init");

#if defined(POLARSSL_CERTS_C)
    ret = x509_crt_parse( &cacert, (const unsigned char *) test_ca_list,
                          strlen( test_ca_list ) );
#else
    ret = 1;
    usart_putstr("POLARSSL_CERTS_C not defined.");
#endif

    if( ret < 0 )
    {
    	char buf[50];
    	sprintf(buf, "x509_crt_parse returned = %d\n", ret );
    	usart_putstr(buf);
    	//usart_putstr( " failed\n  !  x509_crt_parse returned \n\n");
        goto exit;
    }

    usart_putstr( " ok (skipped)\n");

    /*
     * 1. Start the connection
     */
    usart_putstr( "  . Connecting to tcp/192.168.5.10/4433\n" );
    tcp_polarclient_connect();
//    usart_putstr("ssl_set_bio - Before Queue sent item\n");
//    xQueueSend( xQueue1, &item, 0);
//    usart_putstr("ssl_set_bio - After Queue sent item\n");

    if( xLwIPQueue != 0 )
    {
    	unsigned long pxRxedMessage;
        // Receive a message on the created queue.  Block for 10 ticks if a
        // message is not immediately available.
        if( xQueueReceive( xLwIPQueue, &( pxRxedMessage ), (portTickType) 10 ) )
        {
        	usart_putstr( " Received pxRxedMessage\n" );
        }
    }

    usart_putstr( " ok (Conecting)\n" );

    /*
     * 2. Setup stuff
     */
    usart_putstr( "  . Setting up the SSL/TLS structure...\n" );

    if( ( ret = ssl_init( &ssl ) ) != 0 )
    {
    	usart_putstr( " failed\n  ! ssl_init returned \n");
        goto exit;
    }
    usart_putstr( " ok (SSL/TLS)\n" );

    ssl_set_endpoint( &ssl, SSL_IS_CLIENT );
    /* OPTIONAL is not optimal for security,
     * but makes interop easier in this simplified example */
    ssl_set_authmode( &ssl, SSL_VERIFY_OPTIONAL );
    ssl_set_ca_chain( &ssl, &cacert, NULL, "PolarSSL Server 1" );

    ssl_set_rng( &ssl, ctr_drbg_random, &ctr_drbg );
    ssl_set_dbg( &ssl, my_debug, 0 );
    //TODO You will receive that pointer as the first argument to your callback..
    //So just cast it back to the correct pointer and then you have your data..
    usart_putstr("ssl_set_bio - Start\n");
    ssl_set_bio( &ssl, tcp_wrapped_recv, polarclient_pcb,
    		tcp_wrapped_write, polarclient_pcb );
    usart_putstr("ssl_set_bio - End\n");
    xQueueSend( xQueue1, &item, 0);
    //usart_putstr("ssl_set_bio - After Queue sent item\n");

    	/*			----------- New Start -------------
         * 4. Handshake
         */
        usart_putstr( "  . Performing the SSL/TLS handshake...\n" );

        while( ( ret = ssl_handshake( &ssl ) ) != 0 )
        {
            if( ret != POLARSSL_ERR_NET_WANT_READ && ret != POLARSSL_ERR_NET_WANT_WRITE )
            {
                usart_putstr( " failed\n  ! ssl_handshake returned: ");
                char tmp_buf[50];
                sprintf(tmp_buf, "%d\n", ret );
                usart_putstr(tmp_buf);
                goto exit;
            }
        }

        usart_putstr( " ok (ssl_handshake)\n" );

        /*
         * 5. Verify the server certificate
         */
        usart_putstr( "  . Verifying peer X.509 certificate...\n" );

        /* In real life, we may want to bail out when ret != 0 */
        if( ( ret = ssl_get_verify_result( &ssl ) ) != 0 )
        {
            usart_putstr( " failed (Verifying peer X.509)\n" );

            if( ( ret & BADCERT_EXPIRED ) != 0 )
                usart_putstr( "  ! server certificate has expired\n" );

            if( ( ret & BADCERT_REVOKED ) != 0 )
                usart_putstr( "  ! server certificate has been revoked\n" );

            if( ( ret & BADCERT_CN_MISMATCH ) != 0 )
            {
            	usart_putstr( "  ! CN mismatch (expected CN=%s)\n");
            	usart_putstr("   PolarSSL Server 1\n");
            }


            if( ( ret & BADCERT_NOT_TRUSTED ) != 0 )
                usart_putstr( "  ! self-signed or not signed by a trusted CA\n" );

            usart_putstr( "\n" );
        }
        else
            usart_putstr( " ok\n" );

        /*
         * 3. Write the GET request
         */
        usart_putstr( "  > Write to server:" );

        len = sprintf( (char *) buf, GET_REQUEST );

        while( ( ret = ssl_write( &ssl, buf, len ) ) <= 0 )
        {
            if( ret != POLARSSL_ERR_NET_WANT_READ && ret != POLARSSL_ERR_NET_WANT_WRITE )
            {
                usart_putstr(" failed\n  ! ssl_write returned: ");
                char tmp_buf[50];
                sprintf(tmp_buf, "%d\n", ret );
                usart_putstr(tmp_buf);
                goto exit;
            }
        }

        len = ret;
        char tmp_buf[50];
        sprintf(tmp_buf, " %d ", len );
        usart_putstr(tmp_buf);
        usart_putstr( "bytes written\n\n" );
        strcpy(tmp_buf, (char *) buf );
        usart_putstr(tmp_buf);

        /*
         * 7. Read the HTTP response
         */
        usart_putstr( "  < Read from server:" );

        do
        {
            len = sizeof( buf ) - 1;
            memset( buf, 0, sizeof( buf ) );
            ret = ssl_read( &ssl, buf, len );

            if( ret == POLARSSL_ERR_NET_WANT_READ || ret == POLARSSL_ERR_NET_WANT_WRITE )
                continue;

            if( ret == POLARSSL_ERR_SSL_PEER_CLOSE_NOTIFY )
                break;

            if( ret < 0 )
            {
                printf( "failed\n  ! ssl_read returned: ");
                char tmp_buf[50];
                sprintf(tmp_buf, "%d\n", ret );
                usart_putstr(tmp_buf);
                break;
            }

            if( ret == 0 )
            {
                usart_putstr( "\n\nEOF\n\n" );
                break;
            }

            len = ret;
            char tmp_buf[50];
            sprintf(tmp_buf, " %d ", len );
            usart_putstr(tmp_buf);
            usart_putstr( "bytes read\n\n" );
            strcpy(tmp_buf, (char *) buf );
            usart_putstr(tmp_buf);
        }
        while( 1 );

        ssl_close_notify( &ssl );
		//               ----------- New End -------------
exit:

#ifdef POLARSSL_ERROR_C
    if( ret != 0 )
    {
        char error_buf[100];
        polarssl_strerror( ret, error_buf, 100 );
        usart_putstr("Last error was: %d - %s\n\n"); //, ret, error_buf );
    }
#endif

    x509_crt_free( &cacert );
    ssl_free( &ssl );
    entropy_free( &entropy );

    memset( &ssl, 0, sizeof( ssl ) );

    return( ret );
}
/**
* @brief  Connects to the TCP polarssl server
* @param  None
* @retval None
*/
void tcp_polarclient_connect(void)
{
  struct ip_addr DestIPaddr;
  struct polarclient *es_my;
  err_t err;

  getTaskName("tcp_polarclient_connect");

  /* create new tcp pcb */
  polarclient_pcb = tcp_new();
  
  if (polarclient_pcb != NULL)
  {
    IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 );
    
  //  usart_putstr("connect to destination address/port\n");
    getTaskName("connect to destination address/port");
    /* connect to destination address/port */

    err = tcp_connect(polarclient_pcb,&DestIPaddr,DEST_PORT,tcp_polarclient_connected);

    if (err == ERR_OK)
    	usart_putstr("tcp_connect OK\n");
    else
    	usart_putstr("tcp_connect FAIL\n");

    getTaskName("address/port exit");
    usart_putstr("connect to dest. address/port exit \n");
  }
  else
  {
    /* deallocate the pcb */
	usart_putstr("deallocate the pcb\n");
    memp_free(MEMP_TCP_PCB, polarclient_pcb);
#ifdef SERIAL_DEBUG
    printf("\n\r can not create tcp pcb");
#endif 
  }
}

/**
  * @brief Function called when TCP connection established
  * @param tpcb: pointer on the connection control block
  * @param err: when connection correctly established err should be ERR_OK 
  * @retval err_t: returned error 
  */
static err_t tcp_polarclient_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
	//TODO add semaphores
  struct polarclient *es = NULL;
  struct polarclient *es_my;

  	  xQueue1 = xQueueCreate( 1, sizeof( unsigned char ) );
  unsigned char item = 0;

  getTaskName("tcp_polarclient_connected");
  usart_putstr("tcp_polarclient_connected Func - Start\n");

  //xQueuePost

  if (err == ERR_OK)   
  {
    /* allocate structure es to maintain tcp connection informations */
    es = (struct polarclient *)mem_malloc(sizeof(struct polarclient));
    if (es != NULL)
    {
    	usart_putstr("ES_CONNECTED\n");
      es_my->state = ES_CONNECTED;
      es->pcb = tpcb;
      
      sprintf((char*)data, "sending tcp client message %d", (int)message_count);
        
      /* allocate pbuf */
      es->p_tx = pbuf_alloc(PBUF_TRANSPORT, strlen((char*)data) , PBUF_POOL);

      if (es->p_tx)
      {
        /* copy data to pbuf */
        pbuf_take(es->p_tx, (char*)data, strlen((char*)data));
        
        /* pass newly allocated es structure as argument to tpcb */
        tcp_arg(tpcb, es);
  
        /* initialize LwIP tcp_recv callback function */
        usart_putstr("tcp_recv is called from tcp_polarclient_connected\n");
        tcp_recv(tpcb, tcp_polarclient_recv);
  
        /* initialize LwIP tcp_sent callback function */
        tcp_sent(tpcb, tcp_polarclient_sent);
  
        /* initialize LwIP tcp_poll callback function */
        tcp_poll(tpcb, tcp_polarclient_poll, 1);
    
        /* send data */
        tcp_polarclient_send(tpcb,es);

        getTaskName("before xQueue1");

        char buf[50];
            	sprintf(buf, "c_count = %d (Before While(1))\n", c_count );
            	usart_putstr(buf);

        while(1)
        {
        	if(c_count == 0)
        	{
            	usart_putstr("In While(1) with Queues\n");
    			if( xQueueReceive(xQueue1, &item, 500) )
    			{
    				usart_putstr("xQueue1 == 1 - no sleep. (Receive is done)\n");
    				break;
    			}
    			else
    			{
    				usart_putstr("xQueue1 == 0 - go to sleep some (For 500 ticks)\n");
    			}
        	}
        	else
        		break;
        }
                  	sprintf(buf, "c_count = %d (After While(1))\n", c_count );
                  	usart_putstr(buf);
        c_count++;
                  	sprintf(buf, "c_count = %d (After c_count++)\n", c_count );
                  	usart_putstr(buf);

        getTaskName("t.._p.._c.. - return ERR_OK");
        usart_putstr("tcp_polarclient_connected Func - return ERR_OK\n");
        return ERR_OK;

      }
    }
    else
    {
      /* close connection */
    	usart_putstr("tcp_polarclient_connection_close - ERR_MEM\n");
      tcp_polarclient_connection_close(tpcb, es);
      /* return memory allocation error */
      return ERR_MEM;  
    }
  }
  else
  {
    /* close connection */
	  usart_putstr("tcp_polarclient_connection_close\n");
    tcp_polarclient_connection_close(tpcb, es);
  }
  usart_putstr("tcp_polarclient_connected - return err\n");
  return err;
}
    
/**
  * @brief tcp_receiv callback
  * @param arg: argument to be passed to receive callback 
  * @param tpcb: tcp connection control block 
  * @param err: receive error code 
  * @retval err_t: retuned error  
  */
static err_t tcp_polarclient_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{ 
  struct polarclient *es;
  err_t ret_err;
  

  LWIP_ASSERT("arg != NULL",arg != NULL);
  
  es = (struct polarclient *)arg;
  
  /* if we receive an empty tcp frame from server => close connection */
  if (p == NULL)
  {
    /* remote host closed connection */
    es->state = ES_CLOSING;
    if(es->p_tx == NULL)
    {
       /* we're done sending, close connection */
    	getTaskName("Done sending from tcp_polarclient_recv");
    	usart_putstr("we're done sending, close connection\n");
       tcp_polarclient_connection_close(tpcb, es);
    }
    else
    {    
      /* send remaining data*/
      tcp_polarclient_send(tpcb, es);
    }
    ret_err = ERR_OK;
  }   
  /* else : a non empty frame was received from echo server but for some reason err != ERR_OK */
  else if(err != ERR_OK)
  {
    /* free received pbuf*/
    if (p != NULL)
    {
      pbuf_free(p);
    }
    ret_err = err;
  }
  else if(es->state == ES_CONNECTED)
  {
    /* increment message count */
    message_count++;
         
    /* Acknowledge data reception */
    tcp_recved(tpcb, p->tot_len);  
    
    /* Call our wrapper callback */
    //tcp_wrapped_recv(tpcb, p->payload, p->tot_len);

    //TODO free or not???? It seems we should free it
    pbuf_free(p);
    usart_putstr("Call our wrapper callback");
    //tcp_polarclient_connection_close(tpcb, es);
    ret_err = ERR_OK;
  }

  /* data received when connection already closed */
  else
  {
    /* Acknowledge data reception */
    tcp_recved(tpcb, p->tot_len);
    
    /* free pbuf and do nothing */
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  return ret_err;
}

/**
  * @brief function used to send data
  * @param  tpcb: tcp control block
  * @param  es: pointer on structure of type polarclient containing info on data
  *             to be sent
  * @retval None 
  */
static void tcp_polarclient_send(struct tcp_pcb *tpcb, struct polarclient * es)
{
  struct pbuf *ptr;
  err_t wr_err = ERR_OK;
 
  while ((wr_err == ERR_OK) &&
         (es->p_tx != NULL) && 
         (es->p_tx->len <= tcp_sndbuf(tpcb)))
  {
	  usart_putstr("tcp_polarclient_send - NO Errors\n");
    /* get pointer on pbuf from es structure */
    ptr = es->p_tx;

    /* enqueue data for transmission */
    wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);
    
    if (wr_err == ERR_OK)
    { 
      /* continue with next pbuf in chain (if any) */
      es->p_tx = ptr->next;
      
      if(es->p_tx != NULL)
      {
    	  usart_putstr("tcp_polarclient_send - next es->p_tx\n");
        /* increment reference count for es->p */
        pbuf_ref(es->p_tx);
      }
      
      /* free pbuf: will free pbufs up to es->p (because es->p has a reference count > 0) */
      usart_putstr("tcp_polarclient_send - pbuf_free - start\n");
      pbuf_free(ptr);
      usart_putstr("tcp_polarclient_send - pbuf_free - end\n");
   }
   else if(wr_err == ERR_MEM)
   {
	   usart_putstr("tcp_polarclient_send - low memory\n");
      /* we are low on memory, try later, defer to poll */
     es->p_tx = ptr;
   }
   else
   {
     /* other problem ?? */
   }
  }
  usart_putstr("tcp_polarclient_send - END function\n");
}

/**
  * @brief  This function implements the tcp_poll callback function
  * @param  arg: pointer on argument passed to callback
  * @param  tpcb: tcp connection control block
  * @retval err_t: error code
  */
static err_t tcp_polarclient_poll(void *arg, struct tcp_pcb *tpcb)
{
  err_t ret_err;
  struct polarclient *es;

  es = (struct polarclient*)arg;
  if (es != NULL)
  {
    if (es->p_tx != NULL)
    {
      /* there is a remaining pbuf (chain) , try to send data */
      tcp_polarclient_send(tpcb, es);
    }
    else
    {
      /* no remaining pbuf (chain)  */
      if(es->state == ES_CLOSING)
      {
        /* close tcp connection */
    	  usart_putstr("close tcp connection");
        tcp_polarclient_connection_close(tpcb, es);
      }
    }
    ret_err = ERR_OK;
  }
  else
  {
    /* nothing to be done */
    tcp_abort(tpcb);
    ret_err = ERR_ABRT;
  }
  return ret_err;
}

/**
  * @brief  This function implements the tcp_sent LwIP callback (called when ACK
  *         is received from remote host for sent data) 
  * @param  arg: pointer on argument passed to callback
  * @param  tcp_pcb: tcp connection control block
  * @param  len: length of data sent 
  * @retval err_t: returned error code
  */
static err_t tcp_polarclient_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  struct polarclient *es;
  struct polarclient *es_my;
  //xSemaphoreHandle xSem_Sent;

  LWIP_UNUSED_ARG(len);

  es = (struct polarclient *)arg;
  es_my->num_bytes_sent = len;
  
  if(es->p_tx != NULL)
  {
    /* still got pbufs to send */
    tcp_polarclient_send(tpcb, es);
  }

  es_my->state = ES_SENT;

  xSemaphoreGive( xSem_Sent );

  return ERR_OK;
}

/**
  * @brief This function is used to close the tcp connection with server
  * @param tpcb: tcp connection control block
  * @param es: pointer on polarclient structure
  * @retval None
  */
static void tcp_polarclient_connection_close(struct tcp_pcb *tpcb, struct polarclient * es )
{

  usart_putstr("tcp_polarclient_connection_close in Function Start\n");
  /* remove callbacks */
  tcp_recv(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_poll(tpcb, NULL,0);

  if (es != NULL)
  {
    mem_free(es);
  }

  /* close tcp connection */
  tcp_close(tpcb);
  usart_putstr("tcp_polarclient_connection_close in Function End\n");
}

#endif /* LWIP_TCP */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
