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

static void my_debug( void *ctx, int level, const char *str )
{
    if( level < DEBUG_LEVEL )
    {
        usart_putstr( str );
    }
}


/* ECHO protocol states */
enum polarclient_states
{
  ES_NOT_CONNECTED = 0,
  ES_CONNECTED,
  ES_RECEIVED,
  ES_CLOSING,
};


/* structure to be passed as argument to the tcp callbacks */
struct polarclient
{
  enum polarclient_states state; /* connection status */
  struct tcp_pcb *pcb;          /* pointer on the current tcp_pcb */
  struct pbuf *p_tx;            /* pointer on pbuf to be transmitted */
};


/* Private function prototypes -----------------------------------------------*/
static err_t tcp_polarclient_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_polarclient_connection_close(struct tcp_pcb *tpcb, struct polarclient * es);
static err_t tcp_polarclient_poll(void *arg, struct tcp_pcb *tpcb);
static err_t tcp_polarclient_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void tcp_polarclient_send(struct tcp_pcb *tpcb, struct polarclient * es);
static err_t tcp_polarclient_connected(void *arg, struct tcp_pcb *tpcb, err_t err);
static int tcp_wrapped_write( void *ctx, const unsigned char *buf, size_t len);
static int tcp_wrapped_recv( void *ctx, unsigned char *buf, size_t len );
err_t polarssl_init(void);
/* Private functions ---------------------------------------------------------*/

int tcp_wrapped_write( void *ctx, const unsigned char *buf, size_t len) {
	return tcp_write(polarclient_pcb, (const void *)buf, (u16_t)len, 1);
}

int tcp_wrapped_recv( void *ctx, unsigned char *buf, size_t len) {
	//TODO
	int num_bytes = len;
	return num_bytes;
}

err_t polarssl_init(void) {

	err_t ret;
	/*
     * 0. Initialize the RNG and the session data
     */
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

    usart_putstr( " - done\n" );

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
    	sprintf(buf, "x509_crt_parse returned=%d\n", ret );
    	usart_putstr(buf);
    	//usart_putstr( " failed\n  !  x509_crt_parse returned \n\n");
        goto exit;
    }

    usart_putstr( " ok ( skipped)\n");

    /*
     * 1. Start the connection
     */
    usart_putstr( "  . Connecting to tcp/192.168.1.1/4433\n" );
    tcp_polarclient_connect();

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

    usart_putstr( " ok\n" );

    /*
     * 2. Setup stuff
     */
    usart_putstr( "  . Setting up the SSL/TLS structure..." );

    if( ( ret = ssl_init( &ssl ) ) != 0 )
    {
    	usart_putstr( " failed\n  ! ssl_init returned \n");
        goto exit;
    }

    usart_putstr( " ok\n" );

    ssl_set_endpoint( &ssl, SSL_IS_CLIENT );
    /* OPTIONAL is not optimal for security,
     * but makes interop easier in this simplified example */
    ssl_set_authmode( &ssl, SSL_VERIFY_OPTIONAL );
    ssl_set_ca_chain( &ssl, &cacert, NULL, "PolarSSL Server 1" );

    ssl_set_rng( &ssl, ctr_drbg_random, &ctr_drbg );
    ssl_set_dbg( &ssl, my_debug, 0 );
    //TODO You will receive that pointer as the first argument to your callback..
    //So just cast it back to the correct pointer and then you have your data..
    ssl_set_bio( &ssl, tcp_wrapped_recv, polarclient_pcb,
    		tcp_wrapped_write, polarclient_pcb );

exit:

#ifdef POLARSSL_ERROR_C
    if( ret != 0 )
    {
        char error_buf[100];
        polarssl_strerror( ret, error_buf, 100 );
        printf("Last error was: %d - %s\n\n", ret, error_buf );
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
  
  usart_putstr("start tcp_polarclient_connect\n");
  /* create new tcp pcb */
  polarclient_pcb = tcp_new();
  
  if (polarclient_pcb != NULL)
  {
    IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 );
    
    usart_putstr("connect to destination address/port\n");
    /* connect to destination address/port */
    tcp_connect(polarclient_pcb,&DestIPaddr,DEST_PORT,tcp_polarclient_connected);
    usart_putstr("connect to destination address/port exit \n");
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
  * @param tpcb: pointer on the connection contol block
  * @param err: when connection correctly established err should be ERR_OK 
  * @retval err_t: returned error 
  */
static err_t tcp_polarclient_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
  struct polarclient *es = NULL;
  
  usart_putstr("tcp_polarclient_connected\n");

  //xQueuePost

  if (err == ERR_OK)   
  {
    /* allocate structure es to maintain tcp connection informations */
    es = (struct polarclient *)mem_malloc(sizeof(struct polarclient));
  
    if (es != NULL)
    {
      es->state = ES_CONNECTED;
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
        tcp_recv(tpcb, tcp_polarclient_recv);
  
        /* initialize LwIP tcp_sent callback function */
        tcp_sent(tpcb, tcp_polarclient_sent);
  
        /* initialize LwIP tcp_poll callback function */
        tcp_poll(tpcb, tcp_polarclient_poll, 1);
    
        /* send data */
        tcp_polarclient_send(tpcb,es);
        
        return ERR_OK;
      }
    }
    else
    {
      /* close connection */
      tcp_polarclient_connection_close(tpcb, es);
      
      /* return memory allocation error */
      return ERR_MEM;  
    }
  }
  else
  {
    /* close connection */
    tcp_polarclient_connection_close(tpcb, es);
  }
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
    tcp_wrapped_recv(tpcb, p->payload, p->tot_len);

    pbuf_free(p);
    tcp_polarclient_connection_close(tpcb, es);
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
        /* increment reference count for es->p */
        pbuf_ref(es->p_tx);
      }
      
      /* free pbuf: will free pbufs up to es->p (because es->p has a reference count > 0) */
      pbuf_free(ptr);
   }
   else if(wr_err == ERR_MEM)
   {
      /* we are low on memory, try later, defer to poll */
     es->p_tx = ptr;
   }
   else
   {
     /* other problem ?? */
   }
  }
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

  LWIP_UNUSED_ARG(len);

  es = (struct polarclient *)arg;
  
  if(es->p_tx != NULL)
  {
    /* still got pbufs to send */
    tcp_polarclient_send(tpcb, es);
  }

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

  usart_putstr("tcp_polarclient_connection_close\n");
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
  
}

#endif /* LWIP_TCP */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
