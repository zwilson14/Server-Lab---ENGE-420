/*****************************************************************************
* Name        : <Zach Wilson>
* Program     : Lab <7>
* Class       : ENGE 420
* Date        : <2/26/17>
* Description : <Pwm with light sensor and button and http server interface>
* ===========================================================================
*/

#include <asf.h>
#include "asf.h"
#include "string.h"
#include "main.h"
#include "common/include/nm_common.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
#include "button.h"

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 TCP server example --"STRING_EOL \
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

#define BTN_ACTIVE_VAL ((PORT->Group[BTN_ACTIVE_GROUP].IN.reg) & BTN_ACTIVE)
#define BTN_ACTIVE (PORT_PA08)
#define BTN_ACTIVE_GROUP (0)
#define BTN_ACTIVE_PIN (PIN_PA08%32)


//static const char http_content_type[] =	"Content-type: application/json\r\n\r\n";
static const char http_content_type[] =	"Content-type: text/html\r\n\r\n";
static const char http_cors_allow[] = "Access-Control-Allow-Origin: *\r\n" "Access-Control-Allow-Headers: Content-Type, api_key, Authorization\r\n";
static const char http_html_hdr_200[] = "HTTP/1.1 200 OK\r\n";
static const char http_html_hdr_204[] = "HTTP/1.1 204 No Content\r\n";
static const char http_html_hdr_404[] = "HTTP/1.1 404 Not Found\r\n";
static const char http_html_hdr_500[] = "HTTP/1.1 500 Internal Server Error\r\n";
static const char http_connection_close[] = "Connection: close\r\n\r\n";

// This is the return of the button call to see if pressed.
uint8_t button_val = 0;

/////////////////////////////////////////////////////////////////////////////////


void SysTick_Handler(void);
void send_response(char * message);
void process_request(tstrSocketRecvMsg *pstrRecv);



/** UART module for debug. */
static struct usart_module cdc_uart_module;

static uint32_t millis;

/** Message format definitions. */
typedef struct s_msg_wifi_product {
	uint8_t name[9];
} t_msg_wifi_product;

/** Message format declarations. */
static t_msg_wifi_product msg_wifi_product = {
	.name = MAIN_WIFI_M2M_PRODUCT_NAME,
};

/** Receive buffer definition. */
static uint8_t gau8SocketTestBuffer[MAIN_WIFI_M2M_BUFFER_SIZE];

/** Socket for TCP communication */
static SOCKET tcp_server_socket = -1;
static SOCKET tcp_client_socket = -1;

/** Wi-Fi connection state */
static uint8_t wifi_connected;

/**
 * \brief Callback to get the Data from socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg socket event type. Possible values are:
 *  - SOCKET_MSG_BIND
 *  - SOCKET_MSG_LISTEN
 *  - SOCKET_MSG_ACCEPT
 *  - SOCKET_MSG_CONNECT
 *  - SOCKET_MSG_RECV
 *  - SOCKET_MSG_SEND
 *  - SOCKET_MSG_SENDTO
 *  - SOCKET_MSG_RECVFROM
 * \param[in] pvMsg is a pointer to message structure. Existing types are:
 *  - tstrSocketBindMsg
 *  - tstrSocketListenMsg
 *  - tstrSocketAcceptMsg
 *  - tstrSocketConnectMsg
 *  - tstrSocketRecvMsg
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	switch (u8Msg) {
	/* Socket bind */
	case SOCKET_MSG_BIND:
	{
		tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg *)pvMsg;
		if (pstrBind && pstrBind->status == 0) {
			printf("socket_cb: bind success!\r\n");
			listen(tcp_server_socket, 0);
		} else {
			printf("socket_cb: bind error!\r\n");
			close(tcp_server_socket);
			tcp_server_socket = -1;
		}
	}
	break;

	/* Socket listen */
	case SOCKET_MSG_LISTEN:
	{
		tstrSocketListenMsg *pstrListen = (tstrSocketListenMsg *)pvMsg;
		if (pstrListen && pstrListen->status == 0) {
			printf("socket_cb: listen success!\r\n");
			accept(tcp_server_socket, NULL, NULL);
		} else {
			printf("socket_cb: listen error!\r\n");
			close(tcp_server_socket);
			tcp_server_socket = -1;
		}
	}
	break;

	/* Connect accept */
	case SOCKET_MSG_ACCEPT:
	{
		tstrSocketAcceptMsg *pstrAccept = (tstrSocketAcceptMsg *)pvMsg;
		if (pstrAccept) {
			printf("socket_cb: accept success!\r\n");
			accept(tcp_server_socket, NULL, NULL);
			tcp_client_socket = pstrAccept->sock;
			recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);
		} else {
			printf("socket_cb: accept error!\r\n");
			close(tcp_server_socket);
			tcp_server_socket = -1;
		}
	}
	break;

	/* Message send */
	case SOCKET_MSG_SEND:
	{
		printf("socket_cb: send success!\r\n");
		printf("TCP Server Test Complete!\r\n");
		printf("close socket\n");
		close(tcp_client_socket);
		//close(tcp_server_socket);
	}
	break;

	/* Message receive */
	case SOCKET_MSG_RECV:
	{
		tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
		if (pstrRecv && pstrRecv->s16BufferSize > 0) {
			printf("socket_cb: recv success!\r\n");
			//send(tcp_client_socket, &msg_wifi_product, sizeof(t_msg_wifi_product), 0);
			process_request(pstrRecv);
		} else {
			printf("socket_cb: recv error!\r\n");
			close(tcp_server_socket);
			tcp_server_socket = -1;
		}
	}

	break;

	default:
		break;
	}
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
			wifi_connected = 0;
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
		}
	}
	break;

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		wifi_connected = 1;
		printf("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
	}
	break;

	default:
		break;
	}
}










/////////////////////////////////////////////////////////////////////////////////

void configure_adc(void);
void configure_adc_callbacks(void);
void adc_complete_callback(
		struct adc_module *const module);

//! [result_buffer]
#define ADC_SAMPLES 128
uint16_t adc_result_buffer[ADC_SAMPLES];
//! [result_buffer]

//! [module_inst]
struct adc_module adc_instance;
//! [module_inst]

//! [job_complete_callback]
volatile bool adc_read_done = false;

void adc_complete_callback(
		struct adc_module *const module)
{
  adc_read_buffer_job(&adc_instance, adc_result_buffer, ADC_SAMPLES);
	adc_read_done = true;
}
//! [job_complete_callback]

//! [setup]
void configure_adc(void)
{
//! [setup_config]
	struct adc_config config_adc;
//! [setup_config]
//! [setup_config_defaults]
	adc_get_config_defaults(&config_adc);
//! [setup_config_defaults]

//! [setup_modify_conf]
#if (!SAML21) && (!SAML22) && (!SAMC21)
	config_adc.gain_factor     = ADC_GAIN_FACTOR_DIV2;
#endif
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV64;
	config_adc.reference       = ADC_REFERENCE_INTVCC1;
#if (SAMC21)
	config_adc.positive_input  = ADC_POSITIVE_INPUT_PIN0;
#else
	config_adc.positive_input  = ADC_POSITIVE_INPUT_PIN0;
#endif
	config_adc.resolution      = ADC_RESOLUTION_8BIT;
//! [setup_modify_conf]

//! [setup_set_config]
#if (SAMC21)
	adc_init(&adc_instance, ADC1, &config_adc);
#else
	adc_init(&adc_instance, ADC, &config_adc);
#endif
//! [setup_set_config]

//! [setup_enable]
	adc_enable(&adc_instance);
//! [setup_enable]
}

void configure_adc_callbacks(void)
{
//! [setup_register_callback]
	adc_register_callback(&adc_instance,
			adc_complete_callback, ADC_CALLBACK_READ_BUFFER);
//! [setup_register_callback]
//! [setup_enable_callback]
	adc_enable_callback(&adc_instance, ADC_CALLBACK_READ_BUFFER);
//! [setup_enable_callback]
}
//! [setup]










/**
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	struct usart_config usart_conf;

	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	usart_conf.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	usart_conf.baudrate    = 115200;

	stdio_serial_init(&cdc_uart_module, EDBG_CDC_MODULE, &usart_conf);
	usart_enable(&cdc_uart_module);
}


void process_request(tstrSocketRecvMsg *pstrRecv)
{
  char message[255];
  // printf(pstrRecv->pu8Buffer);
  printf("bob");
  //if (!strncmp(gau8SocketTestBuffer, "Get /Button", 11))
  if (!strncmp(pstrRecv->pu8Buffer, "GET /Button ", 12))
  {
    if (button_read() > 0)
    {
      sprintf(message, "Button is on\r\n");
    }
    else
    {
      sprintf(message, "Button is off\r\n");
    }  
  }  
  
  else if (!strncmp(gau8SocketTestBuffer, "GET /ADC ", 9))
  {
    sprintf(message, "ADC Val = %d", adc_result_buffer[0]);
  }  
  
  else 
  {
    sprintf(message, "Try going to /Button or /ADC");
  }
  send_response(message);
  //send_response(pstrRecv->pu8Buffer);
}

void send_response(char * message)
{
	char content_length[255];
    uint8_t msg_length;

    msg_length = strlen(message);
	sprintf(content_length, "Content-Length: ");
	sprintf(&content_length[strlen(content_length)], "%d\r\n", msg_length);

    // Send header
	send(tcp_client_socket, http_html_hdr_200, sizeof(http_html_hdr_200), 0);
	send(tcp_client_socket, http_cors_allow, sizeof(http_cors_allow), 0);
	send(tcp_client_socket, content_length, strlen(content_length), 0);
	send(tcp_client_socket, http_content_type, sizeof(http_content_type), 0);
	
    // Send Content
	send(tcp_client_socket, message, msg_length, 0);

}








/////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Main application function.
 *
 * Initialize system, UART console, network then test function of TCP server.
 *
 * \return program return value.
 */
int main(void)
{
	tstrWifiInitParam param;
	int8_t ret;
	struct sockaddr_in addr;
	/** User define MAC Address. */
	const char main_user_define_mac_address[] = {0xf8, 0xf0, 0x05, 0x12, 0x12, 0x12};
	uint8_t mac_addr[6];

	/* Initialize the board. */
	system_init();
  buttons_init();
  
  //! [setup_init]
  configure_adc();
  configure_adc_callbacks();
  //! [setup_init]

  //! [main]
  //! [enable_global_interrupts]
  system_interrupt_enable_global();
  //! [enable_global_interrupts]
	adc_read_buffer_job(&adc_instance, adc_result_buffer, ADC_SAMPLES);
  
	/* Initialize the UART console. */
	configure_console();
	printf(STRING_HEADER);

	/* Initialize the BSP. */
	nm_bsp_init();
	
	SysTick_Config(48000);

	/* Initialize socket address structure. */
	addr.sin_family = AF_INET;
	addr.sin_port = _htons(MAIN_WIFI_M2M_SERVER_PORT);
	addr.sin_addr.s_addr = 0;

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) {
		}
	}



// 	/* Get MAC Address from OTP. */
// 	m2m_wifi_get_otp_mac_address(mac_addr, &u8IsMacAddrValid);
// 	if (!u8IsMacAddrValid) {
 	printf("USER MAC Address : ");
// 
// 		/* Cannot found MAC Address from OTP. Set user define MAC address. */
 	m2m_wifi_set_mac_address((uint8_t *)main_user_define_mac_address);
// 	} else {
// 		printf("OTP MAC Address : ");
// 	}

	/* Get MAC Address. */
    m2m_wifi_get_mac_address(mac_addr);
	/* Initialize socket module */
	socketInit();
	registerSocketCallback(socket_cb, NULL);

	/* Connect to router. */
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

	while (1) {
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
        if (millis > 1000)
		{
			//printf(".");
		}
		if (wifi_connected == M2M_WIFI_CONNECTED) {
			if (millis > 1000)
		    {
			    //printf("ss: %d\n", tcp_server_socket);
		    }
			if (tcp_server_socket < 0) {
				/* Open TCP server socket */
				if ((tcp_server_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
					printf("main: failed to create TCP server socket error!\r\n");
					continue;
				}

				/* Bind service*/
				bind(tcp_server_socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
			}
			else
			{
			if (millis > 1000)
			{
				//printf("-");
			}				
			}
		}
		if (millis > 1000)
		{
			millis = 0;
		}
	}

	return 0;
}


void SysTick_Handler(void)  
{                             
  millis++;
}

