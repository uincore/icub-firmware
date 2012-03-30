
/* @file       main-transceiver.c
	@brief      This file implements ...
	@author     marco.accame@iit.it
    @date       04/07/2011
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


using namespace std;

#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include <string>
#include <signal.h>

// Ace stuff
#include <ace/ACE.h>
#include "ace/SOCK_Dgram.h"
#include "ace/Addr.h"

#include "main-transceiver.hpp"
#include "hostTransceiver.hpp"

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include "yarp/dev/ControlBoardInterfacesImpl.inl" //ControlBoardHelper

#include "EoMotionControl.h"
#include "eOcfg_EPs_rem_board.h"

//#include "eOcfg_nvsEP_base_con.h"
//#include "eOcfg_nvsEP_mngmnt_con.h"
//#include "eOcfg_nvsEP_joint_con.h"

#define hal_trace_puts(arg)		printf("%s", arg)

// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------

 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_callback_button_1(void);
static void s_callback_button_2(void);
static void s_callback_button_3(void);
static void s_callback_button_4(void);

static void s_eom_hostprotoc_extra_protocoltransceiver_ask_the_board(void);

void copyPid2eo(Pid in, eOmc_PID_t *out);
// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialization) of static variables
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


// Connection infos
//DSocket	UDP_socket, UDP_socket2;

ACE_SOCK_Dgram				*ACE_socket;
ACE_UINT16					port;
ACE_INT8					flags = 0;

Board_connect_info local, remote01;
uint8_t need2sendarop = 0;

pthread_t thread;


int main(int argc, char *argv[])
{
	char str[SIZE];
	// Program data
	memset(&remote01.data, 0x00, SIZE);

	// Utility stuff
	int i;
	ACE_UINT32 ip1,ip2,ip3,ip4;

	//	Register handler to catch CTRL+C
	if(signal(SIGINT, sighandler) == SIG_IGN)
		signal(SIGINT, SIG_IGN);

	// Set default connection data
	local.address_string=string(DEFAULT_LAPTOP_IP);
	remote01.address_string = string(DEFAULT_EMS_IP);
	port = DEFAULT_PORT;


	uint8_t *udppkt_data = NULL;
	uint16_t udppkt_size = 0;

    // parse command line input argument
    if(argc > 1)
    {
    	for(i = 1; i < argc; i++)
    	{

    		if(strcmp("--reply", argv[i]) == 0 )
    		{
    			need2sendarop = 1;
    			printf("I'll reply to any received packet\n");
    			continue;
    		}

    		if(strcmp("--rem-ip", argv[i]) == 0 )
    		{
    			if(i < (argc - 1))
    				remote01.address_string = string(argv[++i]);
    			continue;
    		}

    		if(strcmp("--loc-ip", argv[i]) == 0 )
    		{
    			if(i < (argc - 1))
    				local.address_string = string(argv[++i]);
    			continue;
    		}

    		if(strcmp("--port", argv[i]) == 0 )
    		{
    			if(i < (argc - 1))
    				port = atoi(argv[++i]);
    			continue;
    		}

    		if((strcmp("-h", argv[i]) == 0) || (strcmp("--help", argv[i]) == 0))
    		{
    			usage();
    			return 0;
    		}
    		usage();
    		return ERROR;
    	}
    }

    usage();

    printf("\nConfiguration is : \n\n");
    printf("I need to send a reply = %d\n", need2sendarop);
    // start the udp socket using ace / winsock / psocks
    // Set my own address_string
    sscanf(local.address_string.c_str(),"%d.%d.%d.%d",&ip1,&ip2,&ip3,&ip4);
    local.addr.set(port, (ip1<<24)|(ip2<<16)|(ip3<<8)|ip4 );

    printf("local.address: %s\n", local.address_string.c_str());

	ACE_socket = new ACE_SOCK_Dgram();
	if (-1 == ACE_socket->open(local.addr) )
	{
		printf("eStikEtzi pensa che qualcosa non abbia funzionato!!!\n");
		return -1;
	}

    // Set destination address_string
    sscanf(remote01.address_string.c_str(),"%d.%d.%d.%d",&ip1,&ip2,&ip3,&ip4);
    remote01.addr.set(port, (ip1<<24)|(ip2<<16)|(ip3<<8)|ip4 );

    printf("remote01.address: %s\n", remote01.address_string.c_str());
    printf("port is : %d\n\n", port);

	eOipv4addr_t hostaddr = eo_common_ipv4addr(ip1,ip2,ip3,ip4); // marco (10, 255, 39, 151)
	eOipv4port_t port = port;

    // init object: one per ems
    hostTransceiver_Init(eo_common_ipv4addr(10,255,37,155), eo_common_ipv4addr(10, 39,255,152), port, 512);

    ACE_UINT8		tmp = 1;

    // Start receiver thread
    pthread_create(&thread, NULL, recvThread, (void*) &remote01);

	 // Send a packet to test dummy
	 while(keepGoingOn)
		{
			commands();
			char pressedkey = getchar();
			if (pressedkey != '\n')
			{
				getchar();
				switch (pressedkey)
				{
					case 'q':  	// quit
						keepGoingOn = FALSE;
						break;

					case '1':	//	send one ask rop
						s_callback_button_1();
						break;

					case '2':	// config/reset regular rops
						s_callback_button_2();
						break;

					case '3':	//	send empty rop
						s_callback_button_3();
						break;

					case '4':	//	send a set pid rop
						s_callback_button_4();
						break;

					default:
						printf("Command not known -->> sending an empty ropframe!\n");
						break;
				}

				// Get the actual packet and write it into socket using udppkt_data, udppkt_size
				// if the command isn't known, an empty ropframe wil be sent -- I guess
				hostTransceiver_GetTransmit(&udppkt_data, &udppkt_size);
				ACE_socket->send(udppkt_data, udppkt_size, remote01.addr, flags);
				printf("Sent EmbObj packet, size = %d\n", udppkt_size);
			}
		}

    pthread_cancel(thread);
    return(0);
}


// --------------------------------------------------------------------------------------------------------------------
// - functions with internal scope
// --------------------------------------------------------------------------------------------------------------------

void *recvThread(void * arg)
{
  printf("Thread started\n");

  int  timeout = 2000;
  Board_connect_info		sender;
  ACE_UINT16 				udppkt_size = 0;
  size_t					maxBytes2Read = 512;
  ACE_UINT8					tmp = 0xAA;
  ACE_UINT8					bytesRecv = -1;


  char str[SIZE];


  while (keepGoingOn)
  {
	  // get from sockt
	  sender.addr.get_addr();
	  udppkt_size = ACE_socket->recv((void *) &sender.data, maxBytes2Read, remote01.addr, flags);
	  printf("Received new packet, size = %d\n", udppkt_size);

	  hostTransceiver_SetReceived((ACE_UINT8 *)sender.data, udppkt_size);

	  if(1 == need2sendarop)
	  {
		// eo_ropframe_Clear(p->ropframereply);
		//  hostTransceiver_AddSetROP((ACE_UINT16 ) EOK_cfg_nvsEP_base_endpoint, (ACE_UINT16 ) EOK_cfg_nvsEP_base_NVID__forcerestart, &tmp, 1);
		  hostTransceiver_GetTransmit((ACE_UINT8 **)&sender.data, &udppkt_size);
		  hostTransceiver_GetTransmit((ACE_UINT8 **)&sender.data, &udppkt_size);  // repeated, just to be sure the packet got at the end is empty

		  // write into skt: udppkt_data, udppkt_size
			ACE_socket->send(sender.data, udppkt_size, remote01.addr, flags);		  printf("Sent EmbObj packet, size = %d\n", udppkt_size);
	  }
  }
  pthread_exit(NULL);
  return NULL;
}



void commands(void)
{
  printf("q: quit\n");
  printf("1: send type 1 packet -> ask application info\n");
  printf("2: send type 2 packet -> configure regulars (toggling)\n");
  printf("3: send type 3 packet -> send an empty rop frame\n");
  printf("4: send type 4 packet -> set joint configuration\n");
  printf("\n");
}

void usage(void)
{
  printf("usage: "
		 "\t\t--help		print this help\n");
  printf("\t\t--loc-ip  <xx>	set the ip address of the local machine - needed by yarp - by default %s\n", DEFAULT_LAPTOP_IP);
  printf("\t\t--rem-ip  <xx>	set the ip address of the remote device - by default %s\n", DEFAULT_EMS_IP);
  printf("\t\t--port    <xx>	set the socket port - by default %d\n", DEFAULT_PORT);
  printf("\t\t--reply		if present, then an empty ropframe is sent as an ack for each packet received - by default it's disabled\n");
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of callback functions
// --------------------------------------------------------------------------------------------------------------------

static void s_callback_button_1(void)
{
    char str[128];

    snprintf(str, sizeof(str)-1, "called callback on BUTTON_TAMP: tx a set<regulars>");
    hal_trace_puts(str);

    s_eom_hostprotoc_extra_protocoltransceiver_ask_the_board();
}


static void s_eom_hostprotoc_extra_protocoltransceiver_ask_the_board(void)
{
	char str[128];

	s_eom_hostprotoc_extra_protocoltransceiver_load_occasional_rop(eo_ropcode_ask, EOK_cfg_nvsEP_base_endpoint, EOK_cfg_nvsEP_base_NVID__applicationinfo);
	snprintf(str, sizeof(str)-1, "added a ask<__applicationinfo>\n");
}

static void s_callback_button_2(void)
{
	char str[128];

	snprintf(str, sizeof(str)-1, "called callback on BUTTON_TAMP: tx a set<regulars>\n");
	s_eom_hostprotoc_extra_protocoltransceiver_configure_regular_rops_on_board();
}


static void s_callback_button_3(void )
{
	char str[128];

	snprintf(str, sizeof(str)-1, "called callback on BUTTON_WKUP: tx a ropframe\n");
}

static void s_callback_button_4(void )
{
	char str[128];
	Pid pid;
	eOmc_joint_config_t *cfg =  &eo_cfg_nvsEP_joint_usr_rem_board_mem_local->cfg;
	copyPid2eo(pid, &cfg->pidpos);

    s_eom_hostprotoc_extra_protocoltransceiver_load_occasional_rop(eo_ropcode_set, EOK_cfg_nvsEP_joint_endpoint, EOK_cfg_nvsEP_joint_NVID__cfg);

	snprintf(str, sizeof(str)-1, "called callback on BUTTON_WKUP: tx a ropframe\n");
}

// Utilities

void copyPid2eo(Pid in, eOmc_PID_t *out)
{
	out->kp = in.kp;
	out->ki = in.ki;
	out->kd = in.kd;
	out->limitonintegral = in.max_int;
	out->limitonoutput = in.max_output;
	out->offset = in.offset;
	out->scale = in.scale;
}
// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------
