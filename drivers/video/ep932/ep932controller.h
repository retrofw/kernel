#ifndef EP932CONTROLLER_H
#define EP932CONTROLLER_H

#include "ep932eregdef.h"
#include "ep932_if.h"
#include "hdcp.h"

#define VERSION_MAJOR             0  // Beta
#define VERSION_MINOR             36 //      36

#define EP932C_TIMER_PERIOD       100		//     The EP932Controller.c must be re-compiled if user want to change this value.

typedef enum {
	EP932C_TASK_IDLE = 0,
	EP932C_TASK_ERROR,
	EP932C_TASK_PENDING,
	EP932C_TASK_PGOUT
} ep932c_task_status;

typedef struct _ep932c_register_map {

	// Read
	unsigned short		vendorid;			// 0x00
	unsigned short		deviceid;
	unsigned char		version_major;
	unsigned char		version_minor;
	unsigned char		configuration;

	unsigned char		interrupt_flags;		// 0x01

	unsigned char		system_status;			// 0x02

	unsigned char		hdcp_status;			// 0x03
	unsigned char		hdcp_state;
	unsigned char		hdcp_aksv[5];
	unsigned char		hdcp_bksv[5];
	unsigned char		hdcp_bcaps3[3];
	unsigned char		hdcp_ksv_fifo[5*16];
	unsigned char		hdcp_sha[20];
	unsigned char		hdcp_m0[8];

	unsigned char		edid_status;			// 0x04
	unsigned char		edid_videodataaddr;
	unsigned char		edid_audiodataaddr;
	unsigned char		edid_speakerdataaddr;
	unsigned char		edid_vendordataaddr;
	unsigned char		edid_asfreq;
	unsigned char		edid_achannel;
						
	//unsigned short		VS_Period;				// 0x05 (Video Status)
	//unsigned short		H_Res;
	//unsigned short		V_Res;
	//unsigned short		Ratio_24;
	vdo_params 			video_params_backup;

	//unsigned short		AS_Freq;				// 0x06 (Audio Status)
	//unsigned short		AS_Period;				// 
	ado_params 			audio_params_backup;

	unsigned char		readed_edid[256];		// 0x07 
	// Read / Write
	unsigned char		analog_test_control;	// 0X1C

	unsigned char		power_control;			// 0x20
	unsigned char		system_configuration;

	unsigned char		interrupt_enable;		// 0x21

	unsigned char		video_interface[2];		// 0x22

	unsigned char		audio_interface;		// 0x23

	unsigned char		video_input_format[2];	// 0x24

	unsigned char		audio_input_format;		// 0x25

	unsigned char		end;

} ep932c_register_map, *pep932c_register_map;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

typedef void (*ep932c_callback)(void);

void ep932controller_initial(pep932c_register_map pep932c_regmap, ep932c_callback intcall);

unsigned char ep932controller_task(void);

void ep932controller_timer(void);
void ep_hdmi_dumpmessage(void);

void ep932power_on();
void ep932power_off();
// -----------------------------------------------------------------------------
#endif

