#include <linux/module.h>
#include <linux/sched.h>

#include <asm/jzsoc.h>
#include <linux/timer.h>
#include "ep932api.h"
#include "ep932controller.h"  // HDMI Transmiter

#define      IIC_EP932E 0x70
#define      EP932M_5VEN_PIN (32*5+6)
int hdmi_power_on= 0;
int hdmi_init=1;
int restart_system;
int hdmi_terminate_flag=1;
struct timer_list ep932_timer; 
int hdmi_delay = 2;
int hdmi_hg_out = 0;

ep932c_register_map ep932c_registers;

void ep_ep932m_reset(void)
{
	__gpio_as_output(EP932M_RESET_PIN);
	__gpio_clear_pin(EP932M_RESET_PIN);	// set to output low

	mdelay(400);		// Delay 10ms.

	__gpio_set_pin(EP932M_RESET_PIN);
	mdelay(400);		// Delay 10ms.
}

#undef printk

void hdmi_main (void)
{
	   if(hdmi_power_on == 0){
				//printk("EP932Controller ****************************\n");
				if((hdmi_init == 0) || (hdmi_hg_out == 3 )){
					mod_timer(&ep932_timer,jiffies+3*100);
				}
				else{
					mod_timer(&ep932_timer,jiffies+hdmi_delay);
				}
				hdmi_hg_out = ep932controller_task();
				ep932controller_timer();
	   }
	   else{
	   	
	   }
   return;
}


void ep_hdmi_setaudfmt(hdmi_audfmt_t aud_fmt, hdmi_audfreq  aud_freq)
{
	if(aud_fmt == AUD_I2S)
	{
		ep932c_registers.audio_interface = 0x18;		// 2 channel IIS
		//DBG_PRINTK(("Audio interface is IIS - 2.0 CH, "));
	}
	else
	{
		ep932c_registers.audio_interface = 0x10;		// SPDIF
		//DBG_PRINTK(("Audio interface is SPDIF, "));
	}

	if(aud_freq == 0)
	{
		ep932c_registers.system_configuration = 0x02;
		//DBG_PRINTK(("(AUDIO MUTE)"));
	}
	else
	{
		ep932c_registers.system_configuration = 0x00;
	}
	ep932c_registers.audio_input_format = aud_freq;	// set Audio frequency
	DBG_PRINTK(("freq = "));
	switch(aud_freq)
	{
		case AUD_SF_32000HZ:
			DBG_PRINTK(("32K HZ\r\n"));
			break;
			
		case AUD_SF_44100HZ:
			DBG_PRINTK(("44.1K HZ\r\n"));
			break;
			
		case AUD_SF_48000HZ:
			DBG_PRINTK(("48K HZ\r\n"));
			break;
			
		case AUD_SF_88200HZ:
			DBG_PRINTK(("88.2K HZ\r\n"));
			break;
			
		case AUD_SF_96000HZ:
			DBG_PRINTK(("96K HZ\r\n"));
			break;
			
		case AUD_SF_176400HZ:
			DBG_PRINTK(("176.4K HZ\r\n"));
			break;
			
		case AUD_SF_192000HZ:
			DBG_PRINTK(("192K HZ\r\n"));
			break;
			
	}
}

void ep_hdmi_set_video_timing(int timing)
{

	unsigned char temp_setting = 0;

	DBG_PRINTK(("\r\n\r\n"));

	switch (timing)
	{
/*
		case 0: //TVOUT_MODE_576I:
			DBG_PRINTK(("TVOUT_MODE_576I\r\n"));
			ep932c_registers.video_input_format[0] = 0x15;
			ep932c_registers.video_interface[0] = 0x8E;//0x81;
			ep932c_registers.video_interface[1] = 0x00;//0x0a;

			ep932c_registers.power_control = 0x00;
			break;

	   case 1: //TVOUT_MODE_480I:
		    DBG_PRINTK(("TVOUT_MODE_480I\r\n"));
			ep932c_registers.video_input_format[0] = 0x06;
			ep932c_registers.video_interface[0] = 0x8E;//0x81;
			ep932c_registers.video_interface[1] = 0x00;//0x0a;

			ep932c_registers.power_control = 0x00;
			break;

*/
		default:
        case 3: //TVOUT_MODE_480P:
			DBG_PRINTK(("EP932M_MODE H.V.DE mode - (480P output test)\r\n"));
			ep932c_registers.video_input_format[0] = 0x00;	// video format timing
			
			temp_setting =EDGE_RISING | BSEL_24BIT /*| EDGE_RISING */ /*| FMT_12*/;
			ep932c_registers.video_interface[0] = temp_setting;
			
			DBG_PRINTK(("video_interface_0 = 0x%02X \r\n",(int)ep932c_registers.video_interface[0] ));

			ep932c_registers.video_interface[1] = 0x00; 	// DE,HS,VS, RGB444
			
			ep932c_registers.power_control = 0x00;
			break;
		case 4: //TVOUT_MODE_576P:
		    DBG_PRINTK(("TVOUT_MODE_576P\r\n"));
			ep932c_registers.video_input_format[0] = 0x11;
			ep932c_registers.video_interface[0] = 0x8E;//0x84;
			ep932c_registers.video_interface[1] = 0x00;//0x0a;

			ep932c_registers.power_control = 0x00;
		    break;
		case 5: //TVOUT_MODE_720P50:
			DBG_PRINTK(("TVOUT_MODE_720P50\r\n"));
			ep932c_registers.video_input_format[0] = 0x13;
			ep932c_registers.video_interface[0] = 0x86;
			ep932c_registers.video_interface[1] = 0x00;//0x0a;

			ep932c_registers.power_control = 0x00;
			break;

		 case 6: //TVOUT_MODE_720P60:
			DBG_PRINTK(("TVOUT_MODE_720P60\r\n"));
			ep932c_registers.video_input_format[0] = 0x04;
			ep932c_registers.video_interface[0] = 0x86;
			ep932c_registers.video_interface[1] = 0x00;//0x0a;

			ep932c_registers.power_control = 0x00;
			break;

/*
		case 6: //TVOUT_MODE_1080I25:
		    DBG_PRINTK(("TVOUT_MODE_1080I25\r\n"));
			ep932c_registers.video_input_format[0] = 0x14;
			ep932c_registers.video_interface[0] = 0x84;
			ep932c_registers.video_interface[1] = 0x00;//0x0a;

			ep932c_registers.power_control = 0x00;
			break;

		case 7: //TVOUT_MODE_1080I30:
		    DBG_PRINTK(("TVOUT_MODE_1080I30\r\n"));
			ep932c_registers.video_input_format[0] = 0x05;
			ep932c_registers.video_interface[0] = 0x84;
			ep932c_registers.video_interface[1] = 0x00;//0x0a;

			ep932c_registers.power_control = 0x00;
			break;

		case 8: //TVOUT_MODE_480P:
			DBG_PRINTK(("TVOUT_MODE_480P\r\n"));
			ep932c_registers.video_input_format[0] = 0x02;
			ep932c_registers.video_interface[0] = 0x83;//0x84;
			ep932c_registers.video_interface[1] = 0x00;//0x0a;

			ep932c_registers.power_control = 0x00;
			break;
*/
   	}

	ep_hdmi_setaudfmt(AUD_I2S, AUD_SF_44100HZ);
//	ep_hdmi_setaudfmt(AUD_I2S, 0);

	DBG_PRINTK(("##############################################\r\n"));

}
unsigned char ep_hdmi_deinit(void)
{
   
	ep_ep932m_reset();
}
unsigned char ep_hdmi_init(void)
{

//	int err;

//	__gpio_as_output(EP932M_5VEN_PIN);
//	__gpio_set_pin(EP932M_5VEN_PIN);

	ep_ep932m_reset();
	
  
	ep932controller_initial(&ep932c_registers, NULL);

/*
    err= pthread_create(&hdmi_tid, NULL, hdmi_main, NULL);	// for linux
    if (err !=0){
        printk("can't create hdmi thread: %s \n", strerror(err));
        return 1; //HDMI_ERROR_INIT;
    }
	else
	{
        printk("create hdmi main thread ok\n");
	}
*/
	return 0; //HDMI_SUCCESS;
}

void hdmi_test()
{
	add_timer(&ep932_timer);
}

void hdmi_start(int video_timing)
{
	ep_hdmi_init();				// Variables initial	
	ep_hdmi_set_video_timing(video_timing);		// Video set to H.V.DE mode #
	hdmi_init = 1; 
	init_timer(&ep932_timer);
	ep932_timer.function=&hdmi_main;
	ep932_timer.expires=jiffies+hdmi_delay;
	add_timer(&ep932_timer);
}
void hdmi_stop()
{
	del_timer(&ep932_timer);
	mdelay(400);
	ep_hdmi_deinit();
}

/*
void hdmi_exit(void)
{
cleariisoutput();
os_TaskDelete(PRIO_hdmi_TASK);
deAlloc(TaskhdmiStk);
lcd_reinit_lcd();
}
*/
EXPORT_SYMBOL(hdmi_test);
EXPORT_SYMBOL(hdmi_start);
EXPORT_SYMBOL(hdmi_stop);
