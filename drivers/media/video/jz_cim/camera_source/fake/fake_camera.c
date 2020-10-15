#include <asm/jzsoc.h>

#include "fake_camera.h"
#include "../../jz_cim_core.h"

unsigned int current_frame_count = 0;
unsigned int off_set = 4;

unsigned char yuv422i_color_odd[3][4] = {{0xd6, 0x97, 0xca,0x97}, {0x5f, 0x25, 0x6e, 0x25}, {0x73, 0x8e, 0x37, 0x8e}};
unsigned char yuv422i_color_even[3][4] = {{0x5f, 0x25, 0x6e, 0x25}, {0x73, 0x8e, 0x37, 0x8e}, {0xd6, 0x97, 0xca, 0x97}};

struct fake_sensor_info{ 
	struct camera_sensor_desc fake_sensor_desc;
	unsigned int current_calc_width;
	unsigned int current_calc_height;
	pixel_format_flag_t current_pixel_format;
};

struct fake_sensor_info *current_fake_sensor = NULL;

//*current_fake_sensor;



void fake_power_down(void)
{
}   

void fake_power_up(void)
{
}


void fake_reset(void)
{
}

void fake_init(void)
{
	//	current_format = PIXEL_FORMAT_YUV422I;
}

void fake_hw_init(void)
{ 
}	

int fake_sensor_probe(void)
{
	return 0;
}

int fake_set_balance(balance_flag_t balance_flag,int arg)
{
	return 0;
}

int fake_set_effect(effect_flag_t effect_flag,int arg)
{
	return 0;
}

int fake_set_antibanding(antibanding_flag_t antibanding_flag,int arg)
{
	return 0;
}

int fake_set_flash_mode(flash_mode_flag_t flash_mode_flag,int arg)
{
	return 0;

}

int fake_set_scene_mode(scene_mode_flag_t scene_mode_flag,int arg)
{
	return 0;
}

int fake_set_focus_mode(focus_mode_flag_t flash_mode_flag,int arg)
{
	return 0;
}

int fake_set_output_format(pixel_format_flag_t pixel_format_flag,int arg)
{
	switch(pixel_format_flag){
		case PIXEL_FORMAT_JPEG:
			printk("pixel_format_jpeg");
			break;
		case PIXEL_FORMAT_YUV422SP:
			printk("pixel_format_yuv422sp"); 
			break;
		case PIXEL_FORMAT_YUV420SP:
			printk("pixel_format_yuv420sp");
			break;
		case PIXEL_FORMAT_YUV422I:
			if (current_fake_sensor->current_pixel_format == PIXEL_FORMAT_RGB565)
				current_fake_sensor->current_calc_width /= 2;
			off_set = 4;
			break;
		case PIXEL_FORMAT_RGB565:
			if (current_fake_sensor->current_pixel_format == PIXEL_FORMAT_YUV422I)
				current_fake_sensor->current_calc_width *= 2;
			off_set = 2;
			break;
		default:
			printk("unsupport format");
			return -1;
	}
	current_fake_sensor->current_pixel_format = pixel_format_flag;
	return 0;
}

int fake_sensor_init(void)
{
	return 1;
}


int fake_set_resolution(int width,int height,int bpp,pixel_format_flag_t fmt,camera_mode_t mode)
{
	switch(current_fake_sensor->current_pixel_format){
		case PIXEL_FORMAT_YUV422I:
			current_fake_sensor->current_calc_width = width / 2;
			current_fake_sensor->current_calc_height = height;
			break;
		case PIXEL_FORMAT_RGB565:
			current_fake_sensor->current_calc_width = width;
			current_fake_sensor->current_calc_height = height;
			break;
	}

	fake_set_output_format(PIXEL_FORMAT_YUV422I,0);

	return	0;
}

int fill_frame(unsigned int distination_addr, unsigned int current_frame_count)
{
	unsigned int width_count, height_count, calc_width, calc_height;
	unsigned char (*start_color)[4], (*source_color)[4], (*color_even)[4], (*color_odd)[4];
	pixel_format_flag_t current_format;

	calc_width = current_fake_sensor->current_calc_width;
	calc_height = current_fake_sensor->current_calc_height;
	color_even = &yuv422i_color_even[current_frame_count];
	color_odd = &yuv422i_color_odd[current_frame_count];
	current_format = current_fake_sensor->current_pixel_format;
	start_color = NULL;

	for (height_count = 0; height_count < calc_height; height_count++){
		if ((height_count % UNIT_HEIGHT) == 0){
			if ((height_count / UNIT_HEIGHT) % 2)
				start_color = color_even;
			else
				start_color = color_odd;
		}
		source_color = start_color;
		for (width_count = 0; width_count < calc_width; width_count++){
			switch (current_format)	{

				case PIXEL_FORMAT_YUV422I:
					if (((width_count * 2 % UNIT_WIDTH) == 0)&&width_count){
						if (source_color == color_odd)
							source_color = color_even;
						else
							source_color = color_odd;
					}
					break;
				case PIXEL_FORMAT_RGB565:
					printk("rgb565");
			}
			memcpy((unsigned char *)distination_addr, source_color, off_set);
			distination_addr += off_set;
		}
	}

	return 0;
}

int fake_set_mclk(unsigned int mclk)
{
	return 0;
}

int fake_fill_buffer(unsigned int distination_addr,void *desc)
{
	current_fake_sensor = container_of((struct camera_sensor_desc *)desc, struct fake_sensor_info, fake_sensor_desc);

	fill_frame(distination_addr, current_frame_count);
	current_frame_count++;
	if (current_frame_count == COLOR_NUM)
		current_frame_count = 0;
	return 0;
}

int fake_set_parameter(int cmd, int mode, int arg)
{
	switch(cmd)
	{
		case CPCMD_SET_BALANCE :
			fake_set_balance(mode,arg);
			break;
		case CPCMD_SET_EFFECT :
			fake_set_effect(mode,arg);
			break;
		case CPCMD_SET_ANTIBANDING :
			fake_set_antibanding(mode,arg);
			break;
		case CPCMD_SET_FLASH_MODE :
			fake_set_flash_mode(mode,arg);
			break;
		case CPCMD_SET_SCENE_MODE :
			fake_set_scene_mode(mode,arg);
			break;
		case CPCMD_SET_PIXEL_FORMAT :
			fake_set_output_format(mode,arg);
			break;
		case CPCMD_SET_FOCUS_MODE :
			fake_set_focus_mode(mode,arg);
			break;
	}
}

int fake_set_function(int function)
{
	return 0;
}

int fake_set_power(int state)
{
	switch (state)
	{
		case 0:           
			/* hardware power up first */
			fake_power_up();
			/* software power up later if it implemented */
			break;
		case 1:
			fake_power_down();
			break;
		case 2:
			break;
		default:
			printk("%s : EINVAL! \n",__FUNCTION__);
	}
	return 0;
}


struct camera_sensor_ops fake_sensor_ops = {
	.sensor_init = fake_sensor_init,
	.sensor_set_function =  fake_set_function,
	.sensor_set_resolution = fake_set_resolution, 
	.sensor_set_parameter = fake_set_parameter, 
	.sensor_set_power = fake_set_power,

	.camera_sensor_probe = fake_sensor_probe,
	.camera_fill_buffer = fake_fill_buffer,
};

struct resolution_info fake_resolution_table[] = {
	{2048,1536,16,PIXEL_FORMAT_YUV422I},
	{1024,768,16,PIXEL_FORMAT_YUV422I},
	{640,480,16,PIXEL_FORMAT_YUV422I},
        {352,288,16,PIXEL_FORMAT_YUV422I},
	{320,240,16,PIXEL_FORMAT_YUV422I},
	{176,144,16,PIXEL_FORMAT_YUV422I}
};

struct fake_sensor_info fake_sensor = {
	.fake_sensor_desc={
		.name = "fake",
		.camera_clock = CAM_CLOCK,
		.no_dma = 1,
		.ops = &fake_sensor_ops,
		.flags = {
			.pixel_format_flag = PIXEL_FORMAT_YUV422I | PIXEL_FORMAT_RGB565,
		},
		.resolution_table = fake_resolution_table,
		.resolution_table_nr=ARRAY_SIZE(fake_resolution_table),

		.preview_parm = {1024,768, DEF_PRE_BPP,PIXEL_FORMAT_YUV422I}, 
		.max_preview_parm = {1024,768, MAX_PRE_BPP,PIXEL_FORMAT_YUV422I},

		.capture_parm = {2048,1536 , DEF_CAP_BPP,PIXEL_FORMAT_YUV422I},
		.max_capture_parm = {2048,1536, MAX_CAP_BPP,PIXEL_FORMAT_YUV422I},
	},
	.current_calc_width = DEF_PRE_WIDTH / 2,
	.current_calc_height = DEF_PRE_HEIGHT,
	.current_pixel_format = PIXEL_FORMAT_YUV422I,
};

int fake_register(void)
{
	unsigned int i;
	unsigned char *name = "fake ";
	struct fake_sensor_info *fake_info_tmp;

	for ( i = 0; i < MAX_SENSOR_NUM; i++){
		name[4] = i + '0';
		fake_info_tmp = (struct fake_sensor_info *)kmalloc(sizeof(struct fake_sensor_info), GFP_KERNEL);
		memcpy(fake_info_tmp, &fake_sensor, sizeof(struct fake_sensor_info));
		memcpy(fake_info_tmp->fake_sensor_desc.name, name, 15);
		if (i == 0)
			current_fake_sensor = fake_info_tmp;
		camera_sensor_register(&(fake_info_tmp->fake_sensor_desc));
	}

	return 0;
}

module_init(fake_register);
