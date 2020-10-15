#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>

int main(int argc, char **argv)
{
	int on;
	int led_no;
	int fd;
	unsigned char read_size;

	unsigned char channel;
	unsigned char mode;
	unsigned int value1;
	unsigned int value2;

	/*
	   if (argc != 3 || sscanf(argv[1], "%d", &led_no) != 1 || sscanf(argv[2],"%d", &on) != 1 ||
	   on < 0 || on > 1 || led_no < 0 || led_no > 3) {
	   fprintf(stderr, "Usage: leds led_no 0|1\n");
	   exit(1);
	   }
	 */

	if(argc!=5)
	{
		printf("Input the correct arguments\n");
		exit(1);	
	}

	channel = atoi(argv[1]);
	mode = atoi(argv[2]);
	value1 = atoi(argv[3]);
	value2 = atoi(argv[4]);

	fd = open("/dev/TcuTest",O_RDWR);
	//	if (fd < 0) {
	//		fd = open("/dev/leds", 0);
	//	}
	if (fd < 0) {
		perror("open device leds");
		exit(1);
	}
	else
	{
		printf("\nopen successful!!!\n");
	}
	unsigned char *p0;
	//	unsigned int tcu_mod;
	//	tcu_mod = 1+(1<<3)+(7<<4)+(1<<9)+(10<<10);
	//	tcu_mod = 1+(1<<3)+(7<<4);
	//	tcu_mod = 1+(1<<2);  //this data equal to the register TMOD4
	if(mode == 1)         // FIFO MODE test
	{
		ioctl(fd,4,channel);
		ioctl(fd,5,value1);

		p0=(void *)malloc(16*2*20*4);
		unsigned int *p1;
		p1=(unsigned int *)p0;
		unsigned int i,tmp;
		for (i=0;i<16*2*20;i++)
		{

			tmp = i%16;
			if(tmp)
				*(p1+i)= (tmp<<27)+(tmp<<11);//0x11003300;
			else
				*(p1+i)= 0x10001000;

		}
		printf("\nfinish write!!!!\n");
		read_size=write(fd,p0,16*2*20*4);

		ioctl(fd,0,0);
	}
	else if(mode == 2)    // no fifo mode test 
	{
		ioctl(fd,4,channel);
		ioctl(fd,3,value1);
		ioctl(fd,2,value2);
		ioctl(fd,0,0);
	}
	else if(mode == 3) //bypass mode test
	{
		ioctl(fd,4,channel);
		ioctl(fd,10,0);
	}
	else if(mode == 4) //Get count
	{
		ioctl(fd,4,channel);
		ioctl(fd,13,value1);
	}
	else if(mode == 5) //Select clock
	{
		ioctl(fd,4,channel);
		ioctl(fd,12,value1);
	}
	else if(mode == 6) //Select div
	{
		ioctl(fd,4,channel);
		ioctl(fd,11,value1);
	}
	else if(mode == 7) //Start TCU2 mode
	{
		ioctl(fd,4,channel);
		ioctl(fd,14,0);
	}
	else if(mode == 8) //Stop TCU2 mode
	{
		ioctl(fd,4,channel);
		ioctl(fd,15,0);
	}
	else
	{
		ioctl(fd,4,channel);
		ioctl(fd,1,0);
	}

	close(fd);
	return 0;
}

