#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>
#include <signal.h>


/*
 * ./button_test /dev/100ask_button0
 *
 */

static int fd;

// 2写信号处理函数
static void sig_func(int sig)
{
	int val;
	// 读按键
	read(fd, &val, 4);
	printf("get button: 0x%x\n", val);
}

int main(int argc, char **argv)
{
	int flags;
	
	/* 1. 判断参数 */
	if (argc != 2) 
	{
		printf("Usage: %s <dev>\n", argv[0]);
		return -1;
	}

	// 3注册信号处理函数
	signal(SIGIO, sig_func);


	/* 4打开文件 */
	fd = open(argv[1], O_RDWR);
	if (fd == -1)
	{
		printf("can not open file %s\n", argv[1]);
		return -1;
	}

	// 5把APP的pid告诉驱动程序
	fcntl(fd ,F_SETOWN, getpid());

	// 6读取驱动程序flag并设置FASYNC位为1
	flags = fcntl(fd, F_GETFL);
	fcntl(fd, F_SETFL, flags | FASYNC);
	
	while (1)
	{
		printf("https://blog.csdn.net/Zhouzi_heng\n");
		sleep(2);
	}
	
	close(fd);
	
	return 0;
}


