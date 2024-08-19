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

int main(int argc, char **argv)
{
	int flags;
	int i;
	int val;
	int fd;
	
	/* 1. 判断参数 */
	if (argc != 2) 
	{
		printf("Usage: %s <dev>\n", argv[0]);
		return -1;
	}



	/* 4打开文件-非阻塞方式打开 */
	fd = open(argv[1], O_RDWR | O_NONBLOCK);
	if (fd == -1)
	{
		printf("can not open file %s\n", argv[1]);
		return -1;
	}

	// 先以非阻塞方式读10次，这里会立即执行
	for (i = 0; i < 10; i++)
	{
		if (read(fd, &val, 4) == 4)
		{
			printf("get button: 0x%x\n", val);
		}
		else
		{
			printf("get button: -1\n");
		}
	}
	

	// 读取驱动程序flag并设置阻塞方式
	flags = fcntl(fd, F_GETFL);
	fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
	
	while (1)
	{
		// 这里就是阻塞方式读数据了
		if (read(fd, &val, 4) == 4)
		{
			printf("get button: 0x%x\n", val);
		}
		else
		{
			printf("while get button: -1\n");
		}
	}
	
	close(fd);
	
	return 0;
}


