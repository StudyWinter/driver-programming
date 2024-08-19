#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>

/*
 * ./button_test /dev/100ask_button0
 *
 */
int main(int argc, char **argv)
{
	int fd;
	int val;
	struct pollfd fds[1];
	int timeout_ms = 5000;			// 超时时间
	int res;
	
	/* 1. 判断参数 */
	if (argc != 2) 
	{
		printf("Usage: %s <dev>\n", argv[0]);
		return -1;
	}

	/* 2. 打开文件 */
	fd = open(argv[1], O_RDWR);
	if (fd == -1)
	{
		printf("can not open file %s\n", argv[1]);
		return -1;
	}
	
	fds[0].fd = fd;					// 文件描述符赋值
	fds[0].events = POLLIN;			// 查询读事件
	
	while (1)
	{
		/* 3. 读文件 */
		res = poll(fds, 1, timeout_ms);
		// 返回值为正数，表示有多少个文件符合要求
		// 并且返回值是pollin
		if ((res == 1) && (fds[0].revents & POLLIN))
		{
			read(fd, &val, 4);
			printf("get button : 0x%x\n", val);
		}
		else
		{
			// 超时
			printf("time out\n");
		}

		
		
	}
	
	close(fd);
	
	return 0;
}


