#include <linux/module.h>
#include <linux/poll.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/kmod.h>
#include <linux/gfp.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/timer.h>




// 封装一个结构体
struct gpio_key  {
	int gpio;						// gpio编号
	int irq;						// 中断号
	enum of_gpio_flags flag;		// flag
	struct gpio_desc *gpiod;		// 描述
	struct timer_list key_timer;	// 每个按键都有一个定时器
	struct tasklet_struct tasklet;	// 每一个按键都有一个tasklet
};

// 结构体指针
static struct gpio_key* p_gpio_keys;
static int major = 0;			// 主设备号
static struct class* gpio_key_class;
struct fasync_struct* button_fasync;



// 环形缓冲区
#define BUF_LEN 128
static int g_keys[BUF_LEN];
static int r, w;

// 下一个位置
#define NEXT_POS(x) ((x+1) % BUF_LEN)

// 判断环形缓冲区是否为空
static int is_key_buf_empty(void)
{
	// 相等时为空
	return (r == w);
}

// 判断缓冲区是否满
static int is_key_buf_full(void)
{
	return (r == NEXT_POS(w));
}

// 写数据进去
static void put_key(int key)
{
	if (!is_key_buf_full())
	{
		g_keys[w] = key;
		// w后移
		w = NEXT_POS(w);
	}
}

// 获得数据
static int get_key(void)
{
	int key = 0;
	if (!is_key_buf_empty())
	{
		key = g_keys[r];
		// r后移
		r = NEXT_POS(r);
	}
	return key;
}

// 初始化等待队列
static DECLARE_WAIT_QUEUE_HEAD(gpio_key_wait);

// 按键超时时间函数，发送信号
static void key_timer_expire(struct timer_list *t)
{
	int val;
	int key;
	/* data ==> gpio */
	// 从结构体变量地址反算出整个结构体的地址
	struct gpio_key *gpio_key = from_timer(gpio_key, t, key_timer);
	
	val = gpiod_get_value(gpio_key->gpiod);
	printk("key_timer_expire key %d %d\n", gpio_key->gpio, val);
	// 哪一个按键放在高8位，按下/松开是val
	key = (gpio_key->gpio << 8) | val;
	// 加入数据
	put_key(key);
	
	// 唤醒g_key队列
	wake_up_interruptible(&gpio_key_wait);
	// 中断处理函数中使用kill_fasync发送信号
	kill_fasync(&button_fasync, SIGIO, POLL_IN);
}

// tasklet函数
static void key_tasklet_func(unsigned long data)
{
	/* data ==> gpio */
	struct gpio_key *gpio_key = data;
	int val;

	val = gpiod_get_value(gpio_key->gpiod);
	// 打印信息
	printk("key_tasklet_func key %d %d\n", gpio_key->gpio, val);
}



// 实现自己的read函数，等待按键按下
static ssize_t gpio_key_drv_read (struct file *file, char __user *buf, size_t size, loff_t *offset)
{
	int err;
	int key;
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);

	// 环形缓冲区是空的 && 不阻塞
	if (is_key_buf_empty() && (file->f_flags & O_NONBLOCK))
	{
		return -EAGAIN;
	}
	
	// 当is_key_buf_empty为假时，也就是不空，休眠才被打断，才能get数据
	wait_event_interruptible(gpio_key_wait, !is_key_buf_empty());
	key = get_key();
	err = copy_to_user(buf, &key, 4);
	return 4;
}


// 实现自己的poll函数
static __poll_t gpio_key_drv_poll(struct file *file, struct poll_table_struct *pt)
{
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	// 把当前线程挂入队列，并不会休眠
	poll_wait(file, &gpio_key_wait, pt);
	// 如果队列是空的，就返回0，如果不是空的，就返回读到的数据
	return is_key_buf_empty() ? 0 : POLLIN | POLLRDNORM;
}

static int gpio_key_drv_fasync(int fd, struct file *file, int on)
{
	if (fasync_helper(fd, file, on, &button_fasync) >= 0)
	{
		return 0;
	}
	else
	{
		return -EIO;
	}
}


// 定义自己的file_operations结构体
static struct file_operations gpio_key_drv = {
	.owner = THIS_MODULE,
	.read = gpio_key_drv_read,
	.poll = gpio_key_drv_poll,
	.fasync = gpio_key_drv_fasync,
};


// 中断服务程序，修改定时器时间，启动下半部
static irqreturn_t gpio_key_irq_winter(int irq, void* dev_id)
{
	struct gpio_key *gpio_key = dev_id;
	// 打印耗时，不在这里打印
	// printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	// 启动一个下半部
	tasklet_schedule(&gpio_key->tasklet);
	mod_timer(&gpio_key->key_timer, jiffies + HZ / 50);    // 20ms
	return IRQ_HANDLED;	
}

// probe函数
/* 1. 从platform_device获得GPIO
 * 2. gpio=>irq，gpio资源转换成中断号
 * 3. request_irq
 */
static int gpio_keys_probe(struct platform_device* pdev)
{
	int count, i;
	int err;
	unsigned flags = GPIOF_IN;
	struct device_node* node = pdev->dev.of_node;
	enum of_gpio_flags flag;
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	// 1获得gpio
	count = of_gpio_count(node);
	if (!count)
	{
		printk("%s %s line %d, there isn't any gpio available\n", __FILE__, __FUNCTION__, __LINE__);
		return -1;
	}
	
	// 分配空间，分配好了，内存情况k:kernel，z:zero
	p_gpio_keys = kzalloc(count * sizeof(struct gpio_key), GFP_KERNEL);

	for (i = 0; i < count; i++)
	{
		// 把设备结点中gpio的每个引脚都取出来，再转换成中断号
		// 转换成中断号
		p_gpio_keys[i].gpio = of_get_gpio_flags(node, i, &flag);
		if (p_gpio_keys[i].gpio < 0)
		{
			printk("%s %s line %d, of_get_gpio_flags fail\n", __FILE__, __FUNCTION__, __LINE__);
			return -1;
		}
		p_gpio_keys[i].gpiod = gpio_to_desc(p_gpio_keys[i].gpio);
		p_gpio_keys[i].flag = flag & OF_GPIO_ACTIVE_LOW; // 设备树中的flag:GPIO_ACTIVE_LOW & OF_GPIO_ACTIVE_LOW

		if (flag & OF_GPIO_ACTIVE_LOW)
		{
			flags |= GPIOF_ACTIVE_LOW;
		}
		err = devm_gpio_request_one(&pdev->dev, p_gpio_keys[i].gpio, flags, NULL);
		p_gpio_keys[i].irq  = gpio_to_irq(p_gpio_keys[i].gpio);

		// 设置定时器
		timer_setup(&p_gpio_keys[i].key_timer, key_timer_expire, 0);
		// 设置超时时间，最大值
		p_gpio_keys[i].key_timer.expires = ~0;
		// 添加定时器
		add_timer(&p_gpio_keys[i].key_timer);
		// 初始化tasklet
		tasklet_init(&p_gpio_keys[i].tasklet, key_tasklet_func, &p_gpio_keys[i]);
	}
	for (i = 0; i < count; i++)
	{
		// 双边沿触发，当发生irq中断时，内核会调用gpio_key_irq_winter中断处理函数，同时会把&p_gpio_keys[i])参数传进去
		err = request_irq(p_gpio_keys[i].irq, gpio_key_irq_winter, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "winter_gpio_key", &p_gpio_keys[i]);
	}

	/* 注册file_operations 	*/
	major = register_chrdev(0, "winter_gpio_key", &gpio_key_drv);  /* /dev/100ask_gpio_key */

	gpio_key_class = class_create(THIS_MODULE, "winter_gpio_key_class");
	if (IS_ERR(gpio_key_class)) {
		printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
		unregister_chrdev(major, "winter_gpio_key");
		return PTR_ERR(gpio_key_class);
	}

	device_create(gpio_key_class, NULL, MKDEV(major, 0), NULL, "winter_gpio_key"); /* /dev/100ask_gpio_key */
	return 0;
}

// remove函数-释放
int gpio_keys_remove(struct platform_device* pdev)
{
	struct device_node* node= pdev->dev.of_node;
	int count, i;
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);

	device_destroy(gpio_key_class, MKDEV(major, 0));
	class_destroy(gpio_key_class);
	unregister_chrdev(major, "winter_gpio_key");


	
	count = of_gpio_count(node);
	for (i = 0; i < count; i++)
	{
		free_irq(p_gpio_keys[i].irq, &p_gpio_keys[i]);
		// 删除定时器
		del_timer(&p_gpio_keys[i].key_timer);
		// 删除tasklet
		tasklet_kill(&p_gpio_keys[i].tasklet);
	}
	// 释放空间
	kfree(p_gpio_keys);
	return 0;
}

// 资源。compatible的数据和设备树结点的数据匹配后，就会调用probe函数
static const struct of_device_id winter_gpio_keys[] = {
	{ .compatible = "winter,gpio_key", },
	{ },
};


// 1定义一个platform_driver结构体
static struct platform_driver gpio_key_driver = {
	.probe		= gpio_keys_probe,
	.remove		= gpio_keys_remove,
	.driver		= {
		.name	= "winter_gpio-keys",
		.of_match_table = winter_gpio_keys,
	}
};


// 2入口函数
static int __init gpio_keys_init(void)
{
	int err;
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	// 注册platform_driver结构体
	err = platform_driver_register(&gpio_key_driver);
	return 0;
}

// 出口函数
static void __exit gpio_keys_exit(void)
{
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	platform_driver_unregister(&gpio_key_driver);
}

module_init(gpio_keys_init);
module_exit(gpio_keys_exit);
MODULE_LICENSE("GPL");


