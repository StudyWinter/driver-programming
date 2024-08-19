#include <linux/module.h>

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



// 封装一个结构体
struct gpio_key  {
	int gpio;						// gpio编号
	int irq;						// 中断号
	enum of_gpio_flags flag;		// flag
	struct gpio_desc *gpiod;		// 描述
};

// 结构体指针
static struct gpio_key* p_gpio_keys;


// 中断服务程序，
static irqreturn_t gpio_key_irq_winter(int irq, void* dev_id)
{
	struct gpio_key *gpio_key = dev_id;
	int val;
	val = gpiod_get_value(gpio_key->gpiod);
	printk("key %d %d\n", gpio_key->gpio, val);
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
	}
	for (i = 0; i < count; i++)
	{
		// 双边沿触发，当发生irq中断时，内核会调用gpio_key_irq_winter中断处理函数，同时会把&p_gpio_keys[i])参数传进去
		err = request_irq(p_gpio_keys[i].irq, gpio_key_irq_winter, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "winter_gpio_key", &p_gpio_keys[i]);
	}
	return 0;
}

// remove函数-释放
int gpio_keys_remove(struct platform_device* pdev)
{
	struct device_node* node= pdev->dev.of_node;
	int count, i;
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	count = of_gpio_count(node);
	for (i = 0; i < count; i++)
	{
		free_irq(p_gpio_keys[i].irq, &p_gpio_keys[i]);		
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



