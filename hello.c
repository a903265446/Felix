#include <linux/init.h>
#include <linux/module.h>

static char *name = "zhengjunfei";
module_param(name, charp, S_IRUGO);

static int age = 28;
module_param(age, int, S_IRUGO);

static int __init hello_init(void)
{
	printk(KERN_INFO "hello world enter, name = %s, age = %d\n", name, age);
	return 0;
}
EXPORT_SYMBOL_GPL(hello_init);

static void __exit hello_exit(void)
{
	printk(KERN_INFO "hello world exit\n");
}


module_init(hello_init);
module_exit(hello_exit);
MODULE_AUTHOR("Junfei Zheng <zhengjunfei@oradt.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("a simple hello world module");
MODULE_ALIAS("a simplest module");
