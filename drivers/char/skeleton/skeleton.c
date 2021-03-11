#include <linux/module.h>

static int __init skeleton_init(void)
{
	printk(KERN_INFO "Skeleton init\n");
	return 0;
}

static void __exit  skeleton_exit(void)
{
	printk("Skeleton exit\n");
}

module_init(skeleton_init);
module_exit(skeleton_exit);

MODULE_AUTHOR("Balint Horvath");
MODULE_DESCRIPTION("Driver skeleton");
MODULE_LICENSE("GPL");


