#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("SSP"); 
MODULE_DESCRIPTION("my first driver"); 

static dev_t dev;
static struct cdev pyjama_cdev;
static struct class *pyjama_class;

static  int pyjama_open(struct inode *inode, struct file *file){
    printk("pyjama open\n");
    return 0;
}

static  int pyjama_release(struct inode *inode, struct file *file){
    printk("pyjama close\n");
    return 0;
}

static ssize_t pyjama_read(struct file *file, char __user *buff, size_t count, loff_t *offset){
    char msg[] = "hello there!\n";
    size_t len = strlen(msg);
    if(*offset>=len){
        return 0;
    }
    int result = copy_to_user(buff, msg, len);
    *offset += len;
    return len;
}

static struct file_operations pyjama_fops = {
    .owner = THIS_MODULE,
    .open = pyjama_open,
    .read = pyjama_read,
    .release = pyjama_release
};

static int pyjama_module_init (void) {
    printk("module_init: entry\n");
    alloc_chrdev_region(&dev, 0, 1, "pyjama");
    cdev_init(&pyjama_cdev, &pyjama_fops);
    cdev_add(&pyjama_cdev, dev, 1);

    pyjama_class = class_create("pyjama");
    device_create(pyjama_class, NULL, dev, NULL, "pyjama");
    printk("module_init: exit\n");
    return 0;
}

static void pyjama_module_exit (void) {
    device_destroy(pyjama_class, dev);
    class_destroy(pyjama_class);
    cdev_del(&pyjama_cdev);
    unregister_chrdev_region(dev, 1);
    printk("Good, Bye! (from InPyjama!!)\n");
}

module_init(pyjama_module_init);
module_exit(pyjama_module_exit);