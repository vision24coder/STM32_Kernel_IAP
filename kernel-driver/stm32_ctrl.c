#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/termios.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include "stm32_ioctl.h"

#define STM32_BUF_SIZE 1024
#define UART_DEV_PATH "/dev/ttyACM0"

struct scull_pipe{
    char* buffer;
    char* rp;
    char* wp;
    char* end;
    wait_queue_head_t inq;
    struct semaphore sem;
};

static dev_t dev;
static struct cdev stm32_cdev;
static struct class *stm32_ctrl_class;
static struct scull_pipe my_device;
static atomic_t open_ready = ATOMIC_INIT(1);
static struct tty_struct *stm32_tty;
static struct file *uart_filp;
static struct task_struct *rx_thread;

static int uart_rx_thread(void *data){
    char ch;
    loff_t pos = 0;
    char *next;
    while(!kthread_should_stop()){
        if(kernel_read(uart_filp, &ch, 1, &pos) == 1){
            down(&my_device.sem);
            next = my_device.wp + 1;
            if(next == my_device.end){
                next = my_device.buffer;
            }
            if(next != my_device.rp){
                *my_device.wp = ch;
                my_device.wp = next;
                wake_up_interruptible(&my_device.inq);
            }
            up(&my_device.sem);
            
        }
        else{
            msleep(10);
        }
    }
    return 0;
}

static  int stm32_ctrl_open(struct inode *inode, struct file *file){
    printk("stm32_ctrl in open\n");
    if(! atomic_dec_and_test (&open_ready)){
        atomic_inc(&open_ready);
        return -EBUSY;
    }
    file->private_data = &my_device;

    if(stm32_tty){
        return 0;
    }
    uart_filp = filp_open(UART_DEV_PATH, O_RDWR | O_NOCTTY | O_NONBLOCK, 0);
    
    if(IS_ERR(uart_filp)){
        atomic_inc(&open_ready);
        return PTR_ERR(uart_filp);
    }
    if(!uart_filp->private_data){
        filp_close(uart_filp, NULL);
        uart_filp = NULL;
        atomic_inc(&open_ready);
        return -ENODEV;
    }
    stm32_tty = uart_filp->private_data;
    rx_thread = kthread_run(uart_rx_thread, NULL, "stm32_uart_rx");
    if(IS_ERR(rx_thread)){
        filp_close(uart_filp, NULL);
        uart_filp = NULL;
        stm32_tty = NULL;
        atomic_inc(&open_ready);
        return PTR_ERR(rx_thread);
    }
    return 0;
}

static  int stm32_ctrl_release(struct inode *inode, struct file *file){
    printk("stm32_ctrl in close\n");
    if(uart_filp){
        filp_close(uart_filp, NULL);
        uart_filp = NULL;
        stm32_tty = NULL;
        kthread_stop(rx_thread);
        rx_thread = NULL;
    }
    atomic_set(&open_ready, 1);
    file->private_data = NULL;

    return 0;
}

static ssize_t stm32_ctrl_read(struct file *file, char __user *buf, size_t count, loff_t *offset){
    printk("stm32_ctrl in read\n");
    struct scull_pipe *my_device = file->private_data;
    if(down_interruptible(&my_device->sem)){
        return -ERESTARTSYS;
    }

    while(my_device->rp == my_device->wp){
        up(&my_device->sem);
        if(file->f_flags & O_NONBLOCK){
            return -EAGAIN;
        }
        if(wait_event_interruptible(my_device->inq, my_device->rp != my_device->wp)){
            return -ERESTARTSYS;
        }
        if(down_interruptible(&my_device->sem)){
            return -ERESTARTSYS;
        }
    }
    if(my_device->wp > my_device->rp){
        count = min(count, (size_t)(my_device->wp - my_device->rp));
    }
    else{
        count = min(count, (size_t)(my_device->end - my_device->rp));
   }
    if(copy_to_user(buf, my_device->rp, count)){
        up(&my_device->sem);
        return -EFAULT;
    }
   my_device->rp += count;
    if(my_device->rp == my_device->end){
        my_device->rp = my_device->buffer;
    }
    up(&my_device->sem);
    return count;
}

static ssize_t stm32_ctrl_write(struct file *file, const char __user *buf, size_t count, loff_t *offset){
    printk("stm32_ctrl in write\n");
    int ret;
    char *kbuf;
    loff_t pos = 0;
    if(!stm32_tty){
        return -ENODEV;
    }
    kbuf = kmalloc(count, GFP_KERNEL);
    if(!kbuf){
        return -ENOMEM;
    }
    if(copy_from_user(kbuf, buf, count)){
        kfree(kbuf);
        return -EFAULT;
    }
    ret = kernel_write(uart_filp, kbuf, count, &pos);
    kfree(kbuf);
    return ret;
}

static long stm32_ctrl_ioctl(struct file *file, unsigned int cmd, unsigned long arg){
    printk("stm32_ctrl in ioctl\n");
    int level, mode;
    struct scull_pipe *my_device = file->private_data;
    loff_t pos = 0;
    switch(cmd){
        case STM32_IOC_SET_BAUD: {
            struct ktermios termios;
            int baud;
            if(copy_from_user(&baud, (int __user *)arg, sizeof(int))){
                return -EFAULT;
            }
            if(!stm32_tty){
                return -ENODEV;
            }
            down_write(&stm32_tty->termios_rwsem);
            termios = stm32_tty->termios;
            switch(baud){
                case 9600:
                    tty_encode_baud_rate(stm32_tty, 9600, 9600);
                    kernel_write(uart_filp, "I_9600\0", 7, NULL);
                    break;
                case 115200:
                    tty_encode_baud_rate(stm32_tty, 115200, 115200);
                    kernel_write(uart_filp, "I_115200\0", 9, NULL);
                    break;
                default:
                    up_write(&stm32_tty->termios_rwsem);
                    return -EINVAL;
            }
            tty_set_termios(stm32_tty, &termios);
            up_write(&stm32_tty->termios_rwsem);
            break;
        }
        case STM32_IOC_CLR_BUF:
            down(&my_device->sem);
            my_device->rp = my_device->wp = my_device->buffer;
            up(&my_device->sem);
            break;
        
        case STM32_IOC_GET_STAT:
            kernel_write(uart_filp, "STATUS\0", 7, NULL);
            if(copy_to_user((int __user *)arg, &mode, sizeof(int))){
                return -EFAULT;
            }
            break;

        case STM32_IOC_GET_LEVEL:
            down(&my_device->sem);
            if (my_device->wp >= my_device->rp){
                level = my_device->wp - my_device->rp;
            }
            else{
                level = (my_device->end - my_device->rp) + (my_device->wp - my_device->buffer);
            }    
            up(&my_device->sem);
            if (copy_to_user((int __user *)arg, &level, sizeof(int))){
                return -EFAULT;
            }           
            break;
        
        case STM32_IOC_ENTER_BOOT:
            if(!uart_filp){
                return -ENODEV;
            }
            kernel_write(uart_filp, "ENTER_BOOT\0", 11, NULL);
            break;
        
        case STM32_IOC_RESET_PTR:
            down(&my_device->sem);
            my_device->rp = my_device->wp = my_device->buffer;
            up(&my_device->sem);
            break;

        default:
            return -ENOTTY;
    }
    return 0;
}

static struct file_operations stm32_ctrl_fops = {
    .owner = THIS_MODULE,
    .open = stm32_ctrl_open,
    .read = stm32_ctrl_read,
    .write = stm32_ctrl_write,
    .release = stm32_ctrl_release,
    .unlocked_ioctl = stm32_ctrl_ioctl
};

static int stm32_ctrl_init (void) {
    printk("module_init: entry\n");
    if(alloc_chrdev_region(&dev, 0, 1, "stm32_ctrl")){
        return -EINVAL;
    }
    cdev_init(&stm32_cdev, &stm32_ctrl_fops);
    cdev_add(&stm32_cdev, dev, 1);
    sema_init(&my_device.sem, 1); 
    init_waitqueue_head(&my_device.inq);

    my_device.buffer = kmalloc(STM32_BUF_SIZE, GFP_KERNEL);
    if(!my_device.buffer){
        return -ENOMEM;
    }
    my_device.end = my_device.buffer + STM32_BUF_SIZE;
    my_device.rp = my_device.buffer;
    my_device.wp = my_device.buffer;

    stm32_ctrl_class = class_create("stm32_ctrl");
    device_create(stm32_ctrl_class, NULL, dev, NULL, "stm32_ctrl");
    printk("module_init: exit\n");
    return 0;
}

static void stm32_ctrl_exit (void) {
    device_destroy(stm32_ctrl_class, dev);
    class_destroy(stm32_ctrl_class);
    cdev_del(&stm32_cdev);
    unregister_chrdev_region(dev, 1);
    kfree(my_device.buffer);
    printk("Good, Bye!;)\n");
}

module_init(stm32_ctrl_init);
module_exit(stm32_ctrl_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("SSP");
MODULE_DESCRIPTION("stm32_control_driver");