#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("SSP"); 
MODULE_DESCRIPTION("my first driver"); 

static struct proc_dir_entry *custom_proc_node;

static ssize_t	pyjama_read( struct file *fileptr,
                             char __user *user_space_buffer, 
                             size_t count, 
                             loff_t *offset){
    char msg[] = "Ack!\n";
    size_t len = strlen(msg);
    int result;
    if(*offset >= len){
        return 0;
    }
    result = copy_to_user(user_space_buffer, msg, len);
    *offset += len;
    printk("pyjama_read\n");
    return len;
}

static ssize_t	pyjama_write(struct file *fileptr, 
                             const char __user *user_space_buffer,
                             size_t count, 
                             loff_t *offset){

}

struct proc_ops driver_proc_ops = {
    .proc_read = pyjama_read
    .proc_write = pyjama_write
};


static int pyjama_module_init (void) {
    printk("module_init: entry\n");

    custom_proc_node = proc_create(
                                    "pyjama_driver",
                                     0,
                                     NULL,
                                     &driver_proc_ops
    );
    printk("module_init: exit\n");
    return 0;
}

static void pyjama_module_exit (void) {
    printk("Good, Bye! (from InPyjama!!)\n");
}

module_init(pyjama_module_init);
module_exit(pyjama_module_exit);