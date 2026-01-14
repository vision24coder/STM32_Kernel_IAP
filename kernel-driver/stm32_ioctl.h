#ifndef STM32_IOCTL_H
#define STM32_IOCTL_H

#include <linux/ioctl.h>

#define STM32_IOC_MAGIC 'k'

#define STM32_IOC_SET_BAUD      _IOW(STM32_IOC_MAGIC, 1, int)
#define STM32_IOC_CLR_BUF       _IO(STM32_IOC_MAGIC,  2)
#define STM32_IOC_GET_STAT      _IOR(STM32_IOC_MAGIC, 3, int)
#define STM32_IOC_GET_LEVEL     _IOR(STM32_IOC_MAGIC, 4, int)
#define STM32_IOC_RESET_PTR     _IO(STM32_IOC_MAGIC,  5)
#define STM32_IOC_ENTER_BOOT    _IO(STM32_IOC_MAGIC, 6)

#endif