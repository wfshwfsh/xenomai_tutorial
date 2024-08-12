# GPIO RTDM Driver IO Test

## Driver
* search rpi4 device node compatiable name = ""
* modify gpio-bcm2835.c register compatiable name

## Application
* rt_dev_open, rt_dev_close
* rt_dev_ioctl => ioctl_nrt for setting gpio direction with value
* rt_dev_ioctl => ioctl_nrt for set up irq
* rt_dev_write => write_rt
* rt_dev_write => read_rt

# Validate Result
* Using rt_dev_write from user app in rt_task => MSW not increasing