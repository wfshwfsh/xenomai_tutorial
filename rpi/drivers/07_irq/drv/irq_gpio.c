#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>


// BCM GPIO IN 23 -> Pin 16
#define GPIO_IN  23
// BCM GPIO OUT 22 -> Pin 15
#define GPIO_OUT 22

static int led_trigger = 0;
static int numero_interruption = 0;

static irqreturn_t button_isr(int irq, void *data)
{
	unsigned long flags = 0;
	local_irq_save(flags);
	printk("button_isr !!!!\n");
	gpio_set_value(GPIO_OUT, led_trigger);
	led_trigger = led_trigger ? (0):(1);
	local_irq_restore(flags);
	return IRQ_HANDLED;
}

static int __init exemple_init (void)
{
    int err;
    numero_interruption = gpio_to_irq(GPIO_IN);

    if ((err = gpio_request(GPIO_IN, THIS_MODULE->name)) != 0) {
        return err;
    }
    if ((err = gpio_direction_input(GPIO_IN)) != 0) {
        gpio_free(GPIO_IN);
        return err;
    }
    if ((err = gpio_request(GPIO_OUT, THIS_MODULE->name)) != 0) {
        gpio_free(GPIO_IN);
        return err;
    }
    if ((err = gpio_direction_output(GPIO_OUT, 1)) != 0) {
        gpio_free(GPIO_OUT);
        gpio_free(GPIO_IN);
        return err;
    }

    irq_set_irq_type(numero_interruption,  0x00000003);
    
    if ((err = request_irq(numero_interruption, 
				    button_isr, IRQF_TRIGGER_RISING, 
				    "my_button_int", "my_dev")) != 0){
    	gpio_free(GPIO_OUT);
        gpio_free(GPIO_IN);
	return err;
    }

    return 0; 
}

static void __exit exemple_exit (void)
{
    gpio_set_value(GPIO_OUT, 0);
    free_irq(numero_interruption, "my_dev");

    gpio_free(GPIO_OUT);
    gpio_free(GPIO_IN);
}

module_init(exemple_init);
module_exit(exemple_exit);
MODULE_LICENSE("GPL");
