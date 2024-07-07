#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xd153cc49, "module_layout" },
	{ 0xc1514a3b, "free_irq" },
	{ 0xfe990052, "gpio_free" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0xbc477a2, "irq_set_irq_type" },
	{ 0xfd27d57f, "gpiod_direction_output_raw" },
	{ 0x4ce14821, "gpiod_direction_input" },
	{ 0x47229b5c, "gpio_request" },
	{ 0x1d1517b2, "gpiod_to_irq" },
	{ 0x3d28337e, "inband_irq_restore" },
	{ 0xd697e69a, "trace_hardirqs_on" },
	{ 0x2691af29, "gpiod_set_raw_value" },
	{ 0x4994255b, "gpio_to_desc" },
	{ 0xc5850110, "printk" },
	{ 0xec3d2e1b, "trace_hardirqs_off" },
	{ 0xbcea4a9f, "inband_irq_save" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "FA3B1604F61A6AB254BBF5E");
