cmd_/home/adv/work/xenotest/xmmap/drv/modules.order := {   echo /home/adv/work/xenotest/xmmap/drv/xmmap_module.ko; :; } | awk '!x[$$0]++' - > /home/adv/work/xenotest/xmmap/drv/modules.order
