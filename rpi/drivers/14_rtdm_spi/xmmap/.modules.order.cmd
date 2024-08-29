cmd_/home/adv/work/xenotest/xmmap/modules.order := {   echo /home/adv/work/xenotest/xmmap/xmmap_module.ko; :; } | awk '!x[$$0]++' - > /home/adv/work/xenotest/xmmap/modules.order
