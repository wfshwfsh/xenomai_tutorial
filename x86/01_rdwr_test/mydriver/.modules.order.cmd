cmd_/home/adv/git/xenomai/mydriver/modules.order := {   echo /home/adv/git/xenomai/mydriver/mydriver.ko; :; } | awk '!x[$$0]++' - > /home/adv/git/xenomai/mydriver/modules.order
