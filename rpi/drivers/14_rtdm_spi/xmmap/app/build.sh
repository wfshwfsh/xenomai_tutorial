gcc -g -Wall -I.. -Wl,--no-as-needed \
-Wl,@/usr/xenomai/lib/cobalt.wrappers -Wl,@/usr/xenomai/lib/modechk.wrappers \
/usr/xenomai/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/xenomai/lib/dynlist.ld \
-L/usr/xenomai/lib -lcobalt -lmodechk -lpthread -lrt -o xmmap_user xmmap_user.c 
