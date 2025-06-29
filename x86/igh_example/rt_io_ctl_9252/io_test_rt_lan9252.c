//gcc test.c -o test.out -I/opt/etherlab/include -L/opt/etherlab/lib/ -lethercat
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>

/****************************************************************************/
#include "ecrt.h"

#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <trank/native/timer.h>
/****************************************************************************/

// Application parameters
#define FREQUENCY 100//控制数据发送频率
#define PRIORITY 0
#define USE_RT_TASK 1

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_dig_out = NULL;
static ec_slave_config_state_t sc_dig_out_state = {};

// RT 
#define TASK_PRIO 99 // 99 is Highest RT priority, 0 is Lowest
#define TASK_MODE 0 // No flags
#define TASK_STKSZ 0 // default Stack size

//#define TASK_PERIOD (20*1000*1000) //20ms
#define TASK_PERIOD (10*1000*1000) //6ms
//#define TASK_PERIOD (2000*1000)
//#define TASK_PERIOD (30*1000) // 0.5= 50,000,000 ns
RT_TASK tA;

// Timer
static unsigned int sig_alarms = 0;
static unsigned int user_alarms = 0;

/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;

#define BusCouplerPos  0, 0	//alias,position
#define DigOutSlavePos 0, 1

#define LAN9252_Pos		0, 0
#define Microchip_LAN9252	0xe00004d8, 0x0000000d

#define MISC_LAN9252		0x00000009, 0x00009252

#define Beckhoff_EK9100 0x0000001a, 0x09139100		//vendor ID,product code
#define Beckhoff_EL2016 0x0000001a, 0x09132016

static unsigned int Dig_out=1, Dig_in1=0, Dig_in2=0;


// offsets for PDO entries
static unsigned int off_dig_out, off_dig_out2;
static unsigned int off_dig_in;
static unsigned int off_dig_in2;
static unsigned int el2016_input_offset=0;

const static ec_pdo_entry_reg_t domain1_regs[] = {
    {LAN9252_Pos, MISC_LAN9252, 0x7010, 1, &off_dig_out},
    {LAN9252_Pos, MISC_LAN9252, 0x6000, 1, &off_dig_in},
    {}
};

static unsigned int counter = 0;
static unsigned int blink = 0;

/*****************************************************************************/

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x7010, 0x01, 1}, /* LED 1 */
    {0x7010, 0x02, 1}, /* LED 2 */
    {0x7010, 0x03, 1}, /* LED 3 */
    {0x7010, 0x04, 1}, /* LED 4 */
    {0x7010, 0x05, 1}, /* LED 5 */
    {0x7010, 0x06, 1}, /* LED 6 */
    {0x7010, 0x07, 1}, /* LED 7 */
    {0x7010, 0x08, 1}, /* LED 8 */
    {0x0000, 0x00, 8}, /* Gap */
    {0x6000, 0x01, 1}, /* Switch 1 */
    {0x6000, 0x02, 1}, /* Switch 2 */
    {0x6000, 0x03, 1}, /* Switch 3 */
    {0x6000, 0x04, 1}, /* Switch 4 */
    {0x6000, 0x05, 1}, /* Switch 5 */
    {0x6000, 0x06, 1}, /* Switch 6 */
    {0x6000, 0x07, 1}, /* Switch 7 */
    {0x6000, 0x08, 1}, /* Switch 8 */
    {0x0000, 0x00, 8}, /* Gap */
    {0x6020, 0x01, 1}, /* Underrange */
    {0x6020, 0x02, 1}, /* Overrange */
    {0x6020, 0x03, 2}, /* Limit 1 */
    {0x6020, 0x05, 2}, /* Limit 2 */
    {0x0000, 0x00, 8}, /* Gap */
    {0x1802, 0x07, 1}, /* TxPDOState */
    {0x1802, 0x09, 1}, /* TxPDO Toggle */
    {0x6020, 0x11, 16}, /* Analog input */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1601, 9, slave_0_pdo_entries + 0}, /* DO RxPDO-Map */
    {0x1a00, 9, slave_0_pdo_entries + 9}, /* DI TxPDO-Map */
    {0x1a02, 8, slave_0_pdo_entries + 18}, /* AI TxPDO-Map */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};


/*****************************************************************************/
static int led_fd = 0;

void led_write(int val)
{
	char buf[2];
	snprintf(buf, sizeof(buf), "%d", !!val);
	
	int ret = write(led_fd, buf, strlen(buf));
	if(ret<0){
		rt_printf("can't write: %s\n", strerror(errno));
	}
}

int led_init(void)
{
	int ret=0;
	char cmd1[] = "echo 0 > /sys/class/gpio/export";
	char cmd2[] = "echo out > /sys/class/gpio/gpio0/direction";
	system(cmd1);
	system(cmd2);
	
	led_fd = open("/sys/class/gpio/gpio0/value", /*O_RDWR*/O_WRONLY);
	if (led_fd < 0){
		rt_printf("can't open!\n");
		return -1;
	}
	
	return 0;
}


/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
        rt_printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        rt_printf("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        rt_printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        rt_printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        rt_printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

/*****************************************************************************/

void check_slave_config_states(void)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc_dig_out, &s);

    if (s.al_state != sc_dig_out_state.al_state)
        rt_printf("DigOut: State 0x%02X.\n", s.al_state);
    if (s.online != sc_dig_out_state.online)
        rt_printf("DigOut: %s.\n", s.online ? "online" : "offline");
    if (s.operational != sc_dig_out_state.operational)
        rt_printf("output: %soperational.\n",
                s.operational ? "" : "Not ");

    sc_dig_out_state = s;
}

/*****************************************************************************/
void periodic_task2(void *arg)
{
    long task_period = 1000000; // 1ms
    rt_printf("task_period = %ld \n", task_period);

    RTIME ticks = rt_timer_ns2ticks(task_period);
    rt_printf("ticks = %ld\n", ticks);
    
    int err = rt_task_set_periodic(NULL, TM_NOW, ticks);
    if (err) {
        rt_printf("rt_task_set_periodic failed: %d\n", err);
        return;
    }

    while (1) {
        err = rt_task_wait_period(NULL);
        if (err) {
            rt_printf("rt_task_wait_period failed: %d\n", err);
            continue;
        }
        rt_printf("Periodic task running...\n");
    }
}

void periodic_task (void *arg) {
        int err;
	//long task_period = (long*)arg;
	long task_period = 1000000;
	rt_printf("task_period = %ld \n", task_period);
	static int led_on = 0;
	RTIME ticks;
	RTIME now, previous;
	previous= rt_timer_read();

	//rt_printf("before XENO loop:\n");
	//led_init();
	//led_write(1);
	//sleep(1);
	//led_write(0);
	//sleep(1);
	
	rt_task_set_mode(0, T_WARNSW, NULL);

	rt_printf("Is this task real-time? %s\n", rt_task_self() != NULL ? "YES" : "NO");
	rt_printf("000\n");
	ticks = rt_timer_ns2ticks(task_period);
	rt_printf("ticks = %ld\n", ticks);
	err = rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(task_period));
        
	if(err){
		rt_printf("rt_task_set_periodic failed with code: %d \n", err);
		return;
	}
	
	while ( 1 ) 
	{
		err = rt_task_wait_period( NULL );
		if (err) {
		    rt_printf("rt_task_wait_period() error: %d\n", err);
		    continue; // 或 break
		}
		
		// receive EtherCAT frames
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);
	
		check_domain1_state();
		//led_on = !led_on;
		//led_write(led_on);
		
		blink++;
		if(blink==100){
			/* here: buffer read / write */
			
			//led_on = !led_on;
			//rt_printf("led_on = %d\n", led_on);
			//led_write(led_on);
			
			if(Dig_out<0x100){
				Dig_out=Dig_out*2;
			}
			else{
				Dig_out=1;
			}
			
			Dig_in1 = EC_READ_U8(domain1_pd + off_dig_in);
			EC_WRITE_U8(domain1_pd + off_dig_out, Dig_out);

			rt_printf("Dig_out=0x%04x, Dig_in1=0x%04x \n", Dig_out, Dig_in1);
			blink=0;
		}
	        
		// send process data
		ecrt_domain_queue(domain1);
		ecrt_master_send(master);
		

		now = rt_timer_read();
		//rt_printf("Time elapsed: %ld.%04ld ms\n", \
			(long)(now - previous) / 1000000, \
			(long)(now - previous) % 1000000);
		previous = now;
		// use xenomai timer
		//ecrt_master_application_time( master, rt_timer_read() );
	}
}


/*****************************************************************************/
void cyclic_task()
{
    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    // check process data state (optional)
    check_domain1_state();
	//ecm_getMasterState();
	check_slave_config_states();
	
    if (counter) {
        counter--;
    } else { // do this at 1 Hz
        counter = FREQUENCY;
	
        // check for master state (optional)
        check_master_state();
	
        // check for islave configuration state(s) (optional)
        check_slave_config_states();
	
    }

    // write process data
	rt_printf("off_dig_out = %d\n", off_dig_out);
	rt_printf("Dig_out = %d\n", Dig_out);
	rt_printf("off_dig_out2 = %d\n", off_dig_out2);
	
	blink++;
	if(blink==10){
		if(Dig_out<0x100){
			Dig_out=Dig_out*2;
		}
		else{
			Dig_out=1;
		}
		blink=0;
	}

	//ecm_getAllSlaveInfo();
	int status = EC_READ_U8(domain1_pd+off_dig_in);
	rt_printf("status = %d\n", status);
	rt_printf("domain1_pd = 0x%x\n", domain1_pd);
    EC_WRITE_U8(domain1_pd + off_dig_out, Dig_out);
	//EC_WRITE_U8(domain1_pd + off_dig_out2, Dig_out);

	//off_dig_out2
    // send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

/****************************************************************************/

void signal_handler(int signum) {//进程收到定时信号所做的操作
    switch (signum) {
        case SIGALRM:
            sig_alarms++;
            break;
    }
}

#define APPLOG_DD(...)		printf("[%s %d] ", __func__, __LINE__); printf(__VA_ARGS__); printf("\n")

int ecm_getMasterState()
{
	ec_master_state_t ms;
	ecrt_master_state(master, &ms);

	rt_printf("slaves_responding = %d", ms.slaves_responding);
	rt_printf("al_states = %d", ms.al_states);
	rt_printf("link_up = %d", ms.link_up);
	
	return 0;
}

int ecm_getAllSlaveInfo()
{
	//vendor id, ...
	int i, ret=0;
	ec_slave_info_t slave_info;
	memset(&slave_info, 0, sizeof(slave_info));
	
	ec_master_info_t _master_info;
	memset(&_master_info, 0, sizeof(ec_master_info_t));
	
	ret = ecrt_master(master, &_master_info);
	if(ret){
		rt_printf("failed to obtain master/slave info");
		return -1;
	}
	
	int slave_num = _master_info.slave_count;
	
	for ( i = 0; i < slave_num; i++ )
	{
    	ret = ecrt_master_get_slave(master, i/*slave position*/, &slave_info);
    	if (ret) {
        	rt_printf("Failed to obtain master information");
        	return 0;
    	}


		rt_printf( "%d  %d:%d  0x%08x:0x%08x \n", i, 
				slave_info.alias, slave_info.position,
				slave_info.vendor_id, slave_info.product_code);

		
	}
	
    return 0;
}
/****************************************************************************/
//#define DigOutSlavePos 0,0
//#define MICROCHIP_EVB3LAN9252DIG 0xE00004D8, 0x00000020

int main(int argc, char **argv)
{
    ec_slave_config_t *sc;// 存放从站配置的结构体
    struct sigaction sa;// 描述信号到达时要采取的操作的结构体
    struct itimerval tv;//计算时间，用于定时的结构体

    if(argc < 2)
    {
	perror("error argv must >= 2");
	return -1;
    }
	
    rt_printf("argv[1] = %s \n", argv[1]);
    long task_period = atol(argv[1]);
    rt_printf("task_period = %d \n", task_period);

    master = ecrt_request_master(0);// 请求EtherCAT主机进行实时操作。
    if (!master)	
        return -1;

    domain1 = ecrt_master_create_domain(master);// 创建新的进程数据域
    if (!domain1)
        return -1;

    rt_printf("Configuring PDOs...\n");

    if (!(sc_dig_out = ecrt_master_slave_config(//获取从站配置
   		        //master, DigOutSlavePos, Beckhoff_EL2016))) {
			//第一个参数是所请求的主站实例，第二个包括主站的别名和位置，第三个包括供应商码和产品码
                        master, LAN9252_Pos, MISC_LAN9252))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    //if (ecrt_slave_config_pdos(sc_dig_out, EC_END, el2016_syncs)) {
    if (ecrt_slave_config_pdos(sc_dig_out, EC_END, slave_0_syncs)) {
	    //指定完整的PDO配置。第一个参数是获取的从站配置，第二个参数表示同步管理器配置数，EC_END=~u0,第三个参数表示同步管理器配置数组
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    // Create configuration for bus coupler
    //sc = ecrt_master_slave_config(master, BusCouplerPos, Beckhoff_EK9100);
    //if (!sc)
    //    return -1;

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {//为进程数据域注册一组PDO项。参数一：创建的进程数据域，参数二：pdo注册数组
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    //ecm_getMasterState();
    //check_slave_config_states();
	
    rt_printf("Activating master...\n");
    if (ecrt_master_activate(master))// 激活主站
        return -1;

    if (!(domain1_pd = ecrt_domain_data(domain1))) {//返回域的进程数据
        return -1;
    }


#if USE_RT_TASK
    cpu_set_t mask;

    // set CPU to affine to
    CPU_ZERO(&mask);    // clear all CPUs
    CPU_SET(3, &mask);    // select CPU 3
	
    int e1 = rt_task_create(&tA, "periodicTask", TASK_STKSZ, TASK_PRIO, TASK_MODE);
    //int e2 = rt_task_start(&tA, &periodic_task, NULL);
	
    // set task cpu affinity
    rt_task_set_affinity(&tA, &mask);
	
    int e2 = rt_task_start(&tA, &periodic_task, task_period);
	
    if (e1 | e2) {
	fprintf(stderr, "Error launching periodic task....\n");
	rt_task_delete(&tA);
	exit(1);
    }
	
    rt_printf("Press any key to end....\n");
    getchar();
    rt_task_delete(&tA);

#else
#if PRIORITY
    pid_t pid = getpid();//获得进程PID
    if (setpriority(PRIO_PROCESS, pid, -19))//设置进程优先级
        fprintf(stderr, "Warning: Failed to set priority: %s\n",
                strerror(errno));
#endif

    sa.sa_handler = signal_handler;//指定信号关联函数
    sigemptyset(&sa.sa_mask);// 初始化给出的信号集为空
    sa.sa_flags = 0;
    if (sigaction(SIGALRM, &sa, 0)) {//系统调用，用于更改进程在收到特定信号时采取的操作
        fprintf(stderr, "Failed to install signal handler!\n");
        return -1;
    }

    rt_printf("Starting timer...\n");
    tv.it_interval.tv_sec = 0;//next time
    tv.it_interval.tv_usec = 1000000 / FREQUENCY;
    tv.it_value.tv_sec = 0;//current time
    tv.it_value.tv_usec = 1000;
    if (setitimer(ITIMER_REAL, &tv, NULL)) {//定时功能，以系统真实的时间来计算，每隔一段时间送出SIGALRM信号。
        fprintf(stderr, "Failed to start timer: %s\n", strerror(errno));
        return 1;
    }

    rt_printf("Started.\n");
    while (1) {
        pause();//挂起，等待cpu唤醒
	
        while (sig_alarms != user_alarms) {//意思是每过一段时间才会执行一次，而不是一直死循环
            //ecm_getMasterState();
			//check_slave_config_states();
			cyclic_task();
            user_alarms++;
        }
    }
#endif
    return 0;
}

