//gcc test.c -o test.out -I/opt/etherlab/include -L/opt/etherlab/lib/ -lethercat
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

// Application parameters
#define FREQUENCY 100//控制数据发送频率
#define PRIORITY 0

// Optional features
#define CONFIGURE_PDOS  1

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_dig_out = NULL;
static ec_slave_config_state_t sc_dig_out_state = {};

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

#define Beckhoff_EK9100 0x0000001a, 0x09139100		//vendor ID,product code
#define Beckhoff_EL2016 0x0000001a, 0x09132016

static unsigned int Dig_out=1;

// offsets for PDO entries
static unsigned int off_dig_out;
static unsigned int off_dig_in;
static unsigned int off_dig_out2;

const static ec_pdo_entry_reg_t domain1_regs[] = {
    {LAN9252_Pos, Microchip_LAN9252, 0x3101, 1, &off_dig_out},
    {LAN9252_Pos, Microchip_LAN9252, 0x3001, 1, &off_dig_in},
	//{LAN9252_Pos, Microchip_LAN9252, 0x3101, 1, &off_dig_out2},
	{}
};

static unsigned int counter = 0;
static unsigned int blink = 0;

/*****************************************************************************/

#if CONFIGURE_PDOS

// Digital out ------------------------
#if 0
static ec_pdo_entry_info_t el2016_channels[] = {
    {0x3101, 0x01, 8}, // Output
    //{0x3101, 0x02, 8}, // Output
	{0x3001, 0x01, 8}, // Input
};

static ec_pdo_info_t el2016_pdos[] = {
    {0x1a00, 1, el2016_channels+0},
	{0x1600, 1, el2016_channels+1},
	
};

static ec_sync_info_t el2016_syncs[] = {
    {0, EC_DIR_OUTPUT, 2, el2016_pdos + 0, EC_WD_ENABLE},
    //{1, EC_DIR_OUTPUT, 1, el2016_pdos + 1, EC_WD_ENABLE},
	{1, EC_DIR_INPUT, 1, el2016_pdos + 1, EC_WD_DISABLE},
    {0xff}
};


//static ec_pdo_entry_info_t el2016_channels[] = {
//    {0x3101, 0x01, 8}, // Output
//    {0x3001, 0x01, 8}, // Input
//};
//
//static ec_pdo_info_t el2016_pdos[] = {
//    {0x1a00, 1, el2016_channels+0},
//	{0x1600, 1, el2016_channels+1},
//};
//
//static ec_sync_info_t el2016_syncs[] = {
//    {0, EC_DIR_OUTPUT, 1, el2016_pdos + 0, EC_WD_ENABLE},
//    {1, EC_DIR_INPUT, 1, el2016_pdos + 1, EC_WD_DISABLE},
//    {0xff}
//};
#else
static ec_pdo_entry_info_t el2016_channels[] = {
    {0x3101, 1, 1}, // Value 1
    {0x3101, 2, 1}, // Value 2
    {0x3101, 3, 1}, // Value 3
    {0x3101, 4, 1}, // Value 4
    {0x3101, 5, 1}, // Value 5
    {0x3101, 6, 1}, // Value 6
    {0x3101, 7, 1}, // Value 7
    {0x3101, 8, 1}, // Value 8

};

static ec_pdo_entry_info_t el2016_channels2[] = {
    {0x3001, 1, 1}, // Value 1
    {0x3001, 2, 1}, // Value 2
    {0x3001, 3, 1}, // Value 3
    {0x3001, 4, 1}, // Value 4
    {0x3001, 5, 1}, // Value 5
    {0x3001, 6, 1}, // Value 6
    {0x3001, 7, 1}, // Value 7
    {0x3001, 8, 1}, // Value 8
};

static ec_pdo_info_t el2016_pdos[] = {
	{0x1a00, 8, el2016_channels+0},
	{0x1a00, 8, el2016_channels+1},
	{0x1a00, 8, el2016_channels+2},
	{0x1a00, 8, el2016_channels+3},
	{0x1a00, 8, el2016_channels+4},
	{0x1a00, 8, el2016_channels+5},
	{0x1a00, 8, el2016_channels+6},
	{0x1a00, 8, el2016_channels+7},
	
};

static ec_pdo_info_t el2016_pdos2[] = {
	{0x1600, 8, el2016_channels2+0},
	{0x1600, 8, el2016_channels2+1},
	{0x1600, 8, el2016_channels2+2},
	{0x1600, 8, el2016_channels2+3},
	{0x1600, 8, el2016_channels2+4},
	{0x1600, 8, el2016_channels2+5},
	{0x1600, 8, el2016_channels2+6},
	{0x1600, 8, el2016_channels2+7},
	
};

static ec_sync_info_t el2016_syncs[] = {
    {0, EC_DIR_OUTPUT, 1, el2016_pdos, EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 1, el2016_pdos2, EC_WD_DISABLE},
	//{1, EC_DIR_OUTPUT, 1, el2016_pdos, EC_WD_ENABLE},
    {0xff}
};
#endif

#endif

/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    //if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

/*****************************************************************************/

void check_slave_config_states(void)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc_dig_out, &s);

    if (s.al_state != sc_dig_out_state.al_state)
        printf("DigOut: State 0x%02X.\n", s.al_state);
    if (s.online != sc_dig_out_state.online)
        printf("DigOut: %s.\n", s.online ? "online" : "offline");
    if (s.operational != sc_dig_out_state.operational)
        printf("output: %soperational.\n",
                s.operational ? "" : "Not ");

    sc_dig_out_state = s;
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
	printf("off_dig_out = %d\n", off_dig_out);
	printf("Dig_out = %d\n", Dig_out);
	printf("off_dig_out2 = %d\n", off_dig_out2);
	
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
	printf("status = %d\n", status);
	printf("domain1_pd = 0x%x\n", domain1_pd);
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

	APPLOG_DD("slaves_responding = %d", ms.slaves_responding);
	APPLOG_DD("al_states = %d", ms.al_states);
	APPLOG_DD("link_up = %d", ms.link_up);
	
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
		APPLOG_DD("failed to obtain master/slave info");
		return -1;
	}
	
	int slave_num = _master_info.slave_count;
	
	for ( i = 0; i < slave_num; i++ )
	{
    	ret = ecrt_master_get_slave(master, i/*slave position*/, &slave_info);
    	if (ret) {
        	APPLOG_DD("Failed to obtain master information");
        	return 0;
    	}


		printf( "%d  %d:%d  0x%08x:0x%08x \n", i, 
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

    master = ecrt_request_master(0);// 请求EtherCAT主机进行实时操作。
    if (!master)	
        return -1;

    domain1 = ecrt_master_create_domain(master);// 创建新的进程数据域
    if (!domain1)
        return -1;

    printf("Configuring PDOs...\n");

    if (!(sc_dig_out = ecrt_master_slave_config(//获取从站配置
					//master, DigOutSlavePos, Beckhoff_EL2016))) {//第一个参数是所请求的主站实例，第二个包括主站的别名和位置，第三个包括供应商码和产品码
                    master, LAN9252_Pos, Microchip_LAN9252))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_dig_out, EC_END, el2016_syncs)) {//指定完整的PDO配置。第一个参数是获取的从站配置，第二个参数表示同步管理器配置数，EC_END=~u0,第三个参数表示同步管理器配置数组
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
	
    printf("Activating master...\n");
    if (ecrt_master_activate(master))// 激活主站
        return -1;

    if (!(domain1_pd = ecrt_domain_data(domain1))) {//返回域的进程数据
        return -1;
    }

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

    printf("Starting timer...\n");
    tv.it_interval.tv_sec = 0;//next time
    tv.it_interval.tv_usec = 1000000 / FREQUENCY;
    tv.it_value.tv_sec = 0;//current time
    tv.it_value.tv_usec = 1000;
    if (setitimer(ITIMER_REAL, &tv, NULL)) {//定时功能，以系统真实的时间来计算，每隔一段时间送出SIGALRM信号。
        fprintf(stderr, "Failed to start timer: %s\n", strerror(errno));
        return 1;
    }

    printf("Started.\n");
    while (1) {
        pause();//挂起，等待cpu唤醒
	
        while (sig_alarms != user_alarms) {//意思是每过一段时间才会执行一次，而不是一直死循环
            //ecm_getMasterState();
			//check_slave_config_states();
			cyclic_task();
            user_alarms++;
        }
    }

    return 0;
}

/****************************************************************************/
#if 0

/**
先定义一些结构体，其中有存放从站配置的结构体，类型为ec_slave_config_t{}，用在接收函数ecrt_master_slave_config（）的返回值；用在描述信号到达时要采取的操作的结构体struct sigaction{}；计算时间，用于定时的结构体struct itimerval{}。

下面是关于主站和从站的一些配置
1. 请求EtherCAT主机进行操作，我感觉可以认为是创建一个主站实例对象。
2. 创建进程数据域。
3. 获取从站配置
4. 指定完整的pdo配置
5. 为进程数据域注册一组PDO项。
6. 激活主站
7. 返回域的进程数据
8. 可选选项：设置进程优先级
9. 可选选项：指定进程收到某种信号采取的操作
10. 可选选项：设置定时发送信号
11. 进入循环，开始循环任务

其中对于几个可选选项，设置优先级，是为了让实时性更好；另外两个定时发送的信号作为进程采取操作的条件，而操作内容又作为循环任务的周期时间间隔，不对，这两个并不再是可选项，应该必须要设置的，周期交换数据的时间间隔要大于等于从站处理 数据和传输等时间总和。

下面是在循环任务中做的事情：
1. 从硬件获取接收的帧并处理数据报。
2. 确定域数据报的状态。
3. 检查域的状态（可选项）
4. 周期性检查主站状态和从站配置状态（可选项）
5. 计算将要发送流水灯数据和数据改变周期大小
6. 将数据放入数据域
7. 将主数据报队列中的所有域数据报排队
8. 发送队列中的所有数据报。


其中对于配置PDO所涉及到的几个结构体
typedef struct {
    uint16_t index; /**< PDO entry index. */
    uint8_t subindex; /**< PDO entry subindex. */
    uint8_t bit_length; /**< Size of the PDO entry in bit. */
} ec_pdo_entry_info_t;


typedef struct {
    uint16_t index; /**< PDO index. */
    unsigned int n_entries; /**< Number of PDO entries in entries to map.
                              Zero means, that the default mapping shall be
                              used (this can only be done if the slave is
                              present at bus configuration time). */
    ec_pdo_entry_info_t *entries; /**< Array of PDO entries to map. Can 														either be NULL, or must contain at
                                    least n_entries values. */
} ec_pdo_info_t;


typedef struct {
    uint8_t index; /**< Sync manager index. Must be less
                     than #EC_MAX_SYNC_MANAGERS for a valid sync manager,
                     but can also be 0xff to mark the end of the list. */
    ec_direction_t dir; /**< Sync manager direction. */
    unsigned int n_pdos; /**< Number of PDOs in pdos. */
    ec_pdo_info_t *pdos; /**< Array with PDOs to assign. This must contain
                            at least n_pdos PDOs. */
    ec_watchdog_mode_t watchdog_mode; /**< Watchdog mode. */
} ec_sync_info_t;


typedef struct {
    uint16_t alias; /**< Slave alias address. */
    uint16_t position; /**< Slave position. */
    uint32_t vendor_id; /**< Slave vendor ID. */
    uint32_t product_code; /**< Slave product code. */
    uint16_t index; /**< PDO entry index. */
    uint8_t subindex; /**< PDO entry subindex. */
    unsigned int *offset; /**< Pointer to a variable to store the PDO 					entry's(byte-)offset in the process data. */
    unsigned int *bit_position; /**< Pointer to a variable to store a bit
                                  position (0-7) within the offset. Can be
                                  NULL, in which case an error is raised if 					 							  the PDO entry does not byte-align. */
} ec_pdo_entry_reg_t;


#define EC_WRITE_U8(DATA, VAL) \
    do { \
        *((uint8_t *)(DATA)) = ((uint8_t) (VAL)); \
    } while (0)

*/
#endif