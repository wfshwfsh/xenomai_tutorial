#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

#include <alchemy/task.h>
#include <alchemy/timer.h>
//#include <alchemy/print.h>

#define TASK_NAME       "my_rt_task"
#define TASK_PRIO       99
#define TASK_STACK_SIZE 0
#define TASK_MODE       0

RT_TASK my_task;

volatile int keep_running = 1;

void signal_handler(int sig) {
    keep_running = 0;
}

void periodic_task(void *arg)
{
    RTIME period_ns = 1e9; // 1 秒
    rt_printf("[Task] Started, period = %lld ns\n", period_ns);

    int err = rt_task_set_periodic(NULL, TM_NOW, period_ns);
    if (err) {
        rt_printf("[Task] rt_task_set_periodic failed: %d\n", err);
        return;
    }

    while (keep_running) {
        err = rt_task_wait_period(NULL);
        if (err) {
            printf("[Task] rt_task_wait_period failed: %d\n", err);
            if (err == -ETIMEDOUT) {
                printf("[Task] Overrun occurred! Missed deadline.\n");
                continue;
            } else {
                break; // other error, break
            }
        }
        printf("[Task] Tick at %lld ns\n", rt_timer_read());
    }

    rt_printf("[Task] Exiting...\n");
}

int main(int argc, char* argv[])
{
    signal(SIGINT, signal_handler); // Ctrl+C 結束

    //rt_print_auto_init(1); // 啟用 rt_printf

    int err = rt_task_create(&my_task, TASK_NAME, TASK_STACK_SIZE, TASK_PRIO, TASK_MODE);
    if (err) {
        printf("Failed to create task: %d\n", err);
        return EXIT_FAILURE;
    }

    err = rt_task_start(&my_task, &periodic_task, NULL);
    if (err) {
        printf("Failed to start task: %d\n", err);
        return EXIT_FAILURE;
    }

    printf("Main: task started. Press Ctrl+C to exit.\n");

    while (keep_running) {
        sleep(1);
    }

    printf("Main: exiting.\n");
    return 0;
}
