#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "pwmled.h"

int main(int argc, char* argv[])
{
  int fd = open("/dev/pwmled", O_RDWR);
  int brightness = 0;
  char key = 0;

  while ((key = getchar()) != 'q') {
    switch (key) {
    case '+':
      brightness += brightness < PWMLED_MAX_BRIGHTNESS ? 20 : 0;
      break;
    case '-':
      brightness -= brightness > 0 ? 20 : 0;
      break;
    }

    if (ioctl(fd, PWMLED_CMD_SET_BRIGHTNESS, brightness) < 0) {
      perror("ioctl");
      break;
    }
  }

  close(fd);
  return 0;
}
