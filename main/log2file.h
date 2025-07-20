#ifndef _LOG_H_
#define _LOG_H_

#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/errno.h>
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "sdkconfig.h"



// File names
static const char *LOG_FILE = "/spiflash/log.txt";
static const char *IRR_FILE = "/spiflash/irr.txt";

void Mount_my_Filesystem(char *partition);
void read_log(const char* filename);
void append_log(const char* filename,const char *format, ...);
long getfilesize(const char *filename);


#endif