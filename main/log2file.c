/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


#include "log2file.h"

static const char *TAG = "example";
wl_handle_t wl_handle = WL_INVALID_HANDLE;

// Mount path for the partition
static const char *base_path = "/spiflash";



// Function to dump contents of a directory
static void list_dir(const char *path);

// Best effort recursive function to clean a directory
static void clean_dir(const char *path);
// 
wl_handle_t Mount_Filesystem(char* partition_name)
{
    ESP_LOGI(TAG, "Mounting FAT filesystem");

    // To mount device we need name of device partition, define base_path
    // and allow format partition in case if it is new one and was not formatted before
    const esp_vfs_fat_mount_config_t mount_config = {
            .max_files = 4,
            .format_if_mount_failed = true,
            .allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
            .use_one_fat = false,
    };

    wl_handle_t wl_handle = WL_INVALID_HANDLE;

    esp_err_t err = ESP_OK;

    err = esp_vfs_fat_spiflash_mount_rw_wl(base_path,partition_name, &mount_config, &wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return WL_INVALID_HANDLE;
    }
    return wl_handle;
}


int OpenFile(const char *filename)
{
    ESP_LOGI(TAG, "Creating a file");

    // Unlike C standard library which uses FILE*, POSIX API uses file descriptors for file operations
    int fd = open(filename, O_RDWR | O_CREAT | O_TRUNC, 0);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open file for writing");

    }
    return fd;
}

void Write2File(int fd,char* text)
{
    ESP_LOGI(TAG, "Writing to the file");
    write(fd, text, strlen(text));

    ESP_LOGI(TAG, "Force cached data and metadata to the filesystem");
    fsync(fd);
    close(fd);
}


long getfilesize(const char *filename)
{
    struct stat info;
    // We have to use `stat` instead of `fstat`, because `fstat` currently isn't fully supported
    if (stat(filename, &info) < 0) {
        ESP_LOGE(TAG, "Failed to stat file: %s", strerror(errno));

        return info.st_size;
    }


    return -1;
}

void stat_file(char * filename)
{
    struct stat info;
    // We have to use `stat` instead of `fstat`, because `fstat` currently isn't fully supported
    if (stat(filename, &info) < 0) {
        ESP_LOGE(TAG, "Failed to stat file: %s", strerror(errno));

        return;
    }


    ESP_LOGI(
        TAG,
        "File stats:\n"
        "\tFile size:                %ld bytes\n"
        "\tFile modification time:   %s",
        info.st_size,
        ctime(&info.st_mtime)
    );
}


int ReadFromFile(int fd,char* buf,int size)
{
    ESP_LOGI(TAG, "Reading from file:");

    ssize_t len =  read(fd, buf, size);
    if (len < 0) {
        ESP_LOGE(TAG, "Failed to read file: %s", strerror(errno));
        close(fd);
        return len;
    }

    printf("%.*s\n", len, buf);
    return len;
}


void ReadFromFile2()
{
    ESP_LOGI(TAG, "Reading from file:");
    char buf[1000];
    FILE *ptr_file=fopen("devices.txt","r");
    while (fgets(buf,1000, ptr_file)!=NULL) 		printf("%s",buf);
    fclose(ptr_file);
}
    
  //  ESP_ERROR_CHECK(esp_vfs_fat_spiflash_unmount_rw_wl(base_path, wl_handle));




void list_dir(const char *path)
{
    ESP_LOGI(TAG, "Listing files in %s:", path);

    DIR *dir = opendir(path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory: %s", strerror(errno));
        return;
    }

    printf("%s:\n", path);
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        printf(
            "    %s: %s\n",
            (entry->d_type == DT_DIR)
                ? "directory"
                : "file     ",
            entry->d_name
        );
    }

    closedir(dir);
}

void clean_dir(const char *path)
{
    ESP_LOGI(TAG, "Deleting everything in %s:", path);

    DIR *dir = opendir(path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory: %s", strerror(errno));
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        char full_path[64] = {0};
        snprintf(full_path, sizeof(full_path), "%.20s/%.40s", path, entry->d_name);
        if (entry->d_type == DT_DIR)
            clean_dir(full_path);
        if (remove(full_path) != 0) {
            ESP_LOGE(TAG, "Failed to remove %s: %s", full_path, strerror(errno));
        }
    }

    closedir(dir);
}


void Mount_my_Filesystem(char *partition)
{
  static bool FSmounted=false;
  if (!FSmounted)  
   {
    wl_handle =Mount_Filesystem(partition); 
    FSmounted=true; 
   }  
}




void append_log(const char* filename,const char *format, ...)
{
  va_list arg;
  int done;
  char *buffer=calloc(1,1024);

                char strftime_buf[64];
				time_t now;
				struct tm timeinfo;
				time(&now);
				localtime_r(&now, &timeinfo);
				strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  sprintf(buffer,"%s:",strftime_buf);
  va_start (arg, format);
  done = vsprintf (buffer+strlen(buffer), format, arg);
  va_end (arg);
  Mount_my_Filesystem("user_fs");
  int handle = open(filename, O_RDWR |O_CREAT| O_APPEND , 0);
  Write2File(handle,buffer);
  close(handle);
  free(buffer);
  stat_file(filename);
}

void read_log(const char* filename)
{
char buf[128] = {0};    
Mount_my_Filesystem("user_fs");
int handle2=open(filename, O_RDONLY , 0);
int readed=ReadFromFile(handle2,buf,sizeof(buf)-1);
close(handle2);
ESP_LOGI("TEST","readed:%s",buf);
}
