#ifndef OTA_UPDATE_H_
#define OTA_UPDATE_H_

#include <stdbool.h>

#include "http_rest.h"

#ifdef __cplusplus
extern "C" {
#endif

void ota_update_init(void);

bool http_rest_ota_status(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_ota_begin(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_ota_chunk(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_ota_finalize(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_ota_abort(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_ota_apply(struct fs_file *file, int num_params, char *params[], char *values[]);

#ifdef __cplusplus
}
#endif

#endif  // OTA_UPDATE_H_
