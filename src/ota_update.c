#include "ota_update.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "common.h"
#include "flash_storage.h"
#include "hardware/flash.h"
#include "hardware/regs/psm.h"
#include "hardware/regs/watchdog.h"
#include "hardware/structs/psm.h"
#include "hardware/structs/watchdog.h"
#include "hardware/sync.h"
#include "pico/flash.h"
#include "pico/platform.h"

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (4u * 1024u * 1024u)
#endif

#ifndef OTA_BOOTLOADER_APPLY_SUPPORTED
#define OTA_BOOTLOADER_APPLY_SUPPORTED 1
#endif

#define OTA_PRIMARY_SLOT_BYTES        (2u * 1024u * 1024u)
#define OTA_METADATA_OFFSET           OTA_PRIMARY_SLOT_BYTES
#define OTA_IMAGE_OFFSET              (OTA_METADATA_OFFSET + FLASH_SECTOR_SIZE)
#define OTA_METADATA_MAGIC            0x3155544Fu  // "OTU1" little-endian
#define OTA_METADATA_VERSION          1u
#define OTA_METADATA_FLAG_VERIFIED    (1u << 0)
#define OTA_MAX_ERROR_LEN             96u
#define OTA_MAX_CHUNK_BYTES           (FLASH_PAGE_SIZE * 2u)
#define OTA_APPLY_DELAY_MS            750u
#define OTA_APPLY_TASK_STACK_WORDS    1024u
#define OTA_ALIGN_UP(value, align)    (((value) + ((align) - 1u)) & ~((align) - 1u))

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t image_size;
    uint32_t expected_crc32;
    uint32_t actual_crc32;
    uint32_t flags;
    uint32_t image_offset;
    uint32_t primary_limit;
    uint32_t reserved[8];
} ota_metadata_t;

typedef struct {
    bool active;
    bool verified;
    uint32_t expected_size;
    uint32_t expected_crc32;
    uint32_t actual_crc32;
    uint32_t received_size;
    char last_error[OTA_MAX_ERROR_LEN];
} ota_session_t;

typedef struct {
    uint32_t image_size;
    uint32_t expected_crc32;
    uint32_t actual_crc32;
} ota_apply_params_t;

typedef enum {
    OTA_FLASH_OP_ERASE = 0,
    OTA_FLASH_OP_PROGRAM = 1,
} ota_flash_op_kind_t;

typedef struct {
    ota_flash_op_kind_t kind;
    uint32_t offset;
    uint32_t length;
    const uint8_t *data;
} ota_flash_op_t;

static ota_session_t g_ota_session = {0};
static ota_apply_params_t g_ota_apply_params = {0};
static volatile bool g_ota_apply_scheduled = false;
static ota_flash_op_t g_flash_op = {0};
static uint8_t g_flash_page[OTA_MAX_CHUNK_BYTES] __attribute__((aligned(4)));
static char g_ota_response[1024];

static uint32_t ota_storage_capacity(void) {
    if (PICO_FLASH_SIZE_BYTES <= OTA_IMAGE_OFFSET) {
        return 0;
    }
    return (uint32_t)(PICO_FLASH_SIZE_BYTES - OTA_IMAGE_OFFSET);
}

static uint32_t ota_primary_image_limit(void) {
    if (FLASH_STORAGE_ML_HISTORY_OFFSET < OTA_PRIMARY_SLOT_BYTES) {
        return FLASH_STORAGE_ML_HISTORY_OFFSET;
    }
    return OTA_PRIMARY_SLOT_BYTES;
}

static bool ota_supported(void) {
    return ota_storage_capacity() >= (256u * 1024u) &&
           ota_primary_image_limit() > 0 &&
           OTA_IMAGE_OFFSET < PICO_FLASH_SIZE_BYTES;
}

static const ota_metadata_t *ota_metadata_flash(void) {
    return (const ota_metadata_t *)(XIP_BASE + OTA_METADATA_OFFSET);
}

static const uint8_t *ota_image_flash(void) {
    return (const uint8_t *)(XIP_BASE + OTA_IMAGE_OFFSET);
}

static bool ota_staged_image_has_container_magic(const uint8_t *image, uint32_t image_size) {
    if (image == NULL || image_size < 4u) {
        return false;
    }

    bool looks_like_uf2 = image[0] == 0x55u && image[1] == 0x46u &&
                          image[2] == 0x32u && image[3] == 0x0Au;
    bool looks_like_elf = image[0] == 0x7Fu && image[1] == 0x45u &&
                          image[2] == 0x4Cu && image[3] == 0x46u;
    return looks_like_uf2 || looks_like_elf;
}

static bool ota_metadata_is_valid(const ota_metadata_t *meta) {
    if (!ota_supported() || meta == NULL) {
        return false;
    }
    if (meta->magic != OTA_METADATA_MAGIC || meta->version != OTA_METADATA_VERSION) {
        return false;
    }
    if ((meta->flags & OTA_METADATA_FLAG_VERIFIED) == 0) {
        return false;
    }
    if (meta->image_offset != OTA_IMAGE_OFFSET || meta->primary_limit != ota_primary_image_limit()) {
        return false;
    }
    if (meta->image_size == 0 ||
        meta->image_size > ota_storage_capacity() ||
        meta->image_size > ota_primary_image_limit()) {
        return false;
    }
    return true;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

static uint32_t crc32_buffer(const uint8_t *data, size_t len) {
    return crc32_update(0u, data, len);
}

static void __no_inline_not_in_flash_func(ota_apply_reboot_from_ram)(void) {
    watchdog_hw->ctrl &= ~WATCHDOG_CTRL_ENABLE_BITS;
    psm_hw->wdsel = PSM_WDSEL_BITS & ~(PSM_WDSEL_ROSC_BITS | PSM_WDSEL_XOSC_BITS);
    watchdog_hw->scratch[4] = 0;
    watchdog_hw->ctrl |= WATCHDOG_CTRL_TRIGGER_BITS;
    while (true) {
        __asm volatile ("nop");
    }
}

static void __no_inline_not_in_flash_func(ota_apply_flash_callback)(void *param) {
    ota_apply_params_t *apply = (ota_apply_params_t *)param;
    if (apply == NULL || apply->image_size == 0u || apply->image_size > OTA_PRIMARY_SLOT_BYTES) {
        return;
    }

    const uint32_t image_size = apply->image_size;
    const uint32_t program_size = OTA_ALIGN_UP(image_size, FLASH_PAGE_SIZE);
    const uint32_t erase_size = OTA_ALIGN_UP(program_size, FLASH_SECTOR_SIZE);
    volatile const uint8_t *source = (volatile const uint8_t *)(XIP_BASE + OTA_IMAGE_OFFSET);

    for (uint32_t sector_offset = 0u; sector_offset < erase_size; sector_offset += FLASH_SECTOR_SIZE) {
        flash_range_erase(sector_offset, FLASH_SECTOR_SIZE);

        uint32_t sector_program_end = sector_offset + FLASH_SECTOR_SIZE;
        if (sector_program_end > program_size) {
            sector_program_end = program_size;
        }

        for (uint32_t dest_offset = sector_offset;
             dest_offset < sector_program_end;
             dest_offset += OTA_MAX_CHUNK_BYTES) {
            uint32_t program_count = sector_program_end - dest_offset;
            if (program_count > OTA_MAX_CHUNK_BYTES) {
                program_count = OTA_MAX_CHUNK_BYTES;
            }

            for (uint32_t idx = 0u; idx < program_count; idx++) {
                uint32_t image_idx = dest_offset + idx;
                g_flash_page[idx] = image_idx < image_size ? source[image_idx] : 0xFFu;
            }

            flash_range_program(dest_offset, g_flash_page, program_count);
        }
    }

    flash_range_erase(OTA_METADATA_OFFSET, FLASH_SECTOR_SIZE);
    ota_apply_reboot_from_ram();
}

static void ota_apply_task(void *param) {
    (void)param;

    vTaskDelay(pdMS_TO_TICKS(OTA_APPLY_DELAY_MS));
    int rc = flash_safe_execute(ota_apply_flash_callback,
                                &g_ota_apply_params,
                                UINT32_MAX);
    if (rc != PICO_OK) {
        snprintf(g_ota_session.last_error, sizeof(g_ota_session.last_error),
                 "OTA apply could not enter flash-safe zone rc=%d", rc);
        g_ota_apply_scheduled = false;
    }

    vTaskDelete(NULL);
}

static void ota_set_error(const char *message) {
    if (message == NULL) {
        message = "unknown OTA error";
    }
    snprintf(g_ota_session.last_error, sizeof(g_ota_session.last_error), "%s", message);
}

static void ota_clear_error(void) {
    g_ota_session.last_error[0] = '\0';
}

static const char *param_value(int num_params, char *params[], char *values[], const char *name) {
    for (int idx = 0; idx < num_params; idx++) {
        if (strcmp(params[idx], name) == 0) {
            return values[idx];
        }
    }
    return NULL;
}

static bool parse_u32_param(int num_params, char *params[], char *values[],
                            const char *name, uint32_t *out) {
    const char *text = param_value(num_params, params, values, name);
    if (text == NULL || out == NULL) {
        return false;
    }

    char *end = NULL;
    unsigned long value = strtoul(text, &end, 0);
    if (end == text || *end != '\0' || value > 0xFFFFFFFFul) {
        return false;
    }

    *out = (uint32_t)value;
    return true;
}

static int hex_nibble(char ch) {
    if (ch >= '0' && ch <= '9') {
        return ch - '0';
    }
    if (ch >= 'a' && ch <= 'f') {
        return 10 + (ch - 'a');
    }
    if (ch >= 'A' && ch <= 'F') {
        return 10 + (ch - 'A');
    }
    return -1;
}

static bool decode_hex_page(const char *hex, uint8_t *out, size_t *out_len) {
    if (hex == NULL || out == NULL || out_len == NULL) {
        return false;
    }

    size_t hex_len = strlen(hex);
    if ((hex_len == 0) || ((hex_len & 1u) != 0) || (hex_len > OTA_MAX_CHUNK_BYTES * 2u)) {
        return false;
    }

    size_t decoded_len = hex_len / 2u;
    for (size_t i = 0; i < decoded_len; i++) {
        int hi = hex_nibble(hex[i * 2u]);
        int lo = hex_nibble(hex[i * 2u + 1u]);
        if (hi < 0 || lo < 0) {
            return false;
        }
        out[i] = (uint8_t)((hi << 4) | lo);
    }

    *out_len = decoded_len;
    return true;
}

static void ota_flash_callback(void *param) {
    (void)param;

    if (g_flash_op.kind == OTA_FLASH_OP_ERASE) {
        flash_range_erase(g_flash_op.offset, g_flash_op.length);
    }
    else if (g_flash_op.kind == OTA_FLASH_OP_PROGRAM) {
        flash_range_program(g_flash_op.offset, g_flash_op.data, g_flash_op.length);
    }
}

static bool ota_flash_erase(uint32_t offset, uint32_t length) {
    if ((offset % FLASH_SECTOR_SIZE) != 0 || (length % FLASH_SECTOR_SIZE) != 0 || length == 0) {
        ota_set_error("erase request was not sector aligned");
        return false;
    }

    g_flash_op.kind = OTA_FLASH_OP_ERASE;
    g_flash_op.offset = offset;
    g_flash_op.length = length;
    g_flash_op.data = NULL;

    int rc = flash_safe_execute(ota_flash_callback, NULL, 1000);
    if (rc != PICO_OK) {
        snprintf(g_ota_session.last_error, sizeof(g_ota_session.last_error),
                 "flash erase failed rc=%d", rc);
        return false;
    }

    return true;
}

static bool ota_flash_program_page(uint32_t offset, const uint8_t *page) {
    if ((offset % FLASH_PAGE_SIZE) != 0 || page == NULL) {
        ota_set_error("program request was not page aligned");
        return false;
    }

    g_flash_op.kind = OTA_FLASH_OP_PROGRAM;
    g_flash_op.offset = offset;
    g_flash_op.length = FLASH_PAGE_SIZE;
    g_flash_op.data = page;

    int rc = flash_safe_execute(ota_flash_callback, NULL, 1000);
    if (rc != PICO_OK) {
        snprintf(g_ota_session.last_error, sizeof(g_ota_session.last_error),
                 "flash program failed rc=%d", rc);
        return false;
    }

    return true;
}

static bool ota_write_metadata(uint32_t size, uint32_t expected_crc, uint32_t actual_crc) {
    ota_metadata_t meta;
    memset(&meta, 0xFF, sizeof(meta));
    meta.magic = OTA_METADATA_MAGIC;
    meta.version = OTA_METADATA_VERSION;
    meta.image_size = size;
    meta.expected_crc32 = expected_crc;
    meta.actual_crc32 = actual_crc;
    meta.flags = OTA_METADATA_FLAG_VERIFIED;
    meta.image_offset = OTA_IMAGE_OFFSET;
    meta.primary_limit = ota_primary_image_limit();

    if (!ota_flash_erase(OTA_METADATA_OFFSET, FLASH_SECTOR_SIZE)) {
        return false;
    }

    memset(g_flash_page, 0xFF, sizeof(g_flash_page));
    memcpy(g_flash_page, &meta, sizeof(meta));
    return ota_flash_program_page(OTA_METADATA_OFFSET, g_flash_page);
}

static bool ota_invalidate_metadata(void) {
    if (!ota_supported()) {
        return false;
    }
    return ota_flash_erase(OTA_METADATA_OFFSET, FLASH_SECTOR_SIZE);
}

static void ota_send_json(struct fs_file *file, int payload_len) {
    if (payload_len < 0) {
        payload_len = 0;
    }
    if ((size_t)payload_len >= sizeof(g_ota_response)) {
        payload_len = (int)sizeof(g_ota_response) - 1;
        g_ota_response[payload_len] = '\0';
    }

    file->data = g_ota_response;
    file->len = strlen(g_ota_response);
    file->index = file->len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
}

static bool ota_build_status_json(bool success, const char *message, bool include_error) {
    const ota_metadata_t *meta = ota_supported() ? ota_metadata_flash() : NULL;
    bool meta_valid = ota_metadata_is_valid(meta);
    const char *last_error = include_error ? g_ota_session.last_error : "";
    if (last_error == NULL) {
        last_error = "";
    }
    if (message == NULL) {
        message = "";
    }

    uint32_t staged_size = meta_valid ? meta->image_size : 0u;
    uint32_t staged_crc = meta_valid ? meta->actual_crc32 : 0u;

    snprintf(g_ota_response,
             sizeof(g_ota_response),
             "%s"
             "{\"success\":%s,"
             "\"message\":\"%s\","
             "\"supported\":%s,"
             "\"transport\":\"rest_get_hex\","
             "\"apply_supported\":%s,"
             "\"bootloader_required\":%s,"
             "\"active\":%s,"
             "\"verified\":%s,"
             "\"metadata_valid\":%s,"
             "\"flash_size_bytes\":%lu,"
             "\"primary_limit_bytes\":%lu,"
             "\"staging_offset\":%lu,"
             "\"image_offset\":%lu,"
             "\"capacity_bytes\":%lu,"
             "\"expected_size\":%lu,"
             "\"received_size\":%lu,"
             "\"expected_crc32\":\"%08lX\","
             "\"actual_crc32\":\"%08lX\","
             "\"staged_size\":%lu,"
             "\"staged_crc32\":\"%08lX\","
             "\"last_error\":\"%s\"}",
             http_json_header,
             boolean_to_string(success),
             message,
             boolean_to_string(ota_supported()),
             boolean_to_string(OTA_BOOTLOADER_APPLY_SUPPORTED != 0),
             boolean_to_string(OTA_BOOTLOADER_APPLY_SUPPORTED == 0),
             boolean_to_string(g_ota_session.active),
             boolean_to_string(g_ota_session.verified),
             boolean_to_string(meta_valid),
             (unsigned long)PICO_FLASH_SIZE_BYTES,
             (unsigned long)ota_primary_image_limit(),
             (unsigned long)OTA_METADATA_OFFSET,
             (unsigned long)OTA_IMAGE_OFFSET,
             (unsigned long)ota_storage_capacity(),
             (unsigned long)g_ota_session.expected_size,
             (unsigned long)g_ota_session.received_size,
             (unsigned long)g_ota_session.expected_crc32,
             (unsigned long)g_ota_session.actual_crc32,
             (unsigned long)staged_size,
             (unsigned long)staged_crc,
             last_error);
    return true;
}

void ota_update_init(void) {
    const ota_metadata_t *meta = ota_supported() ? ota_metadata_flash() : NULL;
    printf("OTA: flash=%lu primary_limit=%lu staging=0x%08lX image=0x%08lX capacity=%lu supported=%s apply=%s staged=%s\n",
           (unsigned long)PICO_FLASH_SIZE_BYTES,
           (unsigned long)ota_primary_image_limit(),
           (unsigned long)OTA_METADATA_OFFSET,
           (unsigned long)OTA_IMAGE_OFFSET,
           (unsigned long)ota_storage_capacity(),
           ota_supported() ? "yes" : "no",
           OTA_BOOTLOADER_APPLY_SUPPORTED ? "yes" : "no",
           ota_metadata_is_valid(meta) ? "yes" : "no");
}

bool http_rest_ota_status(struct fs_file *file, int num_params, char *params[], char *values[]) {
    (void)num_params;
    (void)params;
    (void)values;

    ota_build_status_json(true, "ok", true);
    ota_send_json(file, (int)strlen(g_ota_response));
    return true;
}

bool http_rest_ota_begin(struct fs_file *file, int num_params, char *params[], char *values[]) {
    uint32_t size = 0;
    uint32_t crc32 = 0;

    memset(&g_ota_session, 0, sizeof(g_ota_session));

    if (!ota_supported()) {
        ota_set_error("OTA staging is not supported on this flash layout");
        ota_build_status_json(false, "unsupported flash layout", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    if (!parse_u32_param(num_params, params, values, "size", &size) ||
        !parse_u32_param(num_params, params, values, "crc32", &crc32)) {
        ota_set_error("missing size or crc32");
        ota_build_status_json(false, "missing size or crc32", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    if (size == 0 || size > ota_storage_capacity() || size > ota_primary_image_limit()) {
        ota_set_error("image does not fit OTA slots");
        ota_build_status_json(false, "image does not fit OTA slots", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    if (!ota_invalidate_metadata()) {
        ota_build_status_json(false, "could not invalidate previous OTA metadata", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    g_ota_session.active = true;
    g_ota_session.expected_size = size;
    g_ota_session.expected_crc32 = crc32;
    g_ota_session.actual_crc32 = 0;
    g_ota_session.received_size = 0;
    g_ota_session.verified = false;
    ota_clear_error();

    ota_build_status_json(true, "OTA upload started", true);
    ota_send_json(file, (int)strlen(g_ota_response));
    return true;
}

bool http_rest_ota_chunk(struct fs_file *file, int num_params, char *params[], char *values[]) {
    uint32_t offset = 0;
    const char *hex = param_value(num_params, params, values, "data");
    size_t decoded_len = 0;

    if (!g_ota_session.active) {
        ota_set_error("no OTA upload is active");
        ota_build_status_json(false, "no OTA upload is active", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    if (!parse_u32_param(num_params, params, values, "offset", &offset) ||
        !decode_hex_page(hex, g_flash_page, &decoded_len)) {
        ota_set_error("bad chunk request");
        ota_build_status_json(false, "bad chunk request", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    if (offset != g_ota_session.received_size || (offset % FLASH_PAGE_SIZE) != 0) {
        ota_set_error("chunks must be sequential and page aligned");
        ota_build_status_json(false, "chunk offset rejected", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    if (decoded_len > OTA_MAX_CHUNK_BYTES ||
        offset + decoded_len > g_ota_session.expected_size ||
        offset + decoded_len > ota_storage_capacity()) {
        ota_set_error("chunk exceeds expected image size");
        ota_build_status_json(false, "chunk exceeds expected image size", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    if ((decoded_len % FLASH_PAGE_SIZE) != 0 &&
        offset + decoded_len != g_ota_session.expected_size) {
        ota_set_error("non-final chunks must end on a flash page boundary");
        ota_build_status_json(false, "non-final chunk alignment rejected", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    size_t program_len = decoded_len;
    if (program_len % FLASH_PAGE_SIZE != 0) {
        program_len = ((program_len / FLASH_PAGE_SIZE) + 1u) * FLASH_PAGE_SIZE;
    }

    if (decoded_len < program_len) {
        if (offset + decoded_len != g_ota_session.expected_size) {
            ota_set_error("only the final chunk may be partial");
            ota_build_status_json(false, "partial non-final chunk rejected", true);
            ota_send_json(file, (int)strlen(g_ota_response));
            return true;
        }
        memset(g_flash_page + decoded_len, 0xFF, program_len - decoded_len);
    }

    for (size_t page_offset = 0; page_offset < program_len; page_offset += FLASH_PAGE_SIZE) {
        uint32_t flash_offset = OTA_IMAGE_OFFSET + offset + (uint32_t)page_offset;

        if (((offset + page_offset) % FLASH_SECTOR_SIZE) == 0) {
            if (!ota_flash_erase(flash_offset, FLASH_SECTOR_SIZE)) {
                ota_build_status_json(false, "sector erase failed", true);
                ota_send_json(file, (int)strlen(g_ota_response));
                return true;
            }
        }

        if (!ota_flash_program_page(flash_offset, g_flash_page + page_offset)) {
            ota_build_status_json(false, "page program failed", true);
            ota_send_json(file, (int)strlen(g_ota_response));
            return true;
        }
    }

    size_t verify_len = decoded_len;
    const uint8_t *flash_ptr = ota_image_flash() + offset;
    if (memcmp(flash_ptr, g_flash_page, verify_len) != 0) {
        ota_set_error("page verify failed");
        ota_build_status_json(false, "page verify failed", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    g_ota_session.actual_crc32 = crc32_update(g_ota_session.actual_crc32, flash_ptr, verify_len);
    g_ota_session.received_size += (uint32_t)decoded_len;
    ota_clear_error();

    ota_build_status_json(true, "chunk accepted", true);
    ota_send_json(file, (int)strlen(g_ota_response));
    return true;
}

bool http_rest_ota_finalize(struct fs_file *file, int num_params, char *params[], char *values[]) {
    (void)num_params;
    (void)params;
    (void)values;

    if (!g_ota_session.active) {
        ota_set_error("no OTA upload is active");
        ota_build_status_json(false, "no OTA upload is active", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    if (g_ota_session.received_size != g_ota_session.expected_size) {
        ota_set_error("upload is incomplete");
        ota_build_status_json(false, "upload is incomplete", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    uint32_t flash_crc = crc32_buffer(ota_image_flash(), g_ota_session.expected_size);
    g_ota_session.actual_crc32 = flash_crc;

    if (flash_crc != g_ota_session.expected_crc32) {
        ota_set_error("CRC mismatch");
        ota_build_status_json(false, "CRC mismatch", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    if (!ota_write_metadata(g_ota_session.expected_size,
                            g_ota_session.expected_crc32,
                            g_ota_session.actual_crc32)) {
        ota_build_status_json(false, "metadata write failed", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    g_ota_session.verified = true;
    g_ota_session.active = false;
    ota_clear_error();

    ota_build_status_json(true, "image staged and verified", true);
    ota_send_json(file, (int)strlen(g_ota_response));
    return true;
}

bool http_rest_ota_abort(struct fs_file *file, int num_params, char *params[], char *values[]) {
    (void)num_params;
    (void)params;
    (void)values;

    memset(&g_ota_session, 0, sizeof(g_ota_session));
    if (ota_supported()) {
        ota_invalidate_metadata();
    }

    ota_build_status_json(true, "OTA upload aborted", true);
    ota_send_json(file, (int)strlen(g_ota_response));
    return true;
}

bool http_rest_ota_apply(struct fs_file *file, int num_params, char *params[], char *values[]) {
    (void)num_params;
    (void)params;
    (void)values;

    const ota_metadata_t *meta = ota_metadata_flash();
    if (!ota_metadata_is_valid(meta)) {
        ota_set_error("no verified staged firmware is available");
        ota_build_status_json(false, "no verified staged firmware is available", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

#if OTA_BOOTLOADER_APPLY_SUPPORTED
    if (g_ota_apply_scheduled) {
        ota_build_status_json(true, "OTA apply already scheduled", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    uint32_t image_size = meta->image_size;
    uint32_t program_size = OTA_ALIGN_UP(image_size, FLASH_PAGE_SIZE);
    uint32_t erase_size = OTA_ALIGN_UP(program_size, FLASH_SECTOR_SIZE);
    if (program_size == 0u ||
        erase_size > OTA_METADATA_OFFSET ||
        program_size > ota_primary_image_limit()) {
        ota_set_error("staged firmware does not fit the primary slot");
        ota_build_status_json(false, "staged firmware does not fit the primary slot", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    const uint8_t *image = ota_image_flash();
    if (ota_staged_image_has_container_magic(image, image_size)) {
        ota_set_error("staged file looks like UF2/ELF; upload app.bin for OTA");
        ota_build_status_json(false, "upload app.bin, not UF2 or ELF", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    uint32_t staged_crc = crc32_buffer(image, image_size);
    if (staged_crc != meta->expected_crc32 || staged_crc != meta->actual_crc32) {
        ota_set_error("staged firmware CRC no longer matches metadata");
        ota_build_status_json(false, "staged firmware CRC check failed", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    g_ota_apply_params.image_size = image_size;
    g_ota_apply_params.expected_crc32 = meta->expected_crc32;
    g_ota_apply_params.actual_crc32 = staged_crc;
    g_ota_apply_scheduled = true;

    BaseType_t task_created = xTaskCreate(ota_apply_task,
                                          "OTA Apply",
                                          OTA_APPLY_TASK_STACK_WORDS,
                                          NULL,
                                          configMAX_PRIORITIES - 1,
                                          NULL);
    if (task_created != pdPASS) {
        g_ota_apply_scheduled = false;
        ota_set_error("could not create OTA apply task");
        ota_build_status_json(false, "could not create OTA apply task", true);
        ota_send_json(file, (int)strlen(g_ota_response));
        return true;
    }

    ota_clear_error();
    ota_build_status_json(true, "OTA apply scheduled; device will reboot shortly", true);
    ota_send_json(file, (int)strlen(g_ota_response));
    return true;
#else
    ota_set_error("verified image is staged, but this firmware has no OTA bootloader yet");
    ota_build_status_json(false,
                          "verified image staged; install an OTA bootloader transition build once to enable apply",
                          true);
    ota_send_json(file, (int)strlen(g_ota_response));
    return true;
#endif
}
