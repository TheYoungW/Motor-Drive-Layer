#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct mb_dm_frame {
  uint32_t can_id;
  uint8_t data[8];
  uint8_t dlc;
  uint8_t channel;
  uint8_t ext;
  uint8_t canfd;
};

int mb_dm_open(const char* library_path,
               int device_type,
               uint8_t selected_channel,
               uint32_t can_baudrate,
               uint32_t canfd_baudrate,
               void** out,
               char* err_buf,
               size_t err_len);

int mb_dm_send(void* handle,
               uint32_t can_id,
               uint8_t ext,
               uint8_t dlc,
               const uint8_t* data,
               char* err_buf,
               size_t err_len);

int mb_dm_recv(void* handle,
               struct mb_dm_frame* out,
               uint32_t timeout_ms,
               char* err_buf,
               size_t err_len);

void mb_dm_shutdown(void* handle);

#ifdef __cplusplus
}
#endif
