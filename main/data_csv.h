#ifndef PT100_LOGGER_DATA_CSV_H_
#define PT100_LOGGER_DATA_CSV_H_

#include <stdbool.h>
#include <stddef.h>

#include "log_record.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define CSV_SCHEMA_VERSION 1u

typedef bool (*csv_write_fn_t)(const char* bytes, size_t len, void* context);

bool CsvFormatHeader(char* out, size_t out_size, size_t* written_out);
bool CsvFormatRow(const log_record_t* record,
                  const char* node_id,
                  char* out,
                  size_t out_size,
                  size_t* written_out);

bool CsvWriteHeader(csv_write_fn_t writer, void* context);
bool CsvWriteRow(csv_write_fn_t writer,
                 void* context,
                 const log_record_t* record,
                 const char* node_id);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_DATA_CSV_H_
