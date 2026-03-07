#ifndef PT100_LOGGER_DATA_CSV_H_
#define PT100_LOGGER_DATA_CSV_H_

#include <stdbool.h>
#include <stddef.h>

#include "log_record.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define CSV_SCHEMA_VERSION 3u

typedef bool (*csv_write_fn_t)(const char* bytes, size_t len, void* context);

/**
 * @brief Execute CsvFormatHeader.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 * @param written_out Parameter written_out.
 * @return Return the function result.
 */
bool CsvFormatHeader(char* out, size_t out_size, size_t* written_out);
/**
 * @brief Execute CsvFormatRow.
 * @param record Parameter record.
 * @param node_id Parameter node_id.
 * @param out Parameter out.
 * @param out_size Parameter out_size.
 * @param written_out Parameter written_out.
 * @return Return the function result.
 */
bool CsvFormatRow(const log_record_t* record,
                  const char* node_id,
                  char* out,
                  size_t out_size,
                  size_t* written_out);

/**
 * @brief Execute CsvWriteHeader.
 * @param writer Parameter writer.
 * @param context Parameter context.
 * @return Return the function result.
 */
bool CsvWriteHeader(csv_write_fn_t writer, void* context);
/**
 * @brief Execute CsvWriteRow.
 * @param writer Parameter writer.
 * @param context Parameter context.
 * @param record Parameter record.
 * @param node_id Parameter node_id.
 * @return Return the function result.
 */
bool CsvWriteRow(csv_write_fn_t writer,
                 void* context,
                 const log_record_t* record,
                 const char* node_id);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_DATA_CSV_H_
