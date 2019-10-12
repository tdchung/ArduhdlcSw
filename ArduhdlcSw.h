#ifndef arduhdlcSw_h
#define arduhdlcSw_h

#include "Arduino.h"
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <util/crc16.h>


#define DEFAUT_ENCODE_SEGMENT       "01"

// Packet type field - byte 0
#define SBR_PKT_RQST_INPUT_CREATE   'I'   // type[1] d_type[1] pad[2] path[] units[]
#define SBR_PKT_RESP_INPUT_CREATE   'i'   // type[1] status[1] pad[2]

#define SBR_PKT_RQST_OUTPUT_CREATE  'O'   // type[1] d_type[1] pad[2] path[] units[]
#define SBR_PKT_RESP_OUTPUT_CREATE  'o'   // type[1] status[1] pad[2]

#define SBR_PKT_RQST_DELETE         'D'   // type[1] pad[1]    pad[2] path[]
#define SBR_PKT_RESP_DELETE         'd'   // type[1] status[1] pad[2]

#define SBR_PKT_RQST_HANDLER_ADD    'H'   // type[1] pad[1]    pad[2] path[] 
#define SBR_PKT_RESP_HANDLER_ADD    'h'   // type[1] status[1] pad[2]

#define SBR_PKT_RQST_HANDLER_REMOVE 'K'   // type[1] pad[1]    pad[2] path[] 
#define SBR_PKT_RESP_HANDLER_REMOVE 'k'   // type[1] status[1] pad[2]

#define SBR_PKT_RQST_PUSH           'P'   // type[1] d_type[1] pad[2] time[] path[] data[]
#define SBR_PKT_RESP_PUSH           'p'   // type[1] status[1] pad[2]

#define SBR_PKT_RQST_GET            'G'   // type[1] pad[1]    pad[2] path[]
#define SBR_PKT_RESP_GET            'g'   // type[1] status[1] pad[2] time[] data[]

#define SBR_PKT_RQST_EXAMPLE_SET    'E'   // type[1] d_type[1] pad[2] path[] data[]
#define SBR_PKT_RESP_EXAMPLE_SET    'e'   // type[1] status[1] pad[2]

#define SBR_PKT_RQST_SENSOR_CREATE  'S'   // type[1] d_type[1] pad[2] path[] units[]
#define SBR_PKT_RESP_SENSOR_CREATE  's'   // type[1] status[1] pad[2]

#define SBR_PKT_RQST_SENSOR_REMOVE  'R'   // type[1] pad[1]    pad[2] path[]
#define SBR_PKT_RESP_SENSOR_REMOVE  'r'   // type[1] status[1] pad[2]

#define SBR_PKT_NTFY_HANDLER_CALL   'c'   // type[1] d_type[1] pad[2] time[] path[] data[]
#define SBR_PKT_RESP_HANDLER_CALL   'C'   // type[1] status[1] pad[2]

#define SBR_PKT_NTFY_SENSOR_CALL    'b'   // type[1] pad[1]    pad[2] path[]
#define SBR_PKT_RESP_SENSOR_CALL    'B'   // type[1] status[1] pad[2]

#define SBR_PKT_RESP_UNKNOWN_RQST   '?'   // type[1] status[1] pad[2]

// Variable length field identifiers
#define SBR_FIELD_ID_PATH           'P'
#define SBR_FIELD_ID_TIME           'T'
#define SBR_FIELD_ID_UNITS          'U'
#define SBR_FIELD_ID_DATA           'D'

// Data type field - byte 1
#define SBR_DATA_TYPE_TRIGGER       'T'   // trigger - no data
#define SBR_DATA_TYPE_BOOLEAN       'B'   // Boolean - 1 byte:  't' | 'f'
#define SBR_DATA_TYPE_NUMERIC       'N'   // numeric - null-terminated ASCII string, representing double
#define SBR_DATA_TYPE_STRING        'S'   // string  - null-terminated ASCII string
#define SBR_DATA_TYPE_JSON          'J'   // JSON    - null-terminated ASCII string, representing JSON
#define SBR_DATA_TYPE_UNDEF         ' '   // not specified

// typedef enum
// { 
//     SBR_DATA_TYPE_TRIGGER = 0,
//     SBR_DATA_TYPE_BOOLEAN = 1, 
//     SBR_DATA_TYPE_NUMERIC = 2,
//     SBR_DATA_TYPE_STRING  = 3,
//     SBR_DATA_TYPE_JSON    = 4,
//     SBR_DATA_TYPE_UNDEF   = 5 
// } data_type_t;

typedef enum
{ 
    REQUEST_CREATE = 0,
    REQUEST_DELETE = 1, 
    REQUEST_ADD    = 2,
    REQUEST_PUSH   = 3,
    REQUEST_GET    = 4,
    REQUEST_EXAMPLE= 5 
} request_package_t;



typedef void (* sendchar_type) (uint8_t);
typedef void (* frame_handler_type)(const uint8_t *framebuffer, uint16_t framelength);

class ArduhdlcSw
{
  public:
    ArduhdlcSw (sendchar_type, frame_handler_type, uint16_t max_frame_length);
    void charReceiver(uint8_t data);
    void frameDecode(const char *framebuffer, uint8_t frame_length);

    //tdchung
    char encode_dtype(int data_type); // move to private
    int encode_create( char* type, int dtype, char* path, char* unit, char* output);
    int encode_delete(char* type, char* path, char* output);
    int encode_add(char* type, char* path, char* output);
    int encode_push(int dtype, char* path, char* data, char* output);
    int encode_get(char* path, char* output);
    int encode_example(int  dtype, char* path, char* data, char* output);
    void encode_request(int request_tpye); // useless. (;

    char get_resp_package_type(char* data);
    char get_resp_status(char* data);
    int get_resp_path(char* data, int length, char* dataout);
    int get_resp_timestamp(char* data, int length, char* dataout);
    int get_resp_data(char* data, int length, char* dataout);

  private:
    /* User must define a function, that sends a 8bit char over the chosen interface, usart, spi, i2c etc. */
    sendchar_type sendchar_function;
    /* User must define a function, that will process the valid received frame */
    /* This function can act like a command router/dispatcher */
    frame_handler_type frame_handler;
    void sendchar(uint8_t data);

    bool escape_character;
    uint8_t * receive_frame_buffer;
    uint8_t frame_position;
    // 16bit CRC sum for _crc_ccitt_update
    uint16_t frame_checksum;
	uint16_t max_frame_length;

    // tdchung
    uint16_t crc16(char* pData, int length);
    char** str_split(char* a_str, const char a_delim);

};

#endif
