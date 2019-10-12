/*
Arduhdlc is Arduino HDLC library,
create base on: https://github.com/jarkko-hautakorpi/Arduhdlc

tdchung
tdchung.9@gmail.com
12-Oct-2019

*/

#include "Arduino.h"
#include "ArduhdlcSw.h"

/* HDLC Asynchronous framing */
/* The frame boundary octet is 01111110, (7E in hexadecimal notation) */
#define FRAME_BOUNDARY_OCTET 0x7E

/* A "control escape octet", has the bit sequence '01111101', (7D hexadecimal) */
#define CONTROL_ESCAPE_OCTET 0x7D

/* If either of these two octets appears in the transmitted data, an escape octet is sent, */
/* followed by the original data octet with bit 5 inverted */
#define INVERT_OCTET 0x20

/* The frame check sequence (FCS) is a 16-bit CRC-CCITT */
/* AVR Libc CRC function is _crc_ccitt_update() */
/* Corresponding CRC function in Qt (www.qt.io) is qChecksum() */
#define CRC16_CCITT_INIT_VAL 0xFFFF

#define DEFAULT_LENGHT 128

/* 16bit low and high bytes copier */
#define low(x)    ((x) & 0xFF)
#define high(x)   (((x)>>8) & 0xFF)

ArduhdlcSw::ArduhdlcSw (sendchar_type put_char,
                        frame_handler_type hdlc_command_router,
                        uint16_t max_frame_length) : sendchar_function(put_char), frame_handler(hdlc_command_router)
{
    this->frame_position = 0;
	this->max_frame_length = max_frame_length;
	this->receive_frame_buffer = (uint8_t *)malloc(max_frame_length+1); // char *ab = (char*)malloc(12);
    this->frame_checksum = CRC16_CCITT_INIT_VAL;
    this->escape_character = false;
}

// tdchung
// Algorithm  CRC-16/CCITT-FALSE
uint16_t ArduhdlcSw::crc16(char* pData, int length)
{
    uint8_t i;
    uint16_t wCrc = CRC16_CCITT_INIT_VAL;
    while (length--) {
        wCrc ^= *(unsigned char *)pData++ << 8;
        for (i=0; i < 8; i++)
            wCrc = wCrc & 0x8000 ? (wCrc << 1) ^ 0x1021 : wCrc << 1;
    }
    return wCrc & CRC16_CCITT_INIT_VAL;
}

/* Function to send a byte throug USART, I2C, SPI etc.*/
void ArduhdlcSw::sendchar(uint8_t data)
{
    (*this->sendchar_function)(data);
}

/* Function to find valid HDLC frame from incoming data */
void ArduhdlcSw::charReceiver(uint8_t data)
{
    /* FRAME FLAG */
    if(data == FRAME_BOUNDARY_OCTET)
    {
        if(this->escape_character == true)
        {
            this->escape_character = false;
        }
        /* If a valid frame is detected */
        // tdchung, update hander frame
        else if( (this->frame_position >= 2)
                //   &&
                //   ( this->frame_checksum == ((this->receive_frame_buffer[this->frame_position-1] << 8 ) |
                //   (this->receive_frame_buffer[this->frame_position-2] & 0xff)) ) 
                )  // (msb << 8 ) | (lsb & 0xff)
        {
            /* Call the user defined function and pass frame to it */
            // (*frame_handler)(receive_frame_buffer,(uint8_t)(this->frame_position-2));

            // tdchung, update check crc
            uint16_t fcs = this->crc16(receive_frame_buffer, frame_position-2);
            // 
            // if (fcs == ((this->receive_frame_buffer[this->frame_position-1] << 8 )
            //             | (this->receive_frame_buffer[this->frame_position-2] & 0xff)))
            if (fcs == ((this->receive_frame_buffer[this->frame_position-1] ) 
                        | (this->receive_frame_buffer[this->frame_position-2] << 8)))
            {
                (*frame_handler)(receive_frame_buffer,(uint8_t)(this->frame_position));
            }
            else
            {
                // crc not match
            }
        }
        this->frame_position = 0;
        this->frame_checksum = CRC16_CCITT_INIT_VAL;
        return;
    }

    if(this->escape_character)
    {
        this->escape_character = false;
        data ^= INVERT_OCTET;
    }
    else if(data == CONTROL_ESCAPE_OCTET)
    {
        this->escape_character = true;
        return;
    }

    receive_frame_buffer[this->frame_position] = data;

    if(this->frame_position-2 >= 0) {
        this->frame_checksum = _crc_ccitt_update(this->frame_checksum, receive_frame_buffer[this->frame_position-2]);
    }

    this->frame_position++;

    if(this->frame_position == this->max_frame_length)
    {
        this->frame_position = 0;
        this->frame_checksum = CRC16_CCITT_INIT_VAL;
    }
}

/* Wrap given data in HDLC frame and send it out byte at a time*/
void ArduhdlcSw::frameDecode(const char *framebuffer, uint8_t frame_length)
{
    uint8_t data;
    // uint16_t fcs = CRC16_CCITT_INIT_VAL;

    // tdchung. make cpu run slow
    uint16_t fcs = this->crc16(framebuffer, frame_length);

    this->sendchar((uint8_t)FRAME_BOUNDARY_OCTET);

    while(frame_length)
    {
        data = *framebuffer++;
        // fcs = _crc_ccitt_update(fcs, data);
        if((data == CONTROL_ESCAPE_OCTET) || (data == FRAME_BOUNDARY_OCTET))
        {
            this->sendchar((uint8_t)CONTROL_ESCAPE_OCTET);
            data ^= INVERT_OCTET;
        }
        this->sendchar((uint8_t)data);
        frame_length--;
    }
    // data = low(fcs);
    // if((data == CONTROL_ESCAPE_OCTET) || (data == FRAME_BOUNDARY_OCTET))
    // {
    //     this->sendchar((uint8_t)CONTROL_ESCAPE_OCTET);
    //     data ^= (uint8_t)INVERT_OCTET;
    // }
    // this->sendchar((uint8_t)data);
    // data = high(fcs);
    // if((data == CONTROL_ESCAPE_OCTET) || (data == FRAME_BOUNDARY_OCTET))
    // {
    //     this->sendchar(CONTROL_ESCAPE_OCTET);
    //     data ^= INVERT_OCTET;
    // }

    // tdchung, update cacaulate crc16
    data = high(fcs);
    if((data == CONTROL_ESCAPE_OCTET) || (data == FRAME_BOUNDARY_OCTET))
    {
        this->sendchar((uint8_t)CONTROL_ESCAPE_OCTET);
        data ^= (uint8_t)INVERT_OCTET;
    }
    this->sendchar((uint8_t)data);
    // data = high(fcs);
    data = low(fcs);
    if((data == CONTROL_ESCAPE_OCTET) || (data == FRAME_BOUNDARY_OCTET))
    {
        this->sendchar(CONTROL_ESCAPE_OCTET);
        data ^= INVERT_OCTET;
    }

    this->sendchar(data);
    this->sendchar(FRAME_BOUNDARY_OCTET);
}

// privite function
// split string c
char** ArduhdlcSw::str_split(char* a_str, const char a_delim)
{
    char** result    = 0;
    size_t count     = 0;
    char* tmp        = a_str;
    char* last_comma = 0;
    char delim[2];
    delim[0] = a_delim;
    delim[1] = 0;

    /* Count how many elements will be extracted. */
    while (*tmp)
    {
        if (a_delim == *tmp)
        {
            count++;
            last_comma = tmp;
        }
        tmp++;
    }

    /* Add space for trailing token. */
    count += last_comma < (a_str + strlen(a_str) - 1);

    /* Add space for terminating null string so caller
       knows where the list of returned strings ends. */
    count++;

    result = malloc(sizeof(char*) * count);

    if (result)
    {
        size_t idx  = 0;
        char* token = strtok(a_str, delim);

        while (token)
        {
            assert(idx < count);
            *(result + idx++) = strdup(token);
            token = strtok(0, delim);
        }
        assert(idx == count - 1);
        *(result + idx) = 0;
    }

    return result;
}

// encode data type, byte[1]
// useless
char ArduhdlcSw::encode_dtype(int data_type)
{
    switch (data_type)
    {
        case SBR_DATA_TYPE_TRIGGER: // trigger - no data
            // printf("SBR_DATA_TYPE_TRIGGER\n");
            return 'T';
            break;
        case SBR_DATA_TYPE_BOOLEAN: // Boolean - 1 byte:  't' | 'f'
            // printf("SBR_DATA_TYPE_BOOLEAN\n");
            return 'B';
            break;
        case SBR_DATA_TYPE_NUMERIC: // numeric - null-terminated ASCII string, representing double
            // printf("SBR_DATA_TYPE_NUMERIC\n");
            return 'N';
            break;
        case SBR_DATA_TYPE_STRING: // string  - null-terminated ASCII string
            // printf("SBR_DATA_TYPE_STRING\n");
            return 'S';
            break;
        case SBR_DATA_TYPE_JSON: // JSON    - null-terminated ASCII string, representing JSON
            // printf("SBR_DATA_TYPE_JSON\n"); // not specified
            return 'J';
            break;
        case SBR_DATA_TYPE_UNDEF:  // not specified
        default:
            // printf("SBR_DATA_TYPE_UNDEF\n");
            return ' ';
            break;
    }
}


// TODO: update return value
int ArduhdlcSw::encode_create(
    char* type,          //>> input|output|sensor
    int dtype,           //>> 
    char* path,          //>> 
    char* unit,          //>> [units] optional
    char* output         //<< output pointer
)
{
    int result = 0;
    char dataout[DEFAULT_LENGHT] = {0};
    char package_type = 0;
    char data_type = 0; 

    // package type
    if (0 == strcmp(type, "input"))
    {
        package_type = SBR_PKT_RQST_INPUT_CREATE;
    }
    else if (0 == strcmp(type, "output"))
    {
        package_type = SBR_PKT_RQST_OUTPUT_CREATE;
    }
    else if (0 == strcmp(type, "sensor"))
    {
        package_type = SBR_PKT_RQST_SENSOR_CREATE;
    }
    else 
    {
        return 0;
    }

    // data tpye
    data_type = encode_dtype(dtype);

    if (NULL == unit)
    {
        snprintf(dataout, 
                DEFAULT_LENGHT,
                "%c%c%sP%s", 
                package_type, 
                data_type, 
                DEFAUT_ENCODE_SEGMENT, 
                path);
    }
    else
    {
        snprintf(dataout, 
                DEFAULT_LENGHT, 
                "%c%c%sP%s,U%s", 
                package_type, 
                data_type, 
                DEFAUT_ENCODE_SEGMENT, 
                path, 
                unit);
    }

    // strncpy
    strcpy(output, dataout);
    return result;
}


// delete resource|handler|sensor path
// TODO: update return value
int ArduhdlcSw::encode_delete(
    char* type,          //>> resource|handler|sensor
    char* path,          //>> 
    char* output         //<< output
)
{
    int result = 0; 
    char package_type = 0;
    char dataout[DEFAULT_LENGHT] = {0};

    // package type
    if (0 == strcmp(type, "resource"))
    {
        //      printf("resource\n");
        package_type = SBR_PKT_RQST_DELETE;
    }
    else if (0 == strcmp(type, "handler"))
    {
        //      printf("handler\n");
        package_type = SBR_PKT_RQST_HANDLER_REMOVE;

    }
    else if (0 == strcmp(type, "sensor"))
    {
        //      printf("sensor\n");
        package_type = SBR_PKT_RQST_SENSOR_REMOVE;
    }
    else 
    {
        //      printf("invalid\n");
        return 0;
    }

    snprintf(dataout, 
            DEFAULT_LENGHT, 
            "%c.%sP%s", 
            package_type, 
            DEFAUT_ENCODE_SEGMENT, 
            path);
            
    strcpy(output, dataout);
    return result;
}

// delete resource|handler|sensor path
// TODO: update return value
int ArduhdlcSw::encode_add(
    char* type,          //>> handler
    char* path,          //>> 
    char* output          //<<
)
{
    int result = 0; 
    char package_type = 0;
    char dataout[DEFAULT_LENGHT] = {0};

    if (0 == strcmp(type, "handler"))
    {
        // printf("handler\n");
        package_type = SBR_PKT_RQST_HANDLER_ADD;
    }
    else 
    {
        //      printf("invalid\n");
        return 0;
    }

    snprintf(dataout, 
            128, 
            "%c.%sP%s", 
            package_type, 
            DEFAUT_ENCODE_SEGMENT, 
            path);

    strcpy(output, dataout);
    return result;
}


// push data-type path [data]
// push trig|bool|num|str|json <path> [<data>]',
// TODO: update return value
int ArduhdlcSw::encode_push(
    int dtype,            //>> 
    char* path,           //>>
    char* data,           //>> [data] optional
    char* output          //<<
)
{
    int result = 0;
    char dataout[DEFAULT_LENGHT] = {0};
    char data_tpye = 0;
    char package_type = SBR_PKT_RQST_PUSH;

    // data tpye
    data_tpye = encode_dtype(dtype);

    if (NULL == data)
    {
        snprintf(dataout, 
                DEFAULT_LENGHT, 
                "%c%c%s%c%s", 
                package_type, 
                data_tpye, 
                DEFAUT_ENCODE_SEGMENT,
                SBR_FIELD_ID_PATH,
                path);
    }
    else
    {
        snprintf(dataout,
                DEFAULT_LENGHT,
                "%c%c%s%c%s,%c%s", 
                package_type, 
                data_tpye,
                DEFAUT_ENCODE_SEGMENT,
                SBR_FIELD_ID_PATH,
                path, 
                SBR_FIELD_ID_DATA,
                data);
    }

    strcpy(output, dataout);
    return result;
}

// get path
// TODO: update return value
int ArduhdlcSw::encode_get(
    char* path,           //>>
    char* output          //<<
)
{
    int result = 0;
    char dataout[DEFAULT_LENGHT] = {0};
    char package_type = SBR_PKT_RQST_GET;

    // data type ignored
    snprintf(dataout, 
            DEFAULT_LENGHT,
            "%c.%s%c%s",
            package_type,
            DEFAUT_ENCODE_SEGMENT,
            SBR_FIELD_ID_PATH,
            path);

    strcpy(output, dataout);
    return result;
}

// example data-type path [data]
// TODO: update return value
int ArduhdlcSw::encode_example(
    int  dtype,            //>> 
    char* path,            //>>
    char* data,            //>> [data] optional
    char* output           //<<
)
{
    int result = 0;
    char dataout[DEFAULT_LENGHT] = {0};
    char package_type = SBR_PKT_RQST_EXAMPLE_SET;
    char data_tpye = 0;

    // data tpye
    data_tpye = encode_dtype(dtype);

    if (NULL == data)
    {
        snprintf(dataout,
                DEFAULT_LENGHT,
                "%c%c%s%c%s", 
                package_type, 
                data_tpye,
                DEFAUT_ENCODE_SEGMENT,
                SBR_FIELD_ID_PATH,
                path);
    }
    else
    {
        snprintf(dataout,
                DEFAULT_LENGHT,
                "%c%c%s%c%s,%c%s",
                package_type, 
                data_tpye,
                DEFAUT_ENCODE_SEGMENT, 
                SBR_FIELD_ID_PATH,
                path, 
                SBR_FIELD_ID_DATA, 
                data);
    }

    strcpy(output, dataout);
    return result;
}

void ArduhdlcSw::encode_request(int request_tpye)
{
    switch (request_tpye)
    {
        case REQUEST_CREATE:
            // 
            //      printf("REQUEST_CREATE\n");
            break;

        case REQUEST_DELETE:
            //      printf("REQUEST_DELETE\n");
            break;
        
        case REQUEST_ADD:
            //      printf("REQUEST_ADD\n");
            break;
        
        case REQUEST_PUSH:
            //      printf("REQUEST_PUSH\n");
            break;
        
        case REQUEST_GET:
            //      printf("REQUEST_GET\n");
            break;
        
        case REQUEST_EXAMPLE:
            //      printf("REQUEST_EXAMPLE\n");
            break;
        
        default:
            //      printf("default\n");
            break;
    }
}

char ArduhdlcSw::get_resp_package_type(char* data)
{
    return data[0];
}

char ArduhdlcSw::get_resp_status(char* data)
{
    return data[1];
}

int ArduhdlcSw::get_resp_path(char* data, int length, char* dataout)
{
    int result = 0;

    if (length < 4)
    {
        // invalid data
        return 0;
    }

    char** tokens;
    int i;
    char string[DEFAULT_LENGHT] = {0};

    tokens = this->str_split(data+4, ',');
    if (tokens)
    {
        for (i = 0; *(tokens + i); i++)
        {
            snprintf(string, DEFAULT_LENGHT, "%s", *(tokens + i) );
            if ('P' == string[0])
            {
                strcpy(dataout, string+1);
                result = 1;
            }
            else
            {
                result = 0;
                // invalid data path
            }

            free(*(tokens + i));
        }
        free(tokens);
    }
    return result;
}


int ArduhdlcSw::get_resp_timestamp(char* data, int length, char* dataout)
{
    int result = 0;

    if (length < 4)
    {
        // invalid data
        return 0;
    }

    char** tokens;
    int i;
    char string[DEFAULT_LENGHT] = {0};

    tokens = this->str_split(data+4, ',');
    if (tokens)
    {
        for (i = 0; *(tokens + i); i++)
        {
            snprintf(string, DEFAULT_LENGHT, "%s", *(tokens + i) );
            if ('T' == string[0])
            {
                strcpy(dataout, string+1);
                result = 1;
            }
            else
            {
                result = 0;
                // invalid data path
            }

            free(*(tokens + i));
        }
        free(tokens);
    }
    return result;
}


int ArduhdlcSw::get_resp_data(char* data, int length, char* dataout)
{
    int result = 0;

    if (length < 4)
    {
        // invalid data
        return 0;
    }

    char** tokens;
    int i;
    char string[DEFAULT_LENGHT] = {0};

    tokens = this->str_split(data+4, ',');
    if (tokens)
    {
        for (i = 0; *(tokens + i); i++)
        {
            snprintf(string, DEFAULT_LENGHT, "%s", *(tokens + i) );
            if ('D' == string[0])
            {
                strcpy(dataout, string+1);
                result = 1;
            }
            else
            {
                result = 0;
                // invalid data path
            }

            free(*(tokens + i));
        }
        free(tokens);
    }
    return result;
}
