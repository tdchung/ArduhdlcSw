#include "ArduhdlcSw.h"

#define MAX_HDLC_FRAME_LENGTH 256
/* Function to send out byte/char */
void send_character(uint8_t data);

/* Function to handle a valid HDLC frame */
void hdlc_frame_handler(const uint8_t *data, uint16_t length);

/* Initialize Arduhdlc library with three parameters.
1. Character send function, to send out HDLC frame one byte at a time.
2. HDLC frame handler function for received frame.
3. Length of the longest frame used, to allocate buffer in memory */
ArduhdlcSw hdlc(&send_character, &hdlc_frame_handler, MAX_HDLC_FRAME_LENGTH);

/* Function to send out one 8bit character */
void send_character(uint8_t data) {
    Serial.print((char)data);
}

/* Frame handler function. What to do with received data? */
void hdlc_frame_handler(const uint8_t *data, uint16_t length)
{

    // get packet_type
    char packet_type = hdlc.get_resp_package_type((char*) data);
    
    // get response status
    char status = hdlc.get_resp_status((char*) data);

    // get path
    char path[128] = {0};
    if (hdlc.get_resp_path((char*) data, length, path))
    {
        // execute path here
        // example
        // if (0 == strcmp(path, "path/to/path"))
        // do something
    }
}

void setup()
{
    pinMode(1,OUTPUT); // Serial port TX to output
    // initialize serial port to 9600 baud
    Serial.begin(9600);
}

void loop() {
    char my_frame[MAX_HDLC_FRAME_LENGTH] = {0};

    // push resource
    hdlc.encode_push(SBR_DATA_TYPE_STRING, "path/to/push", "helloworld", my_frame);
    
    // send to master
    hdlc.frameDecode(my_frame, strlen(my_frame));
    delay(2000);
}

/*
SerialEvent occurs whenever a new data comes in the
hardware serial RX.  This routine is run between each
time loop() runs, so using delay inside loop can delay
response.  Multiple bytes of data may be available.
*/
void serialEvent() {
    while (Serial.available()) {
        // get the new byte:
        char inChar = (char)Serial.read();
        // Pass all incoming data to hdlc char receiver
        hdlc.charReceiver(inChar);
    }
}
