#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdbool.h>
#include <stddef.h>  

#define F_CPU               16000000UL // 16 MHz
#define BAUD                9600  
#define UBRR_VAL            ((F_CPU / 16 / BAUD) - 1)

#define CHANNEL_CFG         0x00
#define CHANNEL_I2C         0x01
#define CHANNEL_ECHO        0xFF

#define RX_QUEUE_SIZE       4
#define TX_QUEUE_SIZE       4
#define MAX_PAYLOAD         128

#define ANY_ACK             0x00
#define ANY_NACK            0xFF
#define CFG_I2C_SET_FREQ    0x01
#define CFG_I2C_SCAN        0x02

typedef struct{
    uint8_t channel; 
    uint8_t length;
    uint8_t data[MAX_PAYLOAD];
}Packet;

// Points to where new data should be written, and to where
// old data is read
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;
volatile uint8_t tx_head = 0;
volatile uint8_t tx_tail = 0;

volatile Packet rx_queue[RX_QUEUE_SIZE];
volatile Packet tx_queue[TX_QUEUE_SIZE];
volatile Packet* current_tx_packet = NULL;

volatile bool i2c_was_set = false;
volatile uint16_t i2c_frequency_khz = 0;

void i2c_setup(){
    uint32_t i2c_frequency_hz = i2c_frequency_khz * 1000UL;
    
    // Set bitrate (Datasheet page 180) with prescaler = 1
    uint32_t twbr = (((F_CPU / i2c_frequency_hz) - 16) / (2 * 1));
    // Minimum and maximum frequencies
    if (twbr > 255){
        twbr = 255;
    }
    if (i2c_frequency_khz > 400){
        twbr = 0;
    }
    TWBR = (uint8_t) twbr;
    
    // Clear prescaler bits
    TWSR &= ~((1 << TWPS1) | (1 << TWPS0));  
    
    // For interrupt based IO
    // Enable TWI interrupt and ACK control
    // TWCR = (1 << TWIE) | (1 << TWEA) | (1 << TWEN); 
    // Enable global interrupts
    //sei();  

    // Change flag 
    i2c_was_set = true;
}

void i2c_start(){
    // Send START
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);  

    // Wait for completion
    while (!(TWCR & (1 << TWINT))){} 
}

void i2c_stop(){
    // Send STOP
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);  
}

uint8_t i2c_write(uint8_t data){
    // Load data
    TWDR = data;

    // Start transmission
    TWCR = (1 << TWINT) | (1 << TWEN);

    // Wait for completion
    while (!(TWCR & (1 << TWINT)));

    // Return TWI status
    return (TWSR & 0xF8); 
}

uint8_t i2c_read(bool ack){
    TWCR = (1 << TWINT) | (1 << TWEN) | (ack ? (1 << TWEA) : 0);

    // Wait for completion
    while (!(TWCR & (1 << TWINT))){} 

    return TWDR;
}

uint8_t i2c_status(){
    return (TWSR & 0xF8);
}

void i2c_scan() {
    Packet response;
    response.channel = CHANNEL_CFG;
    response.length = 0;
    
    // Will store found addresses (up to 16)
    uint8_t found_devices[16] = {0}; 
    uint8_t found_count = 0;

    // Scan all possible I2C addresses (7-bit, so 0x08 to 0x77)
    for (uint8_t address = 0x08; address < 0x78; address++) {
        i2c_start();

        // Try to write to the address (even if we just want to detect presence)
        uint8_t status = i2c_write((address << 1) | 0x00); // Write operation
        
        // SLA+W transmitted and ACK received
        if (status == 0x18) { 
            // Device responded
            if (found_count < 16) {
                found_devices[found_count++] = address;
            }

            // Properly terminate the successful communication
            i2c_stop(); 
        }else{
            // No response or error. Send stop
            i2c_stop();
        }

        // Small delay between attempts
        _delay_us(100);
    }

    // Prepare response packet
    response.length = found_count;
    for (uint8_t i = 0; i < found_count; i++) {
        response.data[i] = found_devices[i];
    }

    // Send the response
    uart_send_packet(&response);
}


void uart_setup(){
    // Set high and low bytes of baud rate
    UBRR0H = (uint8_t)(UBRR_VAL >> 8); 
    UBRR0L = (uint8_t)UBRR_VAL;  

    // Enable TX and RX
    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); 
    
    // 8-bit data, no parity, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 
}

void uart_send_packet(volatile Packet* packet){
    // Disable interrupt in critical section
    cli();
   
    /* // Wait until there is a packet to send. 
    while (current_tx_packet != NULL){
        // Re-enable while waiting
        sei();
        _delay_us(1);
        cli();
    }*/
    
    uint8_t next_head = (tx_head + 1) % TX_QUEUE_SIZE;
    
    //if (current_tx_packet == NULL){
    // If not full
    if (next_head != tx_tail){
        tx_queue[tx_head] = *packet;
        tx_head = next_head;

        // Enable interrupt if queue was empty
        if (tx_head == ((tx_tail + 1) % TX_QUEUE_SIZE)){
            UCSR0B |= (1 << UDRIE0); 
        }
    }

    sei();
}

void uart_send_ack(uint8_t channel){
    static Packet ack_packet;
    ack_packet.channel = channel;
    ack_packet.length = 1;
    ack_packet.data[0] = ANY_ACK;
    uart_send_packet(&ack_packet);
}

void uart_send_nack(uint8_t channel){
    static Packet nack_packet;
    nack_packet.channel = channel;
    nack_packet.length = 1;
    nack_packet.data[0] = ANY_NACK;
    uart_send_packet(&nack_packet);
}

// Read host data via UART. This ISR works as 
// a state machine that fills a static incoming_packet 
// from UART.
ISR(USART_RX_vect){
    static uint8_t state = 0;
    static uint8_t payload_idx = 0;
    static Packet incoming_packet; 
    uint8_t byte = UDR0;

    switch(state){
        // Channel byte
        case 0:
            if (byte == CHANNEL_I2C || 
                byte == CHANNEL_ECHO || 
                byte == CHANNEL_CFG){
                // Fill channel info
                incoming_packet.channel = byte;
                
                // Go to next state
                state = 1;
            }else{
                // Go to lock until terminator state check and wait there
                state = 5;
            }
            break;
        
        // Payload length byte
        case 1:
            if (byte <= MAX_PAYLOAD && byte > 0){
                incoming_packet.length = byte;
                state = 2;
            
            // Ignore incoming if payload is too big. 
            // Could notify host via CFG channel later.
            }else{
                // Go to lock until terminator state check and wait there
                state = 5; 
            }
            break;

        // Data bytes
        case 2:
            if (payload_idx < incoming_packet.length){
                incoming_packet.data[payload_idx++] = byte;
            }
            
            // check if we can advance or not
            if (payload_idx == incoming_packet.length){
                state = 3;
            }
            break;

        // First terminator byte
        case 3:
            if (byte == '\r'){
                state = 4;
            }else{
                // Lock until terminator
                state = 5;
            }
            break;

        // Second terminator byte
        case 4:
            if (byte == '\n'){
                uint8_t next_head = (rx_head + 1) % RX_QUEUE_SIZE;

                // If not full, enqueue the packet
                if (next_head != rx_tail){
                    rx_queue[rx_head] = incoming_packet;
                    rx_head = next_head;
                
                // Handle buffer overflow
                }else{
                }
                
                // Start over
                state = 0;
                payload_idx = 0;
            
            // Handle incomplete packets by locking until terminator
            }else{
                state = 5;
            }
            break;
        
        // Lock until terminator (first byte)
        case 5:
            if (byte == '\r'){
                state = 6;
            }else{
                state = 5;
            }
            break;

            // Lock until terminator (second byte)
        case 6:
            if (byte == '\n'){
                state = 0;
                payload_idx = 0;
                
                // Warn host about broken packet
                uart_send_nack(CHANNEL_CFG);
            }else{
                // Go back to first byte of the lock
                state = 5;
            }
            break;
    }
}

// Send next byte to host via UART when ready 
ISR(USART_UDRE_vect){
    static uint8_t payload_idx = 0;
    static uint8_t state = 0;

    // Check if there is something to send
    if (!current_tx_packet){
        // Get next packet from queue if available
        if (tx_head != tx_tail){
            current_tx_packet = &tx_queue[tx_tail];
            tx_tail = (tx_tail + 1) % TX_QUEUE_SIZE;

            // Reset for new packet
            payload_idx = 0;
            state = 0;
            
        }else{
            // Disable interrupt if queue empty
            UCSR0B &= ~(1 << UDRIE0);    
            return;
        }
    }

    switch (state){
        // Channel byte
        case 0:
            UDR0 = current_tx_packet->channel;
            state = 1;
            break;

        // Length byte
        case 1:
            UDR0 = current_tx_packet->length;
            state = 2;
            break;

        // Data bytes
        case 2:
            if (payload_idx < current_tx_packet->length){
                UDR0 = current_tx_packet->data[payload_idx++];
            }

            if (payload_idx == current_tx_packet->length){
                state = 3;
            }
            break;

        // First terminator byte
        case 3:
            UDR0 = '\r';
            state = 4;
            break;

        // Second terminator byte
        case 4:
            // Terminate
            payload_idx = 0;
            state = 0;
            UDR0 = '\n';

            // Mark as complete
            current_tx_packet = NULL;
        
            // Disable TX interrupt if queue is empty
            if (tx_head == tx_tail){
                UCSR0B &= ~(1 << UDRIE0);
            }
            break;
    }
}

void process_cfg_command(const uint8_t* payload, uint8_t length){
    if (length < 1){
        uart_send_nack(CHANNEL_CFG);
        return;
    }

    uint8_t command = payload[0];
    Packet response;
    response.channel = CHANNEL_CFG;

    switch(command){
        // ACKs host back
        case ANY_ACK:
            uart_send_ack(CHANNEL_CFG);
            break;

        // Set I2C clock frequency
        case CFG_I2C_SET_FREQ:
            if (length != 3){
                uart_send_nack(CHANNEL_CFG);
            }else{
                i2c_frequency_khz = ((uint16_t)payload[1] << 8) | payload[2]; 

                i2c_setup();

                uart_send_ack(CHANNEL_CFG);
            }
            break;

        // Simple I2C address scan
        case CFG_I2C_SCAN:
            if (i2c_was_set){
                i2c_scan();
            }else{
                uart_send_nack(CHANNEL_CFG);
            }
            break;
        
        // defaults to a NACK
        default:
            uart_send_nack(CHANNEL_CFG);
            break;
    }       
}

void main(){
    uart_setup();
    sei();
    
    while(1){
        // If UART queue is not empty
        if (rx_tail != rx_head){
            // Process a config command
            if (rx_queue[rx_tail].channel == CHANNEL_CFG){
                process_cfg_command(&rx_queue[rx_tail].data, 
                                    rx_queue[rx_tail].length);
            
            // Handle I2C communication
            }else if (rx_queue[rx_tail].channel == CHANNEL_I2C && i2c_was_set == true){
                if (rx_queue[rx_tail].length >= 1){
                    //uint8_t command = rx_queue[rx_tail].data[0];
                    uint8_t address = rx_queue[rx_tail].data[0];
                    
                    uint8_t data_len = rx_queue[rx_tail].length - 1;
                    uint8_t* data = &rx_queue[rx_tail].data[1];

                    i2c_start();
                    
                    // Send address + i2c write bit (0)
                    uint8_t twi_status = i2c_write((address & 0x7F) << 1);
                    if((twi_status != 0x18)) { // Check for SLA+W ACK
                        uart_send_nack(CHANNEL_I2C);
                        i2c_stop();
                        continue;
                    }

                    // Send data
                    for (uint8_t i = 0; i < data_len; i++){
                        twi_status = i2c_write(data[i]);
                        if(twi_status != 0x28) { // Check ACK
                            uart_send_nack(CHANNEL_I2C);
                            break;
                        }
                    }

                    i2c_stop();
                    uart_send_ack(CHANNEL_I2C);
                    
                }else{
                    uart_send_nack(CHANNEL_CFG);
                }

            // Handle echo channel
            }else if(rx_queue[rx_tail].channel == CHANNEL_ECHO){
                // Echo
                uart_send_packet(&rx_queue[rx_tail]);
            }

            // Increment queue's reading position
            rx_tail = (rx_tail + 1) % RX_QUEUE_SIZE;
        }
    }
}




