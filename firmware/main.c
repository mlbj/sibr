#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stddef.h>  

#define F_CPU 16000000UL
#define BAUD 9600
#define UBRR_VAL ((F_CPU / 16 / BAUD) - 1)

#define I2C_CHANNEL 0x01
#define SPI_CHANNEL 0x02
#define CFG_CHANNEL 0x03 

#define RX_QUEUE_SIZE 8
#define TX_QUEUE_SIZE 8
#define MAX_PAYLOAD 32

typedef struct{
    uint8_t channel; // 0x01 (I2C), 0x02 (SPI) or 0x03 (Config)
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

void setup_serial(){
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
    ack_packet.data[0] = 0x00;
    uart_send_packet(&ack_packet);
}

void uart_send_nack(uint8_t channel){
    static Packet nack_packet;
    nack_packet.channel = channel;
    nack_packet.length = 1;
    nack_packet.data[0] = 0xFF;
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
            if (byte == I2C_CHANNEL || 
                byte == SPI_CHANNEL || 
                byte == CFG_CHANNEL){
                incoming_packet.channel = byte;
                state = 1;
            }else{
                // Go to lock until terminator state check and wait there
                state = 5;
            }
            break;
        
        // Payload length byte
        case 1:
            if (byte <= MAX_PAYLOAD){
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

}

void main(){
    setup_serial();
    sei();
    
    while(1){
        // If UART queue is not empty
        if (rx_tail != rx_head){
            // Check if the message was sent for the Config channel
            if (rx_queue[rx_tail].channel == CFG_CHANNEL){
                process_cfg_command(&rx_queue[rx_tail].data, 
                                    rx_queue[rx_tail].length);
            
            // write to either I2C or SPI
            }else{ 
                // Echo
                uart_send_packet(&rx_queue[rx_tail]);
                //uart_send_ack(CFG_CHANNEL);
                //uart_send_nack(CFG_CHANNEL);
            }

            // Increment queue's reading position
            rx_tail = (rx_tail + 1) % RX_QUEUE_SIZE;
        }
    }
}




