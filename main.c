/*
 * CR95HF C.c
 *
 * Created: 31/08/2019 14:19:01
 * Author : zeidp
 */ 
#define  F_CPU 16000000

#define CS_DDR DDRB
#define CS_PORT PORTB
#define CS_PIN PINB2

#define irq_in_DDR DDRB
#define irq_in_PORT PORTB
#define irq_in_PIN PINB1

#define irq_out_DDR DDRB
#define irq_out_PORT PORTB
#define irq_out_PIN PINB0
#define irq_out_PINR PINB

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"

void SPI_init();
char SPI_transmit(uint8_t);
uint8_t CR95HF_init();
uint8_t CR95HF_protocolSelect();
uint8_t CR95HF_inventoryCommand(uint8_t, uint8_t[], uint8_t[]);
uint8_t CR95HF_antiCollision(uint8_t maskLength, uint8_t maskValue[]);
void setBit(uint8_t n, uint8_t maskValue[], uint8_t x);

uint8_t total_tags_found = 0;
uint8_t tags[12][8];

int main(void)
{
	uart_init(); // open the communication to the microcontroller
	io_redirect(); // redirect input and output to the communication
	printf("Hello !!\n");
    if(CR95HF_init()){
		printf("initialization success\n");
	}else{
		printf("initialization failed\n");
	}
	
	if(CR95HF_protocolSelect()){
		printf("protocol select success\n");
	}else{
		printf("protocol select failed\n");
	}
	
	uint8_t mask[8];
	uint8_t numTags = CR95HF_antiCollision(0, mask);
	printf("Number of tags found: %d\n", numTags);
	for(int i = 0; i < numTags; i++){
		for(int j = 0; j < 8; j++){
			printf("%X", tags[i][j]);
		}
		printf("\n");
	}
}

void SPI_init(){
	DDRB |= (1<<PORTB3) | (1<<PORTB5) | (1<<PORTB2); //set sck, mosi and ss to output
	SPCR |= (1<<SPE) | (1<<MSTR) | (1<<SPR0);//enable spi, master, set clock rate fck/128
}

char SPI_transmit(uint8_t data){
	SPDR = data; //transmit data
	while(!(SPSR & (1<<SPIF)));//wait for transition
	return SPDR;//return data received from slave
}

uint8_t CR95HF_init(){
	CS_DDR |= (1 << CS_PIN);  //set cs as output
	CS_PORT |= (1 << CS_PIN);
	irq_in_DDR |= (1 << irq_in_PIN);  //set irq in as output
	irq_in_PORT |= (1 << irq_in_PIN);
	irq_out_DDR &= ~(1 << irq_out_PIN); //set irq out as input
	
	SPI_init();
	
	//wake-up command
	irq_in_PORT &= ~(1 << irq_in_PIN);
	_delay_ms(1);
	irq_in_PORT |= (1 << irq_in_PIN);
	
	
	//Echo command and responce: MCU sends 0x55 to CR95HF, CR95HF should reply with 0x55 otherwise initialization failed
	CS_PORT &= ~(1 << CS_PIN);
	SPI_transmit(0x00);//CR95HF control byte: "Send command to the CR95HF"
	SPI_transmit(0x55);//CR95HF command: "Echo"
	CS_PORT |= (1 << CS_PIN);
	
	//wait for cr95hf
	while((irq_out_PINR & (1<<irq_out_PIN)) != 0) //wait until irq out becomes low
	
	
	//getting responce
	CS_PORT &= ~(1 << CS_PIN);
	SPI_transmit(0x02);//CR95HF control byte: "Read data from the CR95HF"
	char rec = SPI_transmit(0x00);
	CS_PORT |= (1 << CS_PIN);
	
	if(rec == 0x55){
		return 1;
	}else{
		return 0;
	}	
}

uint8_t CR95HF_protocolSelect(){
	//protocol select
	CS_PORT &= ~(1 << CS_PIN);
	SPI_transmit(0x00);//CR95HF control byte: "Send command to the CR95HF"
	SPI_transmit(0x02);//CR95HF command:  "ProtocolSelect"
	SPI_transmit(2);//length of data
	SPI_transmit(0x01);//ISO/IEC 15693  protocol
	SPI_transmit(0b00001101);//Parameters: 26Kbps(H), Wait for SOF, 10% modulation, Single subcarrier
	CS_PORT |= (1 << CS_PIN);
	
	//wait for cr95hf
	while((irq_out_PINR & (1<<irq_out_PIN)) != 0) //wait until irq out becomes low
	
	//getting responce
	CS_PORT &= ~(1 << CS_PIN);
	SPI_transmit(0x02);//CR95HF control byte: "Read data from the CR95HF"
	uint8_t rec1 = SPI_transmit(0x00);
	uint8_t rec2 = SPI_transmit(0x00);
	CS_PORT |= (1 << CS_PIN);
	
	return ((rec1 == 0) & (rec2 == 0));
}

uint8_t CR95HF_inventoryCommand(uint8_t maskLength, uint8_t maskValue[], uint8_t UID[]){
	//this function returns 0 if no tags were detected, returns 1 if only 1 tag is detected, returns 2 if collision is detected
	uint8_t tagsNum = 0;
	
	CS_PORT &= ~(1 << CS_PIN);
	SPI_transmit(0x00);//CR95HF control byte: "Send command to the CR95HF"
	SPI_transmit(0x04);//CR95HF command:  "SendRecv"
	SPI_transmit(3 + ceil(maskLength/8.0));//length of data
	SPI_transmit(0b00100110);//Flags: single subcarrier, high data rate, inventory flag:1, no protocol extension, afi filed is not present, 1 slot, option flag 0, RFU
	SPI_transmit(0x01);//ISO15693 Command: "Inventory"
	SPI_transmit(maskLength); //mask length
	for(int i = 0; i < ceil(maskLength/8.0); i++){
		SPI_transmit(maskValue[i]);
	}
	CS_PORT |= (1 << CS_PIN);
	

	//wait for cr95hf
	while((irq_out_PINR & (1<<irq_out_PIN)) != 0) //wait until irq out becomes low

	//getting responce
	CS_PORT &= ~(1 << CS_PIN);
	SPI_transmit(0x02);//CR95HF control byte: "Read data from the CR95HF"
	uint8_t responce = SPI_transmit(0x00); //result code or error code
	SPI_transmit(0x00);//length of data
	if(responce == 0x80)//succes
	{
		SPI_transmit(0x00);//responce flags
		SPI_transmit(0x00);//responce DSFID
		for(int i = 0; i < 8; i++){
			UID[7-i] = SPI_transmit(0x00);
		}
		SPI_transmit(0x00);//crc
		SPI_transmit(0x00);//crc
		uint8_t lastByte = SPI_transmit(0x00); //RFU[7:2]     1 : crc error if set       0 : collision detected if set
		if(lastByte == 0)//no collision
		{
			tagsNum =  1;
		}else //collision
		{
			tagsNum = 2;
		}
	}else //no tags
	{
		tagsNum = 0;
	}
	CS_PORT |= (1 << CS_PIN);
	return tagsNum;
}

uint8_t CR95HF_antiCollision(uint8_t maskLength, uint8_t maskValue[]){
	uint8_t tagsFound = 0;
	maskLength++;
	uint8_t uid[8];
	
	setBit(maskLength, maskValue, 0);
	uint8_t result = CR95HF_inventoryCommand(maskLength, maskValue, uid);
	if(result == 0){
		//do nothing
	}else if(result == 1){
		tagsFound++;
		for(int i = 0; i < 8; i++){
			tags[total_tags_found][i] = uid[i];
		}
		total_tags_found++;
	}else{
		//recursion
		tagsFound += CR95HF_antiCollision(maskLength, maskValue);
	}

	setBit(maskLength, maskValue, 1);
	result = CR95HF_inventoryCommand(maskLength, maskValue, uid);
	if(result == 0){
		//do nothing
	}else if(result == 1){
		tagsFound++;
		for(int i = 0; i < 8; i++){
			tags[total_tags_found][i] = uid[i];
		}
		total_tags_found++;
	}else{
		//recursion
		tagsFound += CR95HF_antiCollision(maskLength, maskValue);
	}

	return tagsFound;
}

void setBit(uint8_t n, uint8_t maskValue[], uint8_t x){
	//function set nth bit of maskValue to x
	n--;
	if(x == 0){
		maskValue[n/8] &= ~(1 << n%8);
		}else if(x == 1){
		maskValue[n/8] |= (1 << n%8);
	}
}