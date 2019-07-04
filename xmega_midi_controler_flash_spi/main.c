/*
 * xmega_midi_controler_flash_spi.c
 *
 * Created: 2019-07-02 17:02:05
 * Author : Branden
 */ 

#define  F_CPU 32000000UL

#include <avr/io.h>
#include <stdio.h>
#include <avr/delay.h>
#include <string.h>
#include <avr/interrupt.h>

#define CS_ENABLE() (PORTC.OUTCLR = PIN4_bm)
#define CS_DISABLE() (PORTC.OUTSET = PIN4_bm)

#define TIMER_ENABLE() (TCC0.CTRLA = TC_TC0_CLKSEL_DIV4_gc)
#define TIMER_DISABLE() (TCC0.CTRLA = TC_TC0_CLKSEL_OFF_gc)

#define MEM_WRITE_STATUS 0x01
#define MEM_WRITE 0x02
#define MEM_READ 0x03
#define MEM_READ_STATUS 0x05
#define MEM_WRITE_EN 0x06
#define MEM_FAST_READ 0x0B
#define MEM_SECT_ERASE 0x20
#define MEM_CHIP_ERASE 0xC7
#define MEM_128K_ERASE 0xD2
#define MEM_READ_ID 0x9F

#define MEM_STAT_WEN 0x02
#define MEM_STAT_BUSY 0x01

#define RX_WRITE_TEXT 0x10
#define RX_WRITE_DATA 0x20
#define RX_ERASE_ALL 0x30
#define RX_READ_ID 0x01
#define RX_READ_TEXT 0x11
#define RX_READ_DATA 0x21

#define MEM_BLOCK_SIZE 256
#define MEM_SECTOR_SIZE 4096

#define FILE_COUNT_ADDR MEM_SECTOR_SIZE + 4 //File count starts 4 bytes after second sector.

#define MIDI_MAX_MIDI_TRACKS 8

#define MIDI_NOTE_OFF_MASK 0x80 + 0x00
#define MIDI_NOTE_ON_MASK 0x80 + 0x10
#define MIDI_POLY_KEY_MASK 0x80 + 0x20
#define MIDI_CTRL_CHANGE_MASK 0x80 + 0x30
#define MIDI_PROG_CHANGE_MASK 0x80 + 0x40
#define MIDI_CH_PRESSURE_MASK 0x80 + 0x50
#define MIDI_PITCH_BEND_MASK 0x80 + 0x60
#define MIDI_SYS_EX 0xF0

#define MIDI_HEADER_TRACk_COUNT_OFFSET 10
#define MIDI_HEADER_TIME_DIV_OFFSET 12
#define MIDI_FIRST_TRACK_OFFSET 14

#define MIDI_START 0xFA
#define MIDI_CONTINUE 0xFB
#define MIDI_STOP 0xFC 
#define MIDI_UPDATE_MODE 0xFD

#define INPUT_PAUSE_MASK 0x80
#define INPUT_PLAY_MODE_UPDATING_MASK 0x40
#define INPUT_PLAY_MODE_MAKS 0x0C
#define INPUT_PLAY_MODE_SHIFTED_MAKS 0x03
#define INPUT_BUTTON_0_MASK 0x01
#define INPUT_BUTTON_1_MASK 0x02

#define TIMER_PER_FOR_120BPM 15625 / 4  // div 4 because timer clock is also div4

typedef struct track_t
{
	uint32_t addressOffset;
	uint32_t startAddress;
	uint32_t deltaTime;
	uint8_t eventByte;
	uint8_t eventData1;
	uint8_t eventData2;	
} midiTrack;

void uart_putchar(uint8_t c, FILE * stream);
FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

volatile uint16_t _fileCount = 0;
volatile uint16_t _fileIndex = 0 ;
volatile uint32_t _fileAddressOffset = 0;
volatile uint8_t _tractCount = 0;
volatile uint8_t _activeTracks = 0;
volatile int16_t _division; //SIGNED
volatile uint32_t _tempo; //may be able to remove this;
volatile midiTrack _tracks[MIDI_MAX_MIDI_TRACKS];
volatile uint8_t _clockTickFlag = 0;
volatile uint8_t _input0Counts;
volatile uint8_t _input1Counts;
volatile uint8_t _statusFlags; //7 = Paused, 6 = Updating play mode, 5 = reserved, 4 = reserved, 3:2 = play mode, 1 = button 1 down, 0 = button 0 down


void uart_putchar(uint8_t c, FILE * stream)
{
	while(!(USARTE0.STATUS & USART_DREIF_bm)) {}
		
	USARTE0.DATA = c;	
}

void initClk()
{
	OSC.CTRL = OSC_RC32MEN_bm; //enable 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));   //wait for stability
	CCP = CCP_IOREG_gc; //secured access
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc; //choose this osc source as clk
}

void initSPI()
{
	PORTC.DIRSET = PIN4_bm | PIN5_bm | PIN7_bm;
	PORTC.DIRCLR = PIN6_bm;
	SPIC.CTRL =  SPI_CLK2X_bm | SPI_ENABLE_bm | SPI_MASTER_bm | SPI_PRESCALER_DIV16_gc;
}

void initDebugUART()
{
	PORTE.DIRSET = PIN3_bm;
	USARTE0.CTRLB = USART_TXEN_bm;
	USARTE0.CTRLC = USART_CHSIZE_8BIT_gc;
	
}

void initMidiUART()
{
	PORTD.DIRSET = PIN3_bm;
	USARTD0.BAUDCTRLA = 131;
	USARTD0.BAUDCTRLB = 0 << 4; //Set the bsel to 1 and the bscale to 6
	USARTD0.CTRLB = USART_TXEN_bm | USART_CLK2X_bm;
	USARTD0.CTRLC = USART_CHSIZE_8BIT_gc;
	
}

void initMidiTimer()
{
	TCC1.CTRLA = TC_TC1_CLKSEL_DIV64_gc;
	TCC1.INTCTRLA = TC_TC1_OVFINTLVL_HI_gc;
	TCC1.PER = 5000; //Should cause this to tick at about 100Hz (every 10ms)
}

void initInputTimer()
{
	TCC0.CTRLA = TC_TC0_CLKSEL_OFF_gc;
	TCC0.INTCTRLA = TC_TC0_OVFINTLVL_HI_gc;
	TCC0.PER = TIMER_PER_FOR_120BPM;	
}

void initInputADC()
{
	ADCA.CTRLB = ADC_IMPMODE_bm | ADC_CURRLIMIT_LOW_gc | ADC_RESOLUTION_8BIT_gc;
	ADCA.REFCTRL = ADC_REFSEL_AREFA_gc;
	ADCA.PRESCALER = ADC_PRESCALER_DIV512_gc;
	ADCA.CH1.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;
	ADCA.CTRLA = ADC_ENABLE_bm;
	
}


void initInterrupts()
{
	PMIC.CTRL |= PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
}

uint8_t sendMidi(uint8_t data)
{
	while(!(USARTD0.STATUS & USART_DREIF_bm)) {}
		
	USARTD0.DATA = data;
}

uint8_t sendSPI(uint8_t data)
{
	SPIC.DATA = data;
	
	while(!(SPIC.STATUS & SPI_IF_bm)) {}
	
	return SPIC.DATA;
	
}

uint8_t sendDummy()
{
	return sendSPI(0x00);
}

void sendString(uint8_t *chr)
{
	while(*chr)
	{
		sendSPI(*(chr++));
	}
	
}

void memSendAddress(uint32_t address)
{
	sendSPI((uint8_t)((address >> 16) & 0xFF));
	sendSPI((uint8_t)((address >> 8) & 0xFF));
	sendSPI((uint8_t)((address >> 0) & 0xFF));
	//printf("Address: 0x%08lx\r\n", address);
}

uint8_t getMemStatus()
{
	uint8_t out;
	CS_DISABLE();
	CS_ENABLE();
	
	sendSPI(MEM_READ_STATUS);
	out = sendDummy();
	CS_DISABLE();
	return out;
}

void waitForNotBusy()
{
	while((getMemStatus() & MEM_STAT_BUSY)) {}
}

void memEnableWrite()
{
	//printf("Enabling Write\r\n");
	CS_DISABLE();
	CS_ENABLE();
	
	sendSPI(MEM_WRITE_EN);
	
	while(!(getMemStatus() & MEM_STAT_WEN)) {}
	
	CS_DISABLE();
	return;
}

void memErase128kBlock(uint32_t address)
{
	waitForNotBusy();
	
	memEnableWrite();
	
	
	
	CS_DISABLE();
	CS_ENABLE();
	
	sendSPI(MEM_128K_ERASE);
	memSendAddress(address);
	
	CS_DISABLE();
	return;
}

void memEraseSector(uint32_t address)
{
	waitForNotBusy();
	
	memEnableWrite();
	
	
	
	CS_DISABLE();
	CS_ENABLE();
	
	sendSPI(MEM_SECT_ERASE);
	memSendAddress(address);
	
	CS_DISABLE();
	return;
}

void memReadLinesToStdOut(uint32_t address, uint16_t lines)
{
	uint8_t data[16];
	waitForNotBusy();
	
	CS_DISABLE();
	CS_ENABLE();
	sendSPI(MEM_READ);
	memSendAddress(address);;
	
	for(uint8_t i = 0; i < lines; i++)
	{
		for (uint8_t j = 0; j < 16; j++)
		{
			data[j] = sendDummy();
			printf("%02x ", data[j]);
		}
		
		printf("\t");
		
		for (uint8_t j = 0; j < 16; j++)
		{
			printf("%c", data[j]);
		}
		printf("\r\n");
	}
	
	CS_DISABLE();
	return;
}

void memWrite256(uint32_t address, uint8_t data)
{
	waitForNotBusy();
	
	memEnableWrite();

	CS_DISABLE();
	CS_ENABLE();
	
	sendSPI(MEM_WRITE);
	memSendAddress(address);
	
	for(uint16_t i = 0; i < 256; i++)
	{
		sendSPI(data);
	}
	
	CS_DISABLE();
	return;
	
}

void memWriteString(uint32_t address, uint8_t *str)
{
	waitForNotBusy();
	
	memEnableWrite();

	CS_DISABLE();
	CS_ENABLE();
	
	sendSPI(MEM_WRITE);
	memSendAddress(address);
	
	while(*str)
	{
		sendSPI(*(str++));
	}
	
	CS_DISABLE();
	return;
	
}

void memReadToBuffer(uint32_t address, void *buff, uint8_t len)
{
	uint8_t *ptr = buff;
	waitForNotBusy();

	CS_DISABLE();
	CS_ENABLE();
	
	sendSPI(MEM_READ);
	//printf("addr: 0x%08lx \r\n", address);
	memSendAddress(address);
	
	do
	{
		*(ptr++) = sendDummy();
	}while(--len);
	
	CS_DISABLE();
	return;
	
}

void memWriteBuff(uint32_t address, void *buff, uint8_t len)
{
	uint8_t *ptr = buff;
	waitForNotBusy();
	
	memEnableWrite();

	CS_DISABLE();
	CS_ENABLE();
	
	sendSPI(MEM_WRITE);
	memSendAddress(address);
	
	do
	{
		sendSPI(*(ptr++));
	}while(--len);
	
	CS_DISABLE();
	return;
	
}


static uint32_t getVariableLengthValue(uint8_t *first4bytes, uint8_t *out_numBytesUsed)
{
	uint32_t value = 0;
	
	*out_numBytesUsed = 0;
	
	while(*out_numBytesUsed < 4)
	{
		
		value <<= 7;
		//printf("byte %u = %u\r\n", *out_numBytesUsed, *(first4bytes + *out_numBytesUsed));
		if ((*(first4bytes + *out_numBytesUsed) & 0x80))
		{
			value += ((uint32_t)(*(first4bytes + *out_numBytesUsed) & 0x7F));
		}
		else
		{
			value += *(first4bytes + *out_numBytesUsed);
			(*out_numBytesUsed)++;
			return value;
		}
		(*out_numBytesUsed)++;
	}
	//printf("early exit\r\n");
}

//Sets _fileCount
void getFileCount()
{
	uint8_t data[2];
	memReadToBuffer(FILE_COUNT_ADDR, data, 2);
	_fileCount = ((uint16_t)data[0] << 8) + (uint16_t)data[1];	
}

//Uses _fileIndex
//Sets _fileAddressOffset
//Since we don't need the file names, we will automatically add the name length to the offset
void moveToFile()
{
	
	uint8_t data[4];
	uint32_t address = FILE_COUNT_ADDR + 2; //starting address for lookup table is 2 bytes after the size of the total data section.
	_fileAddressOffset = address + (uint32_t)(_fileCount * 4); //first file address is the address of the first lookup table entry + all entries (Which are each 4 bytes)
	
	//if the index we are looking for is 0 or the file count is 0, then we can leave the address we initialized _fileAddressOffset to
	if (_fileIndex > 0 && _fileCount > 0)
	{
		//Make sure the index is not larger than the file count
		if (_fileIndex >= _fileCount)
		{
			_fileIndex = _fileCount - 1;
		}	
		
		//We are basically summing up all of the file sizes until we reach our file index
		for(uint16_t i = 0; i < _fileIndex; i++)
		{
			//read file sizes and add them too the offsets			
			memReadToBuffer(address, data, 4);
			address += 4;
			_fileAddressOffset += ((uint32_t)data[0] << 24) + ((uint32_t)data[1] << 16) + ((uint32_t)data[2] << 8) + ((uint32_t)data[3] << 0);
			printf("Index: %u New _fileAddressOffset 0x%08lx. %02x %02x %02x %02x\r\n",i, _fileAddressOffset, data[0], data[1], data[2], data[3]);
		}
	}
	
	//As a convenience, here we add file name length to offset.
	memReadToBuffer(_fileAddressOffset, data, 2);
	printf("Reading file name length from 0x%08lx. Len: %02x %02x\r\n", _fileAddressOffset, data[0] ,data[1]);
	_fileAddressOffset +=  ((uint32_t)data[0] << 8) + ((uint32_t)data[1] << 0) + 2; //+ 2 for the name length bytes
	printf("New _fileAddressOffset: 0x%08lx	\r\n", _fileAddressOffset);
	
}

//Uses _fileAddressOffset
//Sets _trackCount
void getTrackCount()
{
	uint8_t data[2];

	memReadToBuffer(_fileAddressOffset + MIDI_HEADER_TRACk_COUNT_OFFSET, data, 2);
	printf("Track Count From 0x%08lx, 0x%02x, 0x%02x\r\n", _fileAddressOffset + MIDI_HEADER_TRACk_COUNT_OFFSET, data[0], data[1]);
	_tractCount = ((uint16_t)data[0] << 8) + (uint16_t)data[1];	
	
}


//Uses _fileAddressOffset
//Sets _division
void getTimeDivision()
{
	//If _division is positive it represents Ticks per quarter note. By default the tempo is 120BPM, or 120 quarter notes per minute, 1 quarter note in 0.5 seconds
	//This would mean a tick (a clock tick) would be 0.5/_division. e.g. with a _division of 256, each tick would be 0.001953125 seconds long.
	//With this, an 8th note would have an even length (delta-time for the following Note-off event for that note) of 128 ticks, or a duration of 0.25 seconds.
	//A change in Tempo (meta-event 0xFF with an arg of 51) would effectively change the BPM.
	//For the SMPTE, the _division is calculated by multiplying the first byte by the second byte and then by -1. This means a value of 0xE7 0x28 would be the same as 0x03 0xE8 (or 1000uS, 1mS)	
	uint8_t data[2];

	memReadToBuffer(_fileAddressOffset + MIDI_HEADER_TIME_DIV_OFFSET, data, 2);
	_division = ((uint16_t)data[0] << 8) + (uint16_t)data[1];
	
	
	
	if (_division < 0) //_division is negative so is in SMPTE format
	{
		_division = -1 * (int8_t)data[0] * data[1];
	}	
	
	printf("Division From 0x%08lx, 0x%02x, 0x%02x (%d)\r\n", _fileAddressOffset + MIDI_HEADER_TIME_DIV_OFFSET, data[0], data[1], _division);
}



uint8_t getNextEvent(uint8_t trackIndex)
{
	uint8_t done = 0;
	uint8_t data[4];
	uint8_t i;
	
	_tracks[trackIndex].deltaTime = 0;
	
	//printf("Getting next event\r\n");
	while(!done) //(_tracks[usedTracks].addressOffset < _tracks[usedTracks].trackLength)
	{

			
		//Clear data buff and determine the variable length time
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
		data[3] = 0;

		//The last byte will not have the MSB set.
		data[0] = sendDummy();
		        //printf("Track %u offset 0x%08lx, data[0]: 0x%02x\r\n", trackIndex, _tracks[trackIndex].addressOffset, data[0]);
		_tracks[trackIndex].addressOffset++;
		if ((data[0] & 0x80))
		{
			data[1] = sendDummy();
			_tracks[trackIndex].addressOffset++;
			if ((data[1] & 0x80))
			{
				data[2] = sendDummy();
				_tracks[trackIndex].addressOffset++;
				if ((data[2] & 0x80))
				{
					data[3] = sendDummy();
					_tracks[trackIndex].addressOffset++;
				}
			}
		}
		//printf("Track %u offset 0x%08lx after d\r\n", trackIndex, _tracks[trackIndex].addressOffset);

		//just using 'i' as a temp
		_tracks[trackIndex].deltaTime += getVariableLengthValue(data, &i);

		i = 0;
			
		_tracks[trackIndex].eventByte = sendDummy(); // this should be the event byte
		
		//printf("Evaluating tract %u event 0x%02x: dt: %lu (%02x %02x %02x %02x) \r\n", trackIndex, _tracks[trackIndex].eventByte, _tracks[trackIndex].deltaTime, data[0], data[1], data[2], data[3]);
		
		_tracks[trackIndex].addressOffset++;
			
		//printf("dt: %lu, event: 0x%02x, offset: %lu\r\n", _tracks[trackIndex].deltaTime, _tracks[trackIndex].eventByte, _tracks[trackIndex].addressOffset);

		if ((_tracks[trackIndex].eventByte & 0xF0) == MIDI_NOTE_ON_MASK || (_tracks[trackIndex].eventByte & 0xF0) == MIDI_NOTE_OFF_MASK || (_tracks[trackIndex].eventByte & 0xF0) == MIDI_PITCH_BEND_MASK)
		{
			//These are valid events that we want to handle, so capture the next two bytes and move to the next track
				
			_tracks[trackIndex].eventData1 = sendDummy();
			_tracks[trackIndex].eventData2 = sendDummy();
			_tracks[trackIndex].addressOffset += 2;

			//printf("Event 0x%02x found for track %u\r\n", _tracks[trackIndex].eventByte, trackIndex);
			return 0xFF; 			
		}
		if ((_tracks[trackIndex].eventByte & 0xF0) == MIDI_POLY_KEY_MASK || (_tracks[trackIndex].eventByte & 0xF0) == MIDI_CTRL_CHANGE_MASK)//if the event is one of these, skip the next two bytes and continue.
		{
			sendDummy();
			sendDummy();
			_tracks[trackIndex].addressOffset += 2;
		}
		if ( (_tracks[trackIndex].eventByte & 0xF0) == MIDI_PROG_CHANGE_MASK || (_tracks[trackIndex].eventByte & 0xF0) == MIDI_CH_PRESSURE_MASK) //if the event is one of these, skip the next byte and continue.
		{
			sendDummy();
			_tracks[trackIndex].addressOffset++;
		}
		//Now we have to check other event types to know how far to skip ahead
		else if(_tracks[trackIndex].eventByte == 0xFF) //Meta-Events
		{
			data[0] = sendDummy(); //Get the event type
			_tracks[trackIndex].addressOffset++;
			//Here we are just looking for the number of bytes to jump, except for in the case of FF 51 03 (Tempo)
			switch(data[0])
			{
				case 0x00:
					i = 1;
					break;
				case 0x01: //all of the following are text that we can skip
				case 0x02:
				case 0x03:
				case 0x04:
				case 0x05:
				case 0x06:
				case 0x07:
				case 0x08:
				case 0x09:
					i = sendDummy(); //next byte is len of text
					_tracks[trackIndex].addressOffset++;
					break;
				case 0x20:
					i = 2;
					break;
				case 0x2F: //End of track
					return 0x2F; 
					//_tracks[trackIndex].eventByte = 0x2F;
					//i = 1;
					//break;
				case 0x51:
					// skip the 0x03 control byte
					sendDummy();
					//read in the data bytes for the tempo
					data[0] = sendDummy();
					data[1] = sendDummy();
					data[2] = sendDummy();
					//This is all used to calculate the period value for the timer to correctly set the tempo
					_tempo = ((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + data[2];
					_tempo = ((uint64_t)F_CPU * _tempo) / 1000000;
					TCC0.PER = (_tempo/(uint32_t)_division) / 4; //Divided by 4 so we can fit larger values. Timer clock also div4
					printf("PER: %u\r\n", TCC0.PER);
					_tracks[trackIndex].addressOffset += 4;
					i = 0;
					break;
				case 0x54:
					i = 6;
					break;
				case 0x58:
				i = 5;
				break;
				case 0x59:
				i = 3;
				break;
				case 0xF7:
				//TODO: Finish looking at this length
				while(1)
				{
					_tracks[trackIndex].addressOffset++;
					if (sendDummy() == 0xF7)
					{
						break;
					}
				}
				i = 0;
				break;
			}
				
			_tracks[trackIndex].addressOffset += i;
			//skip some bytes
			while(i--)
			{
				sendDummy();
			}
				
			//printf("Offsett %lu, len, %lu\r\n", _tracks[cnt].addressOffset, _tracks[cnt].trackLength);
		}
			
	}
	
	return 0x00;
}


uint8_t getFirstEvent(uint8_t *trackIndex, uint32_t trackStart)
{
	waitForNotBusy();
	
	CS_DISABLE();
	CS_ENABLE();
	sendSPI(MEM_READ);
	memSendAddress(trackStart);

	
	printf("Next track start 0x%08lx\r\n", trackStart);
	
	if (getNextEvent(*trackIndex) == 0xFF) //Checking 0xFF explicitly because there might be a 0x2F returned.
	{
		(*trackIndex)++;
	}
	
	CS_DISABLE();
	
	return 0x00;
}


//Gets up to the max number of MIDI tracks. Scans each track for notes. Ignores if no notes found
//Uses _trackCount 
//Sets _trackCount, _tracks
uint8_t getMidiTracks()
{
	uint8_t i = 0;
	uint8_t cnt = 0;
	uint8_t usedTracks = 0;
	uint32_t trackStart = (uint32_t)MIDI_FIRST_TRACK_OFFSET + _fileAddressOffset + 4;// + 4 to skip the track header	
	uint8_t data[4];
	uint8_t done = 0;
	uint32_t trackLen = 0;
	
	
	 while (cnt < _tractCount && usedTracks < MIDI_MAX_MIDI_TRACKS) 
	{
		//Clear the track data
		_tracks[cnt].eventByte = 0;
		_tracks[usedTracks].eventData1 = 0;
		_tracks[usedTracks].eventData2 = 0;
		_tracks[usedTracks].addressOffset = 0;
		_tracks[usedTracks].startAddress = 0;

		printf("trackStart: 0x%08lx \r\n", trackStart);
		//Get the length of the track
		memReadToBuffer(trackStart, data, 4); 
		//printf("trackStart: 0x%08lx \r\n", trackStart);

		//_tracks[usedTracks].trackLength = ((uint32_t)data[0] << 24) + ((uint32_t)data[1] << 16) + ((uint32_t)data[2] << 8) + ((uint32_t)data[3] << 0);
		trackLen = ((uint32_t)data[0] << 24) + ((uint32_t)data[1] << 16) + ((uint32_t)data[2] << 8) + ((uint32_t)data[3] << 0);
		printf("len: %02x %02x %02x %02x (%lu)\r\n", data[0], data[1], data[2], data[3], trackLen);
		
		trackStart += 4; //move the trackStart to after the length
		//set the start of the track
		_tracks[usedTracks].startAddress = trackStart;
		
		//scan the track for a starting note. If no note-on found, skip the track
		//Doing a continuous read for better performance
		
		//TODO: Later we will need a way to scan to the next note. See if the following code can be refactored into it's own method
	
		//trackLen = _tracks[usedTracks].trackLength + 4;// + 4 to skip the track header
	 
		getFirstEvent(&usedTracks, trackStart);

		//Update the trackStart here because we will need it later.
		trackStart += trackLen + 4;
				
		cnt++;
		
	}
	
	return usedTracks;
}

void initMemory()
{

	getFileCount();
	_fileAddressOffset = FILE_COUNT_ADDR + 2 + (_fileCount * 4);
	
	printf("File Count: %u, Starting Address 0x%08lx\r\n", _fileCount, _fileAddressOffset);
	

}

uint8_t loadFile(uint16_t index)
{
	_fileIndex = index;
	moveToFile();
	getTrackCount();
	getTimeDivision();
	_tractCount = getMidiTracks();
	_activeTracks = _tractCount;
	printf("Used tracks %u\r\n", _tractCount);
}

//The following static inline methods are made inline so less stack space is used, better performance, and have no internal variables. They are broken out for readability. May have to remove inline if code space is needed

static inline void  startMidi()
{
	sendMidi(MIDI_START);
	sendMidi((_fileIndex >> 8));
	sendMidi(_fileIndex);
	TIMER_ENABLE();
}

static inline void  continueMidi()
{
	sendMidi(MIDI_CONTINUE);
	TIMER_ENABLE();
}

static inline void  stopMidi()
{
	sendMidi(MIDI_STOP);
	TIMER_DISABLE();
}

static inline void  moveToNext()
{
	_fileIndex++;
	if (_fileIndex >= _fileCount)
	{
		_fileIndex = 0;
	}
			
	stopMidi(); //to stop the current notes that are playing
	loadFile(_fileIndex);
	startMidi();	
}

static inline void  moveToPrevious()
{
	if (_fileIndex == 0 || _fileIndex >= _fileCount)
	{
		_fileIndex = _fileCount - 1;
	}
	else
	{
		_fileIndex--;
	}
			
	stopMidi(); //to stop the current notes that are playing
	loadFile(_fileIndex);
	startMidi();	
}

//Three play modes: 0 = continuous play, 1 = repeat, 2 = stop after finished with current
//Updates bits 3:2 of the status
static inline void  updateMidiPlayMode()
{
	if (((_statusFlags >> 2) & INPUT_PLAY_MODE_SHIFTED_MAKS) == 2)
	{
		_statusFlags &= 0xF3; //Set the play mode to 0
	}
	else if (((_statusFlags >> 2) & INPUT_PLAY_MODE_SHIFTED_MAKS) == 1)
	{
		//first reset to 0
		_statusFlags &= 0xF3; //Clear the play mode bits
		_statusFlags |= 0x08; //Set the play mode to 2
		
	}
	else if (((_statusFlags >> 2) & INPUT_PLAY_MODE_SHIFTED_MAKS) == 0)
	{
		_statusFlags &= 0xF3; //Clear the play mode bits
		_statusFlags |= 0x04; //Set the play mode to 1
	}
	
	sendMidi(MIDI_UPDATE_MODE);
	sendMidi(((_statusFlags >> 2) & 0x3));
}

static inline void  handleClockTick()
{
	
	//0: update periods
	//1: send events from each track who's period has expired, if no periods have expired, return
	//2: get next event for each track
	//3: goto 1.
	uint8_t prevEventByte;
	uint8_t prevNote;
	uint8_t out = 0;

	for(uint16_t i = 0; i < _tractCount; i++)
	{

		//getNextEvent requires the memory to be in continuous read mode. This is an optimization to speed up reading the next event
		waitForNotBusy();	
		CS_DISABLE();
		CS_ENABLE();
		sendSPI(MEM_READ);
		memSendAddress(_tracks[i].startAddress + _tracks[i].addressOffset);
												
		//Get the next event and keep sending as long as the deltatime is 0										
		while(1)
		{	
			if (!_tracks[i].eventByte) //if there is no event, skip ahead
			{
				break;
			}
							
			if (!_tracks[i].deltaTime) //A DeltaTime has reached 0;
			{
					
				printf("Track %u send 0x%02x %u %u\r\n", i, _tracks[i].eventByte, _tracks[i].eventData1, _tracks[i].eventData2);
				prevEventByte = _tracks[i].eventByte;
				prevNote = _tracks[i].eventData1;
				sendMidi(_tracks[i].eventByte);
				sendMidi(_tracks[i].eventData1);
				sendMidi(_tracks[i].eventData2);
				
				out = getNextEvent(i);
				if (out == 0)
				{
					break;
				}		
				
				if (out == 0x2F)
				{
					printf("End of track %u\r\n", i);
					_activeTracks--;
					break;
				}
				//printf("prev %02x next %02x prev note %u next note %u next track len %lu\r\n",prevEventByte, _tracks[i].eventByte,  prevNote, _tracks[i].eventData1, _tracks[i].deltaTime);
				if ((prevEventByte & 0xF0) == MIDI_NOTE_ON_MASK && (_tracks[i].eventByte & 0xF0) == MIDI_NOTE_OFF_MASK && _tracks[i].deltaTime == 0 && prevNote == _tracks[i].eventData1) // Skipping a note off if it was 0 length since the last.
				{

					//printf("Note %u Off removed ", prevNote);
					//printf("Event was: Track %u send 0x%02x %u %u\r\n", i, _tracks[i].eventByte, _tracks[i].eventData1, _tracks[i].eventData2);
					out = getNextEvent(i);
					if (out == 0)
					{
						break;
					}
					
					if (out == 0x2F)
					{
						printf("End of track %u\r\n", i);
						_activeTracks--;
						break;
					}
				}
				
										
			}
			else
			{
				break;
			}
		}
			//printf("offset: %lu, lenght %lu\r\n", _tracks[i].addressOffset , _tracks[i].trackLength);
		CS_DISABLE();

		//Just to avoid overflowing deltatime (since it's a 32 bit number). This may actually be slower and it may not matter in the end
		if(_tracks[i].deltaTime)						
		{
			_tracks[i].deltaTime--;	
		}	
		
		//Here we are checking to see if any tracks are left playing and disabling the timer if there are none.
		//This also handles what to do next for play modes.
		if (!_activeTracks)
		{
			TIMER_DISABLE();
			//If mode 2, we just stop
			stopMidi();
			
			if (((_statusFlags >> 2) & INPUT_PLAY_MODE_SHIFTED_MAKS) == 1) //Loop mode
			{
				loadFile(_fileIndex);
				startMidi();						
			}
			else if (((_statusFlags >> 2) & INPUT_PLAY_MODE_SHIFTED_MAKS) == 0) //Continuous mode
			{
				moveToNext();
			}
		}
	}
	
}

static inline void checkButtons()
{
				
	//NOTE: Counts are guaranteed not to roll over
	if (_input0Counts >= 100) //approx 1000ms hold
	{
		if (!(_statusFlags & INPUT_PLAY_MODE_UPDATING_MASK)) //If already in updating mode, do nothing. Basically if the button is still being held, don't try to update play mode again.
		{
			printf("Update midi play mode\r\n");
			_statusFlags &= ~INPUT_BUTTON_0_MASK; //Clear the button 0 flag to ensure this does not allow the midi to become immediately unpaused
			_statusFlags |= INPUT_PLAY_MODE_UPDATING_MASK;
			updateMidiPlayMode();
		}
	}
	else if (_input0Counts > 5 && _input1Counts < 100) //approx 50ms hold
	{
		_statusFlags |= INPUT_BUTTON_0_MASK; //second bit indicate button 0 is pressed

	}
	else if (_input0Counts == 0)
	{
		if ((_statusFlags & INPUT_BUTTON_0_MASK) && !(_statusFlags & INPUT_PLAY_MODE_UPDATING_MASK)) //Only want to move forward or unpause if we didn't just release from a long button 0 hold to update play mode
		{
			if ((_statusFlags & INPUT_PAUSE_MASK)) //In pause mode and button was pressed and released quickly so unpause instead of moving forward
			{
				printf("Continue midi\r\n");
				_statusFlags &= ~INPUT_PAUSE_MASK; //clear midi flag
				continueMidi();
			}
			else
			{
				printf("Move Forward\r\n");
				moveToNext();
			}
		}
			
		//Always reset button down status
		_statusFlags &= ~INPUT_BUTTON_0_MASK; //Clear the button 0 flag
		//Always reset play mode update status
		_statusFlags &= ~INPUT_PLAY_MODE_UPDATING_MASK;
	}
		
		
		
		
	//NOTE: Counts are guaranteed not to roll over
	if (_input1Counts >= 100) //approx 1000ms hold
	{
		if (!(_statusFlags & INPUT_PAUSE_MASK))	//If already in pause mode, don't pause again
		{
			printf("Pause midi play\r\n");
			_statusFlags |= INPUT_PAUSE_MASK; //set pause status
			_statusFlags &= ~INPUT_BUTTON_1_MASK; //Clear the button 1 flag to ensure this does not allow the midi to become immediately unpaused
			stopMidi();
		}
	}
	else if (_input1Counts > 5 && _input1Counts < 100) //approx 50ms hold
	{
		_statusFlags |= INPUT_BUTTON_1_MASK; //second bit indicate button 1 is pressed

	}
	else if (_input1Counts == 0)
	{
		if ( (_statusFlags & INPUT_BUTTON_1_MASK))
		{
			if ((_statusFlags & INPUT_PAUSE_MASK)) //In pause mode and button was pressed and released quickly so unpause instead of moving forward
			{
				printf("Continue midi\r\n");
				_statusFlags &= ~INPUT_PAUSE_MASK; //clear midi flag
				continueMidi();
			}
			else
			{
				printf("Move Back\r\n");
				moveToPrevious();
			}
		}
		//Always reset button down status
		_statusFlags &= ~INPUT_BUTTON_1_MASK; //Clear the button 1 flag
	}	
}

int main(void)
{
	uint8_t data[2];
	uint8_t usedTracks;
	
	cli();
	
	initClk();	
	initSPI();
	initDebugUART();
	initMidiUART();
	initInterrupts();
	initMidiTimer();
	initInputTimer();
	initInputADC();
	
	sei();
	
	stdout = &mystdout;
	
	printf("started\r\n");
	
	initMemory();
	
	//TODO: Remove
	PORTA.DIRSET = PIN5_bm;
		
	_fileIndex = 0xFFFFFF; //so it rolls over to the correct file with the very first button click.
	
    while (1) 
    {
		if (_clockTickFlag)
		{
			handleClockTick();
			_clockTickFlag = 0x00;
		}			
		
		checkButtons();

		
	
    }
}

ISR(TCC0_OVF_vect)
{
	PORTA.OUTTGL = PIN5_bm;
	_clockTickFlag = 0xFF;
}

ISR(TCC1_OVF_vect)
{
	uint8_t res;
	ADCA.CH1.CTRL |= ADC_CH_START_bm;
	
	while ((ADCA.CH1.CTRL & ADC_CH_START_bm)) {}
	
	res = 	ADCA.CH1.RES;
	//printf("ADC: %u\r\n", ADCA.CH1.RES);
	
	
	
	if (res > 120 && res < 250)
	{
		
		if(_input0Counts < 255)
		{

			_input0Counts++;
						//printf("Button 0 held: %u\r\n", _input0Counts);
		}
		_input1Counts = 0;
	}
	else if (res >= 250)
	{
		
		if(_input1Counts < 255)
		{

		  _input1Counts++;
		  			//printf("Button 1 held %u\r\n", _input1Counts);
		}
		_input0Counts = 0;
	}
	else
	{
		_input0Counts = 0;
		_input1Counts = 0;		
	}
}