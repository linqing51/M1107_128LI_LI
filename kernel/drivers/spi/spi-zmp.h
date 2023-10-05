#ifndef __SPI_H__
#define __SPI_H__


enum zmpspi_cs_num {	
	ZMPSPI_ONCHIP_CS = 0,	/*on chip control cs index*/
	/*ZMPSPI_CS1,*/
	/*ZMPSPI_CS2,*/
	ZMPSPI_CS_NUM, 		
};

enum zmpspi_bus_num {
	ZMPSPI_BUS_NUM1,
	ZMPSPI_BUS_NUM2,
	ZMPSPI_MAX_BUS_NUM,
};


#define ZMPSPI_1DATAWIRE 	(0b00<<16)
#define ZMPSPI_2DATAWIRE 	(0b01<<16)
#define ZMPSPI_4DATAWIRE 	(0b10<<16)

#define ZMPSPI_XFER_MODE_DMA 	(1)
#define ZMPSPI_XFER_MODE_CPU 	(2)


#define ZMP_SPICON		(0x00)
#define ZMP_SPICON_WIRE		(0x3<<16)
#define ZMP_SPICON_CLKDIV	(0x7F<<8)
#define ZMP_SPICON_EN	(1<<6)
#define ZMP_SPICON_CS	(1<<5)
#define ZMP_SPICON_MS	(1<<4)
#define ZMP_SPICON_CPHA	(1<<3)
#define ZMP_SPICON_CPOL	(1<<2)
#define ZMP_SPICON_ARRM	(1<<1)
#define ZMP_SPICON_TGDM	(1<<0)

#define ZMP_SPISTA		(0x04)
#define ZMP_SPISTA_TIMEOUT	(1<<10)
#define ZMP_SPISTA_MPROC	(1<<9)
#define ZMP_SPISTA_TRANSF	(1<<8)
#define ZMP_SPISTA_RXOVER	(1<<7)
#define ZMP_SPISTA_RXHFULL	(1<<6)
#define ZMP_SPISTA_RXFULL	(1<<5)
#define ZMP_SPISTA_RXEMP	(1<<4)
#define ZMP_SPISTA_TXUNDER	(1<<3)
#define ZMP_SPISTA_TXHEMP	(1<<2)
#define ZMP_SPISTA_TXFULL	(1<<1)
#define ZMP_SPISTA_TXEMP	(1<<0)

#define ZMP_SPIINT		(0x08)
#define ZMP_SPIINT_TIMEOUT	(1<<10)
#define ZMP_SPIINT_MPROC	(1<<9)
#define ZMP_SPIINT_TRANSF	(1<<8)
#define ZMP_SPIINT_RXOVER	(1<<7)
#define ZMP_SPIINT_RXHFULL	(1<<6)
#define ZMP_SPIINT_RXFULL	(1<<5)
#define ZMP_SPIINT_RXEMP	(1<<4)
#define ZMP_SPIINT_TXUNDER	(1<<3)
#define ZMP_SPIINT_TXHEMP	(1<<2)
#define ZMP_SPIINT_TXFULL	(1<<1)
#define ZMP_SPIINT_TXEMP	(1<<0)

#define ZMP_SPICNT		(0x0C)

#define ZMP_SPIEXTX		(0x10)
#define ZMP_SPIEXTX_BUFEN	(1<<0)
#define ZMP_SPIEXTX_DMAEN	(1<<16)


#define ZMP_SPIEXRX		(0x14)
#define ZMP_SPIEXRX_BUFEN	(1<<0)
#define ZMP_SPIEXRX_DMAEN	(1<<16)

#define ZMP_SPIOUT		(0x18)

#define ZMP_SPIIN		(0x1C)

#endif

