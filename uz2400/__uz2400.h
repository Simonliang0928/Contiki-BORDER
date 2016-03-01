/*******************************************************************************

	UBEC (Uniband Electronic Corp.)
	

	Project : UZ2400 Simple RF API

	File : uz2400.h
	
	Usage :  Special header for the UBEC UZ2400 Zigbee RF.
	
	Platform: U-Zig Power Module with IAR C compiler
	
	Reference: 

		Silicon Laboratories: C8051F124
		UBEC: UZ2400
	
	Note : 	


	Uniband Electronic Corporation
	3F-3, No. 192 Dongguang Rd., Hsinchu, 300 Taiwan
	Tel : 886-3-5729898, Fax : 886-3-5718599
	http: //www.ubec.com.tw


		Copyright (C) 2005 Uniband Electronic Corporation, All rights reserved
					
********************************************************************************/

//Define the address
#define TXBASE  		0x0
#define TX_N 		0x0
#define TX_N_HDR    	0x0
#define TX_N_LEN    	0x1
#define TX_N_BASE   	0x2
#define TX_BCN		0x80
#define TX_G1		0x100
#define TX_G2		0x180

#define REGBASE     	0x200
#define RXBASE      	0x300
#define RX_N			0x300
#define KEYBASE     	0x280
#define KEY_TX_N	0x280
#define KEY_TX_B	0x2a0
#define KEY_TX_G1	0x290
#define KEY_TX_G2	0x2a0
#define KEY_RX_N	0x2b0

// short-address registers (SREGXX)
#define	RXMCR		0x00
#define	PANIDL		0x01
#define	PANIDH		0x02
#define	SADRL		0x03
#define	SADRH		0x04
#define	EADR0		0x05
#define	EADR1		0x06
#define	EADR2		0x07
#define	EADR3		0x08
#define	EADR4		0x09
#define	EADR5		0x0a
#define	EADR6		0x0b
#define	EADR7		0x0c
#define	RXFLUSH	0x0d
#define 	TXSTATE0    	0x0e
#define 	TXSTATE1    	0x0f
#define 	ORDER		0x10
#define 	TXMCR		0x11
#define 	ACKTMOUT 	0x12
#define 	SLALLOC		0x13
#define 	SYMTICKL	0x14
#define 	SYMTICKH	0x15
#define	PAONTIME	0x16 
#define 	PAONSETUP  0x17
#define 	FFOEN		0x18
#define 	CSMACR		0x19
#define	TXBCNTRIG	0x1a
#define	TXNMTRIG	0x1b
#define	TXG1TRIG	0x1c
#define	TXG2TRIG	0x1d
#define	ESLOTG23	0x1e
#define	ESLOTG45	0x1f
#define	ESLOTG67	0x20
#define	TXPEND		0x21
#define	TXBCNINTL	0x22
#define	FRMOFFSET	0x23
#define	TXSR		0x24
#define	TXLNERR 	0x25
#define 	DATE_CLK	0x26
#define	TXOFFSET	0x27
#define	HSYMTMR0	0x28
#define	HSYMTMR1	0x29
#define	SOFTRST	0x2a
#define   BISTCR      	0x2b
#define	SECCR0		0x2c
#define	SECCR1		0x2d
#define   TXPEMISP    	0x2e
#define	SECISR		0x2f
#define	RXSR		0x30
#define	ISRSTS		0x31
#define	INTMSK		0x32
#define   GPIO        	0x33
#define   GPIODIR     	0x34
#define   SLPACK      	0x35
#define   RFCTL       	0x36
#define   SECCR2      	0x37
#define	BBREG0		0x38
#define	BBREG1		0x39
#define	BBREG2		0x3a
#define	BBREG3		0x3b
#define	BBREG4		0x3c
#define	BBREG5		0x3d
#define   BBREG6      	0x3e
#define   RSSITHCCA   0x3f
//=======For New Version========//
#define GATECLK 0x26
#define FIFOEN 0x18
#define PACON1 0x17
#define SECCTRL 0x24D
#define RFCTRL50 0x250
#define RFCTRL51 0x251
#define RFCTRL52 0x252
#define RFCTRL59 0x259
#define RFCTRL73 0x273
#define RFCTRL74 0x274
#define RFCTRL75 0x275
#define RFCTRL76 0x276
#define RFCTRL 0x36





// long-address registers (LREGXX)
#define 	RFCTRL0     		0x200
#define 	RFCTRL1    		0x201
#define 	RFCTRL2     		0x202
#define	RFCTRL3     		0x203
#define 	RFCTRL4     		0x204
#define 	RFCTRL5			0x205
#define 	RFCTRL6     		0x206
#define 	RFCTRL7			0x207
#define 	RFCTRL8     		0x208
#define 	CAL1			0x209
#define 	CAL2			0x20a
#define 	CAL3			0x20b
#define   SFCNTRH     		0x20c
#define   SFCNTRM     		0x20d
#define   SFCNTRL     		0x20e
#define   RFSTATE     		0x20f
#define   RSSI        		0x210
#define   RSSITHGLNA  	0x209
#define   RSSIOFFSET1 	0x20a
#define   RSSIOFFSET2 	0x20b
#define 	CLKIRQCR		0x211
#define	SRCADRMODE 	0x212
#define	SRCADDR0		0x213
#define	SRCADDR1		0x214
#define	SRCADDR2		0x215
#define	SRCADDR3		0x216
#define	SRCADDR4		0x217
#define	SRCADDR5		0x218
#define	SRCADDR6		0x219
#define	SRCADDR7		0x21a
#define   RXFRAMESTATE 	0x21b
#define   SECSTATUS   	0x21c
#define   STCCMP      		0x21d
#define   HLEN        		0x21e
#define   FLEN        		0x21f
#define 	SCLKDIV  		0x220
#define 	WAKETIMEL		0x222
#define 	WAKETIMEH		0x223
#define	TXREMCNTL		0x224
#define	TXREMCNTH		0x225
#define	TXMAINCNTL 	0x226
#define	TXMAINCNTM 	0x227
#define	TXMAINCNTH0 	0x228
#define	TXMAINCNTH1 	0x229
#define	RFMANUALCTRLEN 0x22a
#define	RFMANUALCTRL 	0x22b
#define 	TxDACMANUALCTRL 0x22c
#define	RFMANUALCTRL2 0x22d
#define 	TESTRSSI		0x22e
#define	TESTMODE 		0x22f
#define	ASSO_BCN_LADR0 0x230
#define	ASSO_BCN_LADR1 0x231
#define	ASSO_BCN_LADR2 0x232
#define	ASSO_BCN_LADR3 0x233
#define	ASSO_BCN_LADR4 0x234
#define	ASSO_BCN_LADR5 0x235
#define	ASSO_BCN_LADR6 0x236
#define	ASSO_BCN_LADR7 0x237
#define	ASSO_BCN_SADR0 0x238
#define	ASSO_BCN_SADR1 0x239
#define	ASSO_BCN_CR	0x23a
#define	SEC_PAR_ORDER	0x23b
// 0x40 ~ 0x4c NWL, APL security nonce

//Define the GPIO Address
#define GPIOCS		0x92
#define GPIOCLK		0x93
#define GPIODIN		0x94
#define GPIODOUT	0x95
#define GPIODIO3	0x96
#define GPIODIO4	0x97



