#ifndef _ADS1115CFG_H_
#define _ADS1115CFG_H_

typedef enum
{
ADDR_GND    =	0x48, // address pin low (GND)
ADDR_VDD    =	0x49, // address pin high (VCC)
ADDR_SDA    =	0x4A, // address pin tied to SDA pin
ADDR_SCL	=	0x4B // address pin tied to SCL pin
}ADS1115_ADDRESS

typedef enum
{
RA_CONVERSION	=	0x00,
RA_CONFIG	=	0x01,
RA_LO_THRESH	=	0x02,
RA_HI_THRESH	=	0x03
}ADS1115_RA

typedef enum
{
CFG_OS_BIT	=	_BV(15),
CFG_MUX_MASK	=	(_BV(14) | _BV(13) | _BV(12)),
CFG_MUX_SHIFT	=	12,
CFG_PGA_MASK	=	(_BV(11) | _BV(10) | _BV(9)),
CFG_PGA_SHIFT	=	9,
CFG_MODE_BIT	=	_BV(8),
CFG_DR_MASK	=	(_BV(7) | _BV(6) | _BV(5)),
CFG_DR_SHIFT	=	5,
CFG_COMP_MODE_BIT	=	_BV(4),
CFG_COMP_POL_BIT	=	_BV(3),
CFG_COMP_LAT_BIT	=	_BV(2),
CFG_COMP_QUE_MASK	=	(_BV(1) | _BV(0)),
CFG_COMP_QUE_SHIFT	=	0
}ADS1115_CFG

typedef enum
{
ADS1115_MUX_P0_N1	=	0x00, // default
MUX_P0_N3	=	0x01,
MUX_P1_N3	=	0x02,
MUX_P2_N3	=	0x03,
MUX_P0_NG	=	0x04,
MUX_P1_NG	=	0x05,
MUX_P2_NG	=	0x06,
MUX_P3_NG	=	0x07
}ADS1115_MUX

typedef enum
{
PGA_6P144	=	0x00,
PGA_4P096	=	0x01,
PGA_2P048	=	0x02, // default
PGA_1P024	=	0x03,
PGA_0P512	=	0x04,
PGA_0P256	=	0x05,
PGA_0P256B	=	0x06,
PGA_0P256C	=	0x07
}ADS1115_PGA

typedef enum
{
MV_6P144	=	0.187500,
MV_4P096	=	0.125000,
MV_2P048	=	0.062500, // default
MV_1P024	=	0.031250,
MV_0P512	=	0.015625,
MV_0P256	=	0.007813,
MV_0P256B	=	0.007813,
MV_0P256C	=	0.007813
}ADS1115_MV

typedef enum
{
FSR_6P144	=	6144,
FSR_4P096	=	4096,
FSR_2P048	=	2048,
FSR_1P024	=	1024,
FSR_0P512	=	512,
FSR_0P256	=	256
}ADS1115_FSR

typedef enum
{
CONTINUOUS	=	0x00,
SINGLESHOT	=	0x01 // default
}ADS1115_MODE

typedef enum
{
RATE_8	=	0x00,
RATE_16	=	0x01,
RATE_32	=	0x02,
RATE_64	=	0x03,
RATE_128	=	0x04, // default
RATE_250	=	0x05,
RATE_475	=	0x06,
RATE_860	=	0x07
}ADS1115_RATE

typedef enum
{
COMP_MODE_HYSTERESIS	=	0x00, // default
COMP_MODE_WINDOW	=	0x01,

COMP_POL_ACTIVE_LOW	=	0x00, // default
COMP_POL_ACTIVE_HIGH	=	0x01,

COMP_LAT_NON_LATCHING	=	0x00, // default
COMP_LAT_LATCHING	=	0x01,

COMP_QUE_ASSERT1	=	0x00,
COMP_QUE_ASSERT2	=	0x01,
COMP_QUE_ASSERT4	=	0x02,
COMP_QUE_DISABLE	=	0x03 // default
}ADS1115_COMP
#endif