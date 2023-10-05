#ifndef __GPIO_H__
#define __GPIO_H__

#include <asm/arch-zmp110x/zmp_cpu.h>


/**@brief total number
 */
#define GPIO_NUMBER                     122

/**@brief invalidate gpio
 */
#define INVALID_GPIO                    0xfe

/** @name gpio output level define
 *  define gpio output level
 */
 /*@{*/
#define GPIO_LEVEL_LOW                  0
#define GPIO_LEVEL_HIGH                 1
/* @} */

/** @name gpio dir define
 *  define gpio dir
 */
 /*@{*/
#define GPIO_DIR_INPUT                  0
#define GPIO_DIR_OUTPUT                 1
/* @} */

/** @name gpio interrupt control define
 *  define gpio interrupt enable/disable
 */
 /*@{*/
#define GPIO_INTERRUPT_DISABLE          0
#define GPIO_INTERRUPT_ENABLE           1
/* @} */

/** @name gpio interrupt active level
 *  define gpio interrupt active level
 */
 /*@{*/
#define GPIO_INTERRUPT_LOW_ACTIVE       0
#define GPIO_INTERRUPT_HIGH_ACTIVE      1   
/* @} */

/** @name gpio interrupt type define
 *  define gpio interrupt type
 */
 /*@{*/
#define GPIO_LEVEL_INTERRUPT            0
#define GPIO_EDGE_INTERRUPT             1
/* @} */


/**************** gpio offsets ************************/
#define ZMP_GPIO_GROUP1		(32*0)
#define ZMP_GPIO_GROUP2		(32*1)
#define ZMP_GPIO_GROUP3		(32*2)
#define ZMP_GPIO_GROUP4		(32*3)

#define ZMP_GPIO_GROUP1_NO(offset)		( ZMP_GPIO_GROUP1 + (offset))
#define ZMP_GPIO_GROUP2_NO(offset)		( ZMP_GPIO_GROUP2 + (offset))
#define ZMP_GPIO_GROUP3_NO(offset)		( ZMP_GPIO_GROUP3 + (offset))
#define ZMP_GPIO_GROUP4_NO(offset)		( ZMP_GPIO_GROUP4 + (offset))
	
#define ZMP_GPIO_0			ZMP_GPIO_GROUP1_NO(0)
#define ZMP_GPIO_1			ZMP_GPIO_GROUP1_NO(1)
#define ZMP_GPIO_2			ZMP_GPIO_GROUP1_NO(2)
#define ZMP_GPIO_3			ZMP_GPIO_GROUP1_NO(3)
#define ZMP_GPIO_4			ZMP_GPIO_GROUP1_NO(4)
#define ZMP_GPIO_5			ZMP_GPIO_GROUP1_NO(5)
#define ZMP_GPIO_6			ZMP_GPIO_GROUP1_NO(6)
#define ZMP_GPIO_7			ZMP_GPIO_GROUP1_NO(7)
#define ZMP_GPIO_8			ZMP_GPIO_GROUP1_NO(8)
#define ZMP_GPIO_9			ZMP_GPIO_GROUP1_NO(9)
#define ZMP_GPIO_10			ZMP_GPIO_GROUP1_NO(10)
#define ZMP_GPIO_11			ZMP_GPIO_GROUP1_NO(11)
#define ZMP_GPIO_12			ZMP_GPIO_GROUP1_NO(12)
#define ZMP_GPIO_13			ZMP_GPIO_GROUP1_NO(13)
#define ZMP_GPIO_14			ZMP_GPIO_GROUP1_NO(14)
#define ZMP_GPIO_15			ZMP_GPIO_GROUP1_NO(15)
#define ZMP_GPIO_16			ZMP_GPIO_GROUP1_NO(16)
#define ZMP_GPIO_17			ZMP_GPIO_GROUP1_NO(17)
#define ZMP_GPIO_18			ZMP_GPIO_GROUP1_NO(18)
#define ZMP_GPIO_19			ZMP_GPIO_GROUP1_NO(19)
#define ZMP_GPIO_20			ZMP_GPIO_GROUP1_NO(20)
#define ZMP_GPIO_21			ZMP_GPIO_GROUP1_NO(21)
#define ZMP_GPIO_22			ZMP_GPIO_GROUP1_NO(22)
#define ZMP_GPIO_23			ZMP_GPIO_GROUP1_NO(23)
#define ZMP_GPIO_24			ZMP_GPIO_GROUP1_NO(24)
#define ZMP_GPIO_25			ZMP_GPIO_GROUP1_NO(25)
#define ZMP_GPIO_26			ZMP_GPIO_GROUP1_NO(26)
#define ZMP_GPIO_27			ZMP_GPIO_GROUP1_NO(27)
#define ZMP_GPIO_28			ZMP_GPIO_GROUP1_NO(28)
#define ZMP_GPIO_29			ZMP_GPIO_GROUP1_NO(29)
#define ZMP_GPIO_30			ZMP_GPIO_GROUP1_NO(30)
#define ZMP_GPIO_31			ZMP_GPIO_GROUP1_NO(31)

#define ZMP_GPIO_32			ZMP_GPIO_GROUP2_NO(0)
#define ZMP_GPIO_33			ZMP_GPIO_GROUP2_NO(1)
#define ZMP_GPIO_34			ZMP_GPIO_GROUP2_NO(2)
#define ZMP_GPIO_35			ZMP_GPIO_GROUP2_NO(3)
#define ZMP_GPIO_36			ZMP_GPIO_GROUP2_NO(4)
#define ZMP_GPIO_37			ZMP_GPIO_GROUP2_NO(5)
#define ZMP_GPIO_38			ZMP_GPIO_GROUP2_NO(6)
#define ZMP_GPIO_39			ZMP_GPIO_GROUP2_NO(7)
#define ZMP_GPIO_40			ZMP_GPIO_GROUP2_NO(8)
#define ZMP_GPIO_41			ZMP_GPIO_GROUP2_NO(9)
#define ZMP_GPIO_42			ZMP_GPIO_GROUP2_NO(10)
#define ZMP_GPIO_43			ZMP_GPIO_GROUP2_NO(11)
#define ZMP_GPIO_44			ZMP_GPIO_GROUP2_NO(12)
#define ZMP_GPIO_45			ZMP_GPIO_GROUP2_NO(13)
#define ZMP_GPIO_46			ZMP_GPIO_GROUP2_NO(14)
#define ZMP_GPIO_47			ZMP_GPIO_GROUP2_NO(15)
#define ZMP_GPIO_48			ZMP_GPIO_GROUP2_NO(16)
#define ZMP_GPIO_49			ZMP_GPIO_GROUP2_NO(17)
#define ZMP_GPIO_50			ZMP_GPIO_GROUP2_NO(18)
#define ZMP_GPIO_51			ZMP_GPIO_GROUP2_NO(19)
#define ZMP_GPIO_52			ZMP_GPIO_GROUP2_NO(20)
#define ZMP_GPIO_53			ZMP_GPIO_GROUP2_NO(21)
#define ZMP_GPIO_54			ZMP_GPIO_GROUP2_NO(22)
#define ZMP_GPIO_55			ZMP_GPIO_GROUP2_NO(23)
#define ZMP_GPIO_56			ZMP_GPIO_GROUP2_NO(24)
#define ZMP_GPIO_57			ZMP_GPIO_GROUP2_NO(25)
#define ZMP_GPIO_58			ZMP_GPIO_GROUP2_NO(26)
#define ZMP_GPIO_59			ZMP_GPIO_GROUP2_NO(27)
#define ZMP_GPIO_60			ZMP_GPIO_GROUP2_NO(28)
#define ZMP_GPIO_61			ZMP_GPIO_GROUP2_NO(29)
#define ZMP_GPIO_62			ZMP_GPIO_GROUP2_NO(30)
#define ZMP_GPIO_63			ZMP_GPIO_GROUP2_NO(31)

#define ZMP_GPIO_64			ZMP_GPIO_GROUP3_NO(0)
#define ZMP_GPIO_65			ZMP_GPIO_GROUP3_NO(1)
#define ZMP_GPIO_66			ZMP_GPIO_GROUP3_NO(2)
#define ZMP_GPIO_67			ZMP_GPIO_GROUP3_NO(3)
#define ZMP_GPIO_68			ZMP_GPIO_GROUP3_NO(4)
#define ZMP_GPIO_69			ZMP_GPIO_GROUP3_NO(5)
#define ZMP_GPIO_70			ZMP_GPIO_GROUP3_NO(6)
#define ZMP_GPIO_71			ZMP_GPIO_GROUP3_NO(7)
#define ZMP_GPIO_72			ZMP_GPIO_GROUP3_NO(8)
#define ZMP_GPIO_73			ZMP_GPIO_GROUP3_NO(9)
#define ZMP_GPIO_74			ZMP_GPIO_GROUP3_NO(10)
#define ZMP_GPIO_75			ZMP_GPIO_GROUP3_NO(11)
#define ZMP_GPIO_76			ZMP_GPIO_GROUP3_NO(12)
#define ZMP_GPIO_77			ZMP_GPIO_GROUP3_NO(13)
#define ZMP_GPIO_78			ZMP_GPIO_GROUP3_NO(14)
#define ZMP_GPIO_79			ZMP_GPIO_GROUP3_NO(15)
#define ZMP_GPIO_80			ZMP_GPIO_GROUP3_NO(16)
#define ZMP_GPIO_81			ZMP_GPIO_GROUP3_NO(17)
#define ZMP_GPIO_82			ZMP_GPIO_GROUP3_NO(18)
#define ZMP_GPIO_83			ZMP_GPIO_GROUP3_NO(19)
#define ZMP_GPIO_84			ZMP_GPIO_GROUP2_NO(20)
#define ZMP_GPIO_85			ZMP_GPIO_GROUP3_NO(21)
#define ZMP_GPIO_86			ZMP_GPIO_GROUP3_NO(22)
#define ZMP_GPIO_87			ZMP_GPIO_GROUP3_NO(23)
#define ZMP_GPIO_88			ZMP_GPIO_GROUP3_NO(24)
#define ZMP_GPIO_89			ZMP_GPIO_GROUP3_NO(25)
#define ZMP_GPIO_90			ZMP_GPIO_GROUP3_NO(26)
#define ZMP_GPIO_91			ZMP_GPIO_GROUP3_NO(27)
#define ZMP_GPIO_92			ZMP_GPIO_GROUP3_NO(28)
#define ZMP_GPIO_93			ZMP_GPIO_GROUP2_NO(29)
#define ZMP_GPIO_94			ZMP_GPIO_GROUP3_NO(30)
#define ZMP_GPIO_95			ZMP_GPIO_GROUP3_NO(31)

#define ZMP_GPIO_96			ZMP_GPIO_GROUP3_NO(0)
#define ZMP_GPIO_97			ZMP_GPIO_GROUP3_NO(1)
#define ZMP_GPIO_98			ZMP_GPIO_GROUP3_NO(2)
#define ZMP_GPIO_99			ZMP_GPIO_GROUP3_NO(3)
#define ZMP_GPIO_100			ZMP_GPIO_GROUP3_NO(4)
#define ZMP_GPIO_101			ZMP_GPIO_GROUP3_NO(5)
#define ZMP_GPIO_102			ZMP_GPIO_GROUP3_NO(6)
#define ZMP_GPIO_103			ZMP_GPIO_GROUP3_NO(7)
#define ZMP_GPIO_104			ZMP_GPIO_GROUP3_NO(8)
#define ZMP_GPIO_105			ZMP_GPIO_GROUP3_NO(9)
#define ZMP_GPIO_106			ZMP_GPIO_GROUP3_NO(10)
#define ZMP_GPIO_107			ZMP_GPIO_GROUP3_NO(11)
#define ZMP_GPIO_108			ZMP_GPIO_GROUP3_NO(12)
#define ZMP_GPIO_109			ZMP_GPIO_GROUP3_NO(13)
#define ZMP_GPIO_110			ZMP_GPIO_GROUP3_NO(14)
#define ZMP_GPIO_111			ZMP_GPIO_GROUP3_NO(15)
#define ZMP_GPIO_112			ZMP_GPIO_GROUP3_NO(16)
#define ZMP_GPIO_113			ZMP_GPIO_GROUP3_NO(17)
#define ZMP_GPIO_114			ZMP_GPIO_GROUP3_NO(18)
#define ZMP_GPIO_115			ZMP_GPIO_GROUP3_NO(19)
#define ZMP_GPIO_116			ZMP_GPIO_GROUP2_NO(20)
#define ZMP_GPIO_117			ZMP_GPIO_GROUP3_NO(21)
#define ZMP_GPIO_118			ZMP_GPIO_GROUP3_NO(22)
#define ZMP_GPIO_119			ZMP_GPIO_GROUP3_NO(23)
#define ZMP_GPIO_120			ZMP_GPIO_GROUP3_NO(24)
#define ZMP_GPIO_121			ZMP_GPIO_GROUP3_NO(25)
#define ZMP_GPIO_122			ZMP_GPIO_GROUP3_NO(26)


typedef enum
{
    PULLUP = 0,
    PULLDOWN,
    PULLUPDOWN,
    PUNDEFINED
}T_GPIO_TYPE;


/**
 * @brief share pins
 * 
 */
typedef enum
{
    ePIN_AS_MMCSD = 0,             ///< share pin as MDAT1, 8 lines
    ePIN_AS_I2S,                ///< share pin as I2S bit[24]:0
    ePIN_AS_PWM0,               ///< share pin as PWM0   
    ePIN_AS_PWM1,               ///< share pin as PWM1
    ePIN_AS_PWM2,               ///< share pin as PWM2
    ePIN_AS_PWM3,               ///< share pin as PWM3
    ePIN_AS_PWM4,               ///< share pin as PWM4
	
    ePIN_AS_SDIO,               ///< share pin as SDIO
    ePIN_AS_UART1,              ///< share pin as UART1
    ePIN_AS_UART2,              ///< share pin as UART2
    ePIN_AS_CAMERA,             ///< share pin as CAMERA
    ePIN_AS_SPI0,               ///< share pin as SPI1 bit[25]:0
    ePIN_AS_SPI1,               ///< share pin as SPI2  bit[26]:1
    ePIN_AS_JTAG,               ///< share pin as JTAG
    ePIN_AS_TWI,                ///< share pin as I2C
    ePIN_AS_MAC,                ///< share pin as Ethernet MAC
    ePIN_AS_OPCLK,

    ePIN_AS_DUMMY

}E_GPIO_PIN_SHARE_CONFIG;



typedef struct
{
    E_GPIO_PIN_SHARE_CONFIG func_module;
    unsigned long reg1_bit_mask;
    unsigned long reg1_bit_value;
    unsigned long reg2_bit_mask;
    unsigned long reg2_bit_value;
    unsigned long reg3_bit_mask;
    unsigned long reg3_bit_value;
    unsigned long reg4_bit_mask;
    unsigned long reg4_bit_value;
}
T_SHARE_CFG_FUNC_MODULE;









#endif

