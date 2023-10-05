#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>

/* HW_ID1 Register (PAGE=0, ADDR=0x0E, Reset value=0x02, Read only)
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * |   D7   |   D6   |   D5   |   D4   |   D3   |   D2   |   D1   |   D0   |
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * |                                 HW_IDH                                |
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * HW_ID2 Register (PAGE=0, ADDR=0x0F, Reset value=0x29, Read only)
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * |   D7   |   D6   |   D5   |   D4   |   D3   |   D2   |   D1   |   D0   |
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * |                                 HW_IDL                                |
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * HW_IDH: high 8bits of IC's ID
 * HW_IDL: low  8bits of IC's ID
 */
#define BU21029_HWID_REG (0x0E << 3)
#define SUPPORTED_HWID    0x0229

/* CFR0 Register (PAGE=0, ADDR=0x00, Reset value=0x20)
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * |   D7   |   D6   |   D5   |   D4   |   D3   |   D2   |   D1   |   D0   |
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * |   0    |   0    |  CALIB |  INTRM |   0    |   0    |   0    |   0    |
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * CALIB: 0 = not to use calibration result (*)
 *        1 = use calibration result
 * INTRM: 0 = INT output depend on "pen down" (*)
 *        1 = INT output always "0"
 */
#define BU21029_CFR0_REG (0x00 << 3)
#define CFR0_VALUE        0x00

/* CFR1 Register (PAGE=0, ADDR=0x01, Reset value=0xA6)
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * |   D7   |   D6   |   D5   |   D4   |   D3   |   D2   |   D1   |   D0   |
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * |  MAV   |         AVE[2:0]         |   0    |         SMPL[2:0]        |
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * MAV:  0 = median average filter off
 *       1 = median average filter on (*)
 * AVE:  AVE+1 = number of average samples for MAV,
 *               if AVE>SMPL, then AVE=SMPL (=3)
 * SMPL: SMPL+1 = number of conversion samples for MAV (=7)
 */
#define BU21029_CFR1_REG (0x01 << 3)
#define CFR1_VALUE        0xA6

/* CFR2 Register (PAGE=0, ADDR=0x02, Reset value=0x04)
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * |   D7   |   D6   |   D5   |   D4   |   D3   |   D2   |   D1   |   D0   |
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * |          INTVL_TIME[3:0]          |          TIME_ST_ADC[3:0]         |
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * INTVL_TIME: waiting time between completion of conversion
 *             and start of next conversion, only usable in
 *             autoscan mode (=20.480ms)
 * TIME_ST_ADC: waiting time between application of voltage
 *              to panel and start of A/D conversion (=100us)
 */
#define BU21029_CFR2_REG (0x02 << 3)
#define CFR2_VALUE        0xC9

/* CFR3 Register (PAGE=0, ADDR=0x0B, Reset value=0x72)
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * |   D7   |   D6   |   D5   |   D4   |   D3   |   D2   |   D1   |   D0   |
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * |  RM8   | STRETCH|  PU90K |  DUAL  |           PIDAC_OFS[3:0]          |
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * RM8: 0 = coordinate resolution is 12bit (*)
 *      1 = coordinate resolution is 8bit
 * STRETCH: 0 = SCL_STRETCH function off
 *          1 = SCL_STRETCH function on (*)
 * PU90K: 0 = internal pull-up resistance for touch detection is ~50kohms (*)
 *        1 = internal pull-up resistance for touch detection is ~90kohms
 * DUAL: 0 = dual touch detection off (*)
 *       1 = dual touch detection on
 * PIDAC_OFS: dual touch detection circuit adjustment, it is not necessary
 *            to change this from initial value
 */
#define BU21029_CFR3_REG (0x0B << 3)
#define CFR3_VALUE        0x42

/* LDO Register (PAGE=0, ADDR=0x0C, Reset value=0x00)
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * |   D7   |   D6   |   D5   |   D4   |   D3   |   D2   |   D1   |   D0   |
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * |   0    |         PVDD[2:0]        |   0    |         AVDD[2:0]        |
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * PVDD: output voltage of panel output regulator (=2.000V)
 * AVDD: output voltage of analog circuit regulator (=2.000V)
 */
#define BU21029_LDO_REG  (0x0C << 3)
#define LDO_VALUE         0x77

/* Serial Interface Command Byte 1 (CID=1)
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * |   D7   |   D6   |   D5   |   D4   |   D3   |   D2   |   D1   |   D0   |
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * |   1    |                 CF                |  CMSK  |  PDM   |  STP   |
 * +--------+--------+--------+--------+--------+--------+--------+--------+
 * CF: conversion function, see table 3 in datasheet p6 (=0000, automatic scan)
 * CMSK: 0 = executes convert function (*)
 *       1 = reads the convert result
 * PDM: 0 = power down after convert function stops (*)
 *      1 = keep power on after convert function stops
 * STP: 1 = abort current conversion and power down, set to "0" automatically
 */
#define BU21029_AUTOSCAN  0x80

/* The timeout value needs to be larger than INTVL_TIME + tConv4 (sample and
 * conversion time), where tConv4 is calculated by formula:
 * tPON + tDLY1 + (tTIME_ST_ADC + (tADC * tSMPL) * 2 + tDLY2) * 3
 * see figure 8 in datasheet p15 for details of each field.
 */
#define PEN_UP_TIMEOUT msecs_to_jiffies(50)

#define STOP_DELAY_US  50L
#define START_DELAY_MS 2L
#define BUF_LEN        14L
#define SCALE_12BIT    (1 << 12)
#define MAX_12BIT      ((1 << 12) - 1)
#define DRIVER_NAME    "bu21029"


static const size_t g_bu21029_conver_tab[1024] = {
 46341,  63304,  75975,  86475,  95610, 103785, 111240, 118129, 124559, 130607, 136330, 141774, 146974, 151958, 156749, 161369,
 165831, 170152, 174343, 178414, 182374, 186232, 189995, 193669, 197260, 200772, 204211, 207580, 210884, 214126, 217309, 220436,
 223510, 226532, 229507, 232435, 235318, 238159, 240959, 243720, 246444, 249131, 251783, 254401, 256987, 259541, 262065, 264560,
 267026, 269464, 271876, 274263, 276624, 278960, 281273, 283563, 285831, 288076, 290301, 292505, 294689, 296853, 298998, 301124,
 303233, 305323, 307396, 309452, 311492, 313516, 315523, 317515, 319492, 321454, 323402, 325335, 327255, 329160, 331053, 332932,
 334798, 336652, 338493, 340322, 342140, 343945, 345739, 347522, 349293, 351054, 352804, 354543, 356272, 357991, 359700, 361399,
 363088, 364768, 366438, 368099, 369751, 371394, 373028, 374654, 376270, 377879, 379479, 381071, 382655, 384230, 385798, 387358, /* 7 */
 388911, 390456, 391994, 393524, 395047, 396563, 398071, 399573, 401068, 402557, 404038, 405513, 406981, 408443, 409899, 411348,
 412791, 414228, 415659, 417084, 418503, 419916, 421323, 422725, 424120, 425511, 426896, 428275, 429649, 431017, 432381, 433739, /* 9 */
 435091, 436439, 437782, 439119, 440452, 441780, 443103, 444421, 445734, 447043, 448347, 449646, 450941, 452231, 453517, 454798,
 456075, 457348, 458616, 459880, 461140, 462395, 463647, 464894, 466137, 467377, 468612, 469843, 471070, 472294, 473513, 474729,
 475941, 477149, 478353, 479554, 480751, 481945, 483134, 484321, 485503, 486683, 487858, 489031, 490199, 491365, 492527, 493686,
 494841, 495993, 497142, 498288, 499430, 500569, 501705, 502838, 503968, 505095, 506219, 507339, 508457, 509572, 510683, 511792,
 512898, 514001, 515101, 516198, 517292, 518383, 519472, 520558, 521641, 522721, 523799, 524874, 525946, 527016, 528083, 529147,
 530209, 531268, 532325, 533379, 534430, 535479, 536525, 537569, 538611, 539650, 540686, 541721, 542752, 543782, 544809, 545833,
 546856, 547875, 548893, 549908, 550921, 551932, 552941, 553947, 554951, 555953, 556952, 557950, 558945, 559938, 560929, 561918,
 562905, 563889, 564872, 565852, 566830, 567807, 568781, 569753, 570723, 571691, 572658, 573622, 574584, 575544, 576502, 577459,
 578413, 579366, 580316, 581265, 582212, 583156, 584099, 585041, 585980, 586917, 587853, 588787, 589719, 590649, 591578, 592504,
 593429, 594352, 595274, 596193, 597111, 598028, 598942, 599855, 600766, 601675, 602583, 603489, 604394, 605296, 606197, 607097,
 607995, 608891, 609786, 610679, 611570, 612460, 613348, 614235, 615120, 616004, 616886, 617766, 618645, 619523, 620399, 621273,
 622146, 623017, 623887, 624756, 625623, 626488, 627352, 628215, 629076, 629936, 630794, 631651, 632506, 633360, 634213, 635064,
 635914, 636762, 637610, 638455, 639300, 640143, 640984, 641824, 642663, 643501, 644337, 645172, 646006, 646838, 647669, 648499,
 649327, 650154, 650980, 651805, 652628, 653450, 654271, 655090, 655908, 656725, 657541, 658356, 659169, 659981, 660792, 661601,
 662410, 663217, 664023, 664828, 665632, 666434, 667235, 668035, 668834, 669632, 670429, 671224, 672018, 672812, 673604, 674394,
 675184, 675973, 676760, 677547, 678332, 679116, 679899, 680681, 681462, 682242, 683020, 683798, 684574, 685350, 686124, 686897,
 687670, 688441, 689211, 689980, 690748, 691515, 692281, 693046, 693810, 694573, 695334, 696095, 696855, 697614, 698372, 699128,
 699884, 700639, 701393, 702145, 702897, 703648, 704398, 705147, 705895, 706642, 707388, 708133, 708877, 709620, 710362, 711103,
 711843, 712583, 713321, 714059, 714795, 715531, 716265, 716999, 717732, 718464, 719195, 719925, 720654, 721383, 722110, 722836,
 723562, 724287, 725011, 725734, 726456, 727177, 727897, 728617, 729335, 730053, 730770, 731486, 732201, 732915, 733629, 734341,
 735053, 735764, 736474, 737183, 737892, 738599, 739306, 740012, 740717, 741422, 742125, 742828, 743529, 744231, 744931, 745630,
 746329, 747027, 747724, 748420, 749115, 749810, 750504, 751197, 751889, 752581, 753272, 753962, 754651, 755339, 756027, 756714,
 757400, 758085, 758770, 759454, 760137, 760819, 761501, 762182, 762862, 763542, 764220, 764898, 765575, 766252, 766928, 767603,
 768277, 768950, 769623, 770295, 770967, 771637, 772307, 772977, 773645, 774313, 774980, 775647, 776313, 776978, 777642, 778306,
 778969, 779631, 780293, 780953, 781614, 782273, 782932, 783590, 784248, 784905, 785561, 786216, 786871, 787525, 788179, 788832,
 789484, 790136, 790786, 791437, 792086, 792735, 793383, 794031, 794678, 795324, 795970, 796615, 797260, 797903, 798547, 799189,
 799831, 800472, 801113, 801753, 802392, 803031, 803669, 804307, 804944, 805580, 806216, 806851, 807485, 808119, 808752, 809385,
 810017, 810648, 811279, 811909, 812539, 813168, 813796, 814424, 815052, 815678, 816304, 816930, 817555, 818179, 818803, 819426,
 820049, 820671, 821292, 821913, 822533, 823153, 823772, 824391, 825009, 825626, 826243, 826859, 827475, 828090, 828705, 829319,
 829932, 830545, 831158, 831770, 832381, 832992, 833602, 834212, 834821, 835429, 836037, 836645, 837252, 837858, 838464, 839070,
 839674, 840279, 840883, 841486, 842088, 842691, 843292, 843893, 844494, 845094, 845694, 846293, 846891, 847489, 848087, 848684,
 849280, 849876, 850472, 851067, 851661, 852255, 852848, 853441, 854034, 854626, 855217, 855808, 856398, 856988, 857578, 858167,
 858755, 859343, 859931, 860517, 861104, 861690, 862275, 862860, 863445, 864029, 864613, 865196, 865778, 866360, 866942, 867523,
 868104, 868684, 869264, 869843, 870422, 871000, 871578, 872156, 872732, 873309, 873885, 874460, 875036, 875610, 876184, 876758,
 877331, 877904, 878476, 879048, 879620, 880191, 880761, 881331, 881901, 882470, 883039, 883607, 884175, 884742, 885309, 885875,
 886441, 887007, 887572, 888137, 888701, 889265, 889828, 890391, 890954, 891516, 892078, 892639, 893200, 893760, 894320, 894879,
 895439, 895997, 896555, 897113, 897671, 898228, 898784, 899340, 899896, 900451, 901006, 901561, 902115, 902668, 903221, 903774,
 904327, 904879, 905430, 905981, 906532, 907082, 907632, 908182, 908731, 909280, 909828, 910376, 910923, 911470, 912017, 912563,
 913109, 913655, 914200, 914744, 915289, 915833, 916376, 916919, 917462, 918004, 918546, 919088, 919629, 920170, 920710, 921250,
 921790, 922329, 922868, 923406, 923944, 924482, 925019, 925556, 926093, 926629, 927164, 927700, 928235, 928770, 929304, 929838,
 930371, 930904, 931437, 931970, 932502, 933033, 933565, 934095, 934626, 935156, 935686, 936215, 936745, 937273, 937802, 938330,
 938857, 939385, 939911, 940438, 940964, 941490, 942015, 942541, 943065, 943590, 944114, 944637, 945161, 945684, 946206, 946729,
 947251, 947772, 948293, 948814, 949335, 949855, 950375, 950894, 951413, 951932, 952450, 952969, 953486, 954004, 954521, 955038,
 955554, 956070, 956586, 957101, 957616, 958131, 958645, 959159, 959673, 960186, 960699, 961212, 961724, 962236, 962748, 963259,
 963770, 964281, 964791, 965301, 965811, 966320, 966829, 967338, 967846, 968354, 968862, 969369, 969876, 970383, 970889, 971396,
 971901, 972407, 972912, 973417, 973921, 974425, 974929, 975433, 975936, 976439, 976941, 977444, 977946, 978447, 978949, 979450,
 979950, 980451, 980951, 981451, 981950, 982449, 982948, 983447, 983945, 984443, 984940, 985438, 985935, 986431, 986928, 987424,
 987919, 988415, 988910, 989405, 989899, 990394, 990888, 991381, 991875, 992368, 992860, 993353, 993845, 994337, 994828, 995320,
 995811, 996301, 996792, 997282, 997772, 998261, 998750, 999239, 999728,1000216,1000704,1001192,1001679,1002166,1002653,1003140,
1003626,1004112,1004598,1005083,1005569,1006053,1006538,1007022,1007506,1007990,1008473,1008957,1009439,1009922,1010404,1010886,
1011368,1011850,1012331,1012812,1013292,1013773,1014253,1014732,1015212,1015691,1016170,1016649,1017127,1017605,1018083,1018561,
1019038,1019515,1019992,1020468,1020945,1021421,1021896,1022372,1022847,1023322,1023796,1024271,1024745,1025219,1025692,1026165,
1026638,1027111,1027584,1028056,1028528,1028999,1029471,1029942,1030413,1030883,1031354,1031824,1032294,1032763,1033233,1033702,
1034170,1034639,1035107,1035575,1036043,1036510,1036978,1037445,1037911,1038378,1038844,1039310,1039776,1040241,1040706,1041171,
1041636,1042100,1042565,1043028,1043492,1043956,1044419,1044882,1045344,1045807,1046269,1046731,1047192,1047654,1048115,1048576
};

struct bu21029_ts_data {
	struct i2c_client *client;
	struct input_dev  *in_dev;
	struct timer_list  timer;
	u32                reset_gpios;
	u32                reset_gpios_assert;
	u32                x_plate_ohms;
	u32                max_pressure;
};

struct output_data_format
{
    u8 x_h;
    u8 x_l;
    u8 y_h;
    u8 y_l;
    u8 z1_h;
    u8 z1_l;
    u8 z2_h;
    u8 z2_l;

    u8 px_h;
    u8 px_l;
    u8 py_h;
    u8 py_l;
    u8 gh_h;
    u8 gh_l;
};

struct output_data_combined
{
    int16_t x;
    int16_t y;
    int16_t z1;
    int16_t z2;
    int16_t px;
    int16_t py;
    int16_t gh;
};


static int __mkcmd_reg(int page, int reg_addr, int swrst, int stp)
{
    int cmd = (0<<7) /* CID=0: Read/Write data register  */
            |((reg_addr&0x0f) << 3)
            |((page&0x01)<<2)
            |((0x01&swrst)<<1)
            |((0x01&stp)<<0);
    return cmd;
}

static int __mkcmd_fun(int cf, int cmsk, int pdm, int stp)
{
    int cmd = (1<<7) /* CID=0: Read/Write data register  */
            |((cf&0x0f) << 3)
            |((cmsk&0x01)<<2)
            |((0x01&pdm)<<1)
            |((0x01&stp)<<0);
    return cmd;
}

static void combine_output_data(struct output_data_format *in,
                                  struct output_data_combined *out)
{
    out->x = (((int16_t)in->x_h << 8) | (in->x_l))>>4;
    out->y = (((int16_t)in->y_h << 8) | (in->y_l))>>4;
    out->z1 = (((int16_t)in->z1_h << 8) | (in->z1_l))>>4;
    out->z2 = (((int16_t)in->z2_h << 8) | (in->z2_l))>>4;

    if ((in->px_l & 0x01) == 0x01)
    {
        out->px = ((int16_t)in->px_h << 2) + ((in->px_l & 0xC0) >> 6) - 1024;
    }
    else
    {
        out->px = ((int16_t)in->px_h << 2) + ((in->px_l & 0xC0) >> 6);
    }

    if ((in->py_l & 0x01) == 0x01)
    {
        out->py = ((int16_t)in->py_h << 2) + ((in->py_l & 0xC0) >> 6) - 1024;
    }
    else
    {
        out->py = ((int16_t)in->py_h << 2) + ((in->py_l & 0xC0) >> 6);
    }

    if ((in->gh_l & 0x01) == 0x01)
    {
        out->gh = ((in->gh_h << 4) + ((in->gh_l & 0xF0)>>4))- 4096;
    }
    else
    {
        out->gh = ((in->gh_h << 4) + ((in->gh_l & 0xF0)>>4));
    }

}


#define  X_MINVAL          10
#define  X_MAXVAL           4000

#define  Y_MINVAL           10
#define  Y_MAXVAL          4000

/** \brief 触摸屏压力阀值 有 10欧的外接电阻 */
#define  TOUCH_Z1_THRESHOLD       90    

/** \brief 触摸屏压力阀值 有 10欧的外接电阻 */     
#define  TOUCH_Z2_THRESHOLD       40         
static uint8_t get_touch_state (struct output_data_combined *data)
{
       if(1)//(((4095 - data->z2) > TOUCH_Z2_THRESHOLD) && (data->z1 > TOUCH_Z1_THRESHOLD))
    {
        /*  采样数据超过采集区间所有数据无效*/
        if ((data->x < X_MINVAL) || (data->x > X_MAXVAL) ||
            (data->y < Y_MINVAL) || (data->y > Y_MAXVAL))
        {
            return 0;
        }

        if ((data->px > 70) || (data->py > 70))
        {
            return 0;
        }

        /*  多点触摸及方向判断 */
        if ((data->px > 2) || (data->py > 2))
        {
            /*aw_kprintf("px=%d py=%d\n", px, py);*/
            if (data->gh >= 0)
            {
                return 0x3;                    /* detect two points*/
            } else {
                return 0x2;                    /* detect two points and judge y*/
            }
        }
        else
        {
            return 0x1;                        /* detect one point*/
        }
    }
    else
    {
        return 0;
    }
}


#define X_LIMIT  281273
#define Y_LIMIT  288076
static void cal_touch_position (struct output_data_combined *pos,
                                  uint8_t   touch_state,
                                  u16     *p_x_sample,
                                  u16     *p_y_sample)
{
	
//    if (touch_state > 1) {
	 // printk("px = %d,py = %d\n",pos->px,pos->py);
		int16_t dx = (pos->px > 0) ? (int16_t)(2048 * g_bu21029_conver_tab[pos->px] / X_LIMIT) : 0;   
        int16_t dy = (pos->py > 0) ? (int16_t)(2048 * g_bu21029_conver_tab[pos->py] / Y_LIMIT) : 0; 
  //      printk("%d\n",touch_state);
		if (touch_state > 1) {
            if (0x2 == touch_state) {
                p_x_sample[0] = pos->x + dx;
                p_y_sample[0] = pos->y - dy;
                p_x_sample[1] = pos->x - dx;
                p_y_sample[1] = pos->y + dy;

            }else{
                p_x_sample[0] = pos->x + dx;
                p_y_sample[0] = pos->y + dy;
                p_x_sample[1] = pos->x - dx;
                p_y_sample[1] = pos->y - dy;
            }

    } else {
        p_x_sample[0] = pos->x;
        p_y_sample[0] = pos->y;
    }
//	printk("state is %d\n",touch_state);
//	printk("x = %d,y = %d\n",pos->x,pos->y);
//	printk("dx = %d,dy = %d\n",dx,dy);
}



static int bu21029_touch_report(struct bu21029_ts_data *bu21029)
{
	struct i2c_client *i2c = bu21029->client;
	struct output_data_format out;
	u16 x, y, z1, z2;
	u32 rz;
	int32_t res;
	uint8_t   touch_state,dat1 = 0;
	struct output_data_combined out_combine;
	u16 p_x_sample[2];
	u16 p_y_sample[2];
/*
    i2c_smbus_read_i2c_block_data(i2c,
						  __mkcmd_reg(0, 0x0d, 0, 1),
						  1,&dat1);
   // printk("0x%x\n",dat1);
    if((dat1 & 0x01) != 1){
          printk("0x%x\n",dat1);
          return 0;
    } */ 
//    i2c_smbus_write_byte(i2c,__mkcmd_fun(1, 0, 1, 1));
//    i2c_smbus_write_byte(i2c,__mkcmd_fun(0, 0, 1, 0)); 
	/* read touch data and deassert INT (by restarting the autoscan mode) */
	int error = i2c_smbus_read_i2c_block_data(i2c,
						//  __mkcmd_fun(0, 1, 1, 0),
						BU21029_AUTOSCAN,
						  sizeof(out),
						  (u8 *)&out);
	if (error < 0)
		return error;

	
	x = ((uint16_t)out.x_h << 8) | (out.x_l);
	y = ((uint16_t)out.y_h << 8) | (out.y_l);
	z1 = (((uint16_t)out.z1_h << 8) | (out.z1_l))>>4;
	z2 = (((uint16_t)out.z2_h << 8) | (out.z2_l))>>4;
	x >>= 4;
	y >>= 4;
	
	if (x == 0 && y == 0 && ((z1 == 0) || (z2 == 0))){
		return 0;
	}

	if ((z1 == 0) || (z2 == 0)) {
		return 0;
    }
	
	rz  = z2 - z1;
	rz *= x;
	rz *= bu21029->x_plate_ohms;
	rz /= z1;
	rz  = DIV_ROUND_CLOSEST(rz, SCALE_12BIT);

    if (rz > bu21029->max_pressure)
        return 0;	

	combine_output_data(&out, &out_combine);
	touch_state = get_touch_state(&out_combine);
    if (touch_state == 0) {
         return 0;
    }
	
	cal_touch_position(&out_combine, touch_state, p_x_sample, p_y_sample);
	
    input_mt_slot(bu21029->in_dev, 0);
    input_mt_report_slot_state(bu21029->in_dev, MT_TOOL_FINGER, true);
    input_report_abs(bu21029->in_dev,ABS_MT_POSITION_X, p_x_sample[0]);
    input_report_abs(bu21029->in_dev,ABS_MT_POSITION_Y, p_y_sample[0]);
	input_report_abs(bu21029->in_dev,ABS_MT_PRESSURE, rz);
    	
    if(touch_state > 1)
	{
        input_mt_slot(bu21029->in_dev, 1);
        input_mt_report_slot_state(bu21029->in_dev, MT_TOOL_FINGER, true);
		input_report_abs(bu21029->in_dev,ABS_MT_POSITION_X, p_x_sample[1]);
		input_report_abs(bu21029->in_dev,ABS_MT_POSITION_Y, p_y_sample[1]);
		input_report_abs(bu21029->in_dev,ABS_MT_PRESSURE, rz);
	}else{
        input_mt_slot(bu21029->in_dev, 1);
	    input_mt_report_slot_state(bu21029->in_dev, MT_TOOL_FINGER, false); 
    }

    input_mt_sync_frame(bu21029->in_dev);
	input_sync(bu21029->in_dev);

/*
	if (rz <= bu21029->max_pressure) {
		input_report_abs(bu21029->in_dev, ABS_X, x);
		input_report_abs(bu21029->in_dev, ABS_Y, y);
		input_report_abs(bu21029->in_dev, ABS_PRESSURE, rz);
		input_report_key(bu21029->in_dev, BTN_TOUCH, 1);
		input_sync(bu21029->in_dev);
	}
*/
	

	return 0;
}

static void bu21029_touch_release(unsigned long handle)
{
	struct bu21029_ts_data *bu21029 = (void *)handle;

	input_mt_slot(bu21029->in_dev, 0);	
	input_mt_report_slot_state(bu21029->in_dev, MT_TOOL_FINGER, false); 

	input_mt_slot(bu21029->in_dev, 1);
	input_mt_report_slot_state(bu21029->in_dev, MT_TOOL_FINGER, false); 

	input_mt_sync(bu21029->in_dev);
	input_sync(bu21029->in_dev);
}

static irqreturn_t bu21029_touch_soft_irq(int irq, void *data)
{
	struct bu21029_ts_data *bu21029 = data;
	struct i2c_client *i2c = bu21029->client;
  
	/* report touch and deassert interrupt (will assert again after
	 * INTVL_TIME + tConv4 for continuous touch)
	 */
	int error = bu21029_touch_report(bu21029);

	if (error) {
		dev_err(&i2c->dev, "failed to report (error: %d)\n", error);
		return IRQ_NONE;
	}

	/* reset timer for pen up detection */
	mod_timer(&bu21029->timer, jiffies + PEN_UP_TIMEOUT);

	return IRQ_HANDLED;
}

static void bu21029_reset_chip(struct bu21029_ts_data *bu21029)
{
	gpio_set_value(bu21029->reset_gpios,
		       bu21029->reset_gpios_assert);
	udelay(STOP_DELAY_US);
	gpio_set_value(bu21029->reset_gpios,
		       !bu21029->reset_gpios_assert);
	mdelay(START_DELAY_MS);
}

static int bu21029_init_chip(struct bu21029_ts_data *bu21029)
{
	struct i2c_client *i2c = bu21029->client;
	struct {
		u8 reg;
		u8 value;
	} init_table[13];
	int error, i;
	u16 hwid;

	bu21029_reset_chip(bu21029);

	error = i2c_smbus_read_i2c_block_data(i2c,
					      BU21029_HWID_REG,
					      2,
					      (u8 *)&hwid);
	if (error < 0) {
		dev_err(&i2c->dev, "failed to read HW ID\n");
		return error;
	}
	
	
	if (cpu_to_be16(hwid) != SUPPORTED_HWID) {
		dev_err(&i2c->dev, "unsupported HW ID 0x%x\n", hwid);
		return -ENODEV;
	}

	init_table[0].reg = __mkcmd_reg(0, 0, 0, 0);
	init_table[0].value = 0x20;
	init_table[1].reg = __mkcmd_reg(0, 0x1, 0, 0);
	init_table[1].value = 0xa6;
	init_table[2].reg = __mkcmd_reg(0, 0x2, 0, 0);
	init_table[2].value = 0xc9;
	init_table[3].reg = __mkcmd_reg(0, 0x3, 0, 0);
	init_table[3].value = 0x1f;
	init_table[4].reg = __mkcmd_reg(0, 0x4, 0, 0);
	init_table[4].value = 0x1f;
	init_table[5].reg = __mkcmd_reg(0, 0x5, 0, 0);
	init_table[5].value = 0x10;
	init_table[6].reg = __mkcmd_reg(0, 0x6, 0, 0);
	init_table[6].value = 0x0;
	init_table[7].reg = __mkcmd_reg(0, 0x7, 0, 0);
	init_table[7].value = 0x0;
	init_table[8].reg = __mkcmd_reg(0, 0x8, 0, 0);
	init_table[8].value = 0x0;
	init_table[9].reg = __mkcmd_reg(0, 0x9, 0, 0);
	init_table[9].value = 0x0f;
	init_table[10].reg = __mkcmd_reg(0, 0xa, 0, 0);
	init_table[10].value = 0x0f;
	init_table[11].reg = __mkcmd_reg(0, 0xb, 0, 0);
	init_table[11].value = 0x72;
	init_table[12].reg = __mkcmd_reg(0, 0xc, 0, 0);
	init_table[12].value = 0x0;

	for (i = 0; i < ARRAY_SIZE(init_table); ++i) {
		error = i2c_smbus_write_byte_data(i2c,
						  init_table[i].reg,
						  init_table[i].value);
		if (error < 0) {
			dev_err(&i2c->dev,
				"failed to write 0x%x to register 0x%x\n",
				init_table[i].value,
				init_table[i].reg);
			return error;
		}
	}
	//calibration
	error = i2c_smbus_write_byte(i2c,__mkcmd_fun(5, 0, 0, 0));
	if (error < 0) {
		dev_err(&i2c->dev, "failed to start autoscan\n");
		return error;
	}
	mdelay(10);
	error = i2c_smbus_write_byte(i2c, BU21029_AUTOSCAN);
	if (error < 0) {
		dev_err(&i2c->dev, "failed to start autoscan\n");
		return error;
	}

	return 0;
}

static int bu21029_parse_dt(struct bu21029_ts_data *bu21029)
{
	struct device *dev = &bu21029->client->dev;
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	u32 val32;
	int gpio;

	if (!np) {
		dev_err(dev, "no device tree data\n");
		return -EINVAL;
	}

	gpio = of_get_named_gpio_flags(np, "reset-gpios", 0, &flags);
	if (!gpio_is_valid(gpio)) {
		dev_err(dev, "invalid 'reset-gpios' supplied\n");
		return -EINVAL;
	}
	bu21029->reset_gpios = gpio;
	bu21029->reset_gpios_assert = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;

	if (of_property_read_u32(np, "rohm,x-plate-ohms", &val32)) {
		dev_err(dev, "invalid 'x-plate-ohms' supplied\n");
		return -EINVAL;
	}
	bu21029->x_plate_ohms = val32;

	if (of_property_read_u32(np, "touchscreen-max-pressure", &val32))
		bu21029->max_pressure = MAX_12BIT;
	else
		bu21029->max_pressure = val32;

	return 0;
}

static int bu21029_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct bu21029_ts_data *bu21029;
	struct input_dev *in_dev;
	int error;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE |
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
		dev_err(&client->dev,
			"i2c functionality support is not sufficient\n");
		return -EIO;
	}

	bu21029 = devm_kzalloc(&client->dev, sizeof(*bu21029), GFP_KERNEL);
	if (!bu21029)
		return -ENOMEM;

	in_dev = devm_input_allocate_device(&client->dev);
	if (!in_dev) {
		dev_err(&client->dev, "unable to allocate input device\n");
		return -ENOMEM;
	}

	bu21029->client = client;
	bu21029->in_dev	= in_dev;
//	timer_setup(&bu21029->timer, bu21029_touch_release, 0);
//	setup_timer(&bu21029->timer, bu21029_touch_release,(unsigned long)bu21029);
	error = bu21029_parse_dt(bu21029);
	if (error)
		return error;

	error = devm_gpio_request_one(&client->dev,
				      bu21029->reset_gpios,
				      GPIOF_OUT_INIT_HIGH,
				      DRIVER_NAME);
	if (error) {
		dev_err(&client->dev, "unable to request reset-gpios\n");
		return error;
	}
	setup_timer(&bu21029->timer, bu21029_touch_release,(unsigned long)bu21029);
	error = bu21029_init_chip(bu21029);
	if (error) {
		dev_err(&client->dev, "unable to config bu21029\n");
		return error;
	}

	in_dev->name       = DRIVER_NAME;
	in_dev->id.bustype = BUS_I2C;
	in_dev->dev.parent = &client->dev;
/*
	__set_bit(EV_SYN,       in_dev->evbit);
	__set_bit(EV_KEY,       in_dev->evbit);
	__set_bit(EV_ABS,       in_dev->evbit);
	__set_bit(ABS_X,        in_dev->absbit);
	__set_bit(ABS_Y,        in_dev->absbit);
	__set_bit(ABS_PRESSURE, in_dev->absbit);
	__set_bit(BTN_TOUCH,    in_dev->keybit);
*/
//	input_set_abs_params(in_dev, ABS_X, 0, MAX_12BIT, 0, 0);
//	input_set_abs_params(in_dev, ABS_Y, 0, MAX_12BIT, 0, 0);

	 input_set_abs_params(in_dev, ABS_MT_POSITION_X, 0,MAX_12BIT, 0, 0); 
	 input_set_abs_params(in_dev, ABS_MT_POSITION_Y, 0,MAX_12BIT, 0, 0); 

	input_set_abs_params(in_dev, ABS_MT_PRESSURE,
			     0, bu21029->max_pressure, 0, 0);

    input_mt_init_slots(in_dev,2, 0);

	input_set_drvdata(in_dev, bu21029);

	error = input_register_device(in_dev);
	if (error) {
		dev_err(&client->dev, "unable to register input device\n");
		return error;
	}

	i2c_set_clientdata(client, bu21029);

	error = devm_request_threaded_irq(&client->dev,
					  client->irq,
					  NULL,
					  bu21029_touch_soft_irq,
					  IRQF_ONESHOT,
					  DRIVER_NAME,
					  bu21029);
	if (error) {
		dev_err(&client->dev, "unable to request touch irq\n");
		return error;
	}

	return 0;
}

static int bu21029_remove(struct i2c_client *client)
{
	struct bu21029_ts_data *bu21029 = i2c_get_clientdata(client);

	del_timer_sync(&bu21029->timer);

	return 0;
}

static const struct i2c_device_id bu21029_ids[] = {
	{DRIVER_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, bu21029_ids);

static struct i2c_driver bu21029_driver = {
	.driver = {
		.name  = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.id_table = bu21029_ids,
	.probe    = bu21029_probe,
	.remove   = bu21029_remove,
};
module_i2c_driver(bu21029_driver);

MODULE_AUTHOR("Zhu Yi <yi.zhu5@cn.bosch.com>");
MODULE_DESCRIPTION("Rohm BU21029 touchscreen controller driver");
MODULE_LICENSE("GPL v2");
