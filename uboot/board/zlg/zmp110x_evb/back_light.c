#include <common.h>
#include <linux/compiler.h>
#include <asm/io.h>
#include <div64.h>
#define ZMP_VA_SYSCTRL	0x08000000

#define ZMP_PWM0_CTRL	(ZMP_VA_SYSCTRL+0xB4)
#define ZMP_PWM1_CTRL	(ZMP_VA_SYSCTRL+0xBC)
#define ZMP_PWM2_CTRL	(ZMP_VA_SYSCTRL+0xC4)
#define ZMP_PWM3_CTRL	(ZMP_VA_SYSCTRL+0xCC)
#define ZMP_PWM4_CTRL	(ZMP_VA_SYSCTRL+0xD4)

#define ZMP_PWM_DEVICE ZMP_PWM3_CTRL

#define ZMP_PWM_TIMER_CTRL1 		(0x00)
#define ZMP_PWM_TIMER_CTRL2 		(0x04)

#define ZMP_PWM_HIGH_LEVEL(x) 		((x) << 16)
#define ZMP_PWM_LOW_LEVEL(x) 		(x)

#define ZMP_PT_MODE_PWM (0x2)

#define ZMP_TIMER_TIMEOUT_CLR 		(1<<30)
#define ZMP_TIMER_FEED_TIMER 		(1<<29)
#define ZMP_PWM_TIMER_EN 			(1<<28)
#define ZMP_TIMER_TIMEOUT_STA 		(1<<27)
#define ZMP_TIMER_READ_SEL 			(1<<26)

#define ZMP_TIMER_WORK_MODE_MASK 	(0x3<<24)
#define ZMP_TIMER_WORK_MODE(x) 		((x)<<24)
#define ZMP_PWM_TIMER_PRE_DIV(x) 	((x) << 16)
#define ZMP_PWM_TIMER_PRE_DIV_MASK  ((0xff) << 16)
#define ZMP_PWM_TIMER_PRE_DIV_MAX	0xff
#define ZMP_PWM_MAX_DUTY_CYCLE		65536

#define REAL_CRYSTAL_FREQ 		(12*1000*1000)
#define PWM_MAX_FREQ 		(6*1000*1000)
#define PWM_MIN_FREQ 		(92/256)

#define ZMP_PWM_TIMER_CNT 	(5)
#define NSEC_PER_SEC	1000000000L


static int zmp_pwm_set_duty_cycle(u16 high, u16 low, u32 pre_div)
{
	u32 regval;

	__raw_writel(high << 16 | low,ZMP_PWM_DEVICE + ZMP_PWM_TIMER_CTRL1);

	regval = __raw_readl(ZMP_PWM_DEVICE);
	regval &= ~ZMP_PWM_TIMER_PRE_DIV_MASK;
	regval |= ZMP_PWM_TIMER_PRE_DIV(pre_div);
	__raw_writel(regval,ZMP_PWM_DEVICE);

	return 0;
}

static int zmp_pwm_config(unsigned long long duty_ns, unsigned long long period_ns)
{

	__maybe_unused unsigned int regval,flag=0;
	unsigned int pre_div;
	unsigned int i = 0;
    __maybe_unused unsigned int tmp = 0;
    __maybe_unused unsigned long long hl = 0;
	int high_level= 0;
    int low_level = 0;

	regval = __raw_readl(ZMP_PWM_DEVICE + ZMP_PWM_TIMER_CTRL2);
	regval &= ~ZMP_TIMER_WORK_MODE_MASK;
	regval |= ZMP_TIMER_WORK_MODE(ZMP_PT_MODE_PWM);        /*set pwm/timer as PWM*/
	__raw_writel(regval,ZMP_PWM_DEVICE + ZMP_PWM_TIMER_CTRL2);

    /*
    ** how to calculate period_ns and duty_ns
    ** period_ns = 10^9 / pwm_freq
    ** duty_ns   = duty_cycle * period_ns
    */

    /* if pwm_freq < 100 HZ*/
    if(period_ns > 10000000){
        for (i = ZMP_PWM_TIMER_PRE_DIV_MAX+1; i>=1; i--) {
            unsigned long long tmp = REAL_CRYSTAL_FREQ/1000000 *duty_ns;
            do_div(tmp, 1000*i);
            high_level = tmp -1;
            tmp = REAL_CRYSTAL_FREQ/1000000 * period_ns;
            do_div(tmp, 1000*i);
            low_level  = tmp -2 - high_level;

            if(high_level < 0){
                high_level = 0;
            }
            if(low_level < 0){
                low_level = 0;
            }

            if(high_level > 65535)
                 high_level = 65535;
            if(low_level > 65535)
                 low_level = 65535;
            if ((high_level <= 65535) && (low_level <= 65535))
                break;
        }
    }
    else {
        for (i = 1; i<=ZMP_PWM_TIMER_PRE_DIV_MAX+1; i++) {
            unsigned long tmp = REAL_CRYSTAL_FREQ/1000000 *duty_ns;
            do_div(tmp, 1000*i);
	        high_level = tmp -1;
            tmp = REAL_CRYSTAL_FREQ/1000000 * period_ns;
            do_div(tmp, 1000*i);
            low_level  = tmp -2 - high_level;

            if(high_level < 0){
                high_level = 0;
            }
            if(low_level < 0){
                low_level = 0;
            }
            if ((high_level <= 65535) && (low_level <= 65535))
                break;
        }
    }

	pre_div = i - 1;
//	pr_debug("div:%u, h:%u, l:%u, act_f:%d\n", pre_div, high_level, low_level,  (REAL_CRYSTAL_FREQ) / (pre_div + 1) / ((high_level+1)+(low_level+1)));

	return zmp_pwm_set_duty_cycle(high_level, low_level, pre_div);
}

/* To enable the pwm/timer . */
static int zmp_pwm_enable(void)
{
	u32 regval,regval1;

	regval = ZMP_TIMER_FEED_TIMER | ZMP_PWM_TIMER_EN;
    regval1 = __raw_readl(ZMP_PWM_DEVICE + ZMP_PWM_TIMER_CTRL2);
	regval1 |= regval;
    __raw_writel(regval1,ZMP_PWM_DEVICE + ZMP_PWM_TIMER_CTRL2);

	return 0;
}


static void zmp_pwm_disable(void)
{
	u32 regval;

	regval = __raw_readl(ZMP_PWM_DEVICE + ZMP_PWM_TIMER_CTRL2);
	regval &= ~ZMP_PWM_TIMER_EN;
	__raw_writel(regval,ZMP_PWM_DEVICE + ZMP_PWM_TIMER_CTRL2);
}

static void io_mux_init(void)
{
	u32 regval;
	regval = readl(0x08000180);
	regval |= (2 << 0);
	writel(regval, 0x08000180);

	regval = readl(0x080001d8);
	regval |= (1 << 0);
	writel(regval, 0x080001d8);

	regval = readl(0x080001fc);
	regval |= (1<< 29);
	writel(regval, 0x080001fc);

}

void zmp_pwm_init(unsigned long long duty_ns, unsigned long long period_ns)
{
	io_mux_init();
	zmp_pwm_disable();
	zmp_pwm_config(duty_ns,period_ns);
	zmp_pwm_enable();
}
