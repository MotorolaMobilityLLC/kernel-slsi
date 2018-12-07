#ifndef _AW8695_H_
#define _AW8695_H_

/*********************************************************
 *
 * kernel version
 *
 ********************************************************/
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 4, 1)
#define TIMED_OUTPUT
#endif

/*********************************************************
 *
 * aw8695.h
 *
 ********************************************************/
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#ifdef TIMED_OUTPUT
#include <../../../drivers/staging/android/timed_output.h>
#else
#include <linux/leds.h>
#endif

/*********************************************************
 *
 * marco
 *
 ********************************************************/
#define MAX_I2C_BUFFER_SIZE                 65536

#define AW8695_REG_MAX                      0xff

#define AW8695_SEQUENCER_SIZE               8
#define AW8695_SEQUENCER_LOOP_SIZE          4

#define AW8695_RTP_I2C_SINGLE_MAX_NUM       512

#define HAPTIC_MAX_TIMEOUT                  10000

/* motor config */
//#define LRA_0619
#define LRA_0832

#ifdef LRA_0619
#define AW8695_HAPTIC_F0_PRE                1700    // 170Hz
#define AW8695_HAPTIC_F0_CALI_PERCEN        7       // -7%~7%
#define AW8695_HAPTIC_CONT_DRV_LVL          105     // 105*6.1/256=2.50v
#define AW8695_HAPTIC_CONT_DRV_LVL_OV       125     // 125*6.1/256=2.98v
#define AW8695_HAPTIC_CONT_TD               0x009a
#define AW8695_HAPTIC_CONT_ZC_THR           0x0ff1
#define AW8695_HAPTIC_CONT_NUM_BRK          3
#endif

#ifdef LRA_0832
#define AW8695_HAPTIC_F0_PRE                2350    // 235Hz
#define AW8695_HAPTIC_F0_CALI_PERCEN        7       // -7%~7%
#define AW8695_HAPTIC_CONT_DRV_LVL          98
#define AW8695_HAPTIC_CONT_DRV_LVL_OV       0x70     // 155*6.1/256=3.69v
#define AW8695_HAPTIC_CONT_TD               0x0073
#define AW8695_HAPTIC_CONT_ZC_THR           0x0ff1
#define AW8695_HAPTIC_CONT_NUM_BRK          3
#endif


#define AW8695_HAPTIC_F0_COEFF              260     //2.604167


/* trig config */
#define AW8695_TRG1_ENABLE                  1
#define AW8695_TRG2_ENABLE                  1
#define AW8695_TRG3_ENABLE                  1

#define AW8695_TRG1_DEFAULT_LEVEL           1       // 1: high level; 0: low level
#define AW8695_TRG2_DEFAULT_LEVEL           1       // 1: high level; 0: low level
#define AW8695_TRG3_DEFAULT_LEVEL           1       // 1: high level; 0: low level

#if AW8695_TRG1_ENABLE
#define AW8695_TRG1_DEFAULT_ENABLE          AW8695_BIT_TRGCFG2_TRG1_ENABLE
#else
#define AW8695_TRG1_DEFAULT_ENABLE          AW8695_BIT_TRGCFG2_TRG1_DISABLE
#endif

#if AW8695_TRG2_ENABLE
#define AW8695_TRG2_DEFAULT_ENABLE          AW8695_BIT_TRGCFG2_TRG2_ENABLE
#else
#define AW8695_TRG2_DEFAULT_ENABLE          AW8695_BIT_TRGCFG2_TRG2_DISABLE
#endif

#if AW8695_TRG3_ENABLE
#define AW8695_TRG3_DEFAULT_ENABLE          AW8695_BIT_TRGCFG2_TRG3_ENABLE
#else
#define AW8695_TRG3_DEFAULT_ENABLE          AW8695_BIT_TRGCFG2_TRG3_DISABLE
#endif

#if AW8695_TRG1_DEFAULT_LEVEL
#define AW8695_TRG1_DEFAULT_POLAR           AW8695_BIT_TRGCFG1_TRG1_POLAR_POS
#else
#define AW8695_TRG1_DEFAULT_POLAR           AW8695_BIT_TRGCFG1_TRG1_POLAR_NEG
#endif

#if AW8695_TRG2_DEFAULT_LEVEL
#define AW8695_TRG2_DEFAULT_POLAR           AW8695_BIT_TRGCFG1_TRG2_POLAR_POS
#else
#define AW8695_TRG2_DEFAULT_POLAR           AW8695_BIT_TRGCFG1_TRG2_POLAR_NEG
#endif

#if AW8695_TRG3_DEFAULT_LEVEL
#define AW8695_TRG3_DEFAULT_POLAR           AW8695_BIT_TRGCFG1_TRG3_POLAR_POS
#else
#define AW8695_TRG3_DEFAULT_POLAR           AW8695_BIT_TRGCFG1_TRG3_POLAR_NEG
#endif

enum ram_mode_seq {
    AW8695_SHORT_RAM_100_UP = 1,
    AW8695_LONG_RAM = 2,
    AW8695_SHORT_RAM_100_UP2 = 3,
    AW8695_SHORT_RAM_60_UP = 4,
    AW8695_SHORT_RAM_80_UP = 5,
    AW8695_SHORT_RAM_20_UP = 6,
};

enum aw8695_flags {
    AW8695_FLAG_NONR = 0,
    AW8695_FLAG_SKIP_INTERRUPTS = 1,
};

enum aw8695_haptic_read_write {
    AW8695_HAPTIC_CMD_READ_REG = 0,
    AW8695_HAPTIC_CMD_WRITE_REG = 1,
};


enum aw8695_haptic_work_mode {
    AW8695_HAPTIC_STANDBY_MODE = 0,
    AW8695_HAPTIC_RAM_MODE = 1,
    AW8695_HAPTIC_RTP_MODE = 2,
    AW8695_HAPTIC_TRIG_MODE = 3,
    AW8695_HAPTIC_CONT_MODE = 4,
    AW8695_HAPTIC_RAM_LOOP_MODE = 5,
};

enum aw8695_haptic_bst_mode {
    AW8695_HAPTIC_BYPASS_MODE = 0,
    AW8695_HAPTIC_BOOST_MODE = 1,
};

enum aw8695_haptic_activate_mode {
  AW8695_HAPTIC_ACTIVATE_RAM_MODE = 0,
  AW8695_HAPTIC_ACTIVATE_CONT_MODE = 1,
};


enum aw8695_haptic_vbat_comp_mode {
    AW8695_HAPTIC_VBAT_SW_COMP_MODE = 0,
    AW8695_HAPTIC_VBAT_HW_COMP_MODE = 1,
};

enum aw8695_haptic_f0_flag {
    AW8695_HAPTIC_LRA_F0 = 0,
    AW8695_HAPTIC_CALI_F0 = 1,
};

enum aw8695_haptic_pwm_mode {
    AW8695_PWM_48K = 0,
    AW8695_PWM_24K = 1,
    AW8695_PWM_12K = 2,
};

/*********************************************************
 *
 * struct
 *
 ********************************************************/
struct fileops {
    unsigned char cmd;
    unsigned char reg;
    unsigned char ram_addrh;
    unsigned char ram_addrl;
};

struct ram {
    unsigned int len;
    unsigned int check_sum;
    unsigned int base_addr;
    unsigned char version;
    unsigned char ram_shift;
    unsigned char baseaddr_shift;
};

struct haptic_ctr{
    unsigned char cmd;
    unsigned char play;
    unsigned char wavseq;
    unsigned char loop;
    unsigned char gain;
};

struct haptic_audio{
    struct mutex lock;
    struct hrtimer timer;
    struct work_struct work;
    int delay_val;
    int timer_val;
    unsigned char cnt;
    struct haptic_ctr data[256];
    struct haptic_ctr ctr;
};

struct aw8695 {
    struct regmap *regmap;
    struct i2c_client *i2c;
    struct device *dev;
    struct input_dev *input;

    struct mutex lock;
    struct hrtimer timer;
    struct work_struct vibrator_work;
    struct work_struct rtp_work;
    struct delayed_work ram_work;
#ifdef TIMED_OUTPUT
    struct timed_output_dev to_dev;
#else
    struct led_classdev cdev;
#endif
    struct fileops fileops;
    struct ram ram;

    int reset_gpio;
    int irq_gpio;

    unsigned char hwen_flag;
    unsigned char flags;
    unsigned char chipid;

    unsigned char play_mode;

    unsigned char activate_mode;

    unsigned char auto_boost;

    int state;
    int duration;
    int amplitude;
    int index;
    int vmax;
    int gain;

    unsigned char seq[AW8695_SEQUENCER_SIZE];
    unsigned char loop[AW8695_SEQUENCER_SIZE];

    unsigned int rtp_cnt;
    unsigned int rtp_file_num;

    unsigned char rtp_init;
    unsigned char ram_init;

    unsigned int f0;
    unsigned int f0_pre;
    unsigned int cont_f0;
    unsigned int cont_td;
    unsigned int cont_zc_thr;
    unsigned char cont_drv_lvl;
    unsigned char cont_drv_lvl_ov;
    unsigned char cont_num_brk;
    unsigned char max_pos_beme;
    unsigned char max_neg_beme;
    unsigned char f0_cali_flag;

    struct haptic_audio haptic_audio;
};

struct aw8695_container{
    int len;
    unsigned char data[];
};


/*********************************************************
 *
 * ioctl
 *
 ********************************************************/
struct aw8695_seq_loop {
    unsigned char loop[AW8695_SEQUENCER_SIZE];
};

struct aw8695_que_seq {
    unsigned char index[AW8695_SEQUENCER_SIZE];
};


#define AW8695_HAPTIC_IOCTL_MAGIC         'h'

#define AW8695_HAPTIC_SET_QUE_SEQ         _IOWR(AW8695_HAPTIC_IOCTL_MAGIC, 1, struct aw8695_que_seq*)
#define AW8695_HAPTIC_SET_SEQ_LOOP        _IOWR(AW8695_HAPTIC_IOCTL_MAGIC, 2, struct aw8695_seq_loop*)
#define AW8695_HAPTIC_PLAY_QUE_SEQ        _IOWR(AW8695_HAPTIC_IOCTL_MAGIC, 3, unsigned int)
#define AW8695_HAPTIC_SET_BST_VOL         _IOWR(AW8695_HAPTIC_IOCTL_MAGIC, 4, unsigned int)
#define AW8695_HAPTIC_SET_BST_PEAK_CUR    _IOWR(AW8695_HAPTIC_IOCTL_MAGIC, 5, unsigned int)
#define AW8695_HAPTIC_SET_GAIN            _IOWR(AW8695_HAPTIC_IOCTL_MAGIC, 6, unsigned int)
#define AW8695_HAPTIC_PLAY_REPEAT_SEQ     _IOWR(AW8695_HAPTIC_IOCTL_MAGIC, 7, unsigned int)


#endif

