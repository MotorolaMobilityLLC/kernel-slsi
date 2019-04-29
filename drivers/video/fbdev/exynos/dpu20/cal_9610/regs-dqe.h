#ifndef _REGS_DPU_H_
#define _REGS_DPU_H_

#define DQE_BASE			0x2000
/* DQECON_SET */
#define DQECON				0x0000
#define DQE_APS_SW_RESET_MASK		(1 << 18)
#define DQE_APS_SW_RESET_GET(_v)	(((_v) >> 18) & 0x1)
#define DQE_HSC_SW_RESET_MASK		(1 << 16)
#define DQE_HSC_SW_RESET_GET(_v)	(((_v) >> 16) & 0x1)
#define DQE_APS_ON_MASK			(1 << 4)
#define DQE_APS_ON_GET(_v)		(((_v) >> 4) & 0x1)
#define DQE_HSC_ON_MASK			(1 << 3)
#define DQE_HSC_ON_GET(_v)		(((_v) >> 3) & 0x1)
#define DQE_GAMMA_ON_MASK		(1 << 2)
#define DQE_GAMMA_ON_GET(_v)		(((_v) >> 2) & 0x1)
#define DQE_CGC_ON_MASK			(1 << 1)
#define DQE_CGC_ON_GET(_v)		(((_v) >> 1) & 0x1)

/* IMG_SIZE_SET */
#define DQEIMG_SIZESET			0x0004
#define DQEIMG_VSIZE_F(_v)		((_v) << 16)
#define DQEIMG_VSIZE_MASK		(0x1fff << 16)
#define DQEIMG_VSIZE_GET(_v)		(((_v) >> 16) & 0x1fff)
#define DQEIMG_HSIZE_F(_v)		((_v) << 0)
#define DQEIMG_HSIZE_MASK		(0x1fff << 0)
#define DQEIMG_HSIZE_GET(_v)		(((_v) >> 0) & 0x1fff)

/* CGC_SET */
#define DQECGC1LUT_MAX			(8)
#define DQECGC2LUT_MAX			(9)
#define DQECGC1_RGB_BASE		0x0010
#define DQECGC2_RGB_BASE		0x0410

#define DQECGCLUT_R(_v)			(((_v) & 0x3ff) << 20)
#define DQECGCLUT_G(_v)			(((_v) & 0x3ff) << 10)
#define DQECGCLUT_B(_v)			(((_v) & 0x3ff) << 0)
#define DQECGCLUT_R_MASK		(0x3ff << 20)
#define DQECGCLUT_G_MASK		(0x3ff << 10)
#define DQECGCLUT_B_MASK		(0x3ff << 0)
#define DQECGCLUT_R_GET(_v)		(((_v) >> 20) & 0x3ff)
#define DQECGCLUT_G_GET(_v)		(((_v) >> 10) & 0x3ff)
#define DQECGCLUT_B_GET(_v)		(((_v) >> 0) & 0x3ff)

#define DQECGC1_RED			0x0010
#define DQECGC1_GREEN			0x0014
#define DQECGC1_BLUE			0x0018
#define DQECGC1_CYAN			0x001c
#define DQECGC1_MAGENTA			0x0020
#define DQECGC1_YELLOW			0x0024
#define DQECGC1_WHITE			0x0028
#define DQECGC1_BLACK			0x002c
#define DQECGC_MC_CONTROL		0x0030
#define DQECGC_MC_GAIN(_v)		(((_v) & 0xfff) << 4)
#define DQECGC_MC_EN(_v)		(((_v) & 0x1) << 0)

#define DQECGC2_RED			0x0410
#define DQECGC2_GREEN			0x0414
#define DQECGC2_BLUE			0x0418
#define DQECGC2_CYAN			0x041c
#define DQECGC2_MAGENTA			0x0420
#define DQECGC2_YELLOW			0x0424
#define DQECGC2_WHITE			0x0428
#define DQECGC2_BLACK			0x042c

/* GAMMA_SET */
#define DQEGAMMA_OFFSET			132
#define DQEGAMMALUT_X_Y_BASE		0x0034
#define DQEGAMMALUT_MAX			(33 * 3)
#define DQEGAMMALUT_X(_v)		(((_v) & 0x7ff) << 0)
#define DQEGAMMALUT_Y(_v)		(((_v) & 0x7ff) << 16)
#define DQEGAMMALUT_X_MASK		(0x7ff << 0)
#define DQEGAMMALUT_Y_MASK		(0x7ff << 16)
#define DQEGAMMALUT_X_GET(_v)		(((_v) >> 0) & 0x7ff)
#define DQEGAMMALUT_Y_GET(_v)		(((_v) >> 16) & 0x7ff)

#define DQEGAMMALUT_R_01_00		0x0034
#define DQEGAMMALUT_R_03_02		0x0038
#define DQEGAMMALUT_R_05_04		0x003c
#define DQEGAMMALUT_R_07_06		0x0040
#define DQEGAMMALUT_R_09_08		0x0044
#define DQEGAMMALUT_R_11_10		0x0048
#define DQEGAMMALUT_R_13_12		0x004c
#define DQEGAMMALUT_R_15_14		0x0050
#define DQEGAMMALUT_R_17_16		0x0054
#define DQEGAMMALUT_R_19_18		0x0058
#define DQEGAMMALUT_R_21_20		0x005c
#define DQEGAMMALUT_R_23_22		0x0060
#define DQEGAMMALUT_R_25_24		0x0064
#define DQEGAMMALUT_R_27_26		0x0068
#define DQEGAMMALUT_R_29_28		0x006c
#define DQEGAMMALUT_R_31_30		0x0070
#define DQEGAMMALUT_R_33_32		0x0074
#define DQEGAMMALUT_R_35_34		0x0078
#define DQEGAMMALUT_R_37_36		0x007c
#define DQEGAMMALUT_R_39_38		0x0080
#define DQEGAMMALUT_R_41_40		0x0084
#define DQEGAMMALUT_R_43_42		0x0088
#define DQEGAMMALUT_R_45_44		0x008c
#define DQEGAMMALUT_R_47_46		0x0090
#define DQEGAMMALUT_R_49_48		0x0094
#define DQEGAMMALUT_R_51_50		0x0098
#define DQEGAMMALUT_R_53_52		0x009c
#define DQEGAMMALUT_R_55_54		0x00a0
#define DQEGAMMALUT_R_57_56		0x00a4
#define DQEGAMMALUT_R_59_58		0x00a8
#define DQEGAMMALUT_R_61_60		0x00ac
#define DQEGAMMALUT_R_63_62		0x00b0
#define DQEGAMMALUT_R_64		0x00b4
#define DQEGAMMALUT_G_01_00		0x00b8
#define DQEGAMMALUT_G_03_02		0x00bc
#define DQEGAMMALUT_G_05_04		0x00c0
#define DQEGAMMALUT_G_07_06		0x00c4
#define DQEGAMMALUT_G_09_08		0x00c8
#define DQEGAMMALUT_G_11_10		0x00cc
#define DQEGAMMALUT_G_13_12		0x00d0
#define DQEGAMMALUT_G_15_14		0x00d4
#define DQEGAMMALUT_G_17_16		0x00d8
#define DQEGAMMALUT_G_19_18		0x00dc
#define DQEGAMMALUT_G_21_20		0x00e0
#define DQEGAMMALUT_G_23_22		0x00e4
#define DQEGAMMALUT_G_25_24		0x00e8
#define DQEGAMMALUT_G_27_26		0x00ec
#define DQEGAMMALUT_G_29_28		0x00f0
#define DQEGAMMALUT_G_31_30		0x00f4
#define DQEGAMMALUT_G_33_32		0x00f8
#define DQEGAMMALUT_G_35_34		0x00fc
#define DQEGAMMALUT_G_37_36		0x0100
#define DQEGAMMALUT_G_39_38		0x0104
#define DQEGAMMALUT_G_41_40		0x0108
#define DQEGAMMALUT_G_43_42		0x010c
#define DQEGAMMALUT_G_45_44		0x0110
#define DQEGAMMALUT_G_47_46		0x0114
#define DQEGAMMALUT_G_49_48		0x0118
#define DQEGAMMALUT_G_51_50		0x011c
#define DQEGAMMALUT_G_53_52		0x0120
#define DQEGAMMALUT_G_55_54		0x0124
#define DQEGAMMALUT_G_57_56		0x0128
#define DQEGAMMALUT_G_59_58		0x012c
#define DQEGAMMALUT_G_61_60		0x0130
#define DQEGAMMALUT_G_63_62		0x0134
#define DQEGAMMALUT_G_64		0x0138
#define DQEGAMMALUT_B_01_00		0x013c
#define DQEGAMMALUT_B_03_02		0x0140
#define DQEGAMMALUT_B_05_04		0x0144
#define DQEGAMMALUT_B_07_06		0x0148
#define DQEGAMMALUT_B_09_08		0x014c
#define DQEGAMMALUT_B_11_10		0x0150
#define DQEGAMMALUT_B_13_12		0x0154
#define DQEGAMMALUT_B_15_14		0x0158
#define DQEGAMMALUT_B_17_16		0x015c
#define DQEGAMMALUT_B_19_18		0x0160
#define DQEGAMMALUT_B_21_20		0x0164
#define DQEGAMMALUT_B_23_22		0x0168
#define DQEGAMMALUT_B_25_24		0x016c
#define DQEGAMMALUT_B_27_26		0x0170
#define DQEGAMMALUT_B_29_28		0x0174
#define DQEGAMMALUT_B_31_30		0x0178
#define DQEGAMMALUT_B_33_32		0x017c
#define DQEGAMMALUT_B_35_34		0x0180
#define DQEGAMMALUT_B_37_36		0x0184
#define DQEGAMMALUT_B_39_38		0x0188
#define DQEGAMMALUT_B_41_40		0x018c
#define DQEGAMMALUT_B_43_42		0x0190
#define DQEGAMMALUT_B_45_44		0x0194
#define DQEGAMMALUT_B_47_46		0x0198
#define DQEGAMMALUT_B_49_48		0x019c
#define DQEGAMMALUT_B_51_50		0x01a0
#define DQEGAMMALUT_B_53_52		0x01a4
#define DQEGAMMALUT_B_55_54		0x01a8
#define DQEGAMMALUT_B_57_56		0x01ac
#define DQEGAMMALUT_B_59_58		0x01b0
#define DQEGAMMALUT_B_61_60		0x01b4
#define DQEGAMMALUT_B_63_62		0x01b8
#define DQEGAMMALUT_B_64		0x01bc

/*APS_SET*/
#define DQEAPSLUT_MAX			(15)
#define DQEAPSLUT_BASE			0x01C0

#define DQEAPSLUT_ST(_v)		(((_v) & 0xff) << 16)
#define DQEAPSLUT_NS(_v)		(((_v) & 0xff) << 8)
#define DQEAPSLUT_LT(_v)		(((_v) & 0xff) << 0)
#define DQEAPSLUT_PL_W2(_v)		(((_v) & 0xf) << 16)
#define DQEAPSLUT_PL_W1(_v)		(((_v) & 0xf) << 0)
#define DQEAPSLUT_CTMODE(_v)		(((_v) & 0x3) << 0)
#define DQEAPSLUT_PP_EN(_v)		(((_v) & 0x1) << 0)
#define DQEAPSLUT_TDR_MAX(_v)		(((_v) & 0x3ff) << 16)
#define DQEAPSLUT_TDR_MIN(_v)		(((_v) & 0x3ff) << 0)
#define DQEAPSLUT_AMBIENT_LIGHT(_v)	(((_v) & 0xff) << 0)
#define DQEAPSLUT_BACK_LIGHT(_v)	(((_v) & 0xff) << 0)
#define DQEAPSLUT_DSTEP(_v)		(((_v) & 0x3f) << 0)
#define DQEAPSLUT_SCALE_MODE(_v)	(((_v) & 0x3) << 0)
#define DQEAPSLUT_THRESHOLD_3(_v)	(((_v) & 0x3) << 4)
#define DQEAPSLUT_THRESHOLD_2(_v)	(((_v) & 0x3) << 2)
#define DQEAPSLUT_THRESHOLD_1(_v)	(((_v) & 0x3) << 0)
#define DQEAPSLUT_GAIN_LIMIT(_v)	(((_v) & 0x3ff) << 0)
#define DQEAPSLUT_ROI_SAME(_v)		(((_v) & 0x1) << 2)
#define DQEAPSLUT_UPDATE_METHOD(_v)	(((_v) & 0x1) << 1)
#define DQEAPSLUT_PARTIAL_FRAME(_v)	(((_v) & 0x1) << 0)
#define DQEAPSLUT_ROI_Y1(_v)		(((_v) & 0x1fff) << 16)
#define DQEAPSLUT_ROI_X1(_v)		(((_v) & 0x1fff) << 0)
#define DQEAPSLUT_IBSI_01(_v)		(((_v) & 0xffff) << 16)
#define DQEAPSLUT_IBSI_00(_v)		(((_v) & 0xffff) << 0)
#define DQEAPSLUT_IBSI_11(_v)		(((_v) & 0xffff) << 16)
#define DQEAPSLUT_IBSI_10(_v)		(((_v) & 0xffff) << 0)

#define DQEAPS_GAIN			0x01C0
#define DQEAPS_WEIGHT			0x01C4
#define DQEAPS_CTMODE			0x01C8
#define DQEAPS_PPEN			0x01CC
#define DQEAPS_TDRMINMAX		0x01D0
#define DQEAPS_AMBIENT_LIGHT		0x01D4
#define DQEAPS_BACK_LIGHT		0x01D8
#define DQEAPS_DSTEP			0x01DC
#define DQEAPS_SCALE_MODE		0x01E0
#define DQEAPS_THRESHOLD		0x01E4
#define DQEAPS_GAIN_LIMIT		0x01E8
#define DQEAPS_DIMMING_DONE_INTR	0x01EC

#define DQEAPS_PARTIAL_CON		0x0234

#define DQEAPS_FULL_IMG_SIZESET	0x0238
#define DQEAPS_FULL_IMG_VSIZE_F(_v)	((_v) << 16)
#define DQEAPS_FULL_IMG_VSIZE_MASK	(0x1fff << 16)
#define DQEAPS_FULL_IMG_VSIZE_GET(_v)	(((_v) >> 16) & 0x1fff)
#define DQEAPS_FULL_IMG_HSIZE_F(_v)	((_v) << 0)
#define DQEAPS_FULL_IMG_HSIZE_MASK	(0x1fff << 0)
#define DQEAPS_FULL_IMG_HSIZE_GET(_v)	(((_v) >> 0) & 0x1fff)

#define DQEAPS_PARTIAL_ROI_UP_LEFT_POS	0x023C
#define DQEAPS_PARTIAL_IBSI_01_00	0x0308
#define DQEAPS_PARTIAL_IBSI_11_10	0x030C

/*HSC_SET */
#define DQEHSCLUT_MAX			(11)
#define DQEHSCLUT_BASE			0x0208

#define DQEHSCLUT_B(_v)			(((_v) & 0x3ff) << 20)
#define DQEHSCLUT_G(_v)			(((_v) & 0x3ff) << 10)
#define DQEHSCLUT_R(_v)			(((_v) & 0x3ff) << 0)
#define DQEHSCLUT_Y(_v)			(((_v) & 0x3ff) << 20)
#define DQEHSCLUT_M(_v)			(((_v) & 0x3ff) << 10)
#define DQEHSCLUT_C(_v)			(((_v) & 0x3ff) << 0)
#define DQEHSCLUT_B_MASK		(0x3ff << 20)
#define DQEHSCLUT_G_MASK		(0x3ff << 10)
#define DQEHSCLUT_R_MASK		(0x3ff << 0)
#define DQEHSCLUT_Y_MASK		(0x3ff << 20)
#define DQEHSCLUT_M_MASK		(0x3ff << 10)
#define DQEHSCLUT_C_MASK		(0x3ff << 0)
#define DQEHSCLUT_B_GET(_v)		(((_v) >> 20) & 0x3ff)
#define DQEHSCLUT_G_GET(_v)		(((_v) >> 10) & 0x3ff)
#define DQEHSCLUT_R_GET(_v)		(((_v) >> 0) & 0x3ff)
#define DQEHSCLUT_Y_GET(_v)		(((_v) >> 20) & 0x3ff)
#define DQEHSCLUT_M_GET(_v)		(((_v) >> 10) & 0x3ff)
#define DQEHSCLUT_C_GET(_v)		(((_v) >> 0) & 0x3ff)

#define DQEHSCLUT_ALPHA_SHIFT2(_v)	(((_v) & 0x1f) << 20)
#define DQEHSCLUT_ALPHA_SHIFT1(_v)	(((_v) & 0x1ff) << 8)
#define DQEHSCLUT_ALPHA_SCALE(_v)	(((_v) & 0xf) << 0)

#define DQEHSCLUT_POLY_CURVE3(_v)	(((_v) & 0x3ff) << 20)
#define DQEHSCLUT_POLY_CURVE2(_v)	(((_v) & 0x3ff) << 10)
#define DQEHSCLUT_POLY_CURVE1(_v)	(((_v) & 0x3ff) << 0)

#define DQEHSCLUT_POLY_CURVE6(_v)	(((_v) & 0x3ff) << 20)
#define DQEHSCLUT_POLY_CURVE5(_v)	(((_v) & 0x3ff) << 10)
#define DQEHSCLUT_POLY_CURVE4(_v)	(((_v) & 0x3ff) << 0)

#define DQEHSCLUT_POLY_CURVE8(_v)	(((_v) & 0x3ff) << 10)
#define DQEHSCLUT_POLY_CURVE7(_v)	(((_v) & 0x3ff) << 0)

#define DQEHSCLUT_SKIN_S2(_v)		(((_v) & 0x3ff) << 16)
#define DQEHSCLUT_SKIN_S1(_v)		(((_v) & 0x3ff) << 0)

#define DQEHSCLUT_SKIN_H2(_v)		(((_v) & 0x1ff) << 16)
#define DQEHSCLUT_SKIN_H1(_v)		(((_v) & 0x1ff) << 0)

#define DQEHSCLUT_YCOMP_RATIO(_v)	(((_v) & 0xf) << 12)
#define DQEHSCLUT_TSC_GAIN(_v)		(((_v) & 0x3ff) << 0)

#define DQEHSC_CONTROL			0x0204
#define HSC_PPSC_ON(_v)		(((_v) & 0x1) << 5)
#define HSC_PPSC_ON_MASK		(1 << 5)
#define HSC_YCOMP_ON(_v)		(((_v) & 0x1) << 4)
#define HSC_YCOMP_ON_MASK		(1 << 4)
#define HSC_TSC_ON(_v)			(((_v) & 0x1) << 3)
#define HSC_TSC_ON_MASK		(1 << 3)
#define HSC_DITHER_ON(_v)		(((_v) & 0x1) << 2)
#define HSC_DITHER_ON_MASK		(1 << 2)
#define HSC_PPHC_ON(_v)		(((_v) & 0x1) << 1)
#define HSC_PPHC_ON_MASK		(1 << 1)
#define HSC_SKIN_ON(_v)		(((_v) & 0x1) << 0)
#define HSC_SKIN_ON_MASK		(1 << 0)
#define HSC_ALL_MASK			(0x3f << 0)

#define HSC_PPSC_ON_GET(_v)		(((_v) >> 5) & 0x1)
#define HSC_YCOMP_ON_GET(_v)		(((_v) >> 4) & 0x1)
#define HSC_TSC_ON_GET(_v)		(((_v) >> 3) & 0x1)
#define HSC_DITHER_ON_GET(_v)		(((_v) >> 2) & 0x1)
#define HSC_PPHC_ON_GET(_v)		(((_v) >> 1) & 0x1)
#define HSC_SKIN_ON_GET(_v)		(((_v) >> 0) & 0x1)

#define DQEHSC_PPSCGAIN_RGB		0x0208
#define DQEHSC_PPSCGAIN_CMY		0x020c
#define DQEHSC_ALPHASCALE_SHIF		0x0210
#define DQEHSC_POLY_CURVE0		0x0214
#define DQEHSC_POLY_CURVE1		0x0218
#define DQEHSC_SKIN_S			0x021c
#define DQEHSC_PPHCGAIN_RGB		0x0220
#define DQEHSC_PPHCGAIN_CMY		0x0224
#define DQEHSC_TSC_YCOMP		0x0228
#define DQEHSC_POLY_CURVE2		0x022c
#define DQEHSC_SKIN_H			0x0240

#define DQEHSC_FULL_PXL_NUM		0x0310
#define DQEHSC_FULL_PXL_NUM_MASK	(0x03ffffff << 0)
#define DQEHSC_FULL_PXL_NUM_GET(_v)	(((_v) >> 0) & 0x03ffffff)

#define DQEAPS_FULL_PXL_NUM		0x0320
#define DQEAPS_FULL_PXL_NUM_MASK	(0x03ffffff << 0)
#define DQEAPS_FULL_PXL_NUM_GET(_v)	(((_v) >> 0) & 0x03ffffff)

#define SHADOW_DQE_OFFSET		0x9000

#endif
