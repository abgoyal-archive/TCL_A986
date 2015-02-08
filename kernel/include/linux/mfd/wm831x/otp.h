

#ifndef __MFD_WM831X_OTP_H__
#define __MFD_WM831X_OTP_H__

int wm831x_otp_init(struct wm831x *wm831x);
void wm831x_otp_exit(struct wm831x *wm831x);

#define WM831X_UNIQUE_ID_MASK                   0xFFFF  /* UNIQUE_ID - [15:0] */
#define WM831X_UNIQUE_ID_SHIFT                       0  /* UNIQUE_ID - [15:0] */
#define WM831X_UNIQUE_ID_WIDTH                      16  /* UNIQUE_ID - [15:0] */

#define WM831X_UNIQUE_ID_MASK                   0xFFFF  /* UNIQUE_ID - [15:0] */
#define WM831X_UNIQUE_ID_SHIFT                       0  /* UNIQUE_ID - [15:0] */
#define WM831X_UNIQUE_ID_WIDTH                      16  /* UNIQUE_ID - [15:0] */

#define WM831X_UNIQUE_ID_MASK                   0xFFFF  /* UNIQUE_ID - [15:0] */
#define WM831X_UNIQUE_ID_SHIFT                       0  /* UNIQUE_ID - [15:0] */
#define WM831X_UNIQUE_ID_WIDTH                      16  /* UNIQUE_ID - [15:0] */

#define WM831X_UNIQUE_ID_MASK                   0xFFFF  /* UNIQUE_ID - [15:0] */
#define WM831X_UNIQUE_ID_SHIFT                       0  /* UNIQUE_ID - [15:0] */
#define WM831X_UNIQUE_ID_WIDTH                      16  /* UNIQUE_ID - [15:0] */

#define WM831X_UNIQUE_ID_MASK                   0xFFFF  /* UNIQUE_ID - [15:0] */
#define WM831X_UNIQUE_ID_SHIFT                       0  /* UNIQUE_ID - [15:0] */
#define WM831X_UNIQUE_ID_WIDTH                      16  /* UNIQUE_ID - [15:0] */

#define WM831X_UNIQUE_ID_MASK                   0xFFFF  /* UNIQUE_ID - [15:0] */
#define WM831X_UNIQUE_ID_SHIFT                       0  /* UNIQUE_ID - [15:0] */
#define WM831X_UNIQUE_ID_WIDTH                      16  /* UNIQUE_ID - [15:0] */

#define WM831X_UNIQUE_ID_MASK                   0xFFFF  /* UNIQUE_ID - [15:0] */
#define WM831X_UNIQUE_ID_SHIFT                       0  /* UNIQUE_ID - [15:0] */
#define WM831X_UNIQUE_ID_WIDTH                      16  /* UNIQUE_ID - [15:0] */

#define WM831X_UNIQUE_ID_MASK                   0xFFFF  /* UNIQUE_ID - [15:0] */
#define WM831X_UNIQUE_ID_SHIFT                       0  /* UNIQUE_ID - [15:0] */
#define WM831X_UNIQUE_ID_WIDTH                      16  /* UNIQUE_ID - [15:0] */

#define WM831X_OTP_FACT_ID_MASK                 0xFFFE  /* OTP_FACT_ID - [15:1] */
#define WM831X_OTP_FACT_ID_SHIFT                     1  /* OTP_FACT_ID - [15:1] */
#define WM831X_OTP_FACT_ID_WIDTH                    15  /* OTP_FACT_ID - [15:1] */
#define WM831X_OTP_FACT_FINAL                   0x0001  /* OTP_FACT_FINAL */
#define WM831X_OTP_FACT_FINAL_MASK              0x0001  /* OTP_FACT_FINAL */
#define WM831X_OTP_FACT_FINAL_SHIFT                  0  /* OTP_FACT_FINAL */
#define WM831X_OTP_FACT_FINAL_WIDTH                  1  /* OTP_FACT_FINAL */

#define WM831X_DC3_TRIM_MASK                    0xF000  /* DC3_TRIM - [15:12] */
#define WM831X_DC3_TRIM_SHIFT                       12  /* DC3_TRIM - [15:12] */
#define WM831X_DC3_TRIM_WIDTH                        4  /* DC3_TRIM - [15:12] */
#define WM831X_DC2_TRIM_MASK                    0x0FC0  /* DC2_TRIM - [11:6] */
#define WM831X_DC2_TRIM_SHIFT                        6  /* DC2_TRIM - [11:6] */
#define WM831X_DC2_TRIM_WIDTH                        6  /* DC2_TRIM - [11:6] */
#define WM831X_DC1_TRIM_MASK                    0x003F  /* DC1_TRIM - [5:0] */
#define WM831X_DC1_TRIM_SHIFT                        0  /* DC1_TRIM - [5:0] */
#define WM831X_DC1_TRIM_WIDTH                        6  /* DC1_TRIM - [5:0] */

#define WM831X_CHIP_ID_MASK                     0xFFFF  /* CHIP_ID - [15:0] */
#define WM831X_CHIP_ID_SHIFT                         0  /* CHIP_ID - [15:0] */
#define WM831X_CHIP_ID_WIDTH                        16  /* CHIP_ID - [15:0] */

#define WM831X_OSC_TRIM_MASK                    0x0780  /* OSC_TRIM - [10:7] */
#define WM831X_OSC_TRIM_SHIFT                        7  /* OSC_TRIM - [10:7] */
#define WM831X_OSC_TRIM_WIDTH                        4  /* OSC_TRIM - [10:7] */
#define WM831X_BG_TRIM_MASK                     0x0078  /* BG_TRIM - [6:3] */
#define WM831X_BG_TRIM_SHIFT                         3  /* BG_TRIM - [6:3] */
#define WM831X_BG_TRIM_WIDTH                         4  /* BG_TRIM - [6:3] */
#define WM831X_LPBG_TRIM_MASK                   0x0007  /* LPBG_TRIM - [2:0] */
#define WM831X_LPBG_TRIM_SHIFT                       0  /* LPBG_TRIM - [2:0] */
#define WM831X_LPBG_TRIM_WIDTH                       3  /* LPBG_TRIM - [2:0] */

#define WM831X_CHILD_I2C_ADDR_MASK              0x00FE  /* CHILD_I2C_ADDR - [7:1] */
#define WM831X_CHILD_I2C_ADDR_SHIFT                  1  /* CHILD_I2C_ADDR - [7:1] */
#define WM831X_CHILD_I2C_ADDR_WIDTH                  7  /* CHILD_I2C_ADDR - [7:1] */
#define WM831X_CH_AW                            0x0001  /* CH_AW */
#define WM831X_CH_AW_MASK                       0x0001  /* CH_AW */
#define WM831X_CH_AW_SHIFT                           0  /* CH_AW */
#define WM831X_CH_AW_WIDTH                           1  /* CH_AW */

#define WM831X_CHARGE_TRIM_MASK                 0x003F  /* CHARGE_TRIM - [5:0] */
#define WM831X_CHARGE_TRIM_SHIFT                     0  /* CHARGE_TRIM - [5:0] */
#define WM831X_CHARGE_TRIM_WIDTH                     6  /* CHARGE_TRIM - [5:0] */

#define WM831X_OTP_AUTO_PROG                    0x8000  /* OTP_AUTO_PROG */
#define WM831X_OTP_AUTO_PROG_MASK               0x8000  /* OTP_AUTO_PROG */
#define WM831X_OTP_AUTO_PROG_SHIFT                  15  /* OTP_AUTO_PROG */
#define WM831X_OTP_AUTO_PROG_WIDTH                   1  /* OTP_AUTO_PROG */
#define WM831X_OTP_CUST_ID_MASK                 0x7FFE  /* OTP_CUST_ID - [14:1] */
#define WM831X_OTP_CUST_ID_SHIFT                     1  /* OTP_CUST_ID - [14:1] */
#define WM831X_OTP_CUST_ID_WIDTH                    14  /* OTP_CUST_ID - [14:1] */
#define WM831X_OTP_CUST_FINAL                   0x0001  /* OTP_CUST_FINAL */
#define WM831X_OTP_CUST_FINAL_MASK              0x0001  /* OTP_CUST_FINAL */
#define WM831X_OTP_CUST_FINAL_SHIFT                  0  /* OTP_CUST_FINAL */
#define WM831X_OTP_CUST_FINAL_WIDTH                  1  /* OTP_CUST_FINAL */

#define WM831X_DBE_VALID_DATA_MASK              0xFFFF  /* DBE_VALID_DATA - [15:0] */
#define WM831X_DBE_VALID_DATA_SHIFT                  0  /* DBE_VALID_DATA - [15:0] */
#define WM831X_DBE_VALID_DATA_WIDTH                 16  /* DBE_VALID_DATA - [15:0] */


#endif
