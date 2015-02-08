


#ifndef __INC_PSNR_H
#define __INC_PSNR_H

extern double vp8_mse2psnr(double Samples, double Peak, double Mse);
extern double vp8_calc_psnr(YV12_BUFFER_CONFIG *source, YV12_BUFFER_CONFIG *dest, double *YPsnr, double *UPsnr, double *VPsnr, double *sq_error);

#endif
