//
// Created by lch on 10/9/22.
//

#ifndef DECODE_A_DSPLIB_H
#define DECODE_A_DSPLIB_H
void DSPF_sp_cfftr2_dit(float *x, float *w, unsigned short n);
void DSPF_sp_fir_gen(const float *x, const float *h, float *y, int nh, int ny);
void DSPF_sp_blk_move(const float *x, float *y, const int n);
void DSPF_sp_icfftr2_dif(float* x, float* w, unsigned short n);
void DSPF_sp_fir_cplx(const float *x, const float *h, float *y, int nh, int ny);
void DSPF_sp_convol(const float *x, const float *h, float *y, const short nh, const short ny);
void DSPF_sp_mat_trans(const float *x, const int rows, const int cols, float *y);
float DSPF_sp_vecsum_sq(const float *x, const int nx);
void DSPF_fltoq15(const float *x, short *y, const int n);
void DSPF_sp_dotp_cplx(const float* x, const float* y, int nx, float *re, float *im);
float DSPF_sp_maxval(const float *x, int nx);
void DSPF_sp_mat_mul(float *x1, const int r1, const int c1, float *x2, const int c2, float *y);
void cmltf(float *a, float *b, float *out);
void bit_rev(float* x, int n);// ifft用比特位反转
void gen_w_r2(float* w, int n);// ifft产生旋转因子
#endif //DECODE_A_DSPLIB_H
