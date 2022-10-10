
#include "stdafx.h"
#include "dsplib.h"
#include <math.h>
#include <float.h>
void DSPF_sp_cfftr2_dit(float* x, float* w, unsigned short n)
{
    unsigned short n2, ie, ia, i, j, k, m;
    float rtemp, itemp, c, s;

    n2 = n;
    ie = 1;

    for(k=n; k > 1; k >>= 1)
    {
        n2 >>= 1;
        ia = 0;

        for(j=0; j < ie; j++)
        {
            c = w[2*j];
            s = w[2*j+1];

            for(i=0; i < n2; i++)
            {
                m = ia + n2;
                rtemp = c * x[2*m] + s * x[2*m+1];
                itemp = c * x[2*m+1] - s * x[2*m];
                x[2*m] = x[2*ia] - rtemp;
                x[2*m+1] = x[2*ia+1] - itemp;
                x[2*ia] = x[2*ia] + rtemp;
                x[2*ia+1] = x[2*ia+1] + itemp;
                ia++;
            }

            ia += n2;
        }
        ie <<= 1;
    }
}
void DSPF_sp_fir_gen(const float *x, const float *h, float *y, int nh, int ny)
{
    int i, j;
    float sum;

    for(j = 0; j < ny; j++)
    {
        sum = 0;

        // note: h coeffs given in reverse order: { h[nh-1], h[nh-2], ..., h[0] }
        for(i = 0; i < nh; i++)
            sum += x[i + j] * h[i];

        y[j] = sum;
    }
}
void DSPF_sp_blk_move(const float *x, float *y, const int n)
{
    int i;

    for (i = 0 ; i < n; i++)
        y[i] = x[i];
}
void DSPF_sp_icfftr2_dif(float* x, float* w, unsigned short n)
{
    unsigned short n2, ie, ia, i, j, k, m;
    float rtemp, itemp, c, s;

    n2 = 1;
    ie = n;

    for(k=n; k > 1; k >>= 1)
    {
        ie >>= 1;
        ia = 0;

        for(j=0; j < ie; j++)
        {
            c = w[2*j];
            s = w[2*j+1];

            for(i=0; i < n2; i++)
            {
                m = ia + n2;
                rtemp = x[2*ia] - x[2*m];
                x[2*ia] = x[2*ia] + x[2*m];
                itemp = x[2*ia+1] - x[2*m+1];
                x[2*ia+1] = x[2*ia+1] + x[2*m+1];
                x[2*m] = c*rtemp - s*itemp;
                x[2*m+1] = c*itemp + s*rtemp;
                ia++;
            }

            ia += n2;
        }

        n2 <<= 1;
    }
}
void DSPF_sp_fir_cplx(const float *x, const float *h, float *y, int nh, int ny)
{
    int i, j;
    float imag, real;

    for (i = 0; i < 2*ny; i += 2)
    {
        imag = 0;
        real = 0;

        for (j = 0; j < 2*nh; j += 2)
        {
            real += h[j] * x[i-j] - h[j+1] * x[i+1-j];
            imag += h[j] * x[i+1-j] + h[j+1] * x[i-j];
        }

        y[i] = real;
        y[i+1] = imag;
    }
}
void DSPF_sp_convol(const float *x, const float *h,
                    float *y, const short nh, const short ny)
{
    short i, j;
    float sum;

    for (i = ny; i > 0; i--)
    {
        sum = 0;

        for (j = nh; j > 0; j--)
            sum += x[ny - i + nh - j] * h[j - 1];

        y[ny - i] = sum;
    }
}
void DSPF_sp_mat_trans(const float *x, const int rows, const int cols, float *y)
{
    int i, j;

    for(i = 0; i < cols; i++)
        for(j = 0; j < rows; j++)
            y[i * rows + j] = x[i + cols * j];
}
float DSPF_sp_vecsum_sq(const float *x, const int nx)
{
    int i;
    float sum = 0;

    for(i = 0; i < nx; i++ )
        sum += x[i] * x[i];

    return sum;
}
void DSPF_fltoq15(const float *x, short *y, const int n)
{
    int i, a;

    for(i = 0; i < n; i++)
    {
        a = floor(32768 * x[i]);
        // saturate to 16-bit //
        if (a > 32767) a = 32767;
        if (a < -32768) a = -32768;
        y[i] = (short) a;
    }
}
void DSPF_sp_dotp_cplx(const float* x, const float* y, int nx, float *re, float *im)
{
    float real = 0, imag = 0;
    int i;

    for(i = 0; i < nx; i++)
    {
        real += (x[2 * i] * y[2 * i] - x[2 * i + 1] * y[2 * i + 1]);
        imag += (x[2 * i] * y[2 * i + 1] + x[2 * i + 1] * y[2 * i]);
    }

    *re = real;
    *im = imag;
}
float DSPF_sp_maxval(const float *x, int nx)
{
    int   i, index;
    float max = -FLT_MAX;

    for (i = 0; i < nx; i++)
        if (x[i] > max)
        {
            max = x[i];
            index = i;
        }

    return max;
}
void DSPF_sp_mat_mul(float *x1, const int r1, const int c1, float *x2, const int c2, float *y)
{
    int i, j, k;
    float sum;

    // Multiply each row in x1 by each column in x2.
    // The product of row m in x1 and column n in x2 is placed
    // in position (m,n) in the result.
    for (i = 0; i < r1; i++)
        for (j = 0; j < c2; j++)
        {
            sum = 0;

            for (k = 0; k < c1; k++)
                sum += x1[k + i * c1] * x2[j + k * c2];

            y[j + i * c2] = sum;
        }
}
void cmltf(float *a, float *b, float *out)
{
    float Real;
    float Image;
    Real = a[0] * b[0] - a[1] * b[1];
    Image = a[0] * b[1] + a[1] * b[0];
    out[0] = Real;
    out[1] = Image;
}
// 比特位反转
void bit_rev(float* x, int n)
{
    int i, j, k;
    float rtemp, itemp;
    j = 0;

    for (i = 1; i<(n - 1); i++)
    {
        k = n >> 1;

        while (k <= j)
        {
            j -= k;
            k >>= 1;
        }

        j += k;

        if (i<j)
        {
            rtemp = x[j * 2];
            x[j * 2] = x[i * 2];
            x[i * 2] = rtemp;
            itemp = x[j * 2 + 1];
            x[j * 2 + 1] = x[i * 2 + 1];
            x[i * 2 + 1] = itemp;
        }
    }
}

// 产生旋转因子
void gen_w_r2(float* w, int n)
{
    int i;
    float e = 6.2831852 / n;

    for (i = 0; i<(n >> 1); i++)
    {
        w[2 * i] = cos(i*e);
        w[2 * i + 1] = sin(i*e);
    }
}