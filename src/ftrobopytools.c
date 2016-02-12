#include <Python.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/param.h>
#include <bytesobject.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include "SDL.h"

//////////////////////////////////////////////////////////////////////////////////////////
// Begin of NanoJPEG
//////////////////////////////////////////////////////////////////////////////////////////
// url: http://svn.emphy.de/nanojpeg/trunk/nanojpeg/nanojpeg.c
//////////////////////////////////////////////////////////////////////////////////////////
// NanoJPEG -- KeyJ's Tiny Baseline JPEG Decoder
// version 1.3.2 (2014-02-02)
// by Martin J. Fiedler <martin.fiedler@gmx.net>
//
// This software is published under the terms of KeyJ's Research License,
// version 0.2. Usage of this software is subject to the following conditions:
// 0. There's no warranty whatsoever. The author(s) of this software can not
//    be held liable for any damages that occur when using this software.
// 1. This software may be used freely for both non-commercial and commercial
//    purposes.
// 2. This software may be redistributed freely as long as no fees are charged
//    for the distribution and this license information is included.
// 3. This software may be modified freely except for this license information,
//    which must not be changed in any way.
// 4. If anything other than configuration, indentation or comments have been
//    altered in the code, the original author(s) must receive a copy of the
//    modified code.

typedef enum _nj_result {
    NJ_OK = 0,        // no error, decoding successful
    NJ_NO_JPEG,       // not a JPEG file
    NJ_UNSUPPORTED,   // unsupported format
    NJ_OUT_OF_MEM,    // out of memory
    NJ_INTERNAL_ERR,  // internal error
    NJ_SYNTAX_ERROR,  // syntax error
    __NJ_FINISHED,    // used internally, will never be reported
} nj_result_t;

void njInit(void);
nj_result_t njDecode(const void* jpeg, const int size);
int njGetWidth(void);
int njGetHeight(void);
int njIsColor(void);
unsigned char* njGetImage(void);
int njGetImageSize(void);
void njDone(void);

#define NJ_INLINE static inline
#define NJ_FORCE_INLINE static inline

#include <stdlib.h>
#include <string.h>
#define njAllocMem malloc
#define njFreeMem  free
#define njFillMem  memset
#define njCopyMem  memcpy

typedef struct _nj_code {
    unsigned char bits, code;
} nj_vlc_code_t;

typedef struct _nj_cmp {
    int cid;
    int ssx, ssy;
    int width, height;
    int stride;
    int qtsel;
    int actabsel, dctabsel;
    int dcpred;
    unsigned char *pixels;
} nj_component_t;

typedef struct _nj_ctx {
    nj_result_t error;
    const unsigned char *pos;
    int size;
    int length;
    int width, height;
    int mbwidth, mbheight;
    int mbsizex, mbsizey;
    int ncomp;
    nj_component_t comp[3];
    int qtused, qtavail;
    unsigned char qtab[4][64];
    nj_vlc_code_t vlctab[4][65536];
    int buf, bufbits;
    int block[64];
    int rstinterval;
    unsigned char *rgb;
} nj_context_t;

static nj_context_t nj;

static const char njZZ[64] = { 0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18,
11, 4, 5, 12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28, 35,
42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51, 58, 59, 52, 45,
38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63 };

NJ_FORCE_INLINE unsigned char njClip(const int x) {
    return (x < 0) ? 0 : ((x > 0xFF) ? 0xFF : (unsigned char) x);
}

#define W1 2841
#define W2 2676
#define W3 2408
#define W5 1609
#define W6 1108
#define W7 565

NJ_INLINE void njRowIDCT(int* blk) {
    int x0, x1, x2, x3, x4, x5, x6, x7, x8;
    if (!((x1 = blk[4] << 11)
        | (x2 = blk[6])
        | (x3 = blk[2])
        | (x4 = blk[1])
        | (x5 = blk[7])
        | (x6 = blk[5])
        | (x7 = blk[3])))
    {
        blk[0] = blk[1] = blk[2] = blk[3] = blk[4] = blk[5] = blk[6] = blk[7] = blk[0] << 3;
        return;
    }
    x0 = (blk[0] << 11) + 128;
    x8 = W7 * (x4 + x5);
    x4 = x8 + (W1 - W7) * x4;
    x5 = x8 - (W1 + W7) * x5;
    x8 = W3 * (x6 + x7);
    x6 = x8 - (W3 - W5) * x6;
    x7 = x8 - (W3 + W5) * x7;
    x8 = x0 + x1;
    x0 -= x1;
    x1 = W6 * (x3 + x2);
    x2 = x1 - (W2 + W6) * x2;
    x3 = x1 + (W2 - W6) * x3;
    x1 = x4 + x6;
    x4 -= x6;
    x6 = x5 + x7;
    x5 -= x7;
    x7 = x8 + x3;
    x8 -= x3;
    x3 = x0 + x2;
    x0 -= x2;
    x2 = (181 * (x4 + x5) + 128) >> 8;
    x4 = (181 * (x4 - x5) + 128) >> 8;
    blk[0] = (x7 + x1) >> 8;
    blk[1] = (x3 + x2) >> 8;
    blk[2] = (x0 + x4) >> 8;
    blk[3] = (x8 + x6) >> 8;
    blk[4] = (x8 - x6) >> 8;
    blk[5] = (x0 - x4) >> 8;
    blk[6] = (x3 - x2) >> 8;
    blk[7] = (x7 - x1) >> 8;
}

NJ_INLINE void njColIDCT(const int* blk, unsigned char *out, int stride) {
    int x0, x1, x2, x3, x4, x5, x6, x7, x8;
    if (!((x1 = blk[8*4] << 8)
        | (x2 = blk[8*6])
        | (x3 = blk[8*2])
        | (x4 = blk[8*1])
        | (x5 = blk[8*7])
        | (x6 = blk[8*5])
        | (x7 = blk[8*3])))
    {
        x1 = njClip(((blk[0] + 32) >> 6) + 128);
        for (x0 = 8;  x0;  --x0) {
            *out = (unsigned char) x1;
            out += stride;
        }
        return;
    }
    x0 = (blk[0] << 8) + 8192;
    x8 = W7 * (x4 + x5) + 4;
    x4 = (x8 + (W1 - W7) * x4) >> 3;
    x5 = (x8 - (W1 + W7) * x5) >> 3;
    x8 = W3 * (x6 + x7) + 4;
    x6 = (x8 - (W3 - W5) * x6) >> 3;
    x7 = (x8 - (W3 + W5) * x7) >> 3;
    x8 = x0 + x1;
    x0 -= x1;
    x1 = W6 * (x3 + x2) + 4;
    x2 = (x1 - (W2 + W6) * x2) >> 3;
    x3 = (x1 + (W2 - W6) * x3) >> 3;
    x1 = x4 + x6;
    x4 -= x6;
    x6 = x5 + x7;
    x5 -= x7;
    x7 = x8 + x3;
    x8 -= x3;
    x3 = x0 + x2;
    x0 -= x2;
    x2 = (181 * (x4 + x5) + 128) >> 8;
    x4 = (181 * (x4 - x5) + 128) >> 8;
    *out = njClip(((x7 + x1) >> 14) + 128);  out += stride;
    *out = njClip(((x3 + x2) >> 14) + 128);  out += stride;
    *out = njClip(((x0 + x4) >> 14) + 128);  out += stride;
    *out = njClip(((x8 + x6) >> 14) + 128);  out += stride;
    *out = njClip(((x8 - x6) >> 14) + 128);  out += stride;
    *out = njClip(((x0 - x4) >> 14) + 128);  out += stride;
    *out = njClip(((x3 - x2) >> 14) + 128);  out += stride;
    *out = njClip(((x7 - x1) >> 14) + 128);
}

#define njThrow(e) do { nj.error = e; return; } while (0)
#define njCheckError() do { if (nj.error) return; } while (0)

static int njShowBits(int bits) {
    unsigned char newbyte;
    if (!bits) return 0;
    while (nj.bufbits < bits) {
        if (nj.size <= 0) {
            nj.buf = (nj.buf << 8) | 0xFF;
            nj.bufbits += 8;
            continue;
        }
        newbyte = *nj.pos++;
        nj.size--;
        nj.bufbits += 8;
        nj.buf = (nj.buf << 8) | newbyte;
        if (newbyte == 0xFF) {
            if (nj.size) {
                unsigned char marker = *nj.pos++;
                nj.size--;
                switch (marker) {
                    case 0x00:
                    case 0xFF:
                        break;
                    case 0xD9: nj.size = 0; break;
                    default:
                        if ((marker & 0xF8) != 0xD0)
                            nj.error = NJ_SYNTAX_ERROR;
                        else {
                            nj.buf = (nj.buf << 8) | marker;
                            nj.bufbits += 8;
                        }
                }
            } else
                nj.error = NJ_SYNTAX_ERROR;
        }
    }
    return (nj.buf >> (nj.bufbits - bits)) & ((1 << bits) - 1);
}

NJ_INLINE void njSkipBits(int bits) {
    if (nj.bufbits < bits)
        (void) njShowBits(bits);
    nj.bufbits -= bits;
}

NJ_INLINE int njGetBits(int bits) {
    int res = njShowBits(bits);
    njSkipBits(bits);
    return res;
}

NJ_INLINE void njByteAlign(void) {
    nj.bufbits &= 0xF8;
}

static void njSkip(int count) {
    nj.pos += count;
    nj.size -= count;
    nj.length -= count;
    if (nj.size < 0) nj.error = NJ_SYNTAX_ERROR;
}

NJ_INLINE unsigned short njDecode16(const unsigned char *pos) {
    return (pos[0] << 8) | pos[1];
}

static void njDecodeLength(void) {
    if (nj.size < 2) njThrow(NJ_SYNTAX_ERROR);
    nj.length = njDecode16(nj.pos);
    if (nj.length > nj.size) njThrow(NJ_SYNTAX_ERROR);
    njSkip(2);
}

NJ_INLINE void njSkipMarker(void) {
    njDecodeLength();
    njSkip(nj.length);
}

NJ_INLINE void njDecodeSOF(void) {
    int i, ssxmax = 0, ssymax = 0;
    nj_component_t* c;
    njDecodeLength();
    if (nj.length < 9) njThrow(NJ_SYNTAX_ERROR);
    if (nj.pos[0] != 8) njThrow(NJ_UNSUPPORTED);
    nj.height = njDecode16(nj.pos+1);
    nj.width = njDecode16(nj.pos+3);
    nj.ncomp = nj.pos[5];
    njSkip(6);
    switch (nj.ncomp) {
        case 1:
        case 3:
            break;
        default:
            njThrow(NJ_UNSUPPORTED);
    }
    if (nj.length < (nj.ncomp * 3)) njThrow(NJ_SYNTAX_ERROR);
    for (i = 0, c = nj.comp;  i < nj.ncomp;  ++i, ++c) {
        c->cid = nj.pos[0];
        if (!(c->ssx = nj.pos[1] >> 4)) njThrow(NJ_SYNTAX_ERROR);
        if (c->ssx & (c->ssx - 1)) njThrow(NJ_UNSUPPORTED);  // non-power of two
        if (!(c->ssy = nj.pos[1] & 15)) njThrow(NJ_SYNTAX_ERROR);
        if (c->ssy & (c->ssy - 1)) njThrow(NJ_UNSUPPORTED);  // non-power of two
        if ((c->qtsel = nj.pos[2]) & 0xFC) njThrow(NJ_SYNTAX_ERROR);
        njSkip(3);
        nj.qtused |= 1 << c->qtsel;
        if (c->ssx > ssxmax) ssxmax = c->ssx;
        if (c->ssy > ssymax) ssymax = c->ssy;
    }
    if (nj.ncomp == 1) {
        c = nj.comp;
        c->ssx = c->ssy = ssxmax = ssymax = 1;
    }
    nj.mbsizex = ssxmax << 3;
    nj.mbsizey = ssymax << 3;
    nj.mbwidth = (nj.width + nj.mbsizex - 1) / nj.mbsizex;
    nj.mbheight = (nj.height + nj.mbsizey - 1) / nj.mbsizey;
    for (i = 0, c = nj.comp;  i < nj.ncomp;  ++i, ++c) {
        c->width = (nj.width * c->ssx + ssxmax - 1) / ssxmax;
        c->height = (nj.height * c->ssy + ssymax - 1) / ssymax;
        c->stride = nj.mbwidth * c->ssx << 3;
        if (((c->width < 3) && (c->ssx != ssxmax)) || ((c->height < 3) && (c->ssy != ssymax))) njThrow(NJ_UNSUPPORTED);
        if (!(c->pixels = njAllocMem(c->stride * nj.mbheight * c->ssy << 3))) njThrow(NJ_OUT_OF_MEM);
    }
    if (nj.ncomp == 3) {
        nj.rgb = njAllocMem(nj.width * nj.height * nj.ncomp);
        if (!nj.rgb) njThrow(NJ_OUT_OF_MEM);
    }
    njSkip(nj.length);
}

NJ_INLINE void njDecodeDHT(void) {
    int codelen, currcnt, remain, spread, i, j;
    nj_vlc_code_t *vlc;
    static unsigned char counts[16];
    njDecodeLength();
    while (nj.length >= 17) {
        i = nj.pos[0];
        if (i & 0xEC) njThrow(NJ_SYNTAX_ERROR);
        if (i & 0x02) njThrow(NJ_UNSUPPORTED);
        i = (i | (i >> 3)) & 3;  // combined DC/AC + tableid value
        for (codelen = 1;  codelen <= 16;  ++codelen)
            counts[codelen - 1] = nj.pos[codelen];
        njSkip(17);
        vlc = &nj.vlctab[i][0];
        remain = spread = 65536;
        for (codelen = 1;  codelen <= 16;  ++codelen) {
            spread >>= 1;
            currcnt = counts[codelen - 1];
            if (!currcnt) continue;
            if (nj.length < currcnt) njThrow(NJ_SYNTAX_ERROR);
            remain -= currcnt << (16 - codelen);
            if (remain < 0) njThrow(NJ_SYNTAX_ERROR);
            for (i = 0;  i < currcnt;  ++i) {
                register unsigned char code = nj.pos[i];
                for (j = spread;  j;  --j) {
                    vlc->bits = (unsigned char) codelen;
                    vlc->code = code;
                    ++vlc;
                }
            }
            njSkip(currcnt);
        }
        while (remain--) {
            vlc->bits = 0;
            ++vlc;
        }
    }
    if (nj.length) njThrow(NJ_SYNTAX_ERROR);
}

NJ_INLINE void njDecodeDQT(void) {
    int i;
    unsigned char *t;
    njDecodeLength();
    while (nj.length >= 65) {
        i = nj.pos[0];
        if (i & 0xFC) njThrow(NJ_SYNTAX_ERROR);
        nj.qtavail |= 1 << i;
        t = &nj.qtab[i][0];
        for (i = 0;  i < 64;  ++i)
            t[i] = nj.pos[i + 1];
        njSkip(65);
    }
    if (nj.length) njThrow(NJ_SYNTAX_ERROR);
}

NJ_INLINE void njDecodeDRI(void) {
    njDecodeLength();
    if (nj.length < 2) njThrow(NJ_SYNTAX_ERROR);
    nj.rstinterval = njDecode16(nj.pos);
    njSkip(nj.length);
}

static int njGetVLC(nj_vlc_code_t* vlc, unsigned char* code) {
    int value = njShowBits(16);
    int bits = vlc[value].bits;
    if (!bits) { nj.error = NJ_SYNTAX_ERROR; return 0; }
    njSkipBits(bits);
    value = vlc[value].code;
    if (code) *code = (unsigned char) value;
    bits = value & 15;
    if (!bits) return 0;
    value = njGetBits(bits);
    if (value < (1 << (bits - 1)))
        value += ((-1) << bits) + 1;
    return value;
}

NJ_INLINE void njDecodeBlock(nj_component_t* c, unsigned char* out) {
    unsigned char code = 0;
    int value, coef = 0;
    njFillMem(nj.block, 0, sizeof(nj.block));
    c->dcpred += njGetVLC(&nj.vlctab[c->dctabsel][0], NULL);
    nj.block[0] = (c->dcpred) * nj.qtab[c->qtsel][0];
    do {
        value = njGetVLC(&nj.vlctab[c->actabsel][0], &code);
        if (!code) break;  // EOB
        if (!(code & 0x0F) && (code != 0xF0)) njThrow(NJ_SYNTAX_ERROR);
        coef += (code >> 4) + 1;
        if (coef > 63) njThrow(NJ_SYNTAX_ERROR);
        nj.block[(int) njZZ[coef]] = value * nj.qtab[c->qtsel][coef];
    } while (coef < 63);
    for (coef = 0;  coef < 64;  coef += 8)
        njRowIDCT(&nj.block[coef]);
    for (coef = 0;  coef < 8;  ++coef)
        njColIDCT(&nj.block[coef], &out[coef], c->stride);
}

NJ_INLINE void njDecodeScan(void) {
    int i, mbx, mby, sbx, sby;
    int rstcount = nj.rstinterval, nextrst = 0;
    nj_component_t* c;
    njDecodeLength();
    if (nj.length < (4 + 2 * nj.ncomp)) njThrow(NJ_SYNTAX_ERROR);
    if (nj.pos[0] != nj.ncomp) njThrow(NJ_UNSUPPORTED);
    njSkip(1);
    for (i = 0, c = nj.comp;  i < nj.ncomp;  ++i, ++c) {
        if (nj.pos[0] != c->cid) njThrow(NJ_SYNTAX_ERROR);
        if (nj.pos[1] & 0xEE) njThrow(NJ_SYNTAX_ERROR);
        c->dctabsel = nj.pos[1] >> 4;
        c->actabsel = (nj.pos[1] & 1) | 2;
        njSkip(2);
    }
    if (nj.pos[0] || (nj.pos[1] != 63) || nj.pos[2]) njThrow(NJ_UNSUPPORTED);
    njSkip(nj.length);
    for (mbx = mby = 0;;) {
        for (i = 0, c = nj.comp;  i < nj.ncomp;  ++i, ++c)
            for (sby = 0;  sby < c->ssy;  ++sby)
                for (sbx = 0;  sbx < c->ssx;  ++sbx) {
                    njDecodeBlock(c, &c->pixels[((mby * c->ssy + sby) * c->stride + mbx * c->ssx + sbx) << 3]);
                    njCheckError();
                }
        if (++mbx >= nj.mbwidth) {
            mbx = 0;
            if (++mby >= nj.mbheight) break;
        }
        if (nj.rstinterval && !(--rstcount)) {
            njByteAlign();
            i = njGetBits(16);
            if (((i & 0xFFF8) != 0xFFD0) || ((i & 7) != nextrst)) njThrow(NJ_SYNTAX_ERROR);
            nextrst = (nextrst + 1) & 7;
            rstcount = nj.rstinterval;
            for (i = 0;  i < 3;  ++i)
                nj.comp[i].dcpred = 0;
        }
    }
    nj.error = __NJ_FINISHED;
}

#if NJ_CHROMA_FILTER

#define CF4A (-9)
#define CF4B (111)
#define CF4C (29)
#define CF4D (-3)
#define CF3A (28)
#define CF3B (109)
#define CF3C (-9)
#define CF3X (104)
#define CF3Y (27)
#define CF3Z (-3)
#define CF2A (139)
#define CF2B (-11)
#define CF(x) njClip(((x) + 64) >> 7)

NJ_INLINE void njUpsampleH(nj_component_t* c) {
    const int xmax = c->width - 3;
    unsigned char *out, *lin, *lout;
    int x, y;
    out = njAllocMem((c->width * c->height) << 1);
    if (!out) njThrow(NJ_OUT_OF_MEM);
    lin = c->pixels;
    lout = out;
    for (y = c->height;  y;  --y) {
        lout[0] = CF(CF2A * lin[0] + CF2B * lin[1]);
        lout[1] = CF(CF3X * lin[0] + CF3Y * lin[1] + CF3Z * lin[2]);
        lout[2] = CF(CF3A * lin[0] + CF3B * lin[1] + CF3C * lin[2]);
        for (x = 0;  x < xmax;  ++x) {
            lout[(x << 1) + 3] = CF(CF4A * lin[x] + CF4B * lin[x + 1] + CF4C * lin[x + 2] + CF4D * lin[x + 3]);
            lout[(x << 1) + 4] = CF(CF4D * lin[x] + CF4C * lin[x + 1] + CF4B * lin[x + 2] + CF4A * lin[x + 3]);
        }
        lin += c->stride;
        lout += c->width << 1;
        lout[-3] = CF(CF3A * lin[-1] + CF3B * lin[-2] + CF3C * lin[-3]);
        lout[-2] = CF(CF3X * lin[-1] + CF3Y * lin[-2] + CF3Z * lin[-3]);
        lout[-1] = CF(CF2A * lin[-1] + CF2B * lin[-2]);
    }
    c->width <<= 1;
    c->stride = c->width;
    njFreeMem(c->pixels);
    c->pixels = out;
}

NJ_INLINE void njUpsampleV(nj_component_t* c) {
    const int w = c->width, s1 = c->stride, s2 = s1 + s1;
    unsigned char *out, *cin, *cout;
    int x, y;
    out = njAllocMem((c->width * c->height) << 1);
    if (!out) njThrow(NJ_OUT_OF_MEM);
    for (x = 0;  x < w;  ++x) {
        cin = &c->pixels[x];
        cout = &out[x];
        *cout = CF(CF2A * cin[0] + CF2B * cin[s1]);  cout += w;
        *cout = CF(CF3X * cin[0] + CF3Y * cin[s1] + CF3Z * cin[s2]);  cout += w;
        *cout = CF(CF3A * cin[0] + CF3B * cin[s1] + CF3C * cin[s2]);  cout += w;
        cin += s1;
        for (y = c->height - 3;  y;  --y) {
            *cout = CF(CF4A * cin[-s1] + CF4B * cin[0] + CF4C * cin[s1] + CF4D * cin[s2]);  cout += w;
            *cout = CF(CF4D * cin[-s1] + CF4C * cin[0] + CF4B * cin[s1] + CF4A * cin[s2]);  cout += w;
            cin += s1;
        }
        cin += s1;
        *cout = CF(CF3A * cin[0] + CF3B * cin[-s1] + CF3C * cin[-s2]);  cout += w;
        *cout = CF(CF3X * cin[0] + CF3Y * cin[-s1] + CF3Z * cin[-s2]);  cout += w;
        *cout = CF(CF2A * cin[0] + CF2B * cin[-s1]);
    }
    c->height <<= 1;
    c->stride = c->width;
    njFreeMem(c->pixels);
    c->pixels = out;
}

#else

NJ_INLINE void njUpsample(nj_component_t* c) {
    int x, y, xshift = 0, yshift = 0;
    unsigned char *out, *lin, *lout;
    while (c->width < nj.width) { c->width <<= 1; ++xshift; }
    while (c->height < nj.height) { c->height <<= 1; ++yshift; }
    out = njAllocMem(c->width * c->height);
    if (!out) njThrow(NJ_OUT_OF_MEM);
    lin = c->pixels;
    lout = out;
    for (y = 0;  y < c->height;  ++y) {
        lin = &c->pixels[(y >> yshift) * c->stride];
        for (x = 0;  x < c->width;  ++x)
            lout[x] = lin[x >> xshift];
        lout += c->width;
    }
    c->stride = c->width;
    njFreeMem(c->pixels);
    c->pixels = out;
}

#endif

NJ_INLINE void njConvert(void) {
    int i;
    nj_component_t* c;
    for (i = 0, c = nj.comp;  i < nj.ncomp;  ++i, ++c) {
        #if NJ_CHROMA_FILTER
            while ((c->width < nj.width) || (c->height < nj.height)) {
                if (c->width < nj.width) njUpsampleH(c);
                njCheckError();
                if (c->height < nj.height) njUpsampleV(c);
                njCheckError();
            }
        #else
            if ((c->width < nj.width) || (c->height < nj.height))
                njUpsample(c);
        #endif
        if ((c->width < nj.width) || (c->height < nj.height)) njThrow(NJ_INTERNAL_ERR);
    }
    if (nj.ncomp == 3) {
        // convert to RGB
        int x, yy;
        unsigned char *prgb = nj.rgb;
        const unsigned char *py  = nj.comp[0].pixels;
        const unsigned char *pcb = nj.comp[1].pixels;
        const unsigned char *pcr = nj.comp[2].pixels;
        for (yy = nj.height;  yy;  --yy) {
            for (x = 0;  x < nj.width;  ++x) {
                register int y = py[x] << 8;
                register int cb = pcb[x] - 128;
                register int cr = pcr[x] - 128;
                *prgb++ = njClip((y            + 359 * cr + 128) >> 8);
                *prgb++ = njClip((y -  88 * cb - 183 * cr + 128) >> 8);
                *prgb++ = njClip((y + 454 * cb            + 128) >> 8);
            }
            py += nj.comp[0].stride;
            pcb += nj.comp[1].stride;
            pcr += nj.comp[2].stride;
        }
    } else if (nj.comp[0].width != nj.comp[0].stride) {
        // grayscale -> only remove stride
        unsigned char *pin = &nj.comp[0].pixels[nj.comp[0].stride];
        unsigned char *pout = &nj.comp[0].pixels[nj.comp[0].width];
        int y;
        for (y = nj.comp[0].height - 1;  y;  --y) {
            njCopyMem(pout, pin, nj.comp[0].width);
            pin += nj.comp[0].stride;
            pout += nj.comp[0].width;
        }
        nj.comp[0].stride = nj.comp[0].width;
    }
}

void njInit(void) {
    njFillMem(&nj, 0, sizeof(nj_context_t));
}

void njDone(void) {
    int i;
    for (i = 0;  i < 3;  ++i)
        if (nj.comp[i].pixels) njFreeMem((void*) nj.comp[i].pixels);
    if (nj.rgb) njFreeMem((void*) nj.rgb);
    njInit();
}

nj_result_t njDecode(const void* jpeg, const int size) {
    njDone();
    nj.pos = (const unsigned char*) jpeg;
    nj.size = size & 0x7FFFFFFF;
    if (nj.size < 2) return NJ_NO_JPEG;
    if ((nj.pos[0] ^ 0xFF) | (nj.pos[1] ^ 0xD8)) return NJ_NO_JPEG;
    njSkip(2);
    while (!nj.error) {
        if ((nj.size < 2) || (nj.pos[0] != 0xFF)) return NJ_SYNTAX_ERROR;
        njSkip(2);
        switch (nj.pos[-1]) {
            case 0xC0: njDecodeSOF(); break;
            case 0xC4: njDecodeDHT(); break;
            case 0xDB: njDecodeDQT(); break;
            case 0xDD: njDecodeDRI(); break;
            case 0xDA: njDecodeScan(); break;
            case 0xFE: njSkipMarker(); break;
            default:
                if ((nj.pos[-1] & 0xF0) == 0xE0)
                    njSkipMarker();
                else
                    return NJ_UNSUPPORTED;
        }
    }
    if (nj.error != __NJ_FINISHED) return nj.error;
    nj.error = NJ_OK;
    njConvert();
    return nj.error;
}

int njGetWidth(void)            { return nj.width; }
int njGetHeight(void)           { return nj.height; }
int njIsColor(void)             { return (nj.ncomp != 1); }
unsigned char* njGetImage(void) { return (nj.ncomp == 1) ? nj.comp[0].pixels : nj.rgb; }
int njGetImageSize(void)        { return nj.width * nj.height * nj.ncomp; }

//////////////////////////////////////////////////////////////////////////////////////////
// End of NanoJPEG 
//////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////
// Begin of ftrobopytools
//////////////////////////////////////////////////////////////////////////////////////////
// ftrobopytools -- camera, display and image tools as python module for use with the
// fischertechnik TXT controller
// version 0.62 (2016-02-12)
// by Torsten Stuehn <stuehn@mailbox.org>
//
// The MIT License (MIT)
//
// Copyright (c) 2015, 2016 Torsten Stuehn
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

struct module_state {
  PyObject *error;
};

#if PY_MAJOR_VERSION >= 3
#define GETSTATE(m) ((struct module_state*)PyModule_GetState(m))
#else
#define GETSTATE(m) (&_state)
static struct module_state _state;
#endif

#define N_CAPTURE_BUFS 1

struct buffer {
  void   *start;
  size_t  length;
};

struct buffer *buffers;
SDL_Surface *screen;

unsigned int current_imgwidth  = 0;
unsigned int current_imgheight = 0;

unsigned char * light_reference_image  = NULL;
unsigned char * current_jpg            = NULL;
unsigned char * current_rgb            = NULL;
unsigned char * current_fb_rgb         = NULL;

int current_jpg_is_valid           = 0;
int current_rgb_is_valid           = 0;
int current_fb_rgb_is_valid        = 0;

size_t current_jpg_buf_size        = 0;
size_t current_rgb_buf_size        = 0;
size_t current_fb_rgb_buf_size     = 0;


static PyObject *ftrobopytools_camInit(PyObject *self, PyObject *args);
static PyObject *ftrobopytools_camClose(PyObject *self, PyObject *args);
static PyObject *ftrobopytools_sdlInit(PyObject *self, PyObject *args);
static PyObject *ftrobopytools_sdlClose(PyObject *self, PyObject *args);
static PyObject *ftrobopytools_getJPEGImage(PyObject *self, PyObject *args);
static PyObject *ftrobopytools_detectLines(PyObject *self, PyObject *args);
static PyObject *ftrobopytools_measureRGBColor(PyObject *self, PyObject *args);
static PyObject *ftrobopytools_measureContrast(PyObject *self, PyObject *args);

static PyMethodDef ftrobopytools_methods[] = {
    {"camInit", ftrobopytools_camInit, METH_VARARGS, "Initialize camera."},
    {"camClose", ftrobopytools_camClose, METH_VARARGS, "Close camera device."},
    {"sdlInit", ftrobopytools_sdlInit, METH_VARARGS, "Initialize Standard Display Library (SDL) for the TXT display."},
    {"sdlClose", ftrobopytools_sdlClose, METH_VARARGS, "Close Standard Display Library (SDL) for the TXT display."},
    {"getJPEGImage", ftrobopytools_getJPEGImage, METH_VARARGS, "Get the current JPEG image from the TXT camera"},
    {"detectLines", ftrobopytools_detectLines, METH_VARARGS, "Detect lines (high contrast changes) along a line"},
    {"measureRGBColor", ftrobopytools_measureRGBColor, METH_VARARGS, "measure average r,g,b-color values within a specified rectangle"},
    {"measureContrast", ftrobopytools_measureContrast, METH_VARARGS, "measure average contrast within a specified rectangle"},
    {NULL, NULL, 0, NULL}
};

#if PY_MAJOR_VERSION >= 3
static int ftrobopytools_traverse(PyObject *m, visitproc visit, void *arg) {
    Py_VISIT(GETSTATE(m)->error);
    return 0;
}
static int ftrobopytools_clear(PyObject *m) {
    Py_CLEAR(GETSTATE(m)->error);
    return 0;
}
static struct PyModuleDef moduledef = {
        PyModuleDef_HEAD_INIT,
        "ftrobopytools",
        NULL,
        sizeof(struct module_state),
        ftrobopytools_methods,
        NULL,
        ftrobopytools_traverse,
        ftrobopytools_clear,
        NULL
};
#define INITERROR return NULL

PyObject *
PyInit_ftrobopytools(void)

#else
#define INITERROR return

void
initftrobopytools(void)
#endif
{
#if PY_MAJOR_VERSION >= 3
    PyObject *module = PyModule_Create(&moduledef);
#else
    PyObject *module = Py_InitModule("ftrobopytools", ftrobopytools_methods);
#endif
    if (module == NULL)
        INITERROR;
    struct module_state *st = GETSTATE(module);
    st->error = PyErr_NewException("ftrobopytools.error", NULL, NULL);
    if (st->error == NULL) {
        Py_DECREF(module);
        INITERROR;
    }
#if PY_MAJOR_VERSION >= 3
    return module;
#endif
}

static int xioctl(int fd, int request, void *arg)
{
        int r;
        do {
          r = ioctl (fd, request, arg);
        } while (-1 == r && EINTR == errno);

        return r;
}

void display_rgb(unsigned char * rgb_buf, unsigned int imgwidth, unsigned int imgheight)
{
    Uint16 red, green, blue;
    Uint16 rgb16;
    unsigned char *fbp;
    unsigned char *rgbp;
    unsigned int x, y;

    fbp  = screen->pixels + 2*screen->w;
    rgbp = rgb_buf;
    for (y=0; y<imgwidth; y++) {
        for (x=0; x<imgheight; x++) {
            red   = rgbp[0];
            green = rgbp[1];
            blue  = rgbp[2];
            rgb16 = (red & 248) << 8 | (green & 252) << 3 | (blue & 248) >> 3;
            *((Uint16*)fbp) = rgb16;
            fbp  -= 2 ;
            rgbp += 3 * imgwidth;
        }
        for (x=0; x<screen->w-imgheight; x++) {
            *((Uint16*)fbp) = 0;
            fbp  -= 2;
        }
        fbp  += 4 * screen->w;
        rgbp -= 3 * imgwidth*imgheight - 3;
    }
    for (y=0; y<screen->h-imgwidth; y++) {
        for (x=0; x<(unsigned int)screen->w; x++) {
          *((Uint16*)fbp) = 0;
          fbp -= 2;
        }
        fbp += 4 * screen->w;
    }

    SDL_Flip(screen);
    return;
}

static PyObject *
ftrobopytools_camInit(PyObject *self, PyObject *args)
{
  static int camInitialized = 0;
  static int videv = -1;
  unsigned int fps;
  unsigned int bufidx;
  unsigned int width, height, format;
  int change_format_only;
  PyObject * res;

  if (camInitialized == 1) {
    Py_INCREF(Py_None);
    return Py_None;
  }
    
  if (!PyArg_ParseTuple(args, "IIIII", &fps, &width, &height, &format, &change_format_only))
     return NULL;

  current_imgwidth  = width;
  current_imgheight = height;

  if (format == 0) {
    format = V4L2_PIX_FMT_MJPEG;
  } else {
    format = V4L2_PIX_FMT_YUYV;
  }
    
  if (change_format_only == 0) {
    // videv = open("/dev/video0", O_RDWR | O_NONBLOCK, 0);
    videv = open("/dev/video0", O_RDWR, 0);
  }

  if (videv == -1) {
    struct module_state *st = GETSTATE(self);
    PyErr_SetString(st->error, "error open video device\n");
    return NULL;
  }

  struct v4l2_format fmt = {0};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  fmt.fmt.pix.pixelformat = format;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (xioctl(videv, VIDIOC_S_FMT, &fmt)==-1) { 
    struct module_state *st = GETSTATE(self);
    PyErr_SetString(st->error, "error set pixel format in cam_init\n");
    return NULL;
  }

  struct v4l2_streamparm parm = {0};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator=1;
  parm.parm.capture.timeperframe.denominator=fps;

  if (xioctl(videv, VIDIOC_S_PARM, &parm)==-1) { 
    struct module_state *st = GETSTATE(self);
    PyErr_SetString(st->error, "error set framerate in cam_init\n");
    return NULL;
  }
 
  struct v4l2_requestbuffers req = {0};
  req.count = N_CAPTURE_BUFS;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (xioctl(videv, VIDIOC_REQBUFS, &req)==-1) {
    struct module_state *st = GETSTATE(self);
    PyErr_SetString(st->error, "error request buffer in cam_init\n");
    return NULL;
  }

  if (req.count < N_CAPTURE_BUFS) {
    struct module_state *st = GETSTATE(self);
    PyErr_SetString(st->error, "error insufficient capture buffer memory in cam_init\n");
    return NULL;
  }

  buffers = calloc(req.count, sizeof(*buffers));
  if (!buffers) {
    struct module_state *st = GETSTATE(self);
    PyErr_SetString(st->error, "error out of memory in cam_init\n");
    return NULL;
  }

  for (bufidx = 0; bufidx<req.count; bufidx++) {
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = bufidx;
    if(xioctl(videv, VIDIOC_QUERYBUF, &buf)==-1) {
      struct module_state *st = GETSTATE(self);
      PyErr_SetString(st->error, "error query buffer in cam_init\n");
      return NULL;
    }
    buffers[bufidx].length = buf.length;
    buffers[bufidx].start = mmap (NULL,
                                  buf.length,
                                  PROT_READ | PROT_WRITE,
                                  MAP_SHARED,
                                  videv, buf.m.offset);
    if (buffers[bufidx].start==MAP_FAILED) {
      struct module_state *st = GETSTATE(self);
      PyErr_SetString(st->error, "error mmap failed in cam_init\n");
      return NULL;
    }

    struct v4l2_buffer buf2 = {0};
    buf2.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf2.memory = V4L2_MEMORY_MMAP;
    buf2.index = bufidx;
    if(xioctl(videv, VIDIOC_QBUF, &buf2)==-1) {
      struct module_state *st = GETSTATE(self);
      PyErr_SetString(st->error, "error query buffer2 in cam_init\n");
      return NULL;
    }
  }

  enum v4l2_buf_type type;
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(xioctl(videv, VIDIOC_STREAMON, &type)==-1) {
    struct module_state *st = GETSTATE(self);
    PyErr_SetString(st->error, "error start streaming in cam_init\n");
    return NULL;
  }

  struct v4l2_control control = {0};
  control.id = V4L2_CID_POWER_LINE_FREQUENCY;
  control.value = 1; // 0=off 1=50Hz 2=60Hz;
  if(xioctl(videv, VIDIOC_S_CTRL, &control)==-1) {
    struct module_state *st = GETSTATE(self);
    PyErr_SetString(st->error, "error set power line frequency in cam_init\n");
    return NULL;
  }
  
  struct v4l2_control control2 = {0};
  control2.id = V4L2_CID_SHARPNESS;
  control2.value = 0; // switch off sharpness correction of camera
  if(xioctl(videv, VIDIOC_S_CTRL, &control2)==-1) {
    struct module_state *st = GETSTATE(self);
    PyErr_SetString(st->error, "error set sharpness in cam_init\n");
    return NULL;
  }

  if (current_jpg == NULL && current_rgb == NULL && current_fb_rgb == NULL) {
    current_jpg    = malloc(sizeof(unsigned char)*width*height*3);
    if (current_jpg != NULL) {
    	current_jpg_buf_size = width*height*3;
    	current_jpg_is_valid = 0;
    } else {
  	  struct module_state *st = GETSTATE(self);
  	  PyErr_SetString(st->error, "error in cam_init: out of memory when allocating buffer\n");
  	  return NULL;
    }
    current_rgb    = malloc(sizeof(unsigned char)*width*height*3);
    if (current_rgb != NULL) {
    	current_rgb_buf_size = width*height*3;
    	current_rgb_is_valid = 0;
    } else {
  	  struct module_state *st = GETSTATE(self);
  	  PyErr_SetString(st->error, "error in cam_init: out of memory when allocating buffer\n");
  	  return NULL;
    }
    current_fb_rgb = malloc(sizeof(unsigned char)*width*height*3);
    if (current_fb_rgb != NULL) {
    	current_fb_rgb_buf_size = width*height*3;
    	current_fb_rgb_is_valid = 0;
    } else {
  	  struct module_state *st = GETSTATE(self);
  	  PyErr_SetString(st->error, "error in cam_init: out of memory when allocating buffer\n");
  	  return NULL;
    }
  } else {
	  struct module_state *st = GETSTATE(self);
	  PyErr_SetString(st->error, "error in cam_init: memory for image buffers has already been allocated\n");
	  return NULL;
  }

  if (current_jpg == NULL || current_rgb == NULL || current_fb_rgb == NULL) {
    struct module_state *st = GETSTATE(self);
    PyErr_SetString(st->error, "error out of memory in cam_init\n");
    return NULL;
  }

  // returns the number of the video device
  res = Py_BuildValue("I", videv);
  return res;
}

static PyObject *
ftrobopytools_camClose(PyObject *self, PyObject *args)
{
    int videv;
    int change_format_only;
    int i;

    if (!PyArg_ParseTuple(args, "II", &videv, &change_format_only))
        return NULL;

    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(xioctl(videv, VIDIOC_STREAMOFF, &type) == -1) {
      struct module_state *st = GETSTATE(self);
      PyErr_SetString(st->error, "error stop streaming in cam_close\n");
      return NULL;
    }

    for (i = 0; i < N_CAPTURE_BUFS; i++) {
      if (munmap(buffers[i].start, buffers[i].length) == -1) {
        struct module_state *st = GETSTATE(self);
        PyErr_SetString(st->error, "error unmapping memory buffers in cam_close\n");
        return NULL;
      }
    }

    if (change_format_only == 0) {
      close(videv);
    }
  
    if (current_jpg    != NULL) {
    	free(current_jpg);
    	current_jpg = NULL;
    }
    if (current_rgb    != NULL) {
    	free(current_rgb);
    	current_rgb = NULL;
    }
    if (current_fb_rgb != NULL) {
    	free(current_fb_rgb);
    	current_fb_rgb = NULL;
    }

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject *
ftrobopytools_sdlInit(PyObject *self, PyObject *args)
{
    Uint8  video_bpp;
    Uint32 videoflags;

    if ( SDL_Init(SDL_INIT_VIDEO) < 0 ) {
        struct module_state *st = GETSTATE(self);
        PyErr_SetString(st->error, "Couldn't initialize SDL.\n");
        return NULL;
    }
    
    videoflags = SDL_HWSURFACE | SDL_DOUBLEBUF | SDL_NOFRAME;
    video_bpp  = 16;
    
    if ( (screen=SDL_SetVideoMode(240,320,video_bpp,videoflags)) == NULL ) {
        struct module_state *st = GETSTATE(self);
        PyErr_SetString(st->error, "Couldn't set 240x320 video mode.\n");
        return NULL;
    }

    SDL_ShowCursor(SDL_DISABLE);

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject *
ftrobopytools_sdlClose(PyObject *self, PyObject *args)
{
  SDL_Quit();
  
  Py_INCREF(Py_None);
  return Py_None;
}

static PyObject *
ftrobopytools_getJPEGImage(PyObject *self, PyObject *args)
{
  int videv;
  PyObject * res;
  
  if (!PyArg_ParseTuple(args, "I", &videv))
    return NULL;
  
  struct v4l2_buffer buf = {0};
  
  for (;;) {
    fd_set fds;
    struct timeval tv = {0};
    int r0;
    
    FD_ZERO(&fds);
    FD_SET(videv, &fds);
    
    // set timeout of 2 seconds
    tv.tv_sec = 2;
    r0 = select(videv+1, &fds, NULL, NULL, &tv);
    if(r0 == -1) {
      if (EINTR == errno)
        continue;
      struct module_state *st = GETSTATE(self);
      PyErr_SetString(st->error, "error select frame in getJPEGImage\n");
      return NULL;
    }
    if(r0 == 0) {
      struct module_state *st = GETSTATE(self);
      PyErr_SetString(st->error, "error timeout in getJPEGImage\n");
      return NULL;
    }
    
    memset(&buf, 0, sizeof(struct v4l2_buffer));
    
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if(xioctl(videv, VIDIOC_DQBUF, &buf) == -1) {
      if (errno == EAGAIN)
        continue;
      struct module_state *st = GETSTATE(self);
      PyErr_SetString(st->error, "error retrieving frame in getJPEGImage\n");
      return NULL;
    }
    
    if (buf.index >= N_CAPTURE_BUFS) {
      struct module_state *st = GETSTATE(self);
      PyErr_SetString(st->error, "error buffer index out of range in getJPEGImage\n");
      return NULL;
    }
    
    break;
    
  }
#if PY_MAJOR_VERSION >= 3
  res = Py_BuildValue("y#", buffers[buf.index].start, buffers[buf.index].length);
#else
  res = Py_BuildValue("s#", buffers[buf.index].start, buffers[buf.index].length);
#endif
  if(xioctl(videv, VIDIOC_QBUF, &buf) == -1) {
    struct module_state *st = GETSTATE(self);
    PyErr_SetString(st->error, "error video buffer in getJPEGImage\n");
    return NULL;
  }
  return res;
}

static PyObject *
ftrobopytools_detectLines(PyObject *self, PyObject *args)
{
#define MAXLINES 5
    int videv;
    struct v4l2_buffer buf = {0};
    unsigned int  pos[MAXLINES];
    unsigned int  width[MAXLINES];
    unsigned int  red[MAXLINES];
    unsigned int  green[MAXLINES];
    unsigned int  blue[MAXLINES];
    unsigned char linebuf[10000];
    unsigned int imgwidth, imgheight, yhorizon, xmin, xmax;
    unsigned int threshold;
    unsigned int count, pixelcount;
    unsigned int minwidth, maxwidth;
    unsigned int numlines;
    unsigned int lidx;
    unsigned int ave_red, ave_green, ave_blue;
    unsigned int grad_r, grad_g, grad_b;
    unsigned int grad_rgb, last_grad_rgb;
    unsigned int i, j, k, l;
    int r;
    unsigned int size;
    int showImage;
    int err;
    unsigned char *ret_buf;
    unsigned char *rgbp;
    unsigned char *lri;
    unsigned char *lb;
    int gray;
    int brightness;
    
    PyObject * ret[MAXLINES];
    PyObject * res = Py_None;
    
    if (!PyArg_ParseTuple(args, "IIIIIIIIIIII", &videv, &imgwidth, &imgheight, &yhorizon, &xmin, &xmax, &minwidth, &maxwidth, &numlines, &threshold, &brightness, &showImage))
        return NULL;
    
    for (;;) {
        fd_set fds;
        struct timeval tv = {0};
        int r0;
        
        FD_ZERO(&fds);
        FD_SET(videv, &fds);
        
        // set timeout of 2 seconds
        tv.tv_sec = 2;
        r0 = select(videv+1, &fds, NULL, NULL, &tv);
        if(r0 == -1) {
            if (EINTR == errno)
                continue;
            struct module_state *st = GETSTATE(self);
            PyErr_SetString(st->error, "error select frame in detectLines\n");
            return NULL;
        }
        if(r0 == 0) {
            struct module_state *st = GETSTATE(self);
            PyErr_SetString(st->error, "error timeout in detectLines\n");
            return NULL;
        }
        
        memset(&buf, 0, sizeof(struct v4l2_buffer));
        
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if(xioctl(videv, VIDIOC_DQBUF, &buf) == -1) {
            if (errno == EAGAIN)
                continue;
            struct module_state *st = GETSTATE(self);
            PyErr_SetString(st->error, "error retrieving frame in getrgb\n");
            return NULL;
        }
        
        if (buf.index >= N_CAPTURE_BUFS) {
            struct module_state *st = GETSTATE(self);
            PyErr_SetString(st->error, "error buffer index out of range in getrgb\n");
            return NULL;
        }
        
        break;
        
    }

    if (buffers[buf.index].length > 0) {

        njInit();
        err=njDecode(buffers[buf.index].start, buffers[buf.index].length);
        if ( ( err != NJ_OK ) ) {
            njDone();
            struct module_state *st = GETSTATE(self);
            if(xioctl(videv, VIDIOC_QBUF, &buf) == -1) {
                PyErr_SetString(st->error, "Error video buffer and error decoding jpeg in detectLines\n");
                return NULL;
            }
            switch (err) {
                case NJ_SYNTAX_ERROR  :
                    // PyErr_SetString(st->error, "Error (NJ_SYNTAX_ERROR) decoding jpeg data in detectLines.\n");
                    Py_INCREF(Py_None);
                    return Py_None;
                    break;
                case NJ_NO_JPEG       :
                    // PyErr_SetString(st->error, "Error (NJ_NO_JPEG) decoding jpeg data in detectLines.\n");
                    Py_INCREF(Py_None);
                    return Py_None;
                    break;
                case NJ_OUT_OF_MEM    :
                    PyErr_SetString(st->error, "Error (NJ_OUT_OF_MEM) decoding jpeg data in detectLines.\n");
                    break;
                case NJ_UNSUPPORTED   :
                    PyErr_SetString(st->error, "Error (NJ_UNSUPPORTED) decoding jpeg data in detectLines.\n");
                    break;
                case NJ_INTERNAL_ERR  :
                    PyErr_SetString(st->error, "Error (NJ_INTERNAL_ERR) decoding jpeg data in detectLines.\n");
                    break;
                default :
                    PyErr_SetString(st->error, "Error (unknown) decoding jpeg data in detectLines.\n");
            }
            return NULL;
        }
        ret_buf    = njGetImage();
        rgbp       = ret_buf;
        ret_buf   += imgwidth * 3 * yhorizon + xmin * 3;
        size       = (xmax - xmin) * 3;
        if (light_reference_image == 0) {
          for (j=0; j<size-3; j+=3) {
            linebuf[j]   = ret_buf[j];
            linebuf[j+1] = ret_buf[j+1];
            linebuf[j+2] = ret_buf[j+2];
            ret_buf += 3;
          }
        } else {
            lri = light_reference_image + 3 * yhorizon + xmin * 3;
            lb  = linebuf;
            for (j=0; j<size; j++) {
                gray = ( ( *ret_buf + *(ret_buf+1) + *(ret_buf+2) ) / 3 ) * brightness / 200;
                for (i=0; i<3; i++) {
                    r = *ret_buf * gray / (*lri + 1);
                    if (r < 0) r = 0;
                    else if (r > 255) r = 255;
                    *lb   = (unsigned char) r;
                    ret_buf++; lri++; lb++;
                }
            }
        }
          
        ret_buf = rgbp + imgwidth * 3 * yhorizon;

        lidx          = 0;
        pixelcount    = xmax-xmin;
        ave_red       = 0;
        ave_green     = 0;
        ave_blue      = 0;
        l             = 0;
        k             = 0;
        count         = 0;
        grad_rgb      = 0;
        last_grad_rgb = 0;
        while (l<numlines && l<MAXLINES && k<pixelcount-2) {
            lidx = 3*k;
            grad_r         = abs(linebuf[lidx]   - linebuf[lidx+3]);
            grad_g         = abs(linebuf[lidx+1] - linebuf[lidx+4]);
            grad_b         = abs(linebuf[lidx+2] - linebuf[lidx+5]);
            last_grad_rgb  = grad_rgb;
            grad_rgb       = grad_r + grad_g + grad_b;
            if (grad_rgb >= threshold && last_grad_rgb < threshold && count == 0) {
                    ave_red   = linebuf[lidx];
                    ave_green = linebuf[lidx+1];
                    ave_blue  = linebuf[lidx+2];
                    count     = 1;
                    grad_rgb  = 0;
            } else {
                if (grad_rgb < threshold && last_grad_rgb >= threshold && count >= minwidth && count <= maxwidth) {
                    pos[l]     = xmin+k-count/2;
                    width[l]   = count;
                    red[l]     = ave_red / count;
                    green[l]   = ave_green / count;
                    blue[l]    = ave_blue / count;
                    for (i=1; i<count-2; i++) {
                        ret_buf[3*(pos[l]+i-count/2)]   = 255;
                        ret_buf[3*(pos[l]+i-count/2)+1] = 255;
                        ret_buf[3*(pos[l]+i-count/2)+2] = 255;
                    }
                    ret_buf[3*(pos[l])]   = 255;
                    ret_buf[3*(pos[l])+1] = 255;
                    ret_buf[3*(pos[l])+2] = 0;
                    l++;
                    count      = 0;
                    grad_rgb   = 0;
                } else {
                    if (count > 0) {
                        ave_red     += linebuf[lidx];
                        ave_green   += linebuf[lidx+1];
                        ave_blue    += linebuf[lidx+2];
                        count++;
                    } else {
                        grad_rgb = 0;
                    }
                }
            }
            k++;
        }
        
        for (k=0; k<l; k++) {
            ret[k] = Py_BuildValue("(IIIII)", pos[k], width[k], red[k], green[k], blue[k]);
        }
        
        if      (l==1) res = Py_BuildValue("(O)", ret[0]);
        else if (l==2) res = Py_BuildValue("(OO)", ret[0], ret[1]);
        else if (l==3) res = Py_BuildValue("(OOO)", ret[0], ret[1], ret[2]);
        else if (l==4) res = Py_BuildValue("(OOOO)", ret[0], ret[1], ret[2], ret[3]);
        else if (l==5) res = Py_BuildValue("(OOOOO)", ret[0], ret[1], ret[2], ret[3], ret[4]);
    
        if (showImage == 1) {
            lri     = light_reference_image;
            ret_buf = rgbp;
            size = imgwidth * imgheight - 3;
            for (j=0; j<size; j++) {
                gray = ( ( *ret_buf + *(ret_buf+1) + *(ret_buf+2) ) / 3 ) * brightness / 200;
                for (i=0; i<3; i++) {
                    r = *ret_buf * gray / (*lri + 1);
                    if (r < 0) r = 0;
                    else if (r > 255) r = 255;
                    *ret_buf   = (unsigned char) r;
                    ret_buf++; lri++;
                }
            }
            display_rgb(rgbp, imgwidth, imgheight);
        }
    
        njDone();
        
    }

    if(xioctl(videv, VIDIOC_QBUF, &buf) == -1) {
        struct module_state *st = GETSTATE(self);
        PyErr_SetString(st->error, "error video buffer in detectLines\n");
        return NULL;
    }
    if (res == Py_None) {
        Py_INCREF(Py_None);
    }
    return res;
}

static PyObject *
ftrobopytools_measureRGBColor(PyObject *self, PyObject *args)
{
#define MAXLINES 5
    int videv;
    struct v4l2_buffer buf = {0};
    unsigned int imgwidth, imgheight;
    unsigned int xtopleft, ytopleft, xbottomright, ybottomright;
    unsigned int ave_red, ave_green, ave_blue;
    unsigned int i, j;
    unsigned int rwidth, rheight, pixelcount;
    int yuyv;
    int err;
    unsigned char *ret_buf;
    
    if (!PyArg_ParseTuple(args, "IIIIIIII", &videv, &imgwidth, &imgheight, &xtopleft, &ytopleft, &xbottomright, &ybottomright, &yuyv))
        return NULL;
    
    for (;;) {
        fd_set fds;
        struct timeval tv = {0};
        int r0;
        
        FD_ZERO(&fds);
        FD_SET(videv, &fds);
        
        // set timeout of 2 seconds
        tv.tv_sec = 2;
        r0 = select(videv+1, &fds, NULL, NULL, &tv);
        if(r0 == -1) {
            if (EINTR == errno)
                continue;
            struct module_state *st = GETSTATE(self);
            PyErr_SetString(st->error, "error select frame in measureRGBColor\n");
            return NULL;
        }
        if(r0 == 0) {
            struct module_state *st = GETSTATE(self);
            PyErr_SetString(st->error, "error timeout in measureRGBColor\n");
            return NULL;
        }
        
        memset(&buf, 0, sizeof(struct v4l2_buffer));
        
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if(xioctl(videv, VIDIOC_DQBUF, &buf) == -1) {
            if (errno == EAGAIN)
                continue;
            struct module_state *st = GETSTATE(self);
            PyErr_SetString(st->error, "error retrieving frame in measureRGBColor\n");
            return NULL;
        }
        
        if (buf.index >= N_CAPTURE_BUFS) {
            struct module_state *st = GETSTATE(self);
            PyErr_SetString(st->error, "error buffer index out of range in measureRGBColor\n");
            return NULL;
        }
        
        break;
        
    }
    
    ave_red    = 0;
    ave_green  = 0;
    ave_blue   = 0;
    pixelcount = 0;

    if (buffers[buf.index].length > 0) {
        if (yuyv == 0) {
            njInit();
            err=njDecode(buffers[buf.index].start, buffers[buf.index].length);
            if ( ( err != NJ_OK ) ) {
                njDone();
                struct module_state *st = GETSTATE(self);
                if(xioctl(videv, VIDIOC_QBUF, &buf) == -1) {
                    PyErr_SetString(st->error, "Error video buffer and error decoding jpeg in measureRGBColor\n");
                    return NULL;
                }
                switch (err) {
                    case NJ_SYNTAX_ERROR  :
                        PyErr_SetString(st->error, "Error (NJ_SYNTAX_ERROR) decoding jpeg data in measureRGBColor.\n");
                        Py_INCREF(Py_None);
                        return Py_None;
                        break;
                    case NJ_NO_JPEG       :
                        PyErr_SetString(st->error, "Error (NJ_NO_JPEG) decoding jpeg data in measureRGBColor.\n");
                        break;
                    case NJ_OUT_OF_MEM    :
                        PyErr_SetString(st->error, "Error (NJ_OUT_OF_MEM) decoding jpeg data in measureRGBColor.\n");
                        break;
                    case NJ_UNSUPPORTED   :
                        PyErr_SetString(st->error, "Error (NJ_UNSUPPORTED) decoding jpeg data in measureRGBColor.\n");
                        break;
                    case NJ_INTERNAL_ERR  :
                        PyErr_SetString(st->error, "Error (NJ_INTERNAL_ERR) decoding jpeg data in measureRGBColor.\n");
                        break;
                    default :
                        PyErr_SetString(st->error, "Error (unknown) decoding jpeg data in measureRGBColor.\n");
                }
                return NULL;
            }
            ret_buf    = njGetImage();

            rwidth  = xbottomright - xtopleft;
            rheight = ybottomright - ytopleft;

            ret_buf += (imgwidth * ytopleft + xtopleft) * 3;
            pixelcount = rwidth * rheight;
            for (i=0; i<rheight; i++) {
                for (j=0; j<rwidth-3; j++) {
                    ave_red   += ret_buf[0];
                    ave_green += ret_buf[1];
                    ave_blue  += ret_buf[2];
                    ret_buf   += 3;
                }
                ret_buf += (imgwidth - rwidth) * 3;
            }
        }
    }
  
    if(xioctl(videv, VIDIOC_QBUF, &buf) == -1) {
        struct module_state *st = GETSTATE(self);
        PyErr_SetString(st->error, "error video buffer in measureRGBColor\n");
        return NULL;
    }
    if (pixelcount > 0) {
      return Py_BuildValue("III", ave_red/pixelcount, ave_green/pixelcount, ave_blue/pixelcount);
    } else {
      Py_INCREF(Py_None);
      return Py_None;
    }

}

static PyObject *
ftrobopytools_measureContrast(PyObject *self, PyObject *args)
{
#define MAXLINES 5
  int videv;
  struct v4l2_buffer buf = {0};
  unsigned int imgwidth, imgheight;
  unsigned int xtopleft, ytopleft, xbottomright, ybottomright;
  unsigned int ave_red, ave_green, ave_blue;
  unsigned int i, j;
  unsigned int rwidth, rheight, pixelcount;
  int err;
  int showImage;
  unsigned char *ret_buf;
  unsigned char *rgbp1  = NULL;
  unsigned char *rgbp2 = NULL;
  int ret_size;
  
  if (!PyArg_ParseTuple(args, "IIIIIIII", &videv, &imgwidth, &imgheight, &xtopleft, &ytopleft, &xbottomright, &ybottomright, &showImage))
    return NULL;
  
  for (;;) {
    fd_set fds;
    struct timeval tv = {0};
    int r0;
    
    FD_ZERO(&fds);
    FD_SET(videv, &fds);
    
    // set timeout of 3 seconds
    tv.tv_sec = 3;
    r0 = select(videv+1, &fds, NULL, NULL, &tv);
    if(r0 == -1) {
      if (EINTR == errno)
        continue;
      struct module_state *st = GETSTATE(self);
      PyErr_SetString(st->error, "error select frame in measureContrast\n");
      return NULL;
    }
    if(r0 == 0) {
      struct module_state *st = GETSTATE(self);
      PyErr_SetString(st->error, "error timeout in measureContrast\n");
      return NULL;
    }
    
    memset(&buf, 0, sizeof(struct v4l2_buffer));
    
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if(xioctl(videv, VIDIOC_DQBUF, &buf) == -1) {
      if (errno == EAGAIN)
        continue;
      struct module_state *st = GETSTATE(self);
      PyErr_SetString(st->error, "error retrieving frame in measureContrast\n");
      return NULL;
    }
    
    if (buf.index >= N_CAPTURE_BUFS) {
      struct module_state *st = GETSTATE(self);
      PyErr_SetString(st->error, "error buffer index out of range in measureContrast\n");
      return NULL;
    }
    
    break;
    
  }
  
  ave_red    = 0;
  ave_green  = 0;
  ave_blue   = 0;
  pixelcount = 0;
  
  if (buffers[buf.index].length > 0) {
    njInit();
    err=njDecode(buffers[buf.index].start, buffers[buf.index].length);
    if ( ( err != NJ_OK ) ) {
      njDone();
      struct module_state *st = GETSTATE(self);
      if(xioctl(videv, VIDIOC_QBUF, &buf) == -1) {
        PyErr_SetString(st->error, "Error video buffer and error decoding jpeg in measureContrast\n");
        return NULL;
      }
      switch (err) {
        case NJ_SYNTAX_ERROR  :
          // PyErr_SetString(st->error, "Error (NJ_SYNTAX_ERROR) decoding jpeg data in measureContrast.\n");
          Py_INCREF(Py_None);
          return Py_None;
          break;
        case NJ_NO_JPEG       :
          // PyErr_SetString(st->error, "Error (NJ_NO_JPEG) decoding jpeg data in measureContrast.\n");
          Py_INCREF(Py_None);
          return Py_None;
          break;
        case NJ_OUT_OF_MEM    :
          PyErr_SetString(st->error, "Error (NJ_OUT_OF_MEM) decoding jpeg data in measureContrast.\n");
          break;
        case NJ_UNSUPPORTED   :
          PyErr_SetString(st->error, "Error (NJ_UNSUPPORTED) decoding jpeg data in measureContrast.\n");
          break;
        case NJ_INTERNAL_ERR  :
          PyErr_SetString(st->error, "Error (NJ_INTERNAL_ERR) decoding jpeg data in measureContrast.\n");
          break;
        default :
          PyErr_SetString(st->error, "Error (unknown) decoding jpeg data in measureContrast.\n");
      }
      return NULL;
    }
    
    ret_buf    = njGetImage();
    ret_size   = njGetImageSize();
    
    if (ret_size != (int)(imgheight * imgwidth) *3) {
      njDone();
      struct module_state *st = GETSTATE(self);
      if(xioctl(videv, VIDIOC_QBUF, &buf) == -1) {
        struct module_state *st = GETSTATE(self);
        PyErr_SetString(st->error, "error video buffer in measureContrast\n");
        return NULL;
      }
      PyErr_SetString(st->error, "error imgwidth, imgheight does not match camera format im measureContrast.\n");
      Py_INCREF(Py_None);
      return Py_None;
    }
    
    rwidth  = xbottomright - xtopleft;
    rheight = ybottomright - ytopleft;
    
    rgbp1 = ret_buf + (imgwidth * ytopleft     + xtopleft) * 3;
    rgbp2 = ret_buf + (imgwidth * (ytopleft+1) + xtopleft) * 3;
    pixelcount = (rwidth-1) * (rheight-1);
    for (i=0; i<rheight-1; i++) {
      for (j=0; j<rwidth-6; j++) {
        ave_red   += abs(rgbp1[0] - rgbp1[3]) + abs(rgbp1[0] - rgbp2[0]) + abs(rgbp1[0] - rgbp2[3]);
        ave_green += abs(rgbp1[1] - rgbp1[4]) + abs(rgbp1[1] - rgbp2[1]) + abs(rgbp1[1] - rgbp2[4]);
        ave_blue  += abs(rgbp1[2] - rgbp1[5]) + abs(rgbp1[2] - rgbp2[2]) + abs(rgbp1[2] - rgbp2[5]);
        rgbp1     += 3;
        rgbp2     += 3;
      }
      rgbp1 += (imgwidth - rwidth) * 3;
      rgbp2 += (imgwidth - rwidth) * 3;
    }
    
    if (showImage == 1 && ret_buf != NULL) {
      display_rgb(ret_buf, imgwidth, imgheight);
    }
    njDone();
  }
  
  if(xioctl(videv, VIDIOC_QBUF, &buf) == -1) {
    struct module_state *st = GETSTATE(self);
    PyErr_SetString(st->error, "error video buffer in measureContrast\n");
    return NULL;
  }
  if (pixelcount > 0) {
    return Py_BuildValue("I", ave_red*4/pixelcount + ave_green*4/pixelcount + ave_blue*4/pixelcount);
  } else {
    Py_INCREF(Py_None);
    return Py_None;
  }
  
}

//////////////////////////////////////////////////////////////////////////////////////////
// End of ftrobopytools
//////////////////////////////////////////////////////////////////////////////////////////

