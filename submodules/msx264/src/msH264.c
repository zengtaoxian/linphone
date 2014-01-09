/*
mediastreamer2 x264 plugin
Copyright (C) 2006-2010 Belledonne Communications SARL (simon.morlat@linphone.org)

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/

#include "mediastreamer2/msfilter.h"
#include "mediastreamer2/msticker.h"
#include "mediastreamer2/msvideo.h"
#include "mediastreamer2/rfc3984.h"
#include "mediastreamer2/flv.h"
#include <sys/stat.h>
#include <fcntl.h>
#include "vpu_lib.h"
#include "vpu_io.h"
#define STREAM_BUF_SIZE		0x200000
#if 0
#ifdef _MSC_VER
#include <stdint.h>
#endif

#include <x264.h>
#endif
#ifndef VERSION
#define VERSION "1.4.1"
#endif

#if 0
#define RC_MARGIN 10000 /*bits per sec*/


#define SPECIAL_HIGHRES_BUILD_CRF 28
#endif

#define MS_X264_CONF(required_bitrate, bitrate_limit, resolution, fps) \
	{ required_bitrate, bitrate_limit, { MS_VIDEO_SIZE_ ## resolution ## _W, MS_VIDEO_SIZE_ ## resolution ## _H }, fps, NULL }

static const MSVideoConfiguration x264_conf_list[] = {
#if defined(ANDROID) || (TARGET_OS_IPHONE == 1) || defined(__arm__)
	MS_X264_CONF(0, 512000, QVGA, 25)
#else
	MS_X264_CONF(1024000, 1536000, SVGA, 25),
	MS_X264_CONF( 512000, 1024000,  VGA, 25),
	MS_X264_CONF( 256000,  512000,  VGA, 15),
	MS_X264_CONF( 170000,  256000, QVGA, 15),
	MS_X264_CONF( 128000,  170000, QCIF, 10),
	MS_X264_CONF(  64000,  128000, QCIF,  7),
	MS_X264_CONF(      0,   64000, QCIF,  5)
#endif
};

static const MSVideoConfiguration multicore_x264_conf_list[] = {
#if defined(ANDROID) || (TARGET_OS_IPHONE == 1) || defined(__arm__)
	MS_X264_CONF(2048000, 3072000,       UXGA, 15),
	MS_X264_CONF(1024000, 1536000, SXGA_MINUS, 15),
	MS_X264_CONF( 750000, 1024000,        XGA, 15),
	MS_X264_CONF( 500000,  750000,       SVGA, 15),
	MS_X264_CONF( 300000,  500000,        VGA, 12),
	MS_X264_CONF(      0,  300000,       QVGA, 12)
#else
	MS_X264_CONF(1024000, 1536000, SVGA, 25),
	MS_X264_CONF( 512000, 1024000,  VGA, 25),
	MS_X264_CONF( 256000,  512000,  VGA, 15),
	MS_X264_CONF( 170000,  256000, QVGA, 15),
	MS_X264_CONF( 128000,  170000, QCIF, 10),
	MS_X264_CONF(  64000,  128000, QCIF,  7),
	MS_X264_CONF(      0,   64000, QCIF,  5)
#endif
};

#if 0
/* the goal of this small object is to tell when to send I frames at startup:
at 2 and 4 seconds*/
typedef struct VideoStarter{
	uint64_t next_time;
	int i_frame_count;
}VideoStarter;

static void video_starter_init(VideoStarter *vs){
	vs->next_time=0;
	vs->i_frame_count=0;
}

static void video_starter_first_frame(VideoStarter *vs, uint64_t curtime){
	vs->next_time=curtime+2000;
}

static bool_t video_starter_need_i_frame(VideoStarter *vs, uint64_t curtime){
	if (vs->next_time==0) return FALSE;
	if (curtime>=vs->next_time){
		vs->i_frame_count++;
		if (vs->i_frame_count==1){
			vs->next_time+=2000;
		}else{
			vs->next_time=0;
		}
		return TRUE;
	}
	return FALSE;
}
#endif

typedef struct {
    unsigned int    size;
    unsigned char*  data;
}SPSInfo;

typedef struct {
    unsigned int    size;
    unsigned char*  data;
}PPSInfo;


enum {
    MODE420 = 0,
    MODE422 = 1,
    MODE224 = 2,
    MODE444 = 3,
    MODE400 = 4
};

struct frame_buf {
    int addrY;
    int addrCb;
    int addrCr;
    int strideY;
    int strideC;
    int mvColBuf;
    vpu_mem_desc desc;
};

struct x264_encode {
    EncHandle handle;		/* Encoder handle */
    PhysicalAddress phy_bsbuf_addr; /* Physical bitstream buffer */
    unsigned int virt_bsbuf_addr;		/* Virtual bitstream buffer */
    int enc_picwidth;	/* Encoded Picture width */
    int enc_picheight;	/* Encoded Picture height */
    int src_picwidth;        /* Source Picture width */
    int src_picheight;       /* Source Picture height */
    int fbcount;	/* Total number of framebuffers allocated */
    int src_fbid;	/* Index of frame buffer that contains YUV image */
    FrameBuffer *fb; /* frame buffer base given to encoder */
    struct frame_buf **pfbpool; /* allocated fb pointers are stored here */
};

typedef struct _EncData{
#if 0
	x264_t *enc;
	x264_param_t params;
#endif
	int mode;
	uint64_t framenum;
	Rfc3984Context *packer;
	int keyframe_int;
#if 0
	VideoStarter starter;
#endif
	bool_t generate_keyframe;
	const MSVideoConfiguration *vconf_list;
	MSVideoConfiguration vconf;

    FLVStream* flv;
    struct x264_encode *enc;
    vpu_mem_desc mem_desc;
}EncData;

typedef struct FLVDataTag_s
{
	unsigned char   frameType;       //4bit
	unsigned char   codecId;         //4bit
    unsigned char   pktType;         //1B
    unsigned int    compositionTime; //3B
    SPSInfo         *sps;
    PPSInfo         *pps;
}FLVVideoTag;	

typedef struct FLVTag_s 
{
	unsigned char tagType;          //1B
	unsigned int  dataSize;         //3B 
	unsigned int  timeStamp;        //3B
    unsigned char timestampExtended;//1B
    unsigned int  streamID;         //3B
	FLVVideoTag tagData;
}FLVTag;

static inline int getAVCDecoderConfigurationRecordSize(
        int sequenceParameterSetLength, int pictureParameterSetLength)
{
    return (11+sequenceParameterSetLength+pictureParameterSetLength);
}

static inline void writeAVCDecoderConfigurationRecord(FLVStream *flv, 
        unsigned char* sequenceParameterSetNALUnit,  int sequenceParameterSetLength,
        unsigned char* pictureParameterSetNALUnit,    int pictureParameterSetLength) 
{
        putChar(flv,0x01);//configurationVersion
        putChar(flv,sequenceParameterSetNALUnit[1]);//AVCProfileIndication
        putChar(flv,sequenceParameterSetNALUnit[2]);//profile_compatibility
        putChar(flv,sequenceParameterSetNALUnit[3]);//AVCLevelIndication
        putChar(flv,0xFF);//lengthSizeMinusOne
        putChar(flv,0xE1);//numOfSequenceParameterSets
        putUI16(flv, sequenceParameterSetLength);
        writeData(flv,sequenceParameterSetNALUnit,sequenceParameterSetLength);
        putChar(flv,0x01);//numOfPictureParameterSets
	    putUI16(flv, pictureParameterSetLength);
        writeData(flv,pictureParameterSetNALUnit,pictureParameterSetLength);
}

static void write264FlvTag(FLVStream *flv,FLVTag *tag,MSQueue *naluq, MSQueue *naluq_t)
{
    mblk_t *tm;
    int len;
    if(flvFull(flv,tag->dataSize + 15)) {
       // struct timeval tv1,tv2;
       // gettimeofday(&tv2,NULL);
        flvFlush(flv);
       // gettimeofday(&tv1,NULL);
       // ms_message("[INFO] TIME=%d ", tv1.tv_sec*1000 + tv1.tv_usec/1000 - tv2.tv_sec*1000 - tv2.tv_usec/1000);
    }

	FLVVideoTag *dataTag=&(tag->tagData);

	putChar(flv, tag->tagType);
	putUI24(flv, tag->dataSize);
	putUI24(flv, tag->timeStamp);
	putChar(flv, tag->timestampExtended);
	putUI24(flv, tag->streamID);

	putChar(flv, (dataTag->frameType<<4)|dataTag->codecId);
	putChar(flv, dataTag->pktType);
	putUI24(flv, dataTag->compositionTime);
     while((tm=ms_queue_get(naluq))!=NULL){
        len=tm->b_wptr-tm->b_rptr;
	    putUI32(flv, len);
        writeData(flv,tm->b_rptr,len);

        ms_queue_put(naluq_t,dupmsg(tm));
        freemsg(tm);
     }
	putUI32(flv, tag->dataSize + 11);
}
static void writeFirstFlvTag(FLVStream *flv,FLVTag *tag)
{
    if(flvFull(flv,tag->dataSize + 15)) {
        flvFlush(flv);
    }

	FLVVideoTag *dataTag=&(tag->tagData);

	putChar(flv, tag->tagType);
	putUI24(flv, tag->dataSize);
	putUI24(flv, tag->timeStamp);
	putChar(flv, tag->timestampExtended);
	putUI24(flv, tag->streamID);

	putChar(flv, (dataTag->frameType<<4)|dataTag->codecId);
	putChar(flv, dataTag->pktType);
	putUI24(flv, dataTag->compositionTime);
    writeAVCDecoderConfigurationRecord(flv, dataTag->sps->data,dataTag->sps->size, 
                                                dataTag->pps->data,dataTag->pps->size);
	putUI32(flv, tag->dataSize + 11);
}

static inline struct frame_buf *x264_get_framebuf(void)
{
    return (struct frame_buf *)calloc(1,sizeof(struct frame_buf));
}

static inline void x264_put_framebuf(struct frame_buf *fb)
{
    if(fb){
        free(fb);
        fb=NULL;
    }
}

static struct frame_buf *x264_framebuf_alloc(int stdMode, int format, int strideY, int height)
{
    struct frame_buf *fb;
    int err;
    int divX, divY;

    fb = x264_get_framebuf();
    if (fb == NULL)
        return NULL;

    divX = (format == MODE420 || format == MODE422) ? 2 : 1;
    divY = (format == MODE420 || format == MODE224) ? 2 : 1;

    memset(&(fb->desc), 0, sizeof(vpu_mem_desc));
    fb->desc.size = (strideY * height  + strideY / divX * height / divY * 2);
    fb->desc.size += strideY / divX * height / divY;

    err = IOGetPhyMem(&fb->desc);
    if (err) {
        ms_error("MSH264Enc: Frame buffer allocation failure\n");
        memset(&(fb->desc), 0, sizeof(vpu_mem_desc));
        return NULL;
    }

    fb->addrY = fb->desc.phy_addr;
    fb->addrCb = fb->addrY + strideY * height;
    fb->addrCr = fb->addrCb + strideY / divX * height / divY;
    fb->strideY = strideY;
    fb->strideC =  strideY / divX;

    fb->mvColBuf = fb->addrCr + strideY / divX * height / divY;
    fb->desc.virt_uaddr = IOGetVirtMem(&(fb->desc));
    if (fb->desc.virt_uaddr <= 0) {
        IOFreePhyMem(&fb->desc);
        memset(&(fb->desc), 0, sizeof(vpu_mem_desc));
        return NULL;
    }

    return fb;
}

static void x264_framebuf_free(struct frame_buf *fb)
{
    if (fb->desc.virt_uaddr) {
        IOFreeVirtMem(&fb->desc);
    }

    if (fb->desc.phy_addr) {
        IOFreePhyMem(&fb->desc);
    }

    memset(&(fb->desc), 0, sizeof(vpu_mem_desc));
    x264_put_framebuf(fb);
}

static int x264_encoder_open(struct x264_encode* enc,EncData *d)
{
    EncHandle handle = {0};
    EncOpenParam encop = {0};

    int i;
    RetCode ret;

    /* Fill up parameters for encoding */
    encop.bitstreamBuffer = enc->phy_bsbuf_addr;
    encop.bitstreamBufferSize = STREAM_BUF_SIZE;
    encop.bitstreamFormat = STD_AVC;

    enc->src_picwidth = d->vconf.vsize.width;
    enc->src_picheight = d->vconf.vsize.height;

    enc->enc_picwidth = enc->src_picwidth;
    enc->enc_picheight = enc->src_picheight;

    encop.picWidth = enc->enc_picwidth;
    encop.picHeight = enc->enc_picheight;

    encop.frameRateInfo = d->vconf.fps;
    encop.bitRate = 0;
    encop.gopSize = 0;
    encop.slicemode.sliceMode = 1;	/* 0: 1 slice per picture; 1: Multiple slices per picture */
    encop.slicemode.sliceSizeMode = 0; /* 0: silceSize defined by bits; 1: sliceSize defined by MB number*/
    encop.slicemode.sliceSize = 8000;  /* Size of a slice in bits or MB numbers */

    encop.initialDelay = 0;
    encop.vbvBufferSize = 0;        /* 0 = ignore 8 */
    encop.intraRefresh = 0;
    encop.sliceReport = 0;
    encop.mbReport = 0;
    encop.mbQpReport = 0;
    encop.rcIntraQp = -1;
    encop.userQpMax = 0;
    encop.userQpMin = 0;
    encop.userQpMinEnable = 0;
    encop.userQpMaxEnable = 0;

    encop.userGamma = (Uint32)(0.75*32768);         /*  (0*32768 <= gamma <= 1*32768) */
    encop.RcIntervalMode= 1;        /* 0:normal, 1:frame_level, 2:slice_level, 3: user defined Mb_level */
    encop.MbInterval = 0;
    encop.avcIntra16x16OnlyModeEnable = 0;

    encop.ringBufferEnable = 0;
    encop.dynamicAllocEnable = 0;
    encop.chromaInterleave = 0;

    encop.EncStdParam.avcParam.avc_constrainedIntraPredFlag = 0;
    encop.EncStdParam.avcParam.avc_disableDeblk = 1;
    encop.EncStdParam.avcParam.avc_deblkFilterOffsetAlpha = 6;
    encop.EncStdParam.avcParam.avc_deblkFilterOffsetBeta = 0;
    encop.EncStdParam.avcParam.avc_chromaQpOffset = 10;
    encop.EncStdParam.avcParam.avc_audEnable = 0;
    encop.EncStdParam.avcParam.avc_fmoEnable = 0;
    encop.EncStdParam.avcParam.avc_fmoType = 0;
    encop.EncStdParam.avcParam.avc_fmoSliceNum = 1;
    encop.EncStdParam.avcParam.avc_fmoSliceSaveBufSize = 32; /* FMO_SLICE_SAVE_BUF_SIZE */

    ret = vpu_EncOpen(&handle, &encop);
    if (ret != RETCODE_SUCCESS) {
        ms_error("MSH264Enc: Encoder open failed %d\n", ret);
        return -1;
    }

    enc->handle = handle;
    return 0;
}

static void x264_encoder_close(struct x264_encode *enc)
{
    EncOutputInfo outinfo = {0};
    RetCode ret;

    ret = vpu_EncClose(enc->handle);
    if (ret == RETCODE_FRAME_NOT_COMPLETE) {
        vpu_EncGetOutputInfo(enc->handle, &outinfo);
        vpu_EncClose(enc->handle);
    }
}

static int x264_encoder_configure(struct x264_encode* enc)
{
    EncHandle handle = enc->handle;
    EncInitialInfo initinfo = {0};
    RetCode ret;

    ret = vpu_EncGetInitialInfo(handle, &initinfo);
    if (ret != RETCODE_SUCCESS) {
        ms_error("MSH264Enc: Encoder GetInitialInfo failed\n");
        return -1;
    }

    enc->fbcount = enc->src_fbid = initinfo.minFrameBufferCount;

    return 0;
}

static x264_encoder_allocate_framebuffer(struct x264_encode* enc)
{
    EncHandle handle = enc->handle;
    int i, enc_stride, src_stride;
    int src_fbid = enc->src_fbid;
    int fbcount = enc->fbcount;
    RetCode ret;
    FrameBuffer *fb;
    struct frame_buf **pfbpool;

    fb = enc->fb = calloc(fbcount + 1, sizeof(FrameBuffer));
    if (fb == NULL) {
        ms_error("MSH264Enc: Failed to allocate enc->fb\n");
        return -1;
    }

    pfbpool = enc->pfbpool = calloc(fbcount + 1, sizeof(struct frame_buf *));
    if (pfbpool == NULL) {
        ms_error("MSH264Enc: Failed to allocate enc->pfbpool\n");
        free(fb);
        fb=NULL;
        return -1;
    }

    for (i = 0; i < fbcount; i++) {
        pfbpool[i] = x264_framebuf_alloc(STD_AVC, MODE420, (enc->enc_picwidth + 15) & ~15,  (enc->enc_picheight + 15) & ~15);
        if (pfbpool[i] == NULL) {
            fbcount = i;
            goto err1;
        }

        fb[i].bufY = pfbpool[i]->addrY;
        fb[i].bufCb = pfbpool[i]->addrCb;
        fb[i].bufCr = pfbpool[i]->addrCr;
        fb[i].strideY = pfbpool[i]->strideY;
        fb[i].strideC = pfbpool[i]->strideC;
    }

    /* Must be a multiple of 16 */
    enc_stride = (enc->enc_picwidth + 15) & ~15;
    src_stride = (enc->src_picwidth + 15 ) & ~15;

    ret = vpu_EncRegisterFrameBuffer(handle, fb, fbcount, enc_stride, src_stride);
    if (ret != RETCODE_SUCCESS) {
        ms_error("MSH264Enc: Register frame buffer failed\n");
        goto err1;
    }

    /* Allocate a single frame buffer for source frame */
    pfbpool[src_fbid] = x264_framebuf_alloc(STD_AVC, MODE420, enc->src_picwidth,
            enc->src_picheight);
    if (pfbpool[src_fbid] == NULL) {
        ms_error("MSH264Enc: failed to allocate single framebuf\n");
        goto err1;
    }

    fb[src_fbid].bufY = pfbpool[src_fbid]->addrY;
    fb[src_fbid].bufCb = pfbpool[src_fbid]->addrCb;
    fb[src_fbid].bufCr = pfbpool[src_fbid]->addrCr;
    fb[src_fbid].strideY = pfbpool[src_fbid]->strideY;
    fb[src_fbid].strideC = pfbpool[src_fbid]->strideC;
    enc->fbcount++;

    return 0;

err1:
    for (i = 0; i < fbcount; i++) {
        x264_framebuf_free(pfbpool[i]);
    }

    free(fb);
    free(pfbpool);
    fb=NULL;
    pfbpool=NULL;
    return -1;
}

static void x264_encoder_free_framebuffer(struct x264_encode *enc)
{
	int i;

	for (i = 0; i < enc->fbcount; i++) {
		x264_framebuf_free(enc->pfbpool[i]);
	}

    if(enc->fb) {
        free(enc->fb);
        enc->fb=NULL;
    }
    if(enc->pfbpool){
        free(enc->pfbpool);
        enc->pfbpool=NULL;
    }
}

static int x264_encoder_fill_headers(struct x264_encode *enc,MSQueue *nalus,EncData *d,int *sizeAll,SPSInfo *sps,PPSInfo *pps)
{
	EncHeaderParam enchdr_param = {0};
	EncHandle handle = enc->handle;
	RetCode ret;
	int mbPicNum;
	mblk_t *m;

	unsigned int vbuf;
	unsigned int phy_bsbuf  = enc->phy_bsbuf_addr;
	unsigned int virt_bsbuf = enc->virt_bsbuf_addr;

	/* Must put encode header before encoding */
	enchdr_param.headerType = SPS_RBSP;
	vpu_EncGiveCommand(handle, ENC_PUT_AVC_HEADER, &enchdr_param);

	vbuf = (virt_bsbuf + enchdr_param.buf - phy_bsbuf);
	m=allocb(enchdr_param.size+10,0);
	memcpy(m->b_wptr,(void*)vbuf+4,enchdr_param.size-4);
	m->b_wptr+=enchdr_param.size-4;
	ms_queue_put(nalus,m);
    sps->size=enchdr_param.size-4;
    sps->data=m->b_rptr;
    *sizeAll+=enchdr_param.size;

	enchdr_param.headerType = PPS_RBSP;
	vpu_EncGiveCommand(handle, ENC_PUT_AVC_HEADER, &enchdr_param);

	vbuf = (virt_bsbuf + enchdr_param.buf - phy_bsbuf);
	m=allocb(enchdr_param.size+10,0);
	memcpy(m->b_wptr,(void*)vbuf+4,enchdr_param.size-4);
	m->b_wptr+=enchdr_param.size-4;
	ms_queue_put(nalus,m);
    pps->size=enchdr_param.size-4;
    pps->data=m->b_rptr;
   *sizeAll+=enchdr_param.size;

	return 0;
}

static unsigned char hd[4]={0x00,0x00,0x00,0x01};
static void* x264_find_header(void* buf, unsigned int size)
{
    void *ptr=buf;
    while( size>0 && (ptr=memchr(buf,0,size))!=NULL ){
        if(memcmp(ptr,hd,4)==0) {
            return ptr;
        }
        else {
            ptr++;
            size-=ptr-buf;
            buf=ptr;
        }
    }
    return NULL;
}

static void x264_nals_to_file(FLVStream *flv,MSQueue *naluq, MSQueue *naluq_t)
{
    mblk_t *tm;
     while((tm=ms_queue_get(naluq))!=NULL){
        writeData(flv,hd,4);
        writeData(flv,tm->b_rptr,tm->b_wptr-tm->b_rptr);

        ms_queue_put(naluq_t,dupmsg(tm));
        freemsg(tm);
     }
}

static int x264_encoder_start(EncData *d,MSPicture *pic,MSQueue *nalus,int* sizeAll,SPSInfo *sps,PPSInfo *pps,int* key)
{
    struct x264_encode *enc=d->enc;
	EncHandle handle = enc->handle;
	EncParam  enc_param = {0};
	EncOpenParam encop = {0};
	EncOutputInfo outinfo = {0};
	RetCode ret = 0;
	int src_fbid = enc->src_fbid, img_size;
	FrameBuffer *fb = enc->fb;
	struct frame_buf **pfbpool = enc->pfbpool;
	struct frame_buf *pfb;
	unsigned int yuv_addr;
	unsigned int vbuf;
	mblk_t *m;
    int i;

    if(d->framenum==0)
    {
        ret = x264_encoder_fill_headers(enc,nalus,d,sizeAll,sps,pps);
        if (ret) {
            ms_error("MSH264Enc: Encode fill headers failed\n");
            return -1;
        }
    }

	enc_param.sourceFrame = &enc->fb[src_fbid];
	enc_param.quantParam = 23;
    if(d->framenum%100==0)
    {
	    enc_param.forceIPicture = 1;
    }
    else
    {
	    enc_param.forceIPicture = 0;
    }
	enc_param.skipPicture = 0;
	enc_param.enableAutoSkip = 1;

	enc_param.encLeftOffset = 0;
	enc_param.encTopOffset = 0;
	if ((enc_param.encLeftOffset + enc->enc_picwidth) > enc->src_picwidth) {
		ms_error("MSH264Enc: Configure is failure for width and left offset\n");
		return -1;
	}
	if ((enc_param.encTopOffset + enc->enc_picheight) > enc->src_picheight) {
		ms_error("MSH264Enc: Configure is failure for height and top offset\n");
		return -1;
	}

	img_size = enc->src_picwidth * enc->src_picheight * 3 / 2;

	/* The main encoding loop */
    pfb = pfbpool[src_fbid];
    yuv_addr = pfb->addrY + pfb->desc.virt_uaddr -
            pfb->desc.phy_addr;
    memcpy((void *)yuv_addr, pic->planes[0],img_size);

    ret = vpu_EncStartOneFrame(handle, &enc_param);
    if (ret != RETCODE_SUCCESS) {
        ms_error("MSH264Enc: vpu_EncStartOneFrame failed Err code:%d\n",
                                ret);
        goto err2;
    }

    //struct timeval tv1,tv2;
    //gettimeofday(&tv2,NULL);
    while (vpu_IsBusy()) {
        vpu_WaitForInt(200);
    }
    //gettimeofday(&tv1,NULL);
    //ms_message("[INFO] TIME=%d ",tv1.tv_sec*1000 + tv1.tv_usec/1000 - tv2.tv_sec*1000 - tv2.tv_usec/1000);

    ret = vpu_EncGetOutputInfo(handle, &outinfo);
    if (ret != RETCODE_SUCCESS) {
        ms_error("MSH264Enc: vpu_EncGetOutputInfo failed Err code: %d\n",
                                ret);
        goto err2;
    }

    if (outinfo.skipEncoded)
        ms_message("[INFO] Skip encoding one Frame!\n");

    vbuf = (enc->virt_bsbuf_addr + outinfo.bitstreamBuffer
                - enc->phy_bsbuf_addr);

    //ms_message("[INFO] LEN=%d NUM=%d",outinfo.bitstreamSize,outinfo.numOfSlices);

    if( (*((char*)vbuf+4) & 0x1F) ==5 )
    {
        *key=1;
        //ms_message("[KEY]=%x num=%llu",*((char*)vbuf+4),d->framenum);
    }
    for(i=1;i<=outinfo.numOfSlices;i++)
    {
        void *ptr;
        int size;
        if(i==outinfo.numOfSlices)
        {
            m=allocb(outinfo.bitstreamSize+10,0);
            memcpy(m->b_wptr,(void*)vbuf+4,outinfo.bitstreamSize-4);
            m->b_wptr+=outinfo.bitstreamSize-4;
            ms_queue_put(nalus,m);
            *sizeAll+=outinfo.bitstreamSize;
            //ms_message("[INFO] LEN[%d]=%d",i,outinfo.bitstreamSize);
            break;
        }
        ptr=x264_find_header((void*)vbuf+1000,outinfo.bitstreamSize-1000);
        if(ptr==NULL)
        {
            ms_error("MSH264Enc: x264_find_header failed error");
            goto err2;
        }

        size=ptr-(void*)vbuf;
        m=allocb(size+10,0);
        memcpy(m->b_wptr,(void*)vbuf+4,size-4);
        m->b_wptr+=size-4;
        ms_queue_put(nalus,m);
        *sizeAll+=size;

        vbuf=(unsigned int)ptr;
        outinfo.bitstreamSize-=size;
        //ms_message("[INFO] LEN[%d]=%d",i,size);
    }
err2:

	/* For automation of test case */
	if (ret > 0)
		ret = 0;

	return ret;
}
static int rec_close(MSFilter *f, void *arg){
    EncData *s=(EncData*)f->data;
    ms_mutex_lock(&f->lock);
    s->flv->state=Stopped;
    if (s->flv->fd>=0)	{
        flvFlush(s->flv);
        close(s->flv->fd);
        s->flv->fd=-1;
    }
    ms_mutex_unlock(&f->lock);
    return 0;
}

static int rec_open(MSFilter *f, void *arg){
    EncData *s=(EncData*)f->data;
    const char *filename=(const char*)arg;

    if (s->flv->fd>=0) rec_close(f,NULL);
    ms_mutex_lock(&f->lock);
    s->flv->fd=open(filename,O_WRONLY|O_CREAT|O_TRUNC, S_IRUSR|S_IWUSR);
    if (s->flv->fd<0){
        ms_warning("Cannot open %s: %s",filename,strerror(errno));
        ms_mutex_unlock(&f->lock);
        return -1;
    }
    s->framenum=0;    
    s->flv->state=Started;
    ms_mutex_unlock(&f->lock);
    return 0;
}

static void getFirstFlvTag(FLVTag *tag, SPSInfo *sps,PPSInfo *pps)
{	
	FLVVideoTag *dataTag=&(tag->tagData);
	tag->tagType = FLVTAGTYPE_VIDEO;
	tag->timeStamp = 0;
	tag->timestampExtended = 0;
	tag->streamID = 0;
	dataTag->frameType = FLVFRAME_KEY;
	dataTag->codecId = FLVCODEC_AVC;

    dataTag->pktType=0;
    dataTag->sps=sps;
    dataTag->pps=pps;
    tag->dataSize = 5+getAVCDecoderConfigurationRecordSize( sps->size,pps->size);
    dataTag->compositionTime=0;
}

static void get264FlvTag(int timeStamp,FLVTag *tag, int size,int key)
{	
	FLVVideoTag *dataTag=&(tag->tagData);
	tag->tagType = FLVTAGTYPE_VIDEO;
	tag->timeStamp = timeStamp;
	tag->timestampExtended = 0;
	tag->streamID = 0;
    tag->dataSize = size+5;
    if(key)
		dataTag->frameType = FLVFRAME_KEY;
	else
		dataTag->frameType = FLVFRAME_INTER;
	dataTag->codecId = FLVCODEC_AVC;

    dataTag->pktType=1;
    dataTag->compositionTime=0;
}
static void x264_flv_to_file(MSFilter *f,MSQueue *naluq, MSQueue *naluq_t, 
        int sizeAll,SPSInfo *sps,PPSInfo *pps,int key)
{
     FLVTag tag;
     EncData *d=(EncData*)f->data;
     if(d->framenum == 0)
     {
         d->flv->time=f->ticker->time;
         writeFlvHeader(d->flv,FLVFLAG_VIDEO);
         getFirstFlvTag(&tag,sps,pps);
         writeFirstFlvTag(d->flv,&tag);
     }
     get264FlvTag(f->ticker->time - d->flv->time,&tag,sizeAll,key);
     write264FlvTag(d->flv,&tag,naluq,naluq_t);
}
static void enc_init(MSFilter *f){
	EncData *d=ms_new(EncData,1);
	d->enc=NULL;
	d->keyframe_int=10; /*10 seconds */
	d->mode=0;
	d->framenum=0;
	d->generate_keyframe=FALSE;
	d->packer=NULL;
	d->vconf_list = &x264_conf_list[0];
	d->vconf = ms_video_find_best_configuration_for_bitrate(d->vconf_list, 384000);
	d->flv = newFLVStream(2*1024*1024);
	d->enc = ms_new0(struct x264_encode,1);
	f->data=d;
	
    if(vpu_Init(NULL)!=0)
    {
        ms_error("MSH264Enc: VPU Init error\n");
        return ;
    }

    d->mem_desc.size = STREAM_BUF_SIZE;
    if(IOGetPhyMem(&d->mem_desc)!=0) {
        ms_error("MSH264Enc: Unable to obtain physical memory\n");
        return ;
    }

    /* mmap that physical buffer */
    d->enc->virt_bsbuf_addr = IOGetVirtMem(&d->mem_desc);
    if (d->enc->virt_bsbuf_addr <= 0) {
        IOFreePhyMem(&d->mem_desc);
        ms_error("MSH264Enc: Unable to map physical memory\n");
        return ;
    }

    d->enc->phy_bsbuf_addr = d->mem_desc.phy_addr;
}

static void enc_uninit(MSFilter *f){
	EncData *d=(EncData*)f->data;
    IOFreeVirtMem(&d->mem_desc);
    IOFreePhyMem(&d->mem_desc);
    vpu_UnInit();

    if (d->flv->fd>=0)	
        rec_close(f,NULL);
    ms_free(d->flv->data);
    ms_free(d->flv);
    ms_free(d->enc);
	ms_free(d);
}

#if 0
static void apply_bitrate(MSFilter *f){
	EncData *d=(EncData*)f->data;
	x264_param_t *params=&d->params;
	float bitrate;

	bitrate=(float)d->vconf.required_bitrate*0.92;
	if (bitrate>RC_MARGIN)
		bitrate-=RC_MARGIN;
	
	params->rc.i_rc_method = X264_RC_ABR;
	params->rc.i_bitrate=(int)(bitrate/1000);
	params->rc.f_rate_tolerance=0.1;
	params->rc.i_vbv_max_bitrate=(int) ((bitrate+RC_MARGIN/2)/1000);
	params->rc.i_vbv_buffer_size=params->rc.i_vbv_max_bitrate;
	params->rc.f_vbv_buffer_init=0.5;
}
#endif

static void enc_preprocess(MSFilter *f){
	EncData *d=(EncData*)f->data;
#if 0
	x264_param_t *params=&d->params;
#endif
	d->packer=rfc3984_new();
	rfc3984_set_mode(d->packer,d->mode);
	rfc3984_enable_stap_a(d->packer,FALSE);

    d->framenum=0;    

    /* open the encoder */
    if(x264_encoder_open(d->enc,d)!=0) {
        ms_error("MSH264Enc: encoder open error\n");
        return ;
    }

    /* configure the encoder */
    if(x264_encoder_configure(d->enc)!=0) {
        ms_error("MSH264Enc: encoder configure error\n");
        return ;
    }

    /* allocate memory for the frame buffers */
    if(x264_encoder_allocate_framebuffer(d->enc)!=0) {
        ms_error("MSH264Enc: encoder allocate framebuffer error\n");
        return ;
    }
}

#if 0
static void x264_nals_to_msgb(x264_nal_t *xnals, int num_nals, MSQueue * nalus){
	int i;
	mblk_t *m;
	/*int bytes;*/
	for (i=0;i<num_nals;++i){
		m=allocb(xnals[i].i_payload+10,0);
		
		memcpy(m->b_wptr,xnals[i].p_payload+4,xnals[i].i_payload-4);
		m->b_wptr+=xnals[i].i_payload-4;
		if (xnals[i].i_type==7) {
			ms_message("A SPS is being sent.");
		}else if (xnals[i].i_type==8) {
			ms_message("A PPS is being sent.");
		}
		ms_queue_put(nalus,m);
	}
}
#endif

static void enc_process(MSFilter *f){
	EncData *d=(EncData*)f->data;
	uint32_t ts=f->ticker->time*90LL;
    int sizeAll=0,key=0;
    SPSInfo sps={0};
    PPSInfo pps={0};
    mblk_t *im;
    MSPicture pic;
    MSQueue nalus;
    ms_queue_init(&nalus);
    ms_mutex_lock(&f->lock);
    while((im=ms_queue_get(f->inputs[0]))!=NULL){
        if (ms_yuv_buf_init_from_mblk(&pic,im)==0){
            if (x264_encoder_start(d,&pic,&nalus,&sizeAll,&sps,&pps,&key)==0){
                if (d->flv->state==Started){
                    MSQueue nalus_t;
                    ms_queue_init(&nalus_t);
                    x264_flv_to_file(f,&nalus,&nalus_t,sizeAll,&sps,&pps,key);
                    //x264_nals_to_file(d->flv,&nalus,&nalus_t);
                    rfc3984_pack(d->packer,&nalus_t,f->outputs[0],ts);
                }
                else{
                    rfc3984_pack(d->packer,&nalus,f->outputs[0],ts);
                }

                d->framenum++;
            }else{
                ms_error("MSH264Enc: x264_encoder_start error");
            }
        }
        freemsg(im);
    }
    ms_mutex_unlock(&f->lock);
}

static void enc_postprocess(MSFilter *f){
	EncData *d=(EncData*)f->data;
	if(d->packer!=NULL) {
	    rfc3984_destroy(d->packer);
        d->packer=NULL;
    }
	x264_encoder_free_framebuffer(d->enc);
	x264_encoder_close(d->enc);
}

static int enc_get_br(MSFilter *f, void*arg){
	EncData *d=(EncData*)f->data;
	*(int*)arg=d->vconf.required_bitrate;
	return 0;
}

static int enc_set_configuration(MSFilter *f, void *arg) {
	EncData *d = (EncData *)f->data;
	const MSVideoConfiguration *vconf = (const MSVideoConfiguration *)arg;
	if (vconf != &d->vconf) memcpy(&d->vconf, vconf, sizeof(MSVideoConfiguration));

	if (d->vconf.required_bitrate > d->vconf.bitrate_limit)
		d->vconf.required_bitrate = d->vconf.bitrate_limit;
#if 0
	if (d->enc) {
		ms_filter_lock(f);
		apply_bitrate(f);
		if (x264_encoder_reconfig(d->enc, &d->params) != 0) {
			ms_error("x264_encoder_reconfig() failed.");
		}
		ms_filter_unlock(f);
		return 0;
	}
#endif
	ms_message("Video configuration set: bitrate=%dbits/s, fps=%f, vsize=%dx%d", d->vconf.required_bitrate, d->vconf.fps, d->vconf.vsize.width, d->vconf.vsize.height);
	return 0;
}

static int enc_set_br(MSFilter *f, void *arg) {
	MSVideoConfiguration best_vconf;
	EncData *d = (EncData *)f->data;
	int br = *(int *)arg;
	best_vconf = ms_video_find_best_configuration_for_bitrate(d->vconf_list, br);
	enc_set_configuration(f, &best_vconf);
	return 0;
}

static int enc_set_fps(MSFilter *f, void *arg){
	EncData *d=(EncData*)f->data;
	d->vconf.fps=*(float*)arg;
	enc_set_configuration(f, &d->vconf);
	return 0;
}

static int enc_get_fps(MSFilter *f, void *arg){
	EncData *d=(EncData*)f->data;
	*(float*)arg=d->vconf.fps;
	return 0;
}

static int enc_get_vsize(MSFilter *f, void *arg){
	EncData *d=(EncData*)f->data;
	*(MSVideoSize*)arg=d->vconf.vsize;
	return 0;
}

static int enc_set_vsize(MSFilter *f, void *arg){
	MSVideoConfiguration best_vconf;
	EncData *d = (EncData *)f->data;
	MSVideoSize *vs = (MSVideoSize *)arg;
	best_vconf = ms_video_find_best_configuration_for_size(d->vconf_list, *vs);
	d->vconf.vsize = *vs;
	d->vconf.fps = best_vconf.fps;
	d->vconf.bitrate_limit = best_vconf.bitrate_limit;
	enc_set_configuration(f, &d->vconf);
	return 0;
}

static int enc_add_fmtp(MSFilter *f, void *arg){
	EncData *d=(EncData*)f->data;
	const char *fmtp=(const char *)arg;
	char value[12];
	if (fmtp_get_value(fmtp,"packetization-mode",value,sizeof(value))){
		d->mode=atoi(value);
		ms_message("packetization-mode set to %i",d->mode);
	}
	return 0;
}

static int enc_req_vfu(MSFilter *f, void *arg){
	EncData *d=(EncData*)f->data;
	d->generate_keyframe=TRUE;
	return 0;
}

static int enc_get_configuration_list(MSFilter *f, void *data) {
	EncData *d = (EncData *)f->data;
	const MSVideoConfiguration **vconf_list = (const MSVideoConfiguration **)data;
	*vconf_list = d->vconf_list;
	return 0;
}


static MSFilterMethod enc_methods[] = {
	{ MS_FILTER_SET_FPS,                       enc_set_fps                },
	{ MS_FILTER_SET_BITRATE,                   enc_set_br                 },
	{ MS_FILTER_GET_BITRATE,                   enc_get_br                 },
	{ MS_FILTER_GET_FPS,                       enc_get_fps                },
	{ MS_FILTER_GET_VIDEO_SIZE,                enc_get_vsize              },
	{ MS_FILTER_SET_VIDEO_SIZE,                enc_set_vsize              },
	{ MS_FILTER_ADD_FMTP,                      enc_add_fmtp               },
	{ MS_FILTER_REQ_VFU,                       enc_req_vfu                },
	{ MS_VIDEO_ENCODER_REQ_VFU,                enc_req_vfu                },
	{ MS_VIDEO_ENCODER_GET_CONFIGURATION_LIST, enc_get_configuration_list },
	{ MS_VIDEO_ENCODER_SET_CONFIGURATION,      enc_set_configuration      },
    {	MS_FILTER_REC_OPEN	,	rec_open	},
    {	MS_FILTER_REC_CLOSE	,	rec_close	},
    {	0	,			NULL		}
};

#ifndef _MSC_VER

static MSFilterDesc x264_enc_desc={
	.id=MS_FILTER_PLUGIN_ID,
	.name="MSH264Enc",
	.text="A H264 encoder based on x264 project",
	.category=MS_FILTER_ENCODER,
	.enc_fmt="H264",
	.ninputs=1,
	.noutputs=1,
	.init=enc_init,
	.preprocess=enc_preprocess,
	.process=enc_process,
	.postprocess=enc_postprocess,
	.uninit=enc_uninit,
	.methods=enc_methods
};

#else

static MSFilterDesc x264_enc_desc={
	MS_FILTER_PLUGIN_ID,
	"MSH264Enc",
	"A H264 encoder based on x264 project",
	MS_FILTER_ENCODER,
	"H264",
	1,
	1,
	enc_init,
	enc_preprocess,
	enc_process,
	enc_postprocess,
	enc_uninit,
	enc_methods
};

#endif

MS2_PUBLIC void libmsx264_init(void){
	ms_filter_register(&x264_enc_desc);
	ms_message("ms264-" VERSION " plugin registered.");
}

