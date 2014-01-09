/*
mediastreamer2 library - modular sound and video processing and streaming
Copyright (C) 2010  Belledonne Communications SARL 
Author: Simon Morlat <simon.morlat@linphone.org>

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
#include "mediastreamer2/rfc3984.h"
#include "mediastreamer2/msvideo.h"
#include "mediastreamer2/msticker.h"

#include "ffmpeg-priv.h"

#include "ortp/b64.h"
#include "vpu_lib.h"
#include "vpu_io.h"
#include <linux/videodev2.h>

#define STREAM_BUF_SIZE		0x200000
#define PS_SAVE_SIZE		0x080000
#define STREAM_END_SIZE		0
#define STREAM_FILL_SIZE	0x40000
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

struct x264_decode {
	DecHandle handle;
	PhysicalAddress phy_bsbuf_addr;
	PhysicalAddress phy_ps_buf;
	PhysicalAddress phy_slice_buf;
	int phy_slicebuf_size;
	unsigned int virt_bsbuf_addr;
	int picwidth;
	int picheight;
	int stride;
	int fbcount;
	int minFrameBufferCount;
	int extrafb;
	FrameBuffer *fb;
	struct frame_buf **pfbpool;
	Rect picCropRect;
};

typedef struct _DecData{
	mblk_t *sps,*pps;
	Rfc3984Context unpacker;
	MSPicture outbuf;
	unsigned int packet_num;
	uint8_t *bitstream;
	int bitstream_size;
	bool_t first_image_decoded;

    struct x264_decode *dec;
    vpu_mem_desc mem_desc;
    vpu_mem_desc ps_mem_desc;
    vpu_mem_desc slice_mem_desc;
}DecData;

static inline struct frame_buf *x264_get_framebuf(void)
{
    return (struct frame_buf *)calloc(1,sizeof(struct frame_buf));
}

static inline void x264_put_framebuf(struct frame_buf *fb)
{
    if(fb) {
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

static int x264_decoder_open(struct x264_decode *dec)
{
	RetCode ret;
	DecHandle handle = {0};
	DecOpenParam oparam = {0};

	oparam.bitstreamFormat  =STD_AVC;
	oparam.bitstreamBuffer = dec->phy_bsbuf_addr;
	oparam.bitstreamBufferSize = STREAM_BUF_SIZE;
    oparam.filePlayEnable =1;
    
	oparam.reorderEnable = 0;
	oparam.mp4DeblkEnable = 0;
	oparam.chromaInterleave = 0;
	oparam.mp4Class = 0;
	oparam.mjpg_thumbNailDecEnable = 0;

	oparam.psSaveBuffer = dec->phy_ps_buf;
	oparam.psSaveBufferSize = PS_SAVE_SIZE;

	ret = vpu_DecOpen(&handle, &oparam);
	if (ret != RETCODE_SUCCESS) {
		ms_error("MSH264Dec: vpu_DecOpen failed\n");
		return -1;
	}

	dec->handle = handle;
	return 0;
}


static void x264_decoder_close(struct x264_decode *dec)
{
	DecOutputInfo outinfo = {0};
	RetCode ret;

	ret = vpu_DecClose(dec->handle);
	if (ret == RETCODE_FRAME_NOT_COMPLETE) {
		vpu_DecGetOutputInfo(dec->handle, &outinfo);
		vpu_DecClose(dec->handle);
	}
}

static void x264_decoder_free_framebuffer(struct x264_decode *dec)
{
	int i, totalfb;
	totalfb = dec->fbcount + dec->extrafb;

	for (i = 0; i < totalfb; i++) {
		x264_framebuf_free(dec->pfbpool[i]);
	}

	if (dec->fb) {
		free(dec->fb);
		dec->fb = NULL;
	}
	if (dec->pfbpool) {
		free(dec->pfbpool);
		dec->pfbpool = NULL;
	}

	return;
}

int x264_decoder_allocate_framebuffer(struct x264_decode *dec)
{
	DecBufInfo bufinfo;
	int i, fbcount = dec->fbcount, totalfb;
	RetCode ret;
	DecHandle handle = dec->handle;
	FrameBuffer *fb;
	struct frame_buf **pfbpool;
	int stride;

	totalfb = fbcount + dec->extrafb;

	fb = dec->fb = calloc(totalfb, sizeof(FrameBuffer));
	if (fb == NULL) {
		ms_error("MSH264Dec: Failed to allocate fb\n");
		return -1;
	}

	pfbpool = dec->pfbpool = calloc(totalfb, sizeof(struct frame_buf *));
	if (pfbpool == NULL) {
		ms_error("MSH264Dec: Failed to allocate pfbpool\n");
		free(dec->fb);
		dec->fb = NULL;
		return -1;
	}

    for (i = 0; i < totalfb; i++) {
        pfbpool[i] = x264_framebuf_alloc(STD_AVC, MODE420,
                        dec->stride, dec->picheight);
        if (pfbpool[i] == NULL) {
            totalfb = i;
            goto err;
        }

        fb[i].bufY = pfbpool[i]->addrY;
        fb[i].bufCb = pfbpool[i]->addrCb;
        fb[i].bufCr = pfbpool[i]->addrCr;
        fb[i].bufMvCol = pfbpool[i]->mvColBuf;
    }


	stride = ((dec->stride + 15) & ~15);
	bufinfo.avcSliceBufInfo.sliceSaveBuffer = dec->phy_slice_buf;
	bufinfo.avcSliceBufInfo.sliceSaveBufferSize = dec->phy_slicebuf_size;

	/* User needs to fill max suported macro block value of frame as following*/
	bufinfo.maxDecFrmInfo.maxMbX = dec->stride / 16;
	bufinfo.maxDecFrmInfo.maxMbY = dec->picheight / 16;
	bufinfo.maxDecFrmInfo.maxMbNum = dec->stride * dec->picheight / 256;
	ret = vpu_DecRegisterFrameBuffer(handle, fb, fbcount, stride, &bufinfo);
	if (ret != RETCODE_SUCCESS) {
		ms_error("MSH264Dec: Register frame buffer failed\n");
		goto err;
	}

	return 0;

err:
	for (i = 0; i < totalfb; i++) {
		x264_framebuf_free(pfbpool[i]);
	}

	free(dec->fb);
	free(dec->pfbpool);
	dec->fb = NULL;
	dec->pfbpool = NULL;
	return -1;
}

int x264_decoder_parse(struct x264_decode *dec)
{
	DecInitialInfo initinfo = {0};
	DecHandle handle = dec->handle;
	int align, extended_fbcount=2;
	RetCode ret;

	/* Parse bitstream and get width/height/framerate etc */
	vpu_DecSetEscSeqInit(handle, 1);
	ret = vpu_DecGetInitialInfo(handle, &initinfo);
	vpu_DecSetEscSeqInit(handle, 0);
	if (ret != RETCODE_SUCCESS) {
		ms_error("MSH264Dec: vpu_DecGetInitialInfo failed, ret:%d, errorcode:%lu\n",
		         ret, initinfo.errorcode);
		return -1;
	}

	if (initinfo.streamInfoObtained) {
		ms_message("[INFO]\tH.264 Profile: %d Level: %d Interlace: %d\n",
			initinfo.profile, initinfo.level, initinfo.interlace);

	}

	ms_message("[INFO]\tDecoder: width = %d, height = %d, fps = %lu, count = %u\n",
			initinfo.picWidth, initinfo.picHeight,
			initinfo.frameRateInfo,
			initinfo.minFrameBufferCount);

	/*
	 * We suggest to add two more buffers than minFrameBufferCount:
	 *
	 * vpu_DecClrDispFlag is used to control framebuffer whether can be
	 * used for decoder again. One framebuffer dequeue from IPU is delayed
	 * for performance improvement and one framebuffer is delayed for
	 * display flag clear.
	 *
	 * Performance is better when more buffers are used if IPU performance
	 * is bottleneck.
	 *
	 * Two more buffers may be needed for interlace stream from IPU DVI view
	 */
	dec->minFrameBufferCount = initinfo.minFrameBufferCount;
	if (initinfo.interlace)
		dec->fbcount = initinfo.minFrameBufferCount + extended_fbcount + 2;
	else
		dec->fbcount = initinfo.minFrameBufferCount + extended_fbcount;

	dec->picwidth = ((initinfo.picWidth + 15) & ~15);

	align = 16;
	if (initinfo.interlace == 1)
		align = 32;

	dec->picheight = ((initinfo.picHeight + align - 1) & ~(align - 1));

	if ((dec->picwidth == 0) || (dec->picheight == 0))
		return -1;

	/*
	 * Information about H.264 decoder picture cropping rectangle which
	 * presents the offset of top-left point and bottom-right point from
	 * the origin of frame buffer.
	 *
	 * By using these four offset values, host application can easily
	 * detect the position of target output window. When display cropping
	 * is off, the cropping window size will be 0.
	 *
	 * This structure for cropping rectangles is only valid for H.264
	 * decoder case.
	 */

	/* Add non-h264 crop support, assume left=top=0 */
	if ((dec->picwidth > initinfo.picWidth ||
		dec->picheight > initinfo.picHeight) &&
		(!initinfo.picCropRect.left &&
		!initinfo.picCropRect.top &&
		!initinfo.picCropRect.right &&
		!initinfo.picCropRect.bottom)) {
		initinfo.picCropRect.left = 0;
		initinfo.picCropRect.top = 0;
		initinfo.picCropRect.right = initinfo.picWidth;
		initinfo.picCropRect.bottom = initinfo.picHeight;
	}

	ms_message("[INFO]\tCROP left/top/right/bottom %lu %lu %lu %lu\n",
					initinfo.picCropRect.left,
					initinfo.picCropRect.top,
					initinfo.picCropRect.right,
					initinfo.picCropRect.bottom);

	memcpy(&(dec->picCropRect), &(initinfo.picCropRect),
					sizeof(initinfo.picCropRect));

	/* worstSliceSize is in kilo-byte unit */
	dec->phy_slicebuf_size = initinfo.worstSliceSize * 1024;
	dec->stride = dec->picwidth;

	return 0;
}

static int x264_decoder_start(struct x264_decode *dec,MSFilter *f,int size)
{
    DecData *d=(DecData*)f->data;
	DecHandle handle = dec->handle;
	DecOutputInfo outinfo = {0};
	DecParam decparam = {0};
	struct frame_buf **pfbpool = dec->pfbpool;
	struct frame_buf *pfb = NULL;
	RetCode ret;
	unsigned int yuv_addr,img_size;

	decparam.dispReorderBuf = 0;

	decparam.prescanEnable = 0;
	decparam.prescanMode = 0;

	decparam.skipframeMode = 0;
	decparam.skipframeNum = 0;

    decparam.chunkSize=size;
    decparam.picStreamBufferAddr=dec->phy_bsbuf_addr;
	/*
	 * once iframeSearchEnable is enabled, prescanEnable, prescanMode
	 * and skipframeMode options are ignored.
	 */
	decparam.iframeSearchEnable = 0;


    ret = vpu_DecStartOneFrame(handle, &decparam);
    if (ret != RETCODE_SUCCESS) {
        ms_error("MSH264Dec: DecStartOneFrame failed\n");
        return -1;
    }

    while (vpu_IsBusy()) {
        vpu_WaitForInt(200);
    }

    ret = vpu_DecGetOutputInfo(handle, &outinfo);
    if (ret != RETCODE_SUCCESS) {
        ms_error("MSH264Dec: vpu_DecGetOutputInfo failed Err code is %d\n",ret);
        return -1;
    }

    if (outinfo.decodingSuccess == 0) {
        ms_warning("MSH264Dec: Incomplete finish of decoding process.\n");
        return -1;
    }

    if (outinfo.notSufficientPsBuffer) {
        ms_error("MSH264Dec: PS Buffer overflow\n");
        return -1;
    }

    if (outinfo.notSufficientSliceBuffer) {
        ms_error("MSH264Dec: Slice Buffer overflow\n");
        return -1;
    }

    /* BIT don't have picture to be displayed */
    if ( outinfo.indexFrameDisplay <0 ) {
        ms_error("MSH264Dec: VPU doesn't have picture to be displayed, indexFrameDisplay = %d\n", \
                outinfo.indexFrameDisplay);
        return -1;
    }

    pfb = pfbpool[outinfo.indexFrameDisplay];

    yuv_addr = pfb->addrY + pfb->desc.virt_uaddr -
            pfb->desc.phy_addr;

    img_size = dec->picwidth * dec->picheight * 3 / 2;

    {
        //ms_message("[INFO]\tGetting yuv picture of %ix%i",dec->picwidth,dec->picheight);
        mblk_t *yuv_msg=ms_yuv_buf_alloc(&d->outbuf,dec->picwidth,dec->picheight);
        memcpy(d->outbuf.planes[0], (unsigned char *)yuv_addr,img_size);
        ms_queue_put(f->outputs[0],yuv_msg);
    }

    ret = vpu_DecClrDispFlag(handle,outinfo.indexFrameDisplay);
    if (ret)
    {
        ms_error("MSH264Dec: vpu_DecClrDispFlag failed Error code %d\n", ret);
        return -1;
    }

	return 0;
}

static void dec_init(MSFilter *f){
	DecData *d=(DecData*)ms_new(DecData,1);
	d->sps=NULL;
	d->pps=NULL;
	rfc3984_init(&d->unpacker);
	d->packet_num=0;
	d->bitstream_size=65536;
	d->bitstream=ms_malloc0(d->bitstream_size);
    d->dec = ms_new0(struct x264_decode,1);
	f->data=d;

    if(vpu_Init(NULL)!=0)
    {
        ms_error("MSH264Dec: VPU Init error\n");
        return ;
    }

	d->mem_desc.size = STREAM_BUF_SIZE;
    if(IOGetPhyMem(&d->mem_desc)!=0) {
        ms_error("MSH264Dec: Unable to obtain physical memory\n");
        return ;
    }

	if (IOGetVirtMem(&d->mem_desc) <= 0) {
        ms_error("MSH264Dec: Unable to map physical memory\n");
		IOFreePhyMem(&d->mem_desc);
		return ;
	}

	d->dec->phy_bsbuf_addr = d->mem_desc.phy_addr;
	d->dec->virt_bsbuf_addr = d->mem_desc.virt_uaddr;

	d->ps_mem_desc.size = PS_SAVE_SIZE;
	if(IOGetPhyMem(&d->ps_mem_desc)!=0)
    {
		ms_error("MSH264Dec: Unable to obtain physical ps save mem\n");
        return;
    }
	d->dec->phy_ps_buf = d->ps_mem_desc.phy_addr;

}

static void dec_update(DecData *d,int size)
{
	int ret;
	ret = vpu_DecUpdateBitstreamBuffer(d->dec->handle, size);
	if (ret != RETCODE_SUCCESS) {
		ms_error("MSH264Dec: vpu_DecUpdateBitstreamBuffer failed\n");
		return ;
	}

    /* parse the bitstream */
    ret = x264_decoder_parse(d->dec);
    if (ret) {
        ms_error("MSH264Dec: decoder parse failed\n");
        return;
    }

    /* allocate slice buf */
    d->slice_mem_desc.size = d->dec->phy_slicebuf_size;
    ret = IOGetPhyMem(&d->slice_mem_desc);
    if (ret) {
        ms_error("MSH264Dec:Unable to obtain physical slice save mem\n");
        return;
    }

    d->dec->phy_slice_buf = d->slice_mem_desc.phy_addr;

    /* allocate frame buffers */
    ret = x264_decoder_allocate_framebuffer(d->dec);
    if (ret){
        ms_error("MSH264Dec: x264_decoder_allocate_framebuffer error\n");
        return;
    }
}

static void dec_preprocess(MSFilter* f) {
	DecData *s=(DecData*)f->data;
	s->first_image_decoded = FALSE;

    s->packet_num=0;

	if( x264_decoder_open(s->dec)!=0 ){
        ms_error("MSH264Dec: decoder open error\n");
        return ;
    }
}
static void dec_postprocess(MSFilter *f){
	DecData *s=(DecData*)f->data;
	x264_decoder_free_framebuffer(s->dec);
	IOFreePhyMem(&s->slice_mem_desc);
	x264_decoder_close(s->dec);
}

static void dec_uninit(MSFilter *f){
	DecData *d=(DecData*)f->data;
	rfc3984_uninit(&d->unpacker);
	if (d->sps) freemsg(d->sps);
	if (d->pps) freemsg(d->pps);
	ms_free(d->bitstream);

	IOFreePhyMem(&d->ps_mem_desc);
	IOFreeVirtMem(&d->mem_desc);
	IOFreePhyMem(&d->mem_desc);
    vpu_UnInit();

    ms_free(d->dec);
	ms_free(d);
}
#if 0
static mblk_t *get_as_yuvmsg(MSFilter *f, DecData *s, AVFrame *orig){
	AVCodecContext *ctx=&s->av_context;

	if (s->outbuf.w!=ctx->width || s->outbuf.h!=ctx->height){
		if (s->sws_ctx!=NULL){
			sws_freeContext(s->sws_ctx);
			s->sws_ctx=NULL;
			freemsg(s->yuv_msg);
			s->yuv_msg=NULL;
		}
		ms_message("Getting yuv picture of %ix%i",ctx->width,ctx->height);
		s->yuv_msg=ms_yuv_buf_alloc(&s->outbuf,ctx->width,ctx->height);
		s->outbuf.w=ctx->width;
		s->outbuf.h=ctx->height;
		s->sws_ctx=sws_getContext(ctx->width,ctx->height,ctx->pix_fmt,
			ctx->width,ctx->height,PIX_FMT_YUV420P,SWS_FAST_BILINEAR,
                	NULL, NULL, NULL);
	}
#if LIBSWSCALE_VERSION_INT >= AV_VERSION_INT(0,9,0)	
	if (sws_scale(s->sws_ctx,(const uint8_t * const *)orig->data,orig->linesize, 0,
					ctx->height, s->outbuf.planes, s->outbuf.strides)<0){
#else
	if (sws_scale(s->sws_ctx,(uint8_t **)orig->data,orig->linesize, 0,
					ctx->height, s->outbuf.planes, s->outbuf.strides)<0){
#endif
		ms_error("%s: error in sws_scale().",f->desc->name);
	}
	return dupmsg(s->yuv_msg);
}
#endif
static void update_sps(DecData *d, mblk_t *sps){
	if (d->sps)
		freemsg(d->sps);
	d->sps=dupb(sps);
}

static void update_pps(DecData *d, mblk_t *pps){
	if (d->pps)
		freemsg(d->pps);
	if (pps) d->pps=dupb(pps);
	else d->pps=NULL;
}


static bool_t check_sps_change(DecData *d, mblk_t *sps){
	bool_t ret=FALSE;
	if (d->sps){
		ret=(msgdsize(sps)!=msgdsize(d->sps)) || (memcmp(d->sps->b_rptr,sps->b_rptr,msgdsize(sps))!=0);
		if (ret) {
			ms_message("SPS changed ! %i,%i",msgdsize(sps),msgdsize(d->sps));
			update_sps(d,sps);
			update_pps(d,NULL);
		}
	} else {
		ms_message("Receiving first SPS");
		update_sps(d,sps);
	}
	return ret;
}

static bool_t check_pps_change(DecData *d, mblk_t *pps){
	bool_t ret=FALSE;
	if (d->pps){
		ret=(msgdsize(pps)!=msgdsize(d->pps)) || (memcmp(d->pps->b_rptr,pps->b_rptr,msgdsize(pps))!=0);
		if (ret) {
			ms_message("PPS changed ! %i,%i",msgdsize(pps),msgdsize(d->pps));
			update_pps(d,pps);
		}
	}else {
		ms_message("Receiving first PPS");
		update_pps(d,pps);
	}
	return ret;
}


static void enlarge_bitstream(DecData *d, int new_size){
	d->bitstream_size=new_size;
	d->bitstream=ms_realloc(d->bitstream,d->bitstream_size);
}

static int nalusToFrame(DecData *d, MSQueue *naluq, bool_t *new_sps_pps){
	mblk_t *im;
	uint8_t *dst=d->bitstream,*src,*end;
	int nal_len;
	bool_t start_picture=TRUE;
	uint8_t nalu_type;
	*new_sps_pps=FALSE;
	end=d->bitstream+d->bitstream_size;
	while((im=ms_queue_get(naluq))!=NULL){
		src=im->b_rptr;
		nal_len=im->b_wptr-src;
		if (dst+nal_len+100>end){
			int pos=dst-d->bitstream;
			enlarge_bitstream(d, d->bitstream_size+nal_len+100);
			dst=d->bitstream+pos;
			end=d->bitstream+d->bitstream_size;
		}
		if (src[0]==0 && src[1]==0 && src[2]==0 && src[3]==1){
			int size=im->b_wptr-src;
			/*workaround for stupid RTP H264 sender that includes nal markers */
			memcpy(dst,src,size);
			dst+=size;
		}else{
			nalu_type=(*src) & ((1<<5)-1);
			if (nalu_type==7)
				*new_sps_pps=check_sps_change(d,im) || *new_sps_pps;
			if (nalu_type==8)
				*new_sps_pps=check_pps_change(d,im) || *new_sps_pps;
			if (start_picture || nalu_type==7/*SPS*/ || nalu_type==8/*PPS*/ ){
				*dst++=0;
				start_picture=FALSE;
			}
		
			/*prepend nal marker*/
			*dst++=0;
			*dst++=0;
			*dst++=1;
			*dst++=*src++;
			while(src<(im->b_wptr-3)){
				if (src[0]==0 && src[1]==0 && src[2]<3){
					*dst++=0;
					*dst++=0;
					*dst++=3;
					src+=2;
				}
				*dst++=*src++;
			}
			*dst++=*src++;
			*dst++=*src++;
			*dst++=*src++;
		}
		freemsg(im);
	}
	return dst-d->bitstream;
}

static void dec_process(MSFilter *f){
	DecData *d=(DecData*)f->data;
	mblk_t *im;
	MSQueue nalus;
	ms_queue_init(&nalus);
	while((im=ms_queue_get(f->inputs[0]))!=NULL){
		/*push the sps/pps given in sprop-parameter-sets if any*/
		if (d->packet_num==0 && d->sps && d->pps){
			mblk_set_timestamp_info(d->sps,mblk_get_timestamp_info(im));
			mblk_set_timestamp_info(d->pps,mblk_get_timestamp_info(im));
			rfc3984_unpack(&d->unpacker,d->sps,&nalus);
			rfc3984_unpack(&d->unpacker,d->pps,&nalus);
			d->sps=NULL;
			d->pps=NULL;
		}
		rfc3984_unpack(&d->unpacker,im,&nalus);
		if (!ms_queue_empty(&nalus)){
			int size;
			bool_t need_reinit=FALSE;
			size=nalusToFrame(d,&nalus,&need_reinit);
		    memcpy((void *)d->dec->virt_bsbuf_addr, d->bitstream, size);
			if (d->packet_num==0 )
            {
				dec_update(d,size);
            }
		    d->packet_num++;

	        if( x264_decoder_start(d->dec,f,size)==0){
                if (!d->first_image_decoded) {
                    ms_filter_notify_no_arg(f,MS_VIDEO_DECODER_FIRST_IMAGE_DECODED);
                    d->first_image_decoded = TRUE;
                }
            }
            else {
                ms_error("MSH264Dec: x264_decoder_start error\n");
            }
		}
	}
}

static int dec_add_fmtp(MSFilter *f, void *arg){
	DecData *d=(DecData*)f->data;
	const char *fmtp=(const char *)arg;
	char value[256];
	if (fmtp_get_value(fmtp,"sprop-parameter-sets",value,sizeof(value))){
		char * b64_sps=value;
		char * b64_pps=strchr(value,',');
		if (b64_pps){
			*b64_pps='\0';
			++b64_pps;
			ms_message("Got sprop-parameter-sets : sps=%s , pps=%s",b64_sps,b64_pps);
			d->sps=allocb(sizeof(value),0);
			d->sps->b_wptr+=b64_decode(b64_sps,strlen(b64_sps),d->sps->b_wptr,sizeof(value));
			d->pps=allocb(sizeof(value),0);
			d->pps->b_wptr+=b64_decode(b64_pps,strlen(b64_pps),d->pps->b_wptr,sizeof(value));
		}
	}
	return 0;
}

static int reset_first_image(MSFilter* f, void *data) {
	DecData *d=(DecData*)f->data;
	d->first_image_decoded = FALSE;
	return 0;
}

static int dec_get_vsize(MSFilter *f, void *data) {
	DecData *d = (DecData *)f->data;
	MSVideoSize *vsize = (MSVideoSize *)data;
	if (d->first_image_decoded == TRUE) {
		vsize->width = d->outbuf.w;
		vsize->height = d->outbuf.h;
	} else {
		*vsize = MS_VIDEO_SIZE_UNKNOWN;
	}
	return 0;
}

static MSFilterMethod  h264_dec_methods[]={
	{	MS_FILTER_ADD_FMTP	,	dec_add_fmtp	},
	{   MS_VIDEO_DECODER_RESET_FIRST_IMAGE_NOTIFICATION, reset_first_image },
	{	MS_FILTER_GET_VIDEO_SIZE,	dec_get_vsize	},
	{	0			,	NULL	}
};

#ifndef _MSC_VER

MSFilterDesc ms_h264_dec_desc={
	.id=MS_H264_DEC_ID,
	.name="MSH264Dec",
	.text="A H264 decoder based on ffmpeg project.",
	.category=MS_FILTER_DECODER,
	.enc_fmt="H264",
	.ninputs=1,
	.noutputs=1,
	.init=dec_init,
	.preprocess=dec_preprocess,
	.process=dec_process,
    .postprocess=dec_postprocess,
	.uninit=dec_uninit,
	.methods=h264_dec_methods
};

#else


MSFilterDesc ms_h264_dec_desc={
	MS_H264_DEC_ID,
	"MSH264Dec",
	"A H264 decoder based on ffmpeg project.",
	MS_FILTER_DECODER,
	"H264",
	1,
	1,
	dec_init,
	dec_preprocess,
	dec_process,
	NULL,
	dec_uninit,
	h264_dec_methods
};

#endif

MS_FILTER_DESC_EXPORT(ms_h264_dec_desc)

