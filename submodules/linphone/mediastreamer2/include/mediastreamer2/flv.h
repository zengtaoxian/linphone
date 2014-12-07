#ifndef _flv_h_
#define _flv_h_

#define FLVVERSION_1 1
#define FLVFLAG_VIDEO 0x1
#define FLVFLAG_AUDIO 0x4

#define FLVTAGTYPE_AUDIO 8
#define FLVTAGTYPE_VIDEO 9

#define FLVCODEC_AVC    7

#define FLVFRAME_KEY 0x1
#define FLVFRAME_INTER 0x2

typedef enum{
	Stopped,
	Started
} State;

typedef struct FLVStream_s {
    int fd;
	unsigned int length;
	unsigned int  pos;
	unsigned char *data;
    int time;
    State state;
}FLVStream;

static inline int flvFull(FLVStream *flv, int size)
{
    if(size>=flv->length)
        ms_fatal("MSFileFLV: ***FLVBUFSIZE is too short!***\n");
    return	(flv->pos + size >= flv->length)?1:0;
}

static inline void flvFlush(FLVStream *flv) {
	if(write(flv->fd, flv->data, flv->pos)!=flv->pos)
    {
       ms_warning("MSFileFLV: fail to flvFlush: %s",strerror(errno));
    }
	flv->pos = 0;
}

static inline void writeData(FLVStream *flv, unsigned char *data, int size) {
	memcpy(flv->data + flv->pos, data, size);
	flv->pos += size;
}
static inline void putChar(FLVStream *flv, u_int32_t val) {
	flv->data[flv->pos++]=(val&0xFF);
}
static inline void putUI16(FLVStream *flv, u_int32_t val) {
	putChar(flv, (val >> 8));
	putChar(flv, val);;
}
static inline void putUI24(FLVStream *flv, u_int32_t val) {
	putChar(flv, (val >> 16));
	putUI16(flv, val);
}
static inline void putUI32(FLVStream *flv, u_int32_t val) {
	putChar(flv, (val >> 24));
	putUI24(flv, val);
}

static inline void writeFlvHeader(FLVStream *flv, unsigned char flags) 
{
	putChar(flv, 'F');
	putChar(flv, 'L');
	putChar(flv, 'V');
	putChar(flv, FLVVERSION_1);
	putChar(flv, flags);
	putUI32(flv, 9);
	putUI32(flv, 0);
}
static FLVStream *newFLVStream(unsigned int bufsize) {
	FLVStream *flv;
	flv = (FLVStream *)malloc(sizeof(struct FLVStream_s));
	flv->length = bufsize;
	flv->data = (unsigned char *)malloc(flv->length);
	flv->pos = 0;
    flv->fd  =-1;
    flv->time=-1;
    flv->state=Stopped;
	return flv;
}

#endif
