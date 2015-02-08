

#include <linux/kernel.h>
#include <linux/mm.h>
#include "osd.h"
#include "logging.h"
#include "ring_buffer.h"


/* #defines */


/* Amount of space to write to */
#define BYTES_AVAIL_TO_WRITE(r, w, z) ((w) >= (r)) ? ((z) - ((w) - (r))) : ((r) - (w))


static inline void
GetRingBufferAvailBytes(RING_BUFFER_INFO *rbi, u32 *read, u32 *write)
{
	u32 read_loc, write_loc;

	/* Capture the read/write indices before they changed */
	read_loc = rbi->RingBuffer->ReadIndex;
	write_loc = rbi->RingBuffer->WriteIndex;

	*write = BYTES_AVAIL_TO_WRITE(read_loc, write_loc, rbi->RingDataSize);
	*read = rbi->RingDataSize - *write;
}

static inline u32
GetNextWriteLocation(RING_BUFFER_INFO *RingInfo)
{
	u32 next = RingInfo->RingBuffer->WriteIndex;

	/* ASSERT(next < RingInfo->RingDataSize); */

	return next;
}

static inline void
SetNextWriteLocation(RING_BUFFER_INFO *RingInfo, u32 NextWriteLocation)
{
	RingInfo->RingBuffer->WriteIndex = NextWriteLocation;
}

static inline u32
GetNextReadLocation(RING_BUFFER_INFO *RingInfo)
{
	u32 next = RingInfo->RingBuffer->ReadIndex;

	/* ASSERT(next < RingInfo->RingDataSize); */

	return next;
}

static inline u32
GetNextReadLocationWithOffset(RING_BUFFER_INFO *RingInfo, u32 Offset)
{
	u32 next = RingInfo->RingBuffer->ReadIndex;

	/* ASSERT(next < RingInfo->RingDataSize); */
	next += Offset;
	next %= RingInfo->RingDataSize;

	return next;
}

static inline void
SetNextReadLocation(RING_BUFFER_INFO *RingInfo, u32 NextReadLocation)
{
	RingInfo->RingBuffer->ReadIndex = NextReadLocation;
}


static inline void *
GetRingBuffer(RING_BUFFER_INFO *RingInfo)
{
	return (void *)RingInfo->RingBuffer->Buffer;
}


static inline u32
GetRingBufferSize(RING_BUFFER_INFO *RingInfo)
{
	return RingInfo->RingDataSize;
}

static inline u64
GetRingBufferIndices(RING_BUFFER_INFO *RingInfo)
{
	return (u64)RingInfo->RingBuffer->WriteIndex << 32;
}


void DumpRingInfo(RING_BUFFER_INFO *RingInfo, char *Prefix)
{
	u32 bytesAvailToWrite;
	u32 bytesAvailToRead;

	GetRingBufferAvailBytes(RingInfo,
	&bytesAvailToRead,
	&bytesAvailToWrite);

	DPRINT(VMBUS,
		DEBUG_RING_LVL,
		"%s <<ringinfo %p buffer %p avail write %u "
		"avail read %u read idx %u write idx %u>>",
		Prefix,
		RingInfo,
		RingInfo->RingBuffer->Buffer,
		bytesAvailToWrite,
		bytesAvailToRead,
		RingInfo->RingBuffer->ReadIndex,
		RingInfo->RingBuffer->WriteIndex);
}


/* Internal routines */

static u32
CopyToRingBuffer(
	RING_BUFFER_INFO	*RingInfo,
	u32				StartWriteOffset,
	void				*Src,
	u32				SrcLen);

static u32
CopyFromRingBuffer(
	RING_BUFFER_INFO	*RingInfo,
	void				*Dest,
	u32				DestLen,
	u32				StartReadOffset);



void RingBufferGetDebugInfo(RING_BUFFER_INFO *RingInfo,
			    RING_BUFFER_DEBUG_INFO *DebugInfo)
{
	u32 bytesAvailToWrite;
	u32 bytesAvailToRead;

	if (RingInfo->RingBuffer) {
		GetRingBufferAvailBytes(RingInfo,
					&bytesAvailToRead,
					&bytesAvailToWrite);

		DebugInfo->BytesAvailToRead = bytesAvailToRead;
		DebugInfo->BytesAvailToWrite = bytesAvailToWrite;
		DebugInfo->CurrentReadIndex = RingInfo->RingBuffer->ReadIndex;
		DebugInfo->CurrentWriteIndex = RingInfo->RingBuffer->WriteIndex;
		DebugInfo->CurrentInterruptMask = RingInfo->RingBuffer->InterruptMask;
	}
}


u32 GetRingBufferInterruptMask(RING_BUFFER_INFO *rbi)
{
	return rbi->RingBuffer->InterruptMask;
}

int RingBufferInit(RING_BUFFER_INFO *RingInfo, void *Buffer, u32 BufferLen)
{
	if (sizeof(RING_BUFFER) != PAGE_SIZE)
		return -EINVAL;

	memset(RingInfo, 0, sizeof(RING_BUFFER_INFO));

	RingInfo->RingBuffer = (RING_BUFFER *)Buffer;
	RingInfo->RingBuffer->ReadIndex = RingInfo->RingBuffer->WriteIndex = 0;

	RingInfo->RingSize = BufferLen;
	RingInfo->RingDataSize = BufferLen - sizeof(RING_BUFFER);

	spin_lock_init(&RingInfo->ring_lock);

	return 0;
}

void RingBufferCleanup(RING_BUFFER_INFO *RingInfo)
{
}

int RingBufferWrite(RING_BUFFER_INFO *OutRingInfo,
		    struct scatterlist *sglist, u32 sgcount)
{
	int i = 0;
	u32 byteAvailToWrite;
	u32 byteAvailToRead;
	u32 totalBytesToWrite = 0;

	struct scatterlist *sg;
	volatile u32 nextWriteLocation;
	u64 prevIndices = 0;
	unsigned long flags;

	DPRINT_ENTER(VMBUS);

	for_each_sg(sglist, sg, sgcount, i)
	{
		totalBytesToWrite += sg->length;
	}

	totalBytesToWrite += sizeof(u64);

	spin_lock_irqsave(&OutRingInfo->ring_lock, flags);

	GetRingBufferAvailBytes(OutRingInfo,
				&byteAvailToRead,
				&byteAvailToWrite);

	DPRINT_DBG(VMBUS, "Writing %u bytes...", totalBytesToWrite);

	/* DumpRingInfo(OutRingInfo, "BEFORE "); */

	/* If there is only room for the packet, assume it is full. */
	/* Otherwise, the next time around, we think the ring buffer */
	/* is empty since the read index == write index */
	if (byteAvailToWrite <= totalBytesToWrite) {
		DPRINT_DBG(VMBUS,
			"No more space left on outbound ring buffer "
			"(needed %u, avail %u)",
			totalBytesToWrite,
			byteAvailToWrite);

		spin_unlock_irqrestore(&OutRingInfo->ring_lock, flags);

		DPRINT_EXIT(VMBUS);

		return -1;
	}

	/* Write to the ring buffer */
	nextWriteLocation = GetNextWriteLocation(OutRingInfo);

	for_each_sg(sglist, sg, sgcount, i)
	{
		nextWriteLocation = CopyToRingBuffer(OutRingInfo,
						     nextWriteLocation,
						     sg_virt(sg),
						     sg->length);
	}

	/* Set previous packet start */
	prevIndices = GetRingBufferIndices(OutRingInfo);

	nextWriteLocation = CopyToRingBuffer(OutRingInfo,
					     nextWriteLocation,
					     &prevIndices,
					     sizeof(u64));

	/* Make sure we flush all writes before updating the writeIndex */
	mb();

	/* Now, update the write location */
	SetNextWriteLocation(OutRingInfo, nextWriteLocation);

	/* DumpRingInfo(OutRingInfo, "AFTER "); */

	spin_unlock_irqrestore(&OutRingInfo->ring_lock, flags);

	DPRINT_EXIT(VMBUS);

	return 0;
}


int RingBufferPeek(RING_BUFFER_INFO *InRingInfo, void *Buffer, u32 BufferLen)
{
	u32 bytesAvailToWrite;
	u32 bytesAvailToRead;
	u32 nextReadLocation = 0;
	unsigned long flags;

	spin_lock_irqsave(&InRingInfo->ring_lock, flags);

	GetRingBufferAvailBytes(InRingInfo,
				&bytesAvailToRead,
				&bytesAvailToWrite);

	/* Make sure there is something to read */
	if (bytesAvailToRead < BufferLen) {
		/* DPRINT_DBG(VMBUS,
			"got callback but not enough to read "
			"<avail to read %d read size %d>!!",
			bytesAvailToRead,
			BufferLen); */

		spin_unlock_irqrestore(&InRingInfo->ring_lock, flags);

		return -1;
	}

	/* Convert to byte offset */
	nextReadLocation = GetNextReadLocation(InRingInfo);

	nextReadLocation = CopyFromRingBuffer(InRingInfo,
						Buffer,
						BufferLen,
						nextReadLocation);

	spin_unlock_irqrestore(&InRingInfo->ring_lock, flags);

	return 0;
}


int RingBufferRead(RING_BUFFER_INFO *InRingInfo, void *Buffer,
		   u32 BufferLen, u32 Offset)
{
	u32 bytesAvailToWrite;
	u32 bytesAvailToRead;
	u32 nextReadLocation = 0;
	u64 prevIndices = 0;
	unsigned long flags;

	if (BufferLen <= 0)
		return -EINVAL;

	spin_lock_irqsave(&InRingInfo->ring_lock, flags);

	GetRingBufferAvailBytes(InRingInfo,
				&bytesAvailToRead,
				&bytesAvailToWrite);

	DPRINT_DBG(VMBUS, "Reading %u bytes...", BufferLen);

	/* DumpRingInfo(InRingInfo, "BEFORE "); */

	/* Make sure there is something to read */
	if (bytesAvailToRead < BufferLen) {
		DPRINT_DBG(VMBUS,
			"got callback but not enough to read "
			"<avail to read %d read size %d>!!",
			bytesAvailToRead,
			BufferLen);

		spin_unlock_irqrestore(&InRingInfo->ring_lock, flags);

		return -1;
	}

	nextReadLocation = GetNextReadLocationWithOffset(InRingInfo, Offset);

	nextReadLocation = CopyFromRingBuffer(InRingInfo,
						Buffer,
						BufferLen,
						nextReadLocation);

	nextReadLocation = CopyFromRingBuffer(InRingInfo,
						&prevIndices,
						sizeof(u64),
						nextReadLocation);

	/* Make sure all reads are done before we update the read index since */
	/* the writer may start writing to the read area once the read index */
	/*is updated */
	mb();

	/* Update the read index */
	SetNextReadLocation(InRingInfo, nextReadLocation);

	/* DumpRingInfo(InRingInfo, "AFTER "); */

	spin_unlock_irqrestore(&InRingInfo->ring_lock, flags);

	return 0;
}


static u32
CopyToRingBuffer(
	RING_BUFFER_INFO	*RingInfo,
	u32				StartWriteOffset,
	void				*Src,
	u32				SrcLen)
{
	void *ringBuffer = GetRingBuffer(RingInfo);
	u32 ringBufferSize = GetRingBufferSize(RingInfo);
	u32 fragLen;

	/* wrap-around detected! */
	if (SrcLen > ringBufferSize - StartWriteOffset) {
		DPRINT_DBG(VMBUS, "wrap-around detected!");

		fragLen = ringBufferSize - StartWriteOffset;
		memcpy(ringBuffer + StartWriteOffset, Src, fragLen);
		memcpy(ringBuffer, Src + fragLen, SrcLen - fragLen);
	} else
		memcpy(ringBuffer + StartWriteOffset, Src, SrcLen);

	StartWriteOffset += SrcLen;
	StartWriteOffset %= ringBufferSize;

	return StartWriteOffset;
}


static u32
CopyFromRingBuffer(
	RING_BUFFER_INFO	*RingInfo,
	void				*Dest,
	u32				DestLen,
	u32				StartReadOffset)
{
	void *ringBuffer = GetRingBuffer(RingInfo);
	u32 ringBufferSize = GetRingBufferSize(RingInfo);

	u32 fragLen;

	/* wrap-around detected at the src */
	if (DestLen > ringBufferSize - StartReadOffset) {
		DPRINT_DBG(VMBUS, "src wrap-around detected!");

		fragLen = ringBufferSize - StartReadOffset;

		memcpy(Dest, ringBuffer + StartReadOffset, fragLen);
		memcpy(Dest + fragLen, ringBuffer, DestLen - fragLen);
	} else

		memcpy(Dest, ringBuffer + StartReadOffset, DestLen);


	StartReadOffset += DestLen;
	StartReadOffset %= ringBufferSize;

	return StartReadOffset;
}


/* eof */
