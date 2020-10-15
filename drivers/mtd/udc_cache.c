#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/hdreg.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/blktrans.h>
#include <linux/mutex.h>
#include <asm/jzsoc.h>

#define CACHE_MAX_NUM 256
#define SECTOR_SIZE 512

//#define UDC_CACHE_DEBUG

#ifdef UDC_CACHE_DEBUG
#define dprintk(a...)   printk(a)
#else
#define dprintk(a...)   while(0){}
#endif

typedef struct {
	unsigned short  CacheState;
	unsigned short  UseCount;
	unsigned short  CacheChange;
	unsigned short  CacheReserve;	
	unsigned int	BlockId;
	unsigned char   *aBlockData;
} SSFDC__LB_CACHE;

#define FREE_CACHE 0
#define PREWRITE_CACHE 2
#define OFTEN_USE_CACHE 3
#define SECTOR_SHIFT          9

#define CACHE_TO_UNCATCH(x) ((unsigned int)x | 0xa0000000)
static unsigned int __aBlockData[SECTOR_SIZE * CACHE_MAX_NUM / 4] __attribute__ ((aligned (32)));
static SSFDC__LB_CACHE ssfdc_cache[CACHE_MAX_NUM];
static unsigned short Cur_CacheCount = 0;
int FlushDataState = 0;

#define DMA_ENABLE 0

#ifdef CONFIG_MTD_UBI
static struct ubi_blktrans_dev *g_udc_ubiblk;	//add by ylyuan for lb_cache

extern int udc_ubiblk_readsect(struct ubi_blktrans_dev *, unsigned long, char *, int);
extern int udc_ubiblk_writesect(struct ubi_blktrans_dev *, unsigned long, char *);
extern struct ubi_blktrans_dev *udc_get_ubiblk(void);
extern void udc_flush_writecache(struct ubi_blktrans_dev *dev);

#define _NAND_LB_Write(pCache) udc_ubiblk_writesect(g_udc_ubiblk, pCache->BlockId,pCache->aBlockData)
#define _NAND_LB_Read(Sector,pBuffer)  udc_ubiblk_readsect(g_udc_ubiblk, Sector, pBuffer, SECTOR_SIZE);

#else /* CONFIG_MTD_UBI */
static struct mtdblk_dev *g_udc_mtdblk;

extern int udc_mtdblock_readsect(struct mtdblk_dev *, unsigned long, char *, int);
extern int udc_mtdblock_writesect(struct mtdblk_dev *, unsigned long, char *);
extern struct mtdblk_dev *udc_get_mtdblk(void);
extern struct mtd_info *udc_get_mtd(void);
extern void udc_flush_cache(struct mtdblk_dev *mtdblk);

#define _NAND_LB_Write(pCache) udc_mtdblock_writesect(g_udc_mtdblk, pCache->BlockId,pCache->aBlockData)
#define _NAND_LB_Read(Sector,pBuffer)  udc_mtdblock_readsect(g_udc_mtdblk, Sector, pBuffer, SECTOR_SIZE);
#endif /* CONFIG_MTD_UBI */

#define DMA_ENABLE 0

#if DMA_ENABLE
#define DMA_CHANNEL 5
#define PHYSADDR(x) virt_to_phys((void *)x)
#else
#define lb_memcpy memcpy
#endif

#if DMA_ENABLE
static void lb_memcpy(void *target,void* source,unsigned int len)
{
	int ch = DMA_CHANNEL;
	if(((unsigned int)source < 0xa0000000) && len)
		dma_cache_wback_inv((unsigned long)source, len);
	if(((unsigned int)target < 0xa0000000) && len)
		dma_cache_wback_inv((unsigned long)target, len);

	REG_DMAC_DSAR(ch) = PHYSADDR((unsigned long)source);       
	REG_DMAC_DTAR(ch) = PHYSADDR((unsigned long)target);       
	REG_DMAC_DTCR(ch) = len / 32;	            	    
	REG_DMAC_DRSR(ch) = DMAC_DRSR_RS_AUTO;	        	
	REG_DMAC_DCMD(ch) = DMAC_DCMD_SAI| DMAC_DCMD_DAI | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32|DMAC_DCMD_DS_32BYTE;
	REG_DMAC_DCCSR(ch) = DMAC_DCCSR_EN | DMAC_DCCSR_NDES;
	while (	REG_DMAC_DTCR(ch) );
}
#endif

static void _NAND_LB_InitCache(void)
{
	int i;
	SSFDC__LB_CACHE *pCache = ssfdc_cache;
#if DMA_ENABLE
	unsigned char * ptr = (unsigned char *)CACHE_TO_UNCATCH(__aBlockData);
#else
	unsigned char * ptr = (unsigned char *)(__aBlockData);
#endif
	for(i = 0;i < CACHE_MAX_NUM;i++)
	{
		pCache->CacheState = FREE_CACHE;
		pCache->UseCount = 0;
		pCache->CacheChange = 0;
		pCache->aBlockData = ptr;
		ptr+=SECTOR_SIZE;
		pCache++;
	}
	Cur_CacheCount = 0;
}

static  SSFDC__LB_CACHE * _NAND_LB_GetFreeCache(void)
{
	int ret = 0;
	SSFDC__LB_CACHE *pCacheInfo = &ssfdc_cache[Cur_CacheCount];
	while(1)
	{
		if(ret >= CACHE_MAX_NUM)
			return 0;
		if(pCacheInfo >= &ssfdc_cache[CACHE_MAX_NUM])
		{
			pCacheInfo = ssfdc_cache;
			Cur_CacheCount = 0;
		}
		
		if(pCacheInfo->CacheState == FREE_CACHE)
		{
			return pCacheInfo;
		}
		pCacheInfo++;
		Cur_CacheCount++;
		ret++;
	}
	return 0;
}

static void _NAND_LB_CloseCACHES(unsigned int sectorstart,unsigned int sectorend)
{
	unsigned int i;
	SSFDC__LB_CACHE *pCache = ssfdc_cache;
	for( i = 0;i < CACHE_MAX_NUM;i++){
		if((pCache->CacheState != FREE_CACHE) && (pCache->BlockId >= sectorstart) && (pCache->BlockId < sectorend)){
			pCache->CacheChange = 0;
			pCache->CacheState = FREE_CACHE;			    
			pCache->UseCount = 0;
		}
		pCache++;
	}
}

static void _NAND_LB_FLUSHCACHES(unsigned int sectorstart,unsigned int sectorend)
{
	unsigned int i;
	SSFDC__LB_CACHE *pCache = ssfdc_cache;
	for( i = 0;i < CACHE_MAX_NUM;i++){
		if((pCache->CacheState != FREE_CACHE) && (pCache->BlockId >= sectorstart) && (pCache->BlockId < sectorend)){
			if(pCache->CacheChange)
				_NAND_LB_Write(pCache);
			pCache->CacheChange = 0;
			pCache->CacheState = FREE_CACHE;			    
			pCache->UseCount = 0;
		}
		pCache++;
		
	}
}

inline static int Get_NAND_CacheFreeCount(void)
{
	SSFDC__LB_CACHE *pCache = ssfdc_cache;
	SSFDC__LB_CACHE *pEndCache = &ssfdc_cache[CACHE_MAX_NUM];
	unsigned int count = 0;
	while(pCache < pEndCache)
	{
		if(pCache->CacheState == FREE_CACHE)
			count++;
		pCache++;
	}
	return count;
	
}

static unsigned int _NAND_LB_PreWiteToNand(SSFDC__LB_CACHE *pCache,unsigned short *count,unsigned int update)
{
	SSFDC__LB_CACHE *pWriteCache;
	SSFDC__LB_CACHE *pEndCache = &ssfdc_cache[CACHE_MAX_NUM];
	unsigned int sector = -1;
	unsigned int flag;
	while(1)
	{
		sector = -1;
		flag = 0;
		pWriteCache = ssfdc_cache;
		while(pWriteCache < pEndCache)
		{
			if(pWriteCache->CacheState == update) //PREWRITE_CACHE
			{
				if(pWriteCache->BlockId < sector)
				{
						sector = pWriteCache->BlockId;
						pCache = pWriteCache;
				}
			}else
				flag++;
			pWriteCache++;
		}
		
		if(flag < CACHE_MAX_NUM)
		{	
			if(pCache->CacheChange)
			{
				_NAND_LB_Write(pCache);
				pCache->CacheChange = 0;
			}
			pCache->CacheState = FREE_CACHE;			    
			pCache->UseCount = 0;
			(*count)++;
		}else
			break;		
	}
	return 0;
}

static void _NAND_LB_OftenToNand(SSFDC__LB_CACHE *pCache,unsigned short *count,unsigned int update)
{
	SSFDC__LB_CACHE *pWriteCache = pCache;
	SSFDC__LB_CACHE *pOldCache = pCache;
	SSFDC__LB_CACHE *pEndCache = &ssfdc_cache[CACHE_MAX_NUM];

	dprintk("%s!\n",__FUNCTION__);
	while(pCache)
	{
		if(pCache->CacheState == OFTEN_USE_CACHE)
		{
			if(pWriteCache->CacheState != OFTEN_USE_CACHE)
				pWriteCache = pCache;
			else if(pWriteCache->UseCount > pCache->UseCount)
			{
				pWriteCache = pCache;
			}
		}
		pCache++;
		if(pCache >= pEndCache)
			break;
	}
	if(pWriteCache->CacheState == OFTEN_USE_CACHE)
	{
		(*count)++;
		if(pWriteCache->CacheChange)
			_NAND_LB_Write(pWriteCache);
		pWriteCache->CacheState = FREE_CACHE;			    
		
		pWriteCache->UseCount = 0;
		pWriteCache->CacheChange = 0;
		if(update != -1)
			update--;
		if(update != 0)
			_NAND_LB_OftenToNand(pOldCache,count,update);
	}
}

static int _NAND_LB_FreeCache(unsigned int update)
{
	unsigned short freecount = 0,totalfree = 0;
	
	freecount = 0;
	_NAND_LB_PreWiteToNand(ssfdc_cache,&freecount,PREWRITE_CACHE);
	
	totalfree += freecount;
	dprintk("free count = %d\n",freecount);
	if(freecount == 0)
	{
		freecount = 0;	
		_NAND_LB_PreWiteToNand(ssfdc_cache,&freecount,OFTEN_USE_CACHE);
		totalfree += freecount;
		update = 0;
	}	
	if(update)
	{	
		if(Get_NAND_CacheFreeCount() < CACHE_MAX_NUM * 1 / 4) // because fat is 4 sector
		{
			freecount = 0;	
			_NAND_LB_PreWiteToNand(ssfdc_cache,&freecount,OFTEN_USE_CACHE);
			totalfree += freecount;
		}
	}
	
	dprintk("Free = %d\r\n",totalfree);
	return totalfree;
}

static int _NAND_LB_GetFromCache(unsigned int Sector, void *pBuffer) {

	SSFDC__LB_CACHE *pCache = &ssfdc_cache[Cur_CacheCount];
	SSFDC__LB_CACHE *pUseCache = 0;
	unsigned short i;
	dprintk("sector = %x pBuffer = %x\n",Sector,pBuffer);
	if(pCache >= &ssfdc_cache[CACHE_MAX_NUM])
		pCache = ssfdc_cache;
	
	i = 0;
	while (1) {
		if(pCache->CacheState != FREE_CACHE)
		{
			if (Sector == pCache->BlockId)  {
				dprintk("Cache is use = %d\r\n",pCache->BlockId);
				pUseCache = pCache;
				pCache->UseCount++;
				if(pCache->UseCount == 0)
					pCache->UseCount = -1;
				pCache->CacheState = OFTEN_USE_CACHE;
			}
		}
		pCache--;
		if(pCache < ssfdc_cache)
			pCache = &ssfdc_cache[CACHE_MAX_NUM - 1];
		
		i++;
		if (i >= CACHE_MAX_NUM) {
			break;  /* Sector not in cache */
		}
	}
	if (pUseCache) {
		dprintk("From Cache %d\r\n",Sector);
		lb_memcpy(pBuffer, pUseCache->aBlockData, SECTOR_SIZE);
		return 0;
	}
	return -1;  
}

static void _NAND_LB_ClearCache(void) {
	
	unsigned short freecount = 0;
	dprintk("Clear Cache\r\n");
	
	_NAND_LB_PreWiteToNand(ssfdc_cache,&freecount,PREWRITE_CACHE);
	_NAND_LB_PreWiteToNand(ssfdc_cache,&freecount,OFTEN_USE_CACHE);
}

static void _NAND_LB_CopyToCache(unsigned int Sector, void *pBuffer,unsigned short rw) 
{
	SSFDC__LB_CACHE *pCache = _NAND_LB_GetFreeCache();
	dprintk("Copy to Cache = 0x%08x 0x%08x\r\n",pCache,ssfdc_cache);
	
	if(!pCache)
	{
		_NAND_LB_FreeCache(rw);
		
		pCache = _NAND_LB_GetFreeCache();
	}
	pCache->BlockId = Sector;
	pCache->CacheState = PREWRITE_CACHE;
	pCache->UseCount = 0;
	pCache->CacheChange = rw;
	
	lb_memcpy(pCache->aBlockData,pBuffer,SECTOR_SIZE);
}


static int _NAND_LB_UpdateInCache(unsigned int Sector, void *pBuffer) {
	short i,ret = 0;
	i =  Cur_CacheCount;
	if(Cur_CacheCount > CACHE_MAX_NUM)
		i = 0;
	while(1)
	{
		if(ret >= CACHE_MAX_NUM)
			return -1;
		if(ssfdc_cache[i].CacheState != FREE_CACHE)
		{
			
			if(ssfdc_cache[i].BlockId == Sector)
			{
				dprintk("UpdateInCache = %d\r\n",Sector);
				ssfdc_cache[i].CacheState = OFTEN_USE_CACHE;
				ssfdc_cache[i].UseCount++;
				ssfdc_cache[i].CacheChange = 1;
				lb_memcpy(ssfdc_cache[i].aBlockData,pBuffer,SECTOR_SIZE);
				return 0;
			}
		}
		i--;
		if(i < 0)
			i = CACHE_MAX_NUM - 1;
		ret++;
	}
	return -1;
}

static int NAND_LB_MultiRead(unsigned int Sector, void *pBuffer,unsigned int SectorCount) 
{
	int i,ret,end;
	void *p;

	dprintk("NAND_LB_MultiRead = %d %d \n",Sector,SectorCount);
	end = Sector + SectorCount;
	_NAND_LB_FLUSHCACHES(Sector,end);

	p = pBuffer;
	for (i = Sector; i < end; i ++)
	{
#ifdef CONFIG_MTD_UBI
		ret = udc_ubiblk_readsect(g_udc_ubiblk, i, p, SECTOR_SIZE);
#else
		ret = udc_mtdblock_readsect(g_udc_mtdblk, i, p, SECTOR_SIZE);
#endif
		p += SECTOR_SIZE;
	}
	return ret;
}

static int NAND_LB_Read(unsigned int Sector, void *pBuffer) 
{
	int x;
#if DMA_ENABLE
	unsigned char *ptr = (unsigned char *)CACHE_TO_UNCATCH(pBuffer);
	dma_cache_wback_inv(pBuffer,SECTOR_SIZE);
#else
	unsigned char *ptr = (unsigned char *)pBuffer;
#endif
	dprintk("LB_Read = %d \n",Sector);
	if(_NAND_LB_GetFromCache(Sector,ptr))
	{
		x =  _NAND_LB_Read(Sector,ptr);
		_NAND_LB_CopyToCache(Sector,ptr,0);
	}
	return 512;
}

static int NAND_LB_MultiWrite(unsigned int Sector, void *pBuffer,unsigned int SectorCount) 
{
	int i,ret;
	unsigned char *p;

	_NAND_LB_CloseCACHES(Sector,Sector + SectorCount);
	p = (unsigned char *)pBuffer;
	for (i = Sector; i < Sector + SectorCount; i ++)
	{
#ifdef CONFIG_MTD_UBI
		ret = udc_ubiblk_writesect(g_udc_ubiblk, i, p);
#else
		ret = udc_mtdblock_writesect(g_udc_mtdblk, i, p);
#endif
		p += 512;
	}
	return ret;
}

static int NAND_LB_Write(unsigned int Sector, void *pBuffer) 
{
#if DMA_ENABLE
	unsigned char *ptr = (unsigned char *)CACHE_TO_UNCATCH(pBuffer);
	dma_cache_wback_inv(pBuffer,SECTOR_SIZE);
#else
	unsigned char *ptr = (unsigned char *)pBuffer;
#endif
	dprintk("LB_Write = %x %x\r\n",Sector,pBuffer);
	if(_NAND_LB_UpdateInCache(Sector,ptr))
	{
		_NAND_LB_CopyToCache(Sector,ptr,1);
	}
	return 512;
}
/*********************************************************************
*
*             Global functions
*
***********************************************************************/

int NAND_LB_Init(void) 
{
	dprintk("UDC CACHE Init \n");
	_NAND_LB_InitCache();	
#ifdef CONFIG_MTD_UBI
	g_udc_ubiblk = udc_get_ubiblk();
#else
	g_udc_mtdblk = udc_get_mtdblk();
#endif
	return 0;
}

int NAND_LB_FLASHCACHE(void) 
{
	dprintk("Flush lb cache !\n");
	_NAND_LB_ClearCache();
//	dprintk("Flush mtd cache !\n");
//	udc_flush_cache(g_udc_mtdblk);
	return 0;
}

#if 0 /* not use actually */
int NAND_FLUSHCACHE(void) 
{
	dprintk("Flush cache !\n");
#ifdef CONFIG_MTD_UBI
	udc_flush_writecache(g_udc_ubiblk);
#else
	udc_flush_cache(g_udc_mtdblk);
#endif
	return 0;
}
#endif

int udc_read(unsigned long long offset, unsigned int len, unsigned char *buf)
{
	unsigned long block,sector,i;

	block = offset >> SECTOR_SHIFT;
	sector = len >> SECTOR_SHIFT;
	dprintk("read dev = ia:%x, s:%d c:%d\r\n",buf,block,sector);

	if (sector <= 8)
	{
		for(i = 0;i < sector; i++)
		{
			NAND_LB_Read(block + i,(void *)(buf));
			buf += 512;
		}
	}
	else
		NAND_LB_MultiRead(block, buf, sector);

	return len;
}

int udc_write(unsigned long long offset, unsigned int len, unsigned char *buf)
{
	unsigned long block,sector,i;

	block = offset >> SECTOR_SHIFT;
	sector = len >> SECTOR_SHIFT;
	dprintk("write dev s:%d c:%d\r\n",block,sector);

	if(sector <= 8)
	{
		for(i = 0;i < sector; i++)		
		{
			NAND_LB_Write(block + i,(void *)(buf));
			buf += 512;
			FlushDataState = 1;
		}
	}else
		NAND_LB_MultiWrite(block,(void *)(buf),sector);
	
	return len;
}

EXPORT_SYMBOL_GPL(udc_write);
EXPORT_SYMBOL_GPL(udc_read);
EXPORT_SYMBOL_GPL(NAND_LB_Init);
EXPORT_SYMBOL_GPL(NAND_LB_FLASHCACHE);
EXPORT_SYMBOL_GPL(FlushDataState);
//EXPORT_SYMBOL_GPL(NAND_FLUSHCACHE);
