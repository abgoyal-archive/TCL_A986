
void Memory_Init (void *buf, int size);

void Z_Free (void *ptr);
void *Z_Malloc (int size);			// returns 0 filled memory
void *Z_TagMalloc (int size, int tag);

void Z_DumpHeap (void);
void Z_CheckHeap (void);
int Z_FreeMemory (void);

void *Hunk_Alloc (int size);		// returns 0 filled memory
void *Hunk_AllocName (int size, char *name);

void *Hunk_HighAllocName (int size, char *name);

int	Hunk_LowMark (void);
void Hunk_FreeToLowMark (int mark);

int	Hunk_HighMark (void);
void Hunk_FreeToHighMark (int mark);

void *Hunk_TempAlloc (int size);

void Hunk_Check (void);

typedef struct cache_user_s
{
	void	*data;
} cache_user_t;

void Cache_Flush (void);

void *Cache_Check (cache_user_t *c);
// returns the cached data, and moves to the head of the LRU list
// if present, otherwise returns NULL

void Cache_Free (cache_user_t *c);

void *Cache_Alloc (cache_user_t *c, int size, char *name);
// Returns NULL if all purgable data was tossed and there still
// wasn't enough room.

void Cache_Report (void);



