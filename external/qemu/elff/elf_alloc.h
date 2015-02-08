


#ifndef ELFF_ELF_ALLOC_H_
#define ELFF_ELF_ALLOC_H_

extern "C" {
#include "qemu-common.h"
}
#include "elff-common.h"

class ElfFile;

/* Alignment mask for blocks, allocated with this allocator. */
#define ELFALLOC_ALIGNMENT_MASK 3

#define ELF_ALLOC_CHUNK_SIZE  (32 * 1024)

typedef struct ElfAllocatorChunk {
  /* Previous chunk in the chain of chunks allocated by ElfAllocator instance.
   * For better allocation performance, ElfAllocator keeps its list of
   * allocated chunks in reverse order (relatively to the chunk allocation
   * sequence). So this field in each chunk references the chunk, allocated
   * just prior this one. This field contains NULL for the first allocated
   * chunk.
   */
  ElfAllocatorChunk*  prev;

  /* Address of the next available block in this chunk. */
  void*               avail;

  /* Chunk size. */
  size_t              size;

  /* Number of bytes that remain available in this chunk. */
  size_t              remains;
} ElfAllocatorChunk;

class ElfAllocator {
 public:
  /* Constructs ElfAllocator instance. */
  ElfAllocator();

  /* Destructs ElfAllocator instance. */
  ~ElfAllocator();

  /* Allocates requested number of bytes for a DWARF object.
   * Param:
   *  size - Number of bytes to allocate. Value passed in this parameter
   *    will be rounded up accordingly to ELFALLOC_ALIGNMENT_MASK value,
   *    simplifying alignment adjustments for the allocated blocks.
   * Return:
   *  Address of allocated block of the requested size on success,
   *  or NULL on failure.
   */
  void* alloc(size_t size);

 protected:
  /* Current chunk to allocate memory from. NOTE: chunks are listed here
   * in reverse order (relatively to the chunk allocation sequence).
   */
  ElfAllocatorChunk*  current_chunk_;
};

class DwarfAllocBase {
 public:
  /* Constructs DwarfAllocBase instance. */
  DwarfAllocBase() {
  }

  /* Destructs DwarfAllocBase instance. */
  virtual ~DwarfAllocBase() {
  }

  /* Main operator new.
   * Implements allocation of objects of derived classes from elf's "chunked"
   * allocator, instantiated in ElfFile object (see ElfAllocator class).
   * Param:
   *  size - Number of bytes to allocate for an instance of the derived class.
   *  elf - ELF file instance that owns the allocating object.
   * Return:
   *  Pointer to the allocated memory on success, or NULL on failure.
   */
  void* operator new(size_t size, const ElfFile* elf);

  /* Overwitten operator delete.
   * Since deleting for chunk-allocated objects is a "no-op", we don't do
   * anything in this operator. We, however, are obliged to implement this
   * operator in order to compliment overwritten operator 'new'.
   */
  void operator delete(void* ptr) {
  }

  /* Overwitten operator delete.
   * Since deleting for chunk-allocated objects is a "no-op", we don't do
   * anything in this operator. We, however, are obliged to implement this
   * operator in order to compliment overwritten operator 'new'.
   */
  void operator delete[](void* ptr) {
  }

 private:
  /* Default operator new.
   * We override it making 'private' in order to cause a compiler error on
   * attempts to instantiate objects of derived classes using this version
   * of operator 'new'.
   */
  void* operator new(size_t size) {
    return NULL;
  }
};

#endif  // ELFF_ELF_ALLOC_H_
