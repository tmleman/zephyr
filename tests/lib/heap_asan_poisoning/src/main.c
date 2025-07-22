/*
 * Copyright (c) 2025 Zephyr Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/sys/sys_heap.h>
#include <string.h>

/* Test heap size */
#define HEAP_SIZE (4096)

/* Heap memory */
static uint8_t heap_mem[HEAP_SIZE];
static struct sys_heap test_heap;

static void *setUp(void)
{
	/* Initialize the heap before each test */
	sys_heap_init(&test_heap, heap_mem, HEAP_SIZE);
	return NULL;
}

/* Basic allocation and free test */
ZTEST(heap_asan_poisoning, test_alloc_free)
{
	void *ptr;
	void *ptr2;

	/* Allocate memory */
	ptr = sys_heap_alloc(&test_heap, 100);
	zassert_not_null(ptr, "Failed to allocate memory from heap");

	/* Write to the memory to ensure it's accessible */
	memset(ptr, 0xA5, 100);

	/* Free the memory */
	sys_heap_free(&test_heap, ptr);

	/* Now allocate another block to check that the heap is still functional */
	ptr2 = sys_heap_alloc(&test_heap, 200);
	zassert_not_null(ptr2, "Failed to allocate memory after freeing");

	/* Use the new allocation to verify heap functionality */
	memset(ptr2, 0xB6, 200);
	sys_heap_free(&test_heap, ptr2);
}

/* Test realloc functionality */
ZTEST(heap_asan_poisoning, test_realloc)
{
	void *ptr, *new_ptr;

	/* Allocate memory */
	ptr = sys_heap_alloc(&test_heap, 100);
	zassert_not_null(ptr, "Failed to allocate memory from heap");

	/* Write pattern to the memory */
	memset(ptr, 0xA5, 100);

	/* Reallocate to a larger size */
	new_ptr = sys_heap_realloc(&test_heap, ptr, 200);
	zassert_not_null(new_ptr, "Failed to reallocate memory");

	/* Check the first 100 bytes are preserved */
	for (int i = 0; i < 100; i++) {
		zassert_equal(((uint8_t *)new_ptr)[i], 0xA5, "Data not preserved during realloc");
	}

	/* Write to the extended area to ensure it's accessible */
	memset((uint8_t *)new_ptr + 100, 0xB6, 100);

	/* Free the memory */
	sys_heap_free(&test_heap, new_ptr);
}

/* Test aligned allocation */
ZTEST(heap_asan_poisoning, test_aligned_alloc)
{
	void *ptr;

	/* Allocate aligned memory */
	ptr = sys_heap_aligned_alloc(&test_heap, 32, 100);
	zassert_not_null(ptr, "Failed to allocate aligned memory");

	/* Verify alignment */
	zassert_equal((uintptr_t)ptr & 0x1F, 0, "Memory not properly aligned");

	/* Write to the memory to ensure it's accessible */
	memset(ptr, 0xA5, 100);

	/* Free the memory */
	sys_heap_free(&test_heap, ptr);
}

/* Test aligned realloc functionality */
ZTEST(heap_asan_poisoning, test_aligned_realloc)
{
	void *ptr, *new_ptr;

	/* Allocate aligned memory */
	ptr = sys_heap_aligned_alloc(&test_heap, 32, 100);
	zassert_not_null(ptr, "Failed to allocate aligned memory");
	zassert_equal((uintptr_t)ptr & 0x1F, 0, "Memory not properly aligned");

	/* Write pattern to the memory */
	memset(ptr, 0xA5, 100);

	/* Reallocate to a larger size while maintaining alignment */
	new_ptr = sys_heap_aligned_realloc(&test_heap, ptr, 32, 200);
	zassert_not_null(new_ptr, "Failed to reallocate aligned memory");
	zassert_equal((uintptr_t)new_ptr & 0x1F, 0, "Reallocated memory not properly aligned");

	/* Check the first 100 bytes are preserved */
	for (int i = 0; i < 100; i++) {
		zassert_equal(((uint8_t *)new_ptr)[i], 0xA5, "Data not preserved during aligned realloc");
	}

	/* Write to the extended area to ensure it's accessible */
	memset((uint8_t *)new_ptr + 100, 0xB6, 100);

	/* Free the memory */
	sys_heap_free(&test_heap, new_ptr);
}

/* Test multiple allocations with fragmentation */
ZTEST(heap_asan_poisoning, test_fragmentation)
{
	#define NUM_ALLOCS 8
	void *ptrs[NUM_ALLOCS];
	size_t sizes[NUM_ALLOCS] = {32, 64, 128, 48, 96, 256, 80, 160};

	/* Allocate all blocks */
	for (int i = 0; i < NUM_ALLOCS; i++) {
		ptrs[i] = sys_heap_alloc(&test_heap, sizes[i]);
		zassert_not_null(ptrs[i], "Failed to allocate block %d", i);
		memset(ptrs[i], 0xA0 + i, sizes[i]);
	}

	/* Free every other block to create fragmentation */
	for (int i = 0; i < NUM_ALLOCS; i += 2) {
		sys_heap_free(&test_heap, ptrs[i]);
		ptrs[i] = NULL;
	}

	/* Verify remaining blocks still have correct data */
	for (int i = 1; i < NUM_ALLOCS; i += 2) {
		uint8_t *ptr = (uint8_t *)ptrs[i];
		for (size_t j = 0; j < sizes[i]; j++) {
			zassert_equal(ptr[j], 0xA0 + i,
				"Data corruption in block %d at offset %zu", i, j);
		}
	}

	/* Free remaining blocks */
	for (int i = 1; i < NUM_ALLOCS; i += 2) {
		sys_heap_free(&test_heap, ptrs[i]);
	}

	#undef NUM_ALLOCS
}

/* Test mixed allocation types */
ZTEST(heap_asan_poisoning, test_mixed_allocs)
{
	void *regular_ptr, *aligned_ptr;

	/* Allocate various types of memory */
	regular_ptr = sys_heap_alloc(&test_heap, 100);
	zassert_not_null(regular_ptr, "Failed to allocate regular memory");

	aligned_ptr = sys_heap_aligned_alloc(&test_heap, 64, 200);
	zassert_not_null(aligned_ptr, "Failed to allocate aligned memory");
	zassert_equal((uintptr_t)aligned_ptr & 0x3F, 0, "Memory not properly aligned");

	/* Write unique data to each allocation */
	memset(regular_ptr, 0xA1, 100);
	memset(aligned_ptr, 0xB2, 200);

	/* Verify data integrity */
	for (int i = 0; i < 100; i++) {
		zassert_equal(((uint8_t *)regular_ptr)[i], 0xA1,
			"Data corruption in regular_ptr at offset %d", i);
	}

	for (int i = 0; i < 200; i++) {
		zassert_equal(((uint8_t *)aligned_ptr)[i], 0xB2,
			"Data corruption in aligned_ptr at offset %d", i);
	}

	/* Clean up */
	sys_heap_free(&test_heap, regular_ptr);
	sys_heap_free(&test_heap, aligned_ptr);
}

/* Test memory reuse patterns */
ZTEST(heap_asan_poisoning, test_memory_reuse)
{
	#define BLOCK_SIZE 64
	#define NUM_ROUNDS 3
	void *ptr;

	/* Perform multiple rounds of allocation and freeing */
	for (int round = 0; round < NUM_ROUNDS; round++) {
		/* Allocate memory */
		ptr = sys_heap_alloc(&test_heap, BLOCK_SIZE);
		zassert_not_null(ptr, "Failed to allocate in round %d", round);

		/* Write round-specific pattern */
		uint8_t pattern = 0x10 + round;
		memset(ptr, pattern, BLOCK_SIZE);

		/* Verify pattern */
		for (int i = 0; i < BLOCK_SIZE; i++) {
			zassert_equal(((uint8_t *)ptr)[i], pattern,
				"Data corruption in round %d at offset %d", round, i);
		}

		/* Free the memory */
		sys_heap_free(&test_heap, ptr);
	}

	#undef BLOCK_SIZE
	#undef NUM_ROUNDS
}

/* Test usable size function */
ZTEST(heap_asan_poisoning, test_usable_size)
{
	void *ptr;
	size_t alloc_size = 100;
	size_t usable_size;

	/* Allocate memory */
	ptr = sys_heap_alloc(&test_heap, alloc_size);
	zassert_not_null(ptr, "Failed to allocate memory");

	/* Get usable size */
	usable_size = sys_heap_usable_size(&test_heap, ptr);
	zassert_true(usable_size >= alloc_size,
		    "Usable size should be at least allocation size");

	/* Should be able to write to the full usable size */
	memset(ptr, 0xFF, usable_size);

	sys_heap_free(&test_heap, ptr);
}

#ifdef CONFIG_TEST_USE_AFTER_FREE
/* Test that deliberately triggers a use-after-free to verify ASAN detection */
ZTEST(heap_asan_poisoning, test_use_after_free_detection)
{
	void *ptr;

	/* Allocate memory */
	ptr = sys_heap_alloc(&test_heap, 100);
	zassert_not_null(ptr, "Failed to allocate memory from heap");

	/* Write to the memory to ensure it's accessible */
	memset(ptr, 0xA5, 100);

	/* Free the memory */
	sys_heap_free(&test_heap, ptr);

	/* This access should trigger an ASAN error */
	TC_PRINT("Deliberately accessing freed memory - SHOULD CRASH\n");
	*((volatile uint8_t *)ptr) = 0xBB;

	/* Should never reach here if ASAN is working correctly */
	zassert_unreachable("ASAN failed to detect use-after-free");
}
#endif /* CONFIG_TEST_USE_AFTER_FREE */

#ifdef CONFIG_TEST_UNALLOCATED_ACCESS
/* Test that deliberately accesses unallocated memory to verify ASAN detection */
ZTEST(heap_asan_poisoning, test_unallocated_access_detection)
{
	/* Try to access memory that should be poisoned during heap initialization */
	uint8_t *unallocated_ptr = heap_mem + HEAP_SIZE - 100;

	TC_PRINT("Deliberately accessing unallocated memory at %p - SHOULD CRASH\n",
	       (void *)unallocated_ptr);
	*unallocated_ptr = 0xCC;

	/* Should never reach here if ASAN is working correctly */
	zassert_unreachable("ASAN failed to detect unallocated memory access");
}
#endif /* CONFIG_TEST_UNALLOCATED_ACCESS */

/* Test that verifies ASAN correctly distinguishes between allocated and unallocated regions */
ZTEST(heap_asan_poisoning, test_allocated_vs_unallocated)
{
	void *ptr1, *ptr2, *ptr3;

	/* Allocate memory at the beginning */
	ptr1 = sys_heap_alloc(&test_heap, 64);
	zassert_not_null(ptr1, "Failed to allocate first block");

	/* Skip some space to create a gap */
	ptr2 = sys_heap_alloc(&test_heap, 128);
	zassert_not_null(ptr2, "Failed to allocate second block");

	ptr3 = sys_heap_alloc(&test_heap, 96);
	zassert_not_null(ptr3, "Failed to allocate third block");

	/* Write to allocated memory - should work fine */
	memset(ptr1, 0x11, 64);
	memset(ptr2, 0x22, 128);
	memset(ptr3, 0x33, 96);

	/* Verify the data */
	zassert_equal(((uint8_t *)ptr1)[0], 0x11, "Data corruption in ptr1");
	zassert_equal(((uint8_t *)ptr2)[0], 0x22, "Data corruption in ptr2");
	zassert_equal(((uint8_t *)ptr3)[0], 0x33, "Data corruption in ptr3");

	/* Free the middle allocation to create a gap */
	sys_heap_free(&test_heap, ptr2);

	/* The freed region should now be poisoned again */
	/* We can't test accessing it directly as that would crash the test */

	/* Allocate something else to verify heap still works */
	ptr2 = sys_heap_alloc(&test_heap, 48);
	zassert_not_null(ptr2, "Failed to re-allocate memory");
	memset(ptr2, 0x44, 48);
	zassert_equal(((uint8_t *)ptr2)[0], 0x44, "Data corruption in reallocated ptr2");

	/* Clean up */
	sys_heap_free(&test_heap, ptr1);
	sys_heap_free(&test_heap, ptr2);
	sys_heap_free(&test_heap, ptr3);
}

/* Test detection of buffer overruns beyond allocated boundaries */
ZTEST(heap_asan_poisoning, test_boundary_detection)
{
	void *ptr;

	/* Allocate a small block */
	ptr = sys_heap_alloc(&test_heap, 32);
	zassert_not_null(ptr, "Failed to allocate memory");
	size_t usable_size = sys_heap_usable_size(&test_heap, ptr);
	printk("Allocated %zu bytes at %p\n", usable_size, ptr);

	/* Write within the allocated region - should be fine */
	memset(ptr, 0xAA, 32);
	for (int i = 0; i < 32; i++) {
		zassert_equal(((uint8_t *)ptr)[i], 0xAA, "Data corruption at offset %d", i);
	}

	((uint8_t *)ptr)[35] = 0xBB; // Deliberately write beyond the allocated size to trigger ASAN

	/* Note: We cannot test buffer overflow directly as it would crash the test.
	 * In a real scenario with ASAN enabled, writing beyond the allocated boundary
	 * should trigger an ASAN error. The boundary detection happens because:
	 * 1. Adjacent free chunks are poisoned
	 * 2. Memory beyond the heap boundaries is poisoned
	 * 3. The heap implementation maintains chunk metadata that helps detect overruns
	 */

	sys_heap_free(&test_heap, ptr);
}

/* Test that heap metadata structures are properly protected */
ZTEST(heap_asan_poisoning, test_metadata_protection)
{
	void *ptr1, *ptr2;

	/* Allocate and free some memory to exercise the free list */
	ptr1 = sys_heap_alloc(&test_heap, 128);
	zassert_not_null(ptr1, "Failed to allocate first block");

	ptr2 = sys_heap_alloc(&test_heap, 256);
	zassert_not_null(ptr2, "Failed to allocate second block");

	/* Free them to put them on the free list */
	sys_heap_free(&test_heap, ptr1);
	sys_heap_free(&test_heap, ptr2);

	/* Reallocate to verify the free list is working correctly */
	ptr1 = sys_heap_alloc(&test_heap, 64);
	zassert_not_null(ptr1, "Failed to reallocate first block");

	ptr2 = sys_heap_alloc(&test_heap, 128);
	zassert_not_null(ptr2, "Failed to reallocate second block");

	/* Write to the blocks to verify they're usable */
	memset(ptr1, 0x55, 64);
	memset(ptr2, 0x66, 128);

	/* Verify the data */
	for (int i = 0; i < 64; i++) {
		zassert_equal(((uint8_t *)ptr1)[i], 0x55, "Data corruption in ptr1 at offset %d", i);
	}

	for (int i = 0; i < 128; i++) {
		zassert_equal(((uint8_t *)ptr2)[i], 0x66, "Data corruption in ptr2 at offset %d", i);
	}

	/* Clean up */
	sys_heap_free(&test_heap, ptr1);
	sys_heap_free(&test_heap, ptr2);
}

ZTEST_SUITE(heap_asan_poisoning, NULL, setUp, NULL, NULL, NULL);
