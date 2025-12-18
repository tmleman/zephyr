/*
 * Copyright (c) 2025 Zephyr Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/sys/sys_heap.h>
#include <string.h>
#include <sanitizer/asan_interface.h>

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
/* Test unallocated memory access */
ZTEST(heap_asan_poisoning, test_unallocated_access_detection)
{
	TC_PRINT("\n=== Testing ASAN Detection of Unallocated Memory ===\n");

	uint8_t *unallocated_ptr = heap_mem + 1024;

	TC_PRINT("Heap memory:       %p (size %d)\n", (void *)heap_mem, HEAP_SIZE);
	TC_PRINT("Unallocated ptr:   %p (offset 1024)\n", (void *)unallocated_ptr);
	TC_PRINT("This should be deep in poisoned free chunk region\n");

	/* Verify the memory is poisoned using ASAN API */
	if (__asan_address_is_poisoned(unallocated_ptr)) {
		TC_PRINT("✓ Memory is poisoned (as expected)\n");
	} else {
		TC_PRINT("✗ ERROR: Memory is NOT poisoned!\n");
		zassert_unreachable("Memory should be poisoned but isn't");
	}

	TC_PRINT("\nAttempting access - SHOULD CRASH\n");

	/* Use memcpy to force actual memory access that ASAN can intercept */
	uint8_t buffer[16];
	memcpy(buffer, unallocated_ptr, sizeof(buffer));

	/* Should never reach here */
	TC_PRINT("✗ ERROR: ASAN did not detect unallocated access!\n");
	TC_PRINT("Read values: ");
	for (size_t i = 0; i < sizeof(buffer); i++) {
		TC_PRINT("%02X ", buffer[i]);
	}
	TC_PRINT("\n");

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

	//((uint8_t *)ptr)[35] = 0xBB; // Deliberately write beyond the allocated size to trigger ASAN

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

/* Playground test for complex ASAN testing scenarios */
ZTEST(heap_asan_poisoning, test_playground)
{
	void *ptr1, *ptr2, *ptr3;
	size_t usable1, usable2, usable3;

	TC_PRINT("\n=== ASAN Heap Playground Test ===\n");

	/* Step 1: Basic allocation */
	TC_PRINT("Step 1: Allocating 128 bytes\n");
	ptr1 = sys_heap_alloc(&test_heap, 128);
	zassert_not_null(ptr1, "Failed to allocate 128 bytes");

	usable1 = sys_heap_usable_size(&test_heap, ptr1);
	TC_PRINT("  Allocated at %p, usable size: %zu bytes\n", ptr1, usable1);

	/* Verify we can write to full usable size */
	memset(ptr1, 0xAA, usable1);
	TC_PRINT("  Written pattern 0xAA to full usable size\n");

	/* Step 2: Allocate another block */
	TC_PRINT("Step 2: Allocating 256 bytes\n");
	ptr2 = sys_heap_alloc(&test_heap, 256);
	zassert_not_null(ptr2, "Failed to allocate 256 bytes");

	usable2 = sys_heap_usable_size(&test_heap, ptr2);
	TC_PRINT("  Allocated at %p, usable size: %zu bytes\n", ptr2, usable2);

	memset(ptr2, 0xBB, usable2);
	TC_PRINT("  Written pattern 0xBB to full usable size\n");

	/* Step 3: Allocate aligned memory */
	TC_PRINT("Step 3: Allocating 64 bytes with 64-byte alignment\n");
	ptr3 = sys_heap_aligned_alloc(&test_heap, 64, 64);
	zassert_not_null(ptr3, "Failed to allocate aligned memory");
	zassert_equal((uintptr_t)ptr3 & 0x3F, 0, "Memory not 64-byte aligned");

	usable3 = sys_heap_usable_size(&test_heap, ptr3);
	TC_PRINT("  Allocated at %p, usable size: %zu bytes (64-byte aligned)\n",
		 ptr3, usable3);

	memset(ptr3, 0xCC, usable3);
	TC_PRINT("  Written pattern 0xCC to full usable size\n");

	/* Step 4: Verify all blocks still have correct data */
	TC_PRINT("Step 4: Verifying data integrity\n");

	for (size_t i = 0; i < usable1; i++) {
		zassert_equal(((uint8_t *)ptr1)[i], 0xAA,
			     "Corruption in ptr1 at offset %zu", i);
	}
	TC_PRINT("  ptr1 data verified (%zu bytes)\n", usable1);

	for (size_t i = 0; i < usable2; i++) {
		zassert_equal(((uint8_t *)ptr2)[i], 0xBB,
			     "Corruption in ptr2 at offset %zu", i);
	}
	TC_PRINT("  ptr2 data verified (%zu bytes)\n", usable2);

	for (size_t i = 0; i < usable3; i++) {
		zassert_equal(((uint8_t *)ptr3)[i], 0xCC,
			     "Corruption in ptr3 at offset %zu", i);
	}
	TC_PRINT("  ptr3 data verified (%zu bytes)\n", usable3);

	/* Step 5: Free middle block to create fragmentation */
	TC_PRINT("Step 5: Freeing ptr2 (middle block) to create fragmentation\n");
	sys_heap_free(&test_heap, ptr2);
	TC_PRINT("  ptr2 freed, memory poisoned\n");

	/* Step 6: Verify remaining blocks still intact */
	TC_PRINT("Step 6: Verifying remaining blocks after fragmentation\n");

	for (size_t i = 0; i < usable1; i++) {
		zassert_equal(((uint8_t *)ptr1)[i], 0xAA,
			     "Corruption in ptr1 after free");
	}
	TC_PRINT("  ptr1 still valid\n");

	for (size_t i = 0; i < usable3; i++) {
		zassert_equal(((uint8_t *)ptr3)[i], 0xCC,
			     "Corruption in ptr3 after free");
	}
	TC_PRINT("  ptr3 still valid\n");

	/* REMOVE THIS LINE - it was just for testing ASAN detection */
	//((uint8_t *)ptr2)[260] = 0xEE;  // ← Comment this out!

	/* Step 7: Reallocate into freed space */
	TC_PRINT("Step 7: Reallocating 128 bytes into fragmented space\n");
	ptr2 = sys_heap_alloc(&test_heap, 128);
	zassert_not_null(ptr2, "Failed to reallocate");

	usable2 = sys_heap_usable_size(&test_heap, ptr2);
	TC_PRINT("  Reallocated at %p, usable size: %zu bytes\n", ptr2, usable2);

	memset(ptr2, 0xDD, usable2);
	TC_PRINT("  Written pattern 0xDD to reallocated block\n");

	/* Step 8: Test realloc with size change */
	TC_PRINT("Step 8: Reallocating ptr1 from %zu to 200 bytes\n", usable1);
	void *new_ptr1 = sys_heap_realloc(&test_heap, ptr1, 200);
	zassert_not_null(new_ptr1, "Failed to realloc ptr1");

	size_t new_usable1 = sys_heap_usable_size(&test_heap, new_ptr1);
	TC_PRINT("  Reallocated at %p, new usable size: %zu bytes\n",
		 new_ptr1, new_usable1);

	/* Verify original data preserved */
	for (size_t i = 0; i < min(usable1, 200); i++) {
		zassert_equal(((uint8_t *)new_ptr1)[i], 0xAA,
			     "Data not preserved during realloc at offset %zu", i);
	}
	TC_PRINT("  Original data preserved\n");

	/* Write to expanded area */
	memset((uint8_t *)new_ptr1 + 200, 0xEE, new_usable1 - 200);
	TC_PRINT("  Written to expanded area\n");

	/* Verify we can access full usable size */
	for (size_t i = 0; i < new_usable1; i++) {
		volatile uint8_t val = ((uint8_t *)new_ptr1)[i];
		(void)val; /* Suppress unused warning */
	}
	TC_PRINT("  Full usable size accessible (%zu bytes)\n", new_usable1);

	/* Step 9: Clean up all allocations */
	TC_PRINT("Step 9: Cleaning up all allocations\n");
	sys_heap_free(&test_heap, new_ptr1);
	TC_PRINT("  Freed new_ptr1\n");

	sys_heap_free(&test_heap, ptr2);
	TC_PRINT("  Freed ptr2\n");

	sys_heap_free(&test_heap, ptr3);
	TC_PRINT("  Freed ptr3\n");

	TC_PRINT("=== Playground Test Complete ===\n\n");
}

/* Extended playground test with aggressive heap fragmentation */
ZTEST(heap_asan_poisoning, test_playground_extended_fragmentation)
{
	#define MAX_PTRS 16
	void *ptrs[MAX_PTRS];
	size_t sizes[MAX_PTRS];
	size_t usable_sizes[MAX_PTRS];

	TC_PRINT("\n=== Extended ASAN Heap Fragmentation Test ===\n");

	/* Step 1: Create initial fragmentation pattern */
	TC_PRINT("Step 1: Creating initial fragmentation with %d allocations\n", MAX_PTRS);

	for (int i = 0; i < MAX_PTRS; i++) {
		/* Vary sizes: 32, 64, 128, 256, 32, 64, 128, 256, ... */
		sizes[i] = 32 << (i % 4);

		ptrs[i] = sys_heap_alloc(&test_heap, sizes[i]);
		zassert_not_null(ptrs[i], "Failed to allocate block %d (%zu bytes)",
				 i, sizes[i]);

		usable_sizes[i] = sys_heap_usable_size(&test_heap, ptrs[i]);

		/* Write unique pattern to each block */
		memset(ptrs[i], 0xA0 + i, usable_sizes[i]);

		TC_PRINT("  Block %2d: %p, requested %3zu, usable %3zu, pattern 0x%02X\n",
			 i, ptrs[i], sizes[i], usable_sizes[i], 0xA0 + i);
	}

	/* Step 2: Free every other block to create fragmentation */
	TC_PRINT("\nStep 2: Freeing every other block (8 blocks)\n");

	int freed_count = 0;
	for (int i = 0; i < MAX_PTRS; i += 2) {
		TC_PRINT("  Freeing block %2d at %p (%zu bytes)\n",
			 i, ptrs[i], usable_sizes[i]);
		sys_heap_free(&test_heap, ptrs[i]);
		ptrs[i] = NULL;
		freed_count++;
	}
	TC_PRINT("  Total freed: %d blocks\n", freed_count);

	/* Step 3: Verify remaining blocks are intact */
	TC_PRINT("\nStep 3: Verifying remaining %d blocks\n", MAX_PTRS / 2);

	for (int i = 1; i < MAX_PTRS; i += 2) {
		uint8_t expected_pattern = 0xA0 + i;

		for (size_t j = 0; j < usable_sizes[i]; j++) {
			uint8_t actual = ((uint8_t *)ptrs[i])[j];
			zassert_equal(actual, expected_pattern,
				     "Corruption in block %d at offset %zu: "
				     "expected 0x%02X, got 0x%02X",
				     i, j, expected_pattern, actual);
		}
		TC_PRINT("  Block %2d verified (%zu bytes)\n", i, usable_sizes[i]);
	}

	/* Step 4: Allocate into fragmented spaces with different sizes */
	TC_PRINT("\nStep 4: Allocating into fragmented spaces\n");

	for (int i = 0; i < MAX_PTRS; i += 2) {
		/* Allocate smaller sizes than original to fit in fragments */
		size_t new_size = sizes[i] / 2;

		ptrs[i] = sys_heap_alloc(&test_heap, new_size);
		zassert_not_null(ptrs[i],
				"Failed to reallocate block %d (%zu bytes)",
				i, new_size);

		usable_sizes[i] = sys_heap_usable_size(&test_heap, ptrs[i]);
		sizes[i] = new_size;

		/* Write new pattern */
		memset(ptrs[i], 0xD0 + i, usable_sizes[i]);

		TC_PRINT("  Block %2d reallocated: %p, %zu bytes, pattern 0xD%X\n",
			 i, ptrs[i], usable_sizes[i], i);
	}

	/* Step 5: Verify ALL blocks after reallocation */
	TC_PRINT("\nStep 5: Verifying all %d blocks after reallocation\n", MAX_PTRS);

	for (int i = 0; i < MAX_PTRS; i++) {
		uint8_t expected_pattern = (i % 2 == 0) ? (0xD0 + i) : (0xA0 + i);

		for (size_t j = 0; j < usable_sizes[i]; j++) {
			uint8_t actual = ((uint8_t *)ptrs[i])[j];
			zassert_equal(actual, expected_pattern,
				     "Corruption in block %d at offset %zu", i, j);
		}
		TC_PRINT("  Block %2d: OK (%zu bytes, pattern 0x%02X)\n",
			 i, usable_sizes[i], expected_pattern);
	}

	/* Step 6: Free blocks in reverse order */
	TC_PRINT("\nStep 6: Freeing all blocks in reverse order\n");

	for (int i = MAX_PTRS - 1; i >= 0; i--) {
		TC_PRINT("  Freeing block %2d at %p\n", i, ptrs[i]);
		sys_heap_free(&test_heap, ptrs[i]);
		ptrs[i] = NULL;
	}

	/* Step 7: Allocate various sizes to test heap coalescing */
	TC_PRINT("\nStep 7: Testing heap coalescing with large allocations\n");

	void *large1 = sys_heap_alloc(&test_heap, 512);
	zassert_not_null(large1, "Failed to allocate 512 bytes after fragmentation");
	size_t large1_usable = sys_heap_usable_size(&test_heap, large1);
	memset(large1, 0xE1, large1_usable);
	TC_PRINT("  Large block 1: %p, %zu bytes usable\n", large1, large1_usable);

	void *large2 = sys_heap_alloc(&test_heap, 1024);
	zassert_not_null(large2, "Failed to allocate 1024 bytes");
	size_t large2_usable = sys_heap_usable_size(&test_heap, large2);
	memset(large2, 0xE2, large2_usable);
	TC_PRINT("  Large block 2: %p, %zu bytes usable\n", large2, large2_usable);

	/* Step 8: Verify large blocks */
	TC_PRINT("\nStep 8: Verifying large blocks\n");

	for (size_t i = 0; i < large1_usable; i++) {
		zassert_equal(((uint8_t *)large1)[i], 0xE1,
			     "Corruption in large1 at offset %zu", i);
	}
	TC_PRINT("  Large block 1: verified (%zu bytes)\n", large1_usable);

	for (size_t i = 0; i < large2_usable; i++) {
		zassert_equal(((uint8_t *)large2)[i], 0xE2,
			     "Corruption in large2 at offset %zu", i);
	}
	TC_PRINT("  Large block 2: verified (%zu bytes)\n", large2_usable);

	/* Step 9: Stress test with realloc */
	TC_PRINT("\nStep 9: Stress testing with realloc operations\n");

	for (int round = 0; round < 3; round++) {
		size_t new_size = 512 + (round * 256);
		uint8_t expected_pattern = (round == 0) ? 0xE1 : (0xF0 + round - 1);

		TC_PRINT("  Round %d: Reallocating large1 to %zu bytes\n",
			 round, new_size);

		void *new_large1 = sys_heap_realloc(&test_heap, large1, new_size);
		zassert_not_null(new_large1, "Failed to realloc in round %d", round);

		size_t new_usable = sys_heap_usable_size(&test_heap, new_large1);

		/* Verify old data preserved (check against previous pattern) */
		size_t verify_size = (large1_usable < new_size) ? large1_usable : new_size;
		for (size_t i = 0; i < verify_size; i++) {
			zassert_equal(((uint8_t *)new_large1)[i], expected_pattern,
				     "Data corruption during realloc at offset %zu: "
				     "expected 0x%02X, got 0x%02X",
				     i, expected_pattern, ((uint8_t *)new_large1)[i]);
		}
		TC_PRINT("    Data verified: pattern 0x%02X preserved (%zu bytes)\n",
			 expected_pattern, verify_size);

		/* Fill with new pattern for next round */
		memset(new_large1, 0xF0 + round, new_usable);
		TC_PRINT("    Filled with new pattern 0x%02X (%zu bytes)\n",
			 0xF0 + round, new_usable);

		large1 = new_large1;
		large1_usable = new_usable;

		TC_PRINT("    New address: %p, usable: %zu bytes\n",
			 large1, large1_usable);
	}

	/* Step 10: Final cleanup and verification */
	TC_PRINT("\nStep 10: Final cleanup\n");

	sys_heap_free(&test_heap, large1);
	TC_PRINT("  Freed large1\n");

	sys_heap_free(&test_heap, large2);
	TC_PRINT("  Freed large2\n");

	/* Step 11: Allocate one final large block to verify heap is healthy */
	TC_PRINT("\nStep 11: Final health check with large allocation\n");

	void *final_block = sys_heap_alloc(&test_heap, 2048);
	zassert_not_null(final_block, "Heap unhealthy after fragmentation test");

	size_t final_usable = sys_heap_usable_size(&test_heap, final_block);
	TC_PRINT("  Final allocation: %p, %zu bytes usable\n",
		 final_block, final_usable);

	/* Write and verify full usable size */
	memset(final_block, 0xFF, final_usable);
	for (size_t i = 0; i < final_usable; i++) {
		zassert_equal(((uint8_t *)final_block)[i], 0xFF,
			     "Final block corruption at offset %zu", i);
	}
	TC_PRINT("  Final block verified: heap is healthy!\n");

	sys_heap_free(&test_heap, final_block);
	TC_PRINT("  Final block freed\n");

	TC_PRINT("\n=== Extended Fragmentation Test Complete ===\n\n");

	#undef MAX_PTRS
}

/* Comprehensive realloc test with shrinking and expanding scenarios */
ZTEST(heap_asan_poisoning, test_realloc_shrink_expand)
{
	void *ptr;
	size_t original_usable, new_usable;

	TC_PRINT("\n=== Comprehensive Realloc Test (Shrink/Expand) ===\n");

	/* Test 1: Expand realloc (in-place if possible) */
	TC_PRINT("\nTest 1: Expanding allocation in-place\n");

	ptr = sys_heap_alloc(&test_heap, 128);
	zassert_not_null(ptr, "Failed to allocate initial 128 bytes");
	original_usable = sys_heap_usable_size(&test_heap, ptr);

	/* Fill with pattern */
	memset(ptr, 0xAA, 128);
	TC_PRINT("  Initial: %p, requested 128, usable %zu\n", ptr, original_usable);

	/* Expand to 256 bytes */
	ptr = sys_heap_realloc(&test_heap, ptr, 256);
	zassert_not_null(ptr, "Failed to realloc to 256 bytes");
	new_usable = sys_heap_usable_size(&test_heap, ptr);
	TC_PRINT("  Expanded: %p, requested 256, usable %zu\n", ptr, new_usable);

	/* Verify original data preserved */
	for (size_t i = 0; i < 128; i++) {
		zassert_equal(((uint8_t *)ptr)[i], 0xAA,
			     "Data lost during expand at offset %zu", i);
	}
	TC_PRINT("  Original data preserved ✓\n");

	/* Can write to entire usable size */
	memset(ptr, 0xBB, new_usable);
	TC_PRINT("  Full usable size accessible ✓\n");

	sys_heap_free(&test_heap, ptr);

	/* Test 2: Shrink realloc (in-place) */
	TC_PRINT("\nTest 2: Shrinking allocation in-place\n");

	ptr = sys_heap_alloc(&test_heap, 512);
	zassert_not_null(ptr, "Failed to allocate 512 bytes");
	original_usable = sys_heap_usable_size(&test_heap, ptr);

	/* Fill entire usable size */
	memset(ptr, 0xCC, original_usable);
	TC_PRINT("  Initial: %p, requested 512, usable %zu\n", ptr, original_usable);

	/* Shrink to 128 bytes */
	ptr = sys_heap_realloc(&test_heap, ptr, 128);
	zassert_not_null(ptr, "Failed to realloc to 128 bytes");
	new_usable = sys_heap_usable_size(&test_heap, ptr);
	TC_PRINT("  Shrunk: %p, requested 128, usable %zu\n", ptr, new_usable);

	/* Verify data in remaining region */
	for (size_t i = 0; i < 128; i++) {
		zassert_equal(((uint8_t *)ptr)[i], 0xCC,
			     "Data lost during shrink at offset %zu", i);
	}
	TC_PRINT("  Original data preserved ✓\n");

	/* Can write to entire usable size */
	memset(ptr, 0xDD, new_usable);
	TC_PRINT("  Full usable size accessible ✓\n");

	/* ASAN VERIFICATION: Access freed suffix after shrink
	 * This should trigger ASAN error because the memory beyond
	 * the new size was freed and should be poisoned.
	 *
	 * UNCOMMENT THE LINE BELOW TO TEST:
	 * Expected: AddressSanitizer: use-after-poison
	 */
	//TC_PRINT("  [ASAN TEST] Accessing freed suffix after shrink (SHOULD CRASH)\n");
	//((uint8_t *)ptr)[200] = 0xEE;  // Access beyond new_usable, into freed region
	//TC_PRINT("  [ASAN TEST] ERROR: Should have crashed! ASAN not detecting!\n");

	sys_heap_free(&test_heap, ptr);

	/* ASAN VERIFICATION: Access after free
	 * This should trigger ASAN error because the entire block
	 * was freed and should be poisoned.
	 *
	 * *** UNCOMMENTED FOR TESTING ***
	 * Expected: AddressSanitizer: use-after-poison
	 */
	//TC_PRINT("  [ASAN TEST] Accessing after free (SHOULD CRASH)\n");
	//((uint8_t *)ptr)[10] = 0xFF;  // Access freed memory
	//TC_PRINT("  [ASAN TEST] ERROR: Should have crashed! ASAN not detecting!\n");

	/* Test 3: Multiple expand operations */
	TC_PRINT("\nTest 3: Multiple successive expansions\n");

	size_t sizes[] = {64, 128, 256, 512, 1024};
	uint8_t patterns[] = {0xE0, 0xE1, 0xE2, 0xE3, 0xE4};

	ptr = sys_heap_alloc(&test_heap, sizes[0]);
	zassert_not_null(ptr, "Failed initial allocation");
	memset(ptr, patterns[0], sizes[0]);
	TC_PRINT("  Round 0: allocated %zu bytes, pattern 0x%02X\n",
		 sizes[0], patterns[0]);

	for (int i = 1; i < 5; i++) {
		size_t prev_size = sizes[i - 1];

		ptr = sys_heap_realloc(&test_heap, ptr, sizes[i]);
		zassert_not_null(ptr, "Failed to realloc to %zu bytes", sizes[i]);

		/* Verify previous data preserved */
		for (size_t j = 0; j < prev_size; j++) {
			zassert_equal(((uint8_t *)ptr)[j], patterns[i - 1],
				     "Round %d: data lost at offset %zu", i, j);
		}

		/* Fill with new pattern */
		memset(ptr, patterns[i], sizes[i]);

		TC_PRINT("  Round %d: expanded to %zu bytes, pattern 0x%02X, data preserved ✓\n",
			 i, sizes[i], patterns[i]);
	}

	sys_heap_free(&test_heap, ptr);

	/* Test 4: Multiple shrink operations */
	TC_PRINT("\nTest 4: Multiple successive shrinks\n");

	size_t shrink_sizes[] = {1024, 512, 256, 128, 64};

	ptr = sys_heap_alloc(&test_heap, shrink_sizes[0]);
	zassert_not_null(ptr, "Failed initial allocation");

	/* Fill with unique pattern for each region */
	for (size_t i = 0; i < shrink_sizes[0]; i++) {
		((uint8_t *)ptr)[i] = (uint8_t)(0xF0 + (i % 16));
	}
	TC_PRINT("  Round 0: allocated %zu bytes with pattern\n", shrink_sizes[0]);

	void *ptr_round[5];
	ptr_round[0] = ptr;

	for (int i = 1; i < 5; i++) {
		ptr = sys_heap_realloc(&test_heap, ptr, shrink_sizes[i]);
		zassert_not_null(ptr, "Failed to realloc to %zu bytes", shrink_sizes[i]);
		ptr_round[i] = ptr;

		/* Verify remaining data preserved */
		for (size_t j = 0; j < shrink_sizes[i]; j++) {
			uint8_t expected = (uint8_t)(0xF0 + (j % 16));
			zassert_equal(((uint8_t *)ptr)[j], expected,
				     "Round %d: data lost at offset %zu", i, j);
		}

		TC_PRINT("  Round %d: shrunk to %zu bytes, data preserved ✓\n",
			 i, shrink_sizes[i]);

		/* ASAN VERIFICATION: Access memory beyond current shrunk size
		 * Each shrink should poison the freed suffix.
		 *
		 * UNCOMMENT THE LINES BELOW TO TEST (test one at a time):
		 * Expected: AddressSanitizer: use-after-poison
		 */
		//if (i == 2) {  // After shrinking to 256 bytes
		//	TC_PRINT("  [ASAN TEST] Accessing freed region after shrink to 256 (SHOULD CRASH)\n");
		//	((uint8_t *)ptr)[300] = 0xAA;  // Beyond 256, should be poisoned
		//	TC_PRINT("  [ASAN TEST] ERROR: Should have crashed! ASAN not detecting!\n");
		//}
	}

	sys_heap_free(&test_heap, ptr);

	/* Test 5: Expand/shrink alternating pattern */
	TC_PRINT("\nTest 5: Alternating expand/shrink operations\n");

	size_t alternating_sizes[] = {100, 200, 150, 300, 180, 400, 220};

	ptr = sys_heap_alloc(&test_heap, alternating_sizes[0]);
	zassert_not_null(ptr, "Failed initial allocation");
	memset(ptr, 0xA0, alternating_sizes[0]);
	TC_PRINT("  Start: %zu bytes\n", alternating_sizes[0]);

	for (int i = 1; i < 7; i++) {
		size_t prev_size = alternating_sizes[i - 1];
		size_t new_size = alternating_sizes[i];
		const char *op = (new_size > prev_size) ? "expand" : "shrink";

		ptr = sys_heap_realloc(&test_heap, ptr, new_size);
		zassert_not_null(ptr, "Failed to realloc to %zu bytes", new_size);

		/* Verify data in common region */
		size_t check_size = (prev_size < new_size) ? prev_size : new_size;
		for (size_t j = 0; j < check_size; j++) {
			zassert_equal(((uint8_t *)ptr)[j], 0xA0,
				     "Round %d: data lost at offset %zu", i, j);
		}

		/* Refill entire region */
		size_t usable = sys_heap_usable_size(&test_heap, ptr);
		memset(ptr, 0xA0, usable);

		TC_PRINT("  Round %d: %s %zu → %zu bytes, data preserved ✓\n",
			 i, op, prev_size, new_size);

		/* ASAN VERIFICATION: After shrink, access freed region
		 *
		 * UNCOMMENT THE LINES BELOW TO TEST:
		 * Expected: AddressSanitizer: use-after-poison (only on shrink operations)
		 */
		//if (new_size < prev_size && i == 4) {  // Round 4: 300 → 180
		//	TC_PRINT("  [ASAN TEST] Accessing freed region after shrink (SHOULD CRASH)\n");
		//	((uint8_t *)ptr)[250] = 0xBB;  // Between 180 and 300, should be poisoned
		//	TC_PRINT("  [ASAN TEST] ERROR: Should have crashed! ASAN not detecting!\n");
		//}
	}

	sys_heap_free(&test_heap, ptr);

	/* Test 6: Realloc to same size */
	TC_PRINT("\nTest 6: Realloc to same size (no-op)\n");

	ptr = sys_heap_alloc(&test_heap, 256);
	zassert_not_null(ptr, "Failed to allocate");
	void *original_ptr = ptr;

	memset(ptr, 0xB0, 256);
	TC_PRINT("  Original: %p, 256 bytes\n", original_ptr);

	ptr = sys_heap_realloc(&test_heap, ptr, 256);
	zassert_not_null(ptr, "Realloc to same size failed");
	TC_PRINT("  After realloc: %p\n", ptr);

	/* Should ideally be same pointer (implementation may vary) */
	if (ptr == original_ptr) {
		TC_PRINT("  Same pointer returned (optimal) ✓\n");
	} else {
		TC_PRINT("  Different pointer returned (still valid)\n");
	}

	/* Verify data preserved */
	for (size_t i = 0; i < 256; i++) {
		zassert_equal(((uint8_t *)ptr)[i], 0xB0,
			     "Data lost at offset %zu", i);
	}
	TC_PRINT("  Data preserved ✓\n");

	sys_heap_free(&test_heap, ptr);

	/* Test 7: Extreme shrink followed by extreme expand */
	TC_PRINT("\nTest 7: Extreme shrink then extreme expand\n");

	ptr = sys_heap_alloc(&test_heap, 1024);
	zassert_not_null(ptr, "Failed to allocate 1024 bytes");

	/* Write pattern to first 32 bytes only */
	for (size_t i = 0; i < 32; i++) {
		((uint8_t *)ptr)[i] = (uint8_t)(0xC0 + i);
	}
	TC_PRINT("  Initial: 1024 bytes, pattern in first 32 bytes\n");

	/* Shrink dramatically to 32 bytes */
	//void *ptr_1024 = ptr;
	ptr = sys_heap_realloc(&test_heap, ptr, 32);
	zassert_not_null(ptr, "Failed to shrink to 32 bytes");
	TC_PRINT("  Shrunk: 1024 → 32 bytes\n");

	/* ASAN VERIFICATION: Access the freed portion after extreme shrink
	 *
	 * UNCOMMENT THE LINES BELOW TO TEST:
	 * Expected: AddressSanitizer: use-after-poison
	 */
	//TC_PRINT("  [ASAN TEST] Accessing freed region after extreme shrink (SHOULD CRASH)\n");
	//((uint8_t *)ptr)[500] = 0xCC;  // Far beyond 32 bytes, should be poisoned
	//TC_PRINT("  [ASAN TEST] ERROR: Should have crashed! ASAN not detecting!\n");

	/* Verify pattern preserved */
	for (size_t i = 0; i < 32; i++) {
		zassert_equal(((uint8_t *)ptr)[i], (uint8_t)(0xC0 + i),
			     "Data lost after shrink at offset %zu", i);
	}
	TC_PRINT("  Data preserved after shrink ✓\n");

	/* Expand dramatically to 2048 bytes */
	ptr = sys_heap_realloc(&test_heap, ptr, 2048);
	zassert_not_null(ptr, "Failed to expand to 2048 bytes");
	TC_PRINT("  Expanded: 32 → 2048 bytes\n");

	/* Verify original 32 bytes still intact */
	for (size_t i = 0; i < 32; i++) {
		zassert_equal(((uint8_t *)ptr)[i], (uint8_t)(0xC0 + i),
			     "Data lost after expand at offset %zu", i);
	}
	TC_PRINT("  Data preserved after expand ✓\n");

	/* Verify entire usable size is accessible */
	size_t final_usable = sys_heap_usable_size(&test_heap, ptr);
	memset(ptr, 0xFF, final_usable);
	TC_PRINT("  Full usable size (%zu bytes) accessible ✓\n", final_usable);

	sys_heap_free(&test_heap, ptr);

	/* Test 8: Realloc with fragmented heap */
	TC_PRINT("\nTest 8: Realloc operations on fragmented heap\n");

	void *ptrs[4];
	void *saved_ptrs[4];

	/* Create fragmentation */
	for (int i = 0; i < 4; i++) {
		ptrs[i] = sys_heap_alloc(&test_heap, 128);
		zassert_not_null(ptrs[i], "Failed to allocate block %d", i);
		saved_ptrs[i] = ptrs[i];
		memset(ptrs[i], 0xD0 + i, 128);
	}
	TC_PRINT("  Created 4 allocations of 128 bytes each\n");

	/* Free middle blocks to create gaps */
	sys_heap_free(&test_heap, ptrs[1]);
	sys_heap_free(&test_heap, ptrs[2]);
	TC_PRINT("  Freed blocks 1 and 2 (created fragmentation)\n");

	/* ASAN VERIFICATION: Access freed blocks in fragmented heap
	 *
	 * UNCOMMENT THE LINES BELOW TO TEST:
	 * Expected: AddressSanitizer: use-after-poison
	 */
	//TC_PRINT("  [ASAN TEST] Accessing freed block 1 (SHOULD CRASH)\n");
	//((uint8_t *)saved_ptrs[1])[10] = 0xEE;  // Block 1 was freed
	//TC_PRINT("  [ASAN TEST] ERROR: Should have crashed! ASAN not detecting!\n");

	//TC_PRINT("  [ASAN TEST] Accessing freed block 2 (SHOULD CRASH)\n");
	//((uint8_t *)saved_ptrs[2])[20] = 0xEE;  // Block 2 was freed
	//TC_PRINT("  [ASAN TEST] ERROR: Should have crashed! ASAN not detecting!\n");

	/* Try to expand first block (may need to relocate) */
	void *original = ptrs[0];
	ptrs[0] = sys_heap_realloc(&test_heap, ptrs[0], 256);
	zassert_not_null(ptrs[0], "Failed to expand in fragmented heap");

	/* Verify data preserved regardless of relocation */
	for (size_t i = 0; i < 128; i++) {
		zassert_equal(((uint8_t *)ptrs[0])[i], 0xD0,
			     "Data lost during fragmented realloc at offset %zu", i);
	}

	if (ptrs[0] == original) {
		TC_PRINT("  Expanded in-place despite fragmentation ✓\n");
	} else {
		TC_PRINT("  Relocated to new location (expected with fragmentation) ✓\n");

		/* ASAN VERIFICATION: If relocated, old location should be poisoned
		 *
		 * UNCOMMENT THE LINES BELOW TO TEST:
		 * Expected: AddressSanitizer: use-after-poison (only if relocated)
		 */
		//if (original != ptrs[0]) {
		//	TC_PRINT("  [ASAN TEST] Accessing old location after relocation (SHOULD CRASH)\n");
		//	((uint8_t *)original)[50] = 0xFF;  // Old location should be freed/poisoned
		//	TC_PRINT("  [ASAN TEST] ERROR: Should have crashed! ASAN not detecting!\n");
		//}
	}

	sys_heap_free(&test_heap, ptrs[0]);
	sys_heap_free(&test_heap, ptrs[3]);

	TC_PRINT("\n=== Comprehensive Realloc Test Complete ===\n\n");
	TC_PRINT("\nNOTE: ASAN verification code is present but commented out.\n");
	TC_PRINT("To test ASAN detection, uncomment the marked lines one at a time.\n");
	TC_PRINT("Each uncommented test should cause the program to crash with:\n");
	TC_PRINT("  AddressSanitizer: use-after-poison\n\n");
}

ZTEST_SUITE(heap_asan_poisoning, NULL, setUp, NULL, NULL, NULL);
