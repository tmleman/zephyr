# Heap ASAN Poisoning Test

This test verifies the integration of Address Sanitizer (ASAN) memory poisoning with Zephyr's heap implementation.

## Purpose

The test validates that:

1. Memory allocated from the heap is properly unpoisoned and accessible
2. Memory freed to the heap is properly poisoned and inaccessible
3. Unallocated heap memory is initially poisoned and inaccessible
4. Reallocation works correctly with poisoning and unpoisoning
5. Aligned allocation and reallocation work correctly with ASAN
6. Heap internal functions properly ignore ASAN instrumentation where necessary

## Requirements

- Compiler with ASAN support (e.g., Clang)
- CONFIG_SYS_HEAP_ASAN_POISONING enabled

## Test Descriptions

### Normal Tests (Expected to Pass)

- **test_alloc_free**: Basic allocation and free operations
- **test_realloc**: Memory reallocation functionality
- **test_aligned_alloc**: Aligned memory allocation
- **test_aligned_realloc**: Aligned memory reallocation
- **test_fragmentation**: Multiple allocations with fragmentation patterns
- **test_mixed_allocs**: Mixed regular and aligned allocations
- **test_memory_reuse**: Memory reuse patterns across multiple rounds
- **test_usable_size**: Heap usable size functionality

### Error Detection Tests (Expected to Crash)

- **test_use_after_free_detection**: Deliberately triggers use-after-free errors
- **test_unallocated_access_detection**: Deliberately accesses unallocated memory

## How to Run

### Normal Tests

```shell
west build -b native_sim tests/lib/heap_asan_poisoning
west build -t run
```

### Error Detection Tests

#### Use-After-Free Detection:
```shell
west build -b native_sim tests/lib/heap_asan_poisoning -- -DOVERLAY_CONFIG=overlay-test-use-after-free.conf
west build -t run
```

#### Unallocated Memory Access Detection:
```shell
west build -b native_sim tests/lib/heap_asan_poisoning -- -DOVERLAY_CONFIG=overlay-test-unallocated-access.conf
west build -t run
```

Note: Error detection tests require ASAN support and are expected to crash with ASAN errors.

## Implementation Details

The ASAN poisoning implementation provides:

- **Initial heap poisoning** during heap initialization
- **Smart poisoning/unpoisoning** of free chunks while preserving metadata
- **Comprehensive error detection** for use-after-free and unallocated access
- **Minimal performance overhead** with selective function instrumentation
- **Full compatibility** with existing heap features and allocation types
- This ensures that accessing unallocated memory triggers ASAN errors

### 2. Smart Poisoning/Unpoisoning
- Free chunks are poisoned in their user data area while preserving free list metadata
- Chunks are unpoisoned when removed from the free list during allocation
- This provides comprehensive coverage without interfering with heap operations

### 3. Metadata Protection
- Heap metadata structures (z_heap struct, bucket arrays, chunk headers) are carefully unpoisoned
- Free list pointers within chunks are preserved from poisoning
- This ensures heap internal operations work correctly while maximizing error detection

### 4. Enhanced Error Detection
The implementation now detects:
- **Use-after-free**: Accessing memory after it has been freed
- **Unallocated access**: Accessing memory that was never allocated
- **Buffer overruns**: Writing beyond allocated boundaries (via adjacent poisoned regions)
- **Heap corruption**: Invalid access to heap metadata

### 5. Performance Considerations
- ASAN instrumentation is disabled on critical heap internal functions using `HEAP_NO_SANITIZE_ADDRESS`
- Poisoning operations are only performed when CONFIG_SYS_HEAP_ASAN_POISONING is enabled
- Minimal overhead is added to allocation/free operations

### 6. Integration with Existing Features
- Compatible with heap validation, runtime statistics, and heap listeners
- Works with both regular and aligned allocations
- Supports reallocation with proper poisoning state management

This enhanced implementation provides comprehensive memory error detection while maintaining the performance and functionality of the Zephyr heap system.
