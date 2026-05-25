# SPDX-License-Identifier: Apache-2.0

zephyr_constants_library(
  NAME heap_constants
  INCLUDES ${ARCH_DIR}/common/include
  SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/heap_constants.c
)

if(CONFIG_SYS_HEAP_ASAN_POISONING)
  # When ASAN poisoning of the heap is enabled, the heap implementation
  # itself must be excluded from ASAN instrumentation: the chunk headers,
  # free-list links and canaries live inside the user-poisoned regions and
  # the heap code must read/write them freely. Public API entry points use
  # the ASAN runtime API (__asan_*) to poison/unpoison the user-visible
  # regions so that instrumented application code still gets caught when
  # accessing freed or unallocated bytes.
  zephyr_library_named(heap)
  zephyr_library_sources(heap.c)
  zephyr_library_sources_ifdef(CONFIG_SYS_HEAP_RUNTIME_STATS heap_stats.c)
  zephyr_library_sources_ifdef(CONFIG_SYS_HEAP_INFO          heap_info.c)
  zephyr_library_sources_ifdef(CONFIG_SYS_HEAP_VALIDATE      heap_validate.c)
  zephyr_library_sources_ifdef(CONFIG_SYS_HEAP_STRESS        heap_stress.c)
  zephyr_library_sources_ifdef(CONFIG_SYS_HEAP_ARRAY_SIZE    heap_array.c)
  zephyr_library_compile_options(-fno-sanitize=address)
else()
  zephyr_sources(heap.c)
  zephyr_sources_ifdef(CONFIG_SYS_HEAP_RUNTIME_STATS heap_stats.c)
  zephyr_sources_ifdef(CONFIG_SYS_HEAP_INFO          heap_info.c)
  zephyr_sources_ifdef(CONFIG_SYS_HEAP_VALIDATE      heap_validate.c)
  zephyr_sources_ifdef(CONFIG_SYS_HEAP_STRESS        heap_stress.c)
  zephyr_sources_ifdef(CONFIG_SYS_HEAP_ARRAY_SIZE    heap_array.c)
endif()

zephyr_sources_ifdef(CONFIG_SHARED_MULTI_HEAP shared_multi_heap.c)
zephyr_sources_ifdef(CONFIG_MULTI_HEAP        multi_heap.c)
zephyr_sources_ifdef(CONFIG_HEAP_LISTENER     heap_listener.c)
