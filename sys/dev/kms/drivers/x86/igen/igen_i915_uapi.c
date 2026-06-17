/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * Linux i915 driver-specific uAPI shim.
 *
 * Mesa's iris driver only loads against a DRM device whose driver name
 * is "i915".  Once attached it issues a series of I915_GETPARAM /
 * I915_QUERY probes plus a small handful of GEM ioctls during screen
 * creation.  We answer them with Kabylake-GT2-sane values so iris can
 * reach the point of allocating its first BO and calling
 * EXECBUFFER2 — that's where the real GPU command-submission work
 * begins and is intentionally a TODO for now.
 *
 * Every value below is grounded in a Linux i915 reference: the iris
 * source tree's iris_screen_create + brw_get_device_info paths, and
 * the FreeBSD-arm64 kbl pci_id_driver_map table.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/proc.h>

#include <kms/drm_device.h>
#include <kms/drm_file.h>
#include <kms/drm_gem.h>

#include "igen_internal.h"

/* --- ioctl numbers (Linux i915_drm.h) --- */
#define	DRM_COMMAND_BASE			0x40

#define	I915_GETPARAM_NR			0x06
#define	I915_GEM_CREATE_NR			0x1b
#define	I915_GEM_SET_TILING_NR			0x21
#define	I915_GEM_GET_TILING_NR			0x22
#define	I915_GEM_GET_APERTURE_NR		0x23
#define	I915_GEM_CONTEXT_CREATE_NR		0x2d
#define	I915_GEM_CONTEXT_DESTROY_NR		0x2e
#define	I915_GEM_CONTEXT_GETPARAM_NR		0x34
#define	I915_GEM_CONTEXT_SETPARAM_NR		0x35
#define	I915_QUERY_NR				0x39
#define	I915_GEM_CREATE_EXT_NR			0x3c
#define	I915_GEM_MMAP_OFFSET_NR			0x40
#define	I915_REG_READ_NR			0x31
#define	I915_GET_RESET_STATS_NR			0x32
#define	I915_GEM_USERPTR_NR			0x33
#define	I915_GEM_VM_CREATE_NR			0x3a
#define	I915_GEM_VM_DESTROY_NR			0x3b
#define	I915_GEM_CONTEXT_CREATE_EXT_NR		0x3c
#define	I915_GEM_THROTTLE_NR			0x18
#define	I915_GEM_WAIT_NR			0x2c
#define	I915_GEM_BUSY_NR			0x17
#define	I915_GEM_MADVISE_NR			0x26
#define	I915_GEM_SET_DOMAIN_NR			0x1f
#define	I915_GEM_SW_FINISH_NR			0x20

#define	DRM_I915_QUERY_TOPOLOGY_INFO		1
#define	DRM_I915_QUERY_ENGINE_INFO		2
#define	DRM_I915_QUERY_MEMORY_REGIONS		4

#define	I915_ENGINE_CLASS_RENDER		0
#define	I915_ENGINE_CLASS_COPY			1
#define	I915_ENGINE_CLASS_VIDEO			2
#define	I915_ENGINE_CLASS_VIDEO_ENHANCE		3

#define	I915_MEMORY_CLASS_SYSTEM		0

/* --- I915_PARAM_* values (Linux i915_drm.h) --- */
#define	I915_PARAM_CHIPSET_ID			4
#define	I915_PARAM_HAS_GEM			5
#define	I915_PARAM_NUM_FENCES_AVAIL		6
#define	I915_PARAM_HAS_EXECBUF2			9
#define	I915_PARAM_HAS_BSD			10
#define	I915_PARAM_HAS_BLT			11
#define	I915_PARAM_HAS_RELAXED_FENCING		12
#define	I915_PARAM_HAS_LLC			17
#define	I915_PARAM_HAS_ALIASING_PPGTT		18
#define	I915_PARAM_HAS_WAIT_TIMEOUT		19
#define	I915_PARAM_HAS_SEMAPHORES		20
#define	I915_PARAM_HAS_PRIME_VMAP_FLUSH		21
#define	I915_PARAM_HAS_VEBOX			22
#define	I915_PARAM_HAS_EXEC_NO_RELOC		25
#define	I915_PARAM_HAS_EXEC_HANDLE_LUT		26
#define	I915_PARAM_CMD_PARSER_VERSION		28
#define	I915_PARAM_MMAP_VERSION			30
#define	I915_PARAM_HAS_BSD2			31
#define	I915_PARAM_REVISION			32
#define	I915_PARAM_SUBSLICE_TOTAL		33
#define	I915_PARAM_EU_TOTAL			34
#define	I915_PARAM_HAS_GPU_RESET		35
#define	I915_PARAM_HAS_EXEC_SOFTPIN		37
#define	I915_PARAM_HAS_POOLED_EU			38
#define	I915_PARAM_MIN_EU_IN_POOL		39
#define	I915_PARAM_MMAP_GTT_VERSION		40
#define	I915_PARAM_HAS_SCHEDULER			41
#define	I915_PARAM_HUC_STATUS			42
#define	I915_PARAM_HAS_EXEC_ASYNC		43
#define	I915_PARAM_HAS_EXEC_FENCE		44
#define	I915_PARAM_HAS_EXEC_CAPTURE		45
#define	I915_PARAM_SLICE_MASK			46
#define	I915_PARAM_SUBSLICE_MASK			47
#define	I915_PARAM_HAS_EXEC_BATCH_FIRST		48
#define	I915_PARAM_HAS_EXEC_FENCE_ARRAY		49
#define	I915_PARAM_HAS_CONTEXT_ISOLATION		50
#define	I915_PARAM_CS_TIMESTAMP_FREQUENCY	51
#define	I915_PARAM_MMAP_GTT_COHERENT		52
#define	I915_PARAM_PERF_REVISION		54
#define	I915_PARAM_OA_TIMESTAMP_FREQUENCY	56

/* I915 scheduler capability bits.  Reported via HAS_SCHEDULER. */
#define	I915_SCHEDULER_CAP_ENABLED		(1 << 0)
#define	I915_SCHEDULER_CAP_PRIORITY		(1 << 1)
#define	I915_SCHEDULER_CAP_PREEMPTION		(1 << 2)

struct drm_i915_getparam {
	int32_t		param;
	void		*value;	/* user pointer to int */
};

struct drm_i915_gem_create {
	uint64_t	size;
	uint32_t	handle;
	uint32_t	pad;
};

struct drm_i915_gem_set_tiling {
	uint32_t	handle;
	uint32_t	tiling_mode;
	uint32_t	stride;
	uint32_t	swizzle_mode;
};

struct drm_i915_gem_get_tiling {
	uint32_t	handle;
	uint32_t	tiling_mode;
	uint32_t	swizzle_mode;
	uint32_t	phys_swizzle_mode;
};

struct drm_i915_gem_get_aperture {
	uint64_t	aper_size;
	uint64_t	aper_available_size;
};

struct drm_i915_gem_context_create {
	uint32_t	ctx_id;
	uint32_t	pad;
};

struct drm_i915_gem_context_param {
	uint32_t	ctx_id;
	uint32_t	size;
	uint64_t	param;
	uint64_t	value;
};

struct drm_i915_query_item {
	uint64_t	query_id;
	int32_t		length;
	uint32_t	flags;
	uint64_t	data_ptr;
};

struct drm_i915_query {
	uint32_t	num_items;
	uint32_t	flags;
	uint64_t	items_ptr;
};

struct drm_i915_gem_mmap_offset {
	uint32_t	handle;
	uint32_t	pad;
	uint64_t	offset;
	uint64_t	flags;
};

struct drm_i915_gem_userptr {
	uint64_t	user_ptr;
	uint64_t	user_size;
	uint32_t	flags;
	uint32_t	handle;
};

struct drm_i915_gem_busy {
	uint32_t	handle;
	uint32_t	busy;
};

struct drm_i915_gem_wait {
	uint32_t	bo_handle;
	uint32_t	flags;
	int64_t		timeout_ns;
};

struct drm_i915_gem_madvise {
	uint32_t	handle;
	uint32_t	madv;
	uint32_t	retained;
};

struct drm_i915_gem_set_domain {
	uint32_t	handle;
	uint32_t	read_domains;
	uint32_t	write_domain;
};

struct drm_i915_gem_vm_control {
	uint32_t	extensions;
	uint32_t	flags;
	uint32_t	vm_id;
};

struct drm_i915_reset_stats {
	uint32_t	ctx_id;
	uint32_t	flags;
	uint32_t	reset_count;
	uint32_t	batch_active;
	uint32_t	batch_pending;
	uint32_t	pad;
};

struct drm_i915_reg_read {
	uint64_t	offset;
	uint64_t	val;
};

struct i915_engine_class_instance {
	uint16_t	engine_class;
	uint16_t	engine_instance;
};

struct drm_i915_engine_info {
	struct i915_engine_class_instance engine;
	uint32_t	rsvd0;
	uint64_t	flags;
	uint64_t	capabilities;
	uint64_t	logical_instance;
	uint64_t	rsvd1[3];
};

struct drm_i915_query_engine_info_hdr {
	uint32_t	num_engines;
	uint32_t	rsvd[3];
	/* engines[num_engines] follows */
};

struct drm_i915_gem_memory_class_instance {
	uint16_t	memory_class;
	uint16_t	memory_instance;
};

struct drm_i915_memory_region_info {
	struct drm_i915_gem_memory_class_instance region;
	uint32_t	rsvd0;
	uint64_t	probed_size;
	uint64_t	unallocated_size;
	uint64_t	rsvd1[8];
};

struct drm_i915_query_memory_regions_hdr {
	uint32_t	num_regions;
	uint32_t	rsvd[3];
	/* regions[num_regions] follows */
};

struct drm_i915_query_topology_info_hdr {
	uint16_t	flags;
	uint16_t	max_slices;
	uint16_t	max_subslices;
	uint16_t	max_eus_per_subslice;
	uint16_t	subslice_offset;
	uint16_t	subslice_stride;
	uint16_t	eu_offset;
	uint16_t	eu_stride;
	/* data[] follows: slice_mask, subslice_mask[max_slices],
	 * eu_mask[max_slices * max_subslices]. */
};

/*
 * Resolve and return the integer value for a single I915_PARAM_*.
 * Anything we haven't taught yet returns EINVAL — iris treats that as
 * "feature not supported," same as Linux i915 would for a brand-new
 * param it hasn't backported.
 */
static int
igen_i915_getparam_value(struct igen_softc *sc, int32_t param, int32_t *out)
{
	switch (param) {
	case I915_PARAM_CHIPSET_ID:
		*out = sc->pci_id;
		return (0);
	case I915_PARAM_REVISION:
		*out = 0;	/* rev A0 — close enough for Mesa probes */
		return (0);
	case I915_PARAM_HAS_GEM:
	case I915_PARAM_HAS_EXECBUF2:
	case I915_PARAM_HAS_BLT:
	case I915_PARAM_HAS_LLC:		/* KBL has eLLC + LLC */
	case I915_PARAM_HAS_ALIASING_PPGTT:	/* full-PPGTT on gen9+ */
	case I915_PARAM_HAS_WAIT_TIMEOUT:
	case I915_PARAM_HAS_VEBOX:
	case I915_PARAM_HAS_BSD:
	case I915_PARAM_HAS_BSD2:
	case I915_PARAM_HAS_RELAXED_FENCING:
	case I915_PARAM_HAS_PRIME_VMAP_FLUSH:
	case I915_PARAM_HAS_EXEC_NO_RELOC:
	case I915_PARAM_HAS_EXEC_HANDLE_LUT:
	case I915_PARAM_HAS_EXEC_SOFTPIN:
	case I915_PARAM_HAS_EXEC_ASYNC:
	case I915_PARAM_HAS_EXEC_FENCE:
	case I915_PARAM_HAS_EXEC_CAPTURE:
	case I915_PARAM_HAS_EXEC_BATCH_FIRST:
	case I915_PARAM_HAS_EXEC_FENCE_ARRAY:
	case I915_PARAM_HAS_CONTEXT_ISOLATION:
	case I915_PARAM_HAS_GPU_RESET:
	case I915_PARAM_MMAP_GTT_COHERENT:
		*out = 1;
		return (0);
	case I915_PARAM_NUM_FENCES_AVAIL:
		*out = 32;	/* SKL/KBL global fence count */
		return (0);
	case I915_PARAM_CMD_PARSER_VERSION:
		*out = 10;	/* "modern" — iris skips compat shims */
		return (0);
	case I915_PARAM_MMAP_VERSION:
		*out = 1;
		return (0);
	case I915_PARAM_MMAP_GTT_VERSION:
		*out = 4;
		return (0);
	case I915_PARAM_HAS_SCHEDULER:
		*out = I915_SCHEDULER_CAP_ENABLED |
		    I915_SCHEDULER_CAP_PRIORITY |
		    I915_SCHEDULER_CAP_PREEMPTION;
		return (0);
	case I915_PARAM_HUC_STATUS:
		*out = 0;	/* No HuC firmware loaded yet */
		return (0);
	case I915_PARAM_HAS_POOLED_EU:
	case I915_PARAM_MIN_EU_IN_POOL:
		*out = 0;
		return (0);
	case I915_PARAM_SLICE_MASK:
		*out = 0x1;	/* one slice on KBL GT2 */
		return (0);
	case I915_PARAM_SUBSLICE_MASK:
		*out = 0x7;	/* three subslices on KBL GT2 */
		return (0);
	case I915_PARAM_SUBSLICE_TOTAL:
		*out = 3;
		return (0);
	case I915_PARAM_EU_TOTAL:
		*out = 24;	/* KBL GT2 has 24 EUs */
		return (0);
	case I915_PARAM_CS_TIMESTAMP_FREQUENCY:
	case I915_PARAM_OA_TIMESTAMP_FREQUENCY:
		*out = 12000000;	/* 12 MHz CS/OA timestamp on gen9 */
		return (0);
	case I915_PARAM_PERF_REVISION:
		*out = 5;
		return (0);
	case I915_PARAM_HAS_SEMAPHORES:
		*out = 0;	/* deprecated; iris doesn't require it */
		return (0);
	}
	return (EINVAL);
}

static int
igen_i915_getparam(struct igen_softc *sc, struct drm_i915_getparam *gp)
{
	int32_t val;
	int error;

	error = igen_i915_getparam_value(sc, gp->param, &val);
	if (error != 0)
		return (error);
	return (copyout(&val, gp->value, sizeof(val)));
}

/*
 * GEM_CREATE: just allocates a backing object of the requested size
 * and returns a per-file handle.  Iris uses this for every BO it
 * creates — shaders, vertex buffers, render targets, ...  We forward
 * to the dumb-buffer allocator; format ignorance is fine here, the
 * BO doesn't have a pixel-format identity at this layer.
 */
static int
igen_i915_gem_create(struct drm_file *file, struct drm_i915_gem_create *gc)
{
	struct drm_gem_object *obj;
	uint32_t handle;
	int error;

	if (gc->size == 0)
		return (EINVAL);
	obj = kms_gem_object_create(file->dev, gc->size);
	if (obj == NULL)
		return (ENOMEM);
	error = kms_gem_handle_create(file, obj, &handle);
	kms_gem_object_put(obj);
	if (error != 0)
		return (error);
	gc->handle = handle;
	gc->pad = 0;
	return (0);
}

static int
igen_i915_gem_set_tiling(struct drm_file *file __unused,
    struct drm_i915_gem_set_tiling *st)
{
	/*
	 * Accept any tiling request but only ever store linear.  iris asks
	 * for X/Y tiling on render targets; the lie keeps init flowing,
	 * the cost is reduced GPU memory bandwidth — which doesn't matter
	 * until we wire EXECBUFFER2.
	 */
	st->tiling_mode = 0;	/* I915_TILING_NONE */
	st->swizzle_mode = 0;	/* I915_BIT_6_SWIZZLE_NONE */
	return (0);
}

static int
igen_i915_gem_get_tiling(struct drm_file *file __unused,
    struct drm_i915_gem_get_tiling *gt)
{
	gt->tiling_mode = 0;
	gt->swizzle_mode = 0;
	gt->phys_swizzle_mode = 0;
	return (0);
}

static int
igen_i915_gem_get_aperture(struct drm_file *file __unused,
    struct drm_i915_gem_get_aperture *ga)
{
	/*
	 * Report a generous aperture so iris doesn't refuse to allocate
	 * intermediate buffers.  Actual GTT capacity on KBL is 4 GiB
	 * virtual; we expose 256 MiB which matches the size most i915
	 * userspace expects for the mappable window.
	 */
	ga->aper_size = (uint64_t)256 << 20;
	ga->aper_available_size = (uint64_t)256 << 20;
	return (0);
}

static int
igen_i915_gem_context_create(struct drm_file *file __unused,
    struct drm_i915_gem_context_create *cc)
{
	/*
	 * iris asks for one HW context per GL context.  We don't have HW
	 * context isolation wired yet so every request returns the same
	 * sentinel id — iris treats this as one shared context, fine for
	 * single-application scenarios.
	 */
	cc->ctx_id = 1;
	cc->pad = 0;
	return (0);
}

static int
igen_i915_gem_context_destroy(struct drm_file *file __unused,
    struct drm_i915_gem_context_create *cc __unused)
{
	return (0);
}

static int
igen_i915_gem_context_getparam(struct drm_file *file __unused,
    struct drm_i915_gem_context_param *cp)
{
	/*
	 * Default everything to zero — iris reads the GTT size, scheduler
	 * priority, and a few other params.  Returning zero advertises
	 * "feature off but recognised."  Real values land alongside
	 * EXECBUFFER2.
	 */
	cp->value = 0;
	return (0);
}

static int
igen_i915_gem_context_setparam(struct drm_file *file __unused,
    struct drm_i915_gem_context_param *cp __unused)
{
	return (0);
}

/*
 * Build the response for I915_QUERY_ENGINE_INFO into a kernel-side
 * buffer and return the required size.  KBL exposes render, copy,
 * video and vebox engines; userspace iris cares only that the render
 * engine is present.  Caller passes (NULL, 0) to size-query.
 */
static int32_t
igen_i915_query_engine_info_fill(uint8_t *buf, int32_t buf_len)
{
	const uint32_t num_engines = 4;
	int32_t need;

	need = (int32_t)(sizeof(struct drm_i915_query_engine_info_hdr) +
	    num_engines * sizeof(struct drm_i915_engine_info));
	if (buf == NULL || buf_len == 0)
		return (need);
	if (buf_len < need)
		return (-EINVAL);

	struct drm_i915_query_engine_info_hdr *h =
	    (struct drm_i915_query_engine_info_hdr *)buf;
	struct drm_i915_engine_info *e = (struct drm_i915_engine_info *)(h + 1);

	memset(buf, 0, need);
	h->num_engines = num_engines;
	e[0].engine.engine_class = I915_ENGINE_CLASS_RENDER;
	e[0].engine.engine_instance = 0;
	e[1].engine.engine_class = I915_ENGINE_CLASS_COPY;
	e[1].engine.engine_instance = 0;
	e[2].engine.engine_class = I915_ENGINE_CLASS_VIDEO;
	e[2].engine.engine_instance = 0;
	e[3].engine.engine_class = I915_ENGINE_CLASS_VIDEO_ENHANCE;
	e[3].engine.engine_instance = 0;
	return (need);
}

/*
 * Build the response for I915_QUERY_MEMORY_REGIONS.  We expose one
 * SYSTEM region covering main RAM; KBL has no LMEM so no DEVICE
 * region.  probed_size is best-effort — iris uses it for budgeting
 * but does not enforce equality with anything.
 */
static int32_t
igen_i915_query_memory_regions_fill(uint8_t *buf, int32_t buf_len)
{
	const uint32_t num_regions = 1;
	int32_t need;

	need = (int32_t)(sizeof(struct drm_i915_query_memory_regions_hdr) +
	    num_regions * sizeof(struct drm_i915_memory_region_info));
	if (buf == NULL || buf_len == 0)
		return (need);
	if (buf_len < need)
		return (-EINVAL);

	struct drm_i915_query_memory_regions_hdr *h =
	    (struct drm_i915_query_memory_regions_hdr *)buf;
	struct drm_i915_memory_region_info *r =
	    (struct drm_i915_memory_region_info *)(h + 1);

	memset(buf, 0, need);
	h->num_regions = num_regions;
	r[0].region.memory_class = I915_MEMORY_CLASS_SYSTEM;
	r[0].region.memory_instance = 0;
	r[0].probed_size = (uint64_t)1 << 32;	/* 4 GiB nominal */
	r[0].unallocated_size = (uint64_t)1 << 32;
	return (need);
}

/*
 * I915_QUERY_TOPOLOGY_INFO for KBL GT2: one slice, three subslices,
 * eight EUs per subslice (24 total).  Userspace iris uses these masks
 * to drive shader dispatch and compute thread counts.
 */
static int32_t
igen_i915_query_topology_info_fill(uint8_t *buf, int32_t buf_len)
{
	const uint16_t max_slices = 1;
	const uint16_t max_subslices = 3;
	const uint16_t max_eus_per_subslice = 8;
	const uint16_t slice_mask_bytes = (max_slices + 7) / 8;
	const uint16_t subslice_mask_bytes = (max_subslices + 7) / 8;
	const uint16_t eu_mask_bytes = (max_eus_per_subslice + 7) / 8;
	const uint16_t subslice_data = max_slices * subslice_mask_bytes;
	const uint16_t eu_data =
	    max_slices * max_subslices * eu_mask_bytes;
	int32_t need;
	uint8_t *data;

	need = (int32_t)sizeof(struct drm_i915_query_topology_info_hdr) +
	    slice_mask_bytes + subslice_data + eu_data;
	if (buf == NULL || buf_len == 0)
		return (need);
	if (buf_len < need)
		return (-EINVAL);

	struct drm_i915_query_topology_info_hdr *h =
	    (struct drm_i915_query_topology_info_hdr *)buf;
	memset(buf, 0, need);
	h->flags = 0;
	h->max_slices = max_slices;
	h->max_subslices = max_subslices;
	h->max_eus_per_subslice = max_eus_per_subslice;
	h->subslice_offset = slice_mask_bytes;
	h->subslice_stride = subslice_mask_bytes;
	h->eu_offset = slice_mask_bytes + subslice_data;
	h->eu_stride = eu_mask_bytes;

	data = buf + sizeof(*h);
	data[0] = 0x01;			/* one slice present */
	data[h->subslice_offset] = 0x07;	/* three subslices */
	data[h->eu_offset + 0] = 0xff;	/* 8 EUs in subslice 0 */
	data[h->eu_offset + 1] = 0xff;	/* 8 EUs in subslice 1 */
	data[h->eu_offset + 2] = 0xff;	/* 8 EUs in subslice 2 */
	return (need);
}

static int
igen_i915_query_one(struct drm_i915_query_item *item)
{
	uint8_t *buf;
	int32_t need;
	int error;

	switch (item->query_id) {
	case DRM_I915_QUERY_ENGINE_INFO:
	case DRM_I915_QUERY_MEMORY_REGIONS:
	case DRM_I915_QUERY_TOPOLOGY_INFO:
		break;
	default:
		/*
		 * Mark unknown query ids as "no data" without an error so
		 * iris treats them as missing and moves on.  Anything we
		 * have a real fill for falls through to the per-id dispatch
		 * below.
		 */
		item->length = 0;
		return (0);
	}

	switch (item->query_id) {
	case DRM_I915_QUERY_ENGINE_INFO:
		need = igen_i915_query_engine_info_fill(NULL, 0);
		break;
	case DRM_I915_QUERY_MEMORY_REGIONS:
		need = igen_i915_query_memory_regions_fill(NULL, 0);
		break;
	case DRM_I915_QUERY_TOPOLOGY_INFO:
		need = igen_i915_query_topology_info_fill(NULL, 0);
		break;
	default:
		__unreachable();
	}

	if (item->length == 0) {
		item->length = need;
		return (0);
	}

	if (item->length < need)
		return (EINVAL);
	if (item->data_ptr == 0)
		return (EINVAL);

	buf = malloc(need, M_TEMP, M_WAITOK | M_ZERO);
	switch (item->query_id) {
	case DRM_I915_QUERY_ENGINE_INFO:
		(void)igen_i915_query_engine_info_fill(buf, need);
		break;
	case DRM_I915_QUERY_MEMORY_REGIONS:
		(void)igen_i915_query_memory_regions_fill(buf, need);
		break;
	case DRM_I915_QUERY_TOPOLOGY_INFO:
		(void)igen_i915_query_topology_info_fill(buf, need);
		break;
	}

	error = copyout(buf, (void *)(uintptr_t)item->data_ptr, need);
	free(buf, M_TEMP);
	if (error != 0)
		return (error);
	item->length = need;
	return (0);
}

static int
igen_i915_query(struct drm_file *file __unused, struct drm_i915_query *q)
{
	struct drm_i915_query_item item;
	int error;
	uint32_t i;

	for (i = 0; i < q->num_items; i++) {
		void *uptr = (void *)(uintptr_t)(q->items_ptr +
		    (uint64_t)i * sizeof(item));

		error = copyin(uptr, &item, sizeof(item));
		if (error != 0)
			return (error);
		error = igen_i915_query_one(&item);
		if (error != 0)
			return (error);
		error = copyout(&item, uptr, sizeof(item));
		if (error != 0)
			return (error);
	}
	return (0);
}

/*
 * Map a GEM handle to the mmap offset userspace passes to mmap() on
 * the DRM cdev.  Our cdev_pager hook (kms_gem_object_lookup_offset)
 * already keys mappings off the same offset the GEM object was
 * assigned at create time; the iris driver just needs to read it back.
 */
static int
igen_i915_gem_mmap_offset(struct drm_file *file,
    struct drm_i915_gem_mmap_offset *mo)
{
	struct drm_gem_object *obj;

	obj = kms_gem_handle_lookup(file, mo->handle);
	if (obj == NULL)
		return (ENOENT);
	mo->offset = obj->mmap_offset;
	kms_gem_object_put(obj);
	return (0);
}

/*
 * USERPTR: wrap a userspace memory region in a kernel BO so the GPU
 * can DMA from it.  We don't have GPU command submission yet, but
 * libdrm_intel's bufmgr probes USERPTR at init by allocating a page,
 * binding it, then immediately closing it; returning ENODEV here
 * makes it disable that bufmgr feature and continue rather than
 * declare "i915 kernel driver may not be sane!" and abort.
 */
static int
igen_i915_gem_userptr(struct drm_file *file __unused,
    struct drm_i915_gem_userptr *up __unused)
{
	return (ENODEV);
}

static int
igen_i915_gem_busy(struct drm_file *file __unused,
    struct drm_i915_gem_busy *b)
{
	b->busy = 0;
	return (0);
}

static int
igen_i915_gem_wait(struct drm_file *file __unused,
    struct drm_i915_gem_wait *w __unused)
{
	return (0);
}

static int
igen_i915_gem_madvise(struct drm_file *file __unused,
    struct drm_i915_gem_madvise *m)
{
	m->retained = 1;
	return (0);
}

static int
igen_i915_gem_set_domain(struct drm_file *file __unused,
    struct drm_i915_gem_set_domain *sd __unused)
{
	return (0);
}

static int
igen_i915_gem_sw_finish(struct drm_file *file __unused, void *data __unused)
{
	return (0);
}

static int
igen_i915_gem_throttle(struct drm_file *file __unused, void *data __unused)
{
	return (0);
}

/*
 * VM_CREATE: full-PPGTT VM allocation for an isolated address space.
 * Newer Mesa attaches a VM to each context for proper isolation; if
 * the ioctl returns ENOTTY iris falls back to the legacy aliasing
 * PPGTT path which is what we model today.
 */
static int
igen_i915_gem_vm_create(struct drm_file *file __unused,
    struct drm_i915_gem_vm_control *vc)
{
	vc->vm_id = 1;
	return (0);
}

static int
igen_i915_gem_vm_destroy(struct drm_file *file __unused,
    struct drm_i915_gem_vm_control *vc __unused)
{
	return (0);
}

static int
igen_i915_get_reset_stats(struct drm_file *file __unused,
    struct drm_i915_reset_stats *rs)
{
	rs->reset_count = 0;
	rs->batch_active = 0;
	rs->batch_pending = 0;
	return (0);
}

static int
igen_i915_reg_read(struct drm_file *file __unused,
    struct drm_i915_reg_read *rr)
{
	rr->val = 0;
	return (0);
}

/*
 * Driver-level ioctl fallback registered as drm_driver.ioctl.  The
 * core kms_ioctl forwards anything its main switch doesn't recognise
 * here so the i915 namespace (cmd nr >= 0x40) lands in one place.
 */
int
igen_i915_ioctl(struct drm_file *file, u_long cmd, void *data)
{
	struct igen_softc *sc = file->dev->driver_priv;
	uint8_t nr = cmd & 0xff;

	if (nr < DRM_COMMAND_BASE)
		return (ENOTTY);

	switch (nr - DRM_COMMAND_BASE) {
	case I915_GETPARAM_NR:
		return (igen_i915_getparam(sc,
		    (struct drm_i915_getparam *)data));
	case I915_GEM_CREATE_NR:
		return (igen_i915_gem_create(file,
		    (struct drm_i915_gem_create *)data));
	case I915_GEM_SET_TILING_NR:
		return (igen_i915_gem_set_tiling(file,
		    (struct drm_i915_gem_set_tiling *)data));
	case I915_GEM_GET_TILING_NR:
		return (igen_i915_gem_get_tiling(file,
		    (struct drm_i915_gem_get_tiling *)data));
	case I915_GEM_GET_APERTURE_NR:
		return (igen_i915_gem_get_aperture(file,
		    (struct drm_i915_gem_get_aperture *)data));
	case I915_GEM_CONTEXT_CREATE_NR:
		return (igen_i915_gem_context_create(file,
		    (struct drm_i915_gem_context_create *)data));
	case I915_GEM_CONTEXT_DESTROY_NR:
		return (igen_i915_gem_context_destroy(file,
		    (struct drm_i915_gem_context_create *)data));
	case I915_GEM_CONTEXT_GETPARAM_NR:
		return (igen_i915_gem_context_getparam(file,
		    (struct drm_i915_gem_context_param *)data));
	case I915_GEM_CONTEXT_SETPARAM_NR:
		return (igen_i915_gem_context_setparam(file,
		    (struct drm_i915_gem_context_param *)data));
	case I915_QUERY_NR:
		return (igen_i915_query(file,
		    (struct drm_i915_query *)data));
	case I915_GEM_MMAP_OFFSET_NR:
		return (igen_i915_gem_mmap_offset(file,
		    (struct drm_i915_gem_mmap_offset *)data));
	case I915_GEM_USERPTR_NR:
		return (igen_i915_gem_userptr(file,
		    (struct drm_i915_gem_userptr *)data));
	case I915_GEM_BUSY_NR:
		return (igen_i915_gem_busy(file,
		    (struct drm_i915_gem_busy *)data));
	case I915_GEM_WAIT_NR:
		return (igen_i915_gem_wait(file,
		    (struct drm_i915_gem_wait *)data));
	case I915_GEM_MADVISE_NR:
		return (igen_i915_gem_madvise(file,
		    (struct drm_i915_gem_madvise *)data));
	case I915_GEM_SET_DOMAIN_NR:
		return (igen_i915_gem_set_domain(file,
		    (struct drm_i915_gem_set_domain *)data));
	case I915_GEM_SW_FINISH_NR:
		return (igen_i915_gem_sw_finish(file, data));
	case I915_GEM_THROTTLE_NR:
		return (igen_i915_gem_throttle(file, data));
	case I915_GEM_VM_CREATE_NR:
		return (igen_i915_gem_vm_create(file,
		    (struct drm_i915_gem_vm_control *)data));
	case I915_GEM_VM_DESTROY_NR:
		return (igen_i915_gem_vm_destroy(file,
		    (struct drm_i915_gem_vm_control *)data));
	case I915_GEM_CONTEXT_CREATE_EXT_NR:
		return (igen_i915_gem_context_create(file,
		    (struct drm_i915_gem_context_create *)data));
	case I915_GET_RESET_STATS_NR:
		return (igen_i915_get_reset_stats(file,
		    (struct drm_i915_reset_stats *)data));
	case I915_REG_READ_NR:
		return (igen_i915_reg_read(file,
		    (struct drm_i915_reg_read *)data));
	}
	return (ENOTTY);
}
