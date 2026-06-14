/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Dynamic GBM provider selection — Mesa or libgbm-kms, picked at run time.
 *
 * Goal: prove the kernel surface (kms framework) is provider-agnostic.
 * The same /dev/dri/cardN works with either:
 *   - Mesa's libgbm.so.1 (when graphics/mesa-libs is installed)
 *   - libgbm-kms.so      (this tree)
 *
 * Selection order:
 *   1. GBM_PROVIDER env var: 'kms' or 'mesa' picks explicitly
 *   2. If unset, prefer mesa if libgbm.so.1 is present, else fall back to kms
 *
 * The picked provider is loaded with dlopen + dlsym -- no link-time
 * dependency on either.  A compositor can do the same to keep its
 * binary portable across systems where only one or the other is
 * installed.
 *
 * Usage: gbm_dual [/dev/dri/cardN]
 *        env GBM_PROVIDER=kms gbm_dual /dev/dri/card0
 *        env GBM_PROVIDER=mesa gbm_dual /dev/dri/card0
 */

#include <sys/types.h>
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

/* Mesa's gbm.h types are ABI-compatible with ours; we only use the
 * subset that both implement, so a single set of function-pointer
 * prototypes binds either provider. */
union gbm_bo_handle {
	void	*ptr;
	int32_t	 s32;
	uint32_t u32;
	int64_t	 s64;
	uint64_t u64;
};

struct gbm_device;
struct gbm_bo;

struct gbm_provider {
	const char *name;
	void	*lib;
	struct gbm_device *(*create_device)(int fd);
	void	 (*device_destroy)(struct gbm_device *);
	const char *(*device_get_backend_name)(struct gbm_device *);
	struct gbm_bo *(*bo_create)(struct gbm_device *, uint32_t w,
	    uint32_t h, uint32_t fmt, uint32_t flags);
	void	 (*bo_destroy)(struct gbm_bo *);
	uint32_t (*bo_get_width)(struct gbm_bo *);
	uint32_t (*bo_get_height)(struct gbm_bo *);
	uint32_t (*bo_get_stride)(struct gbm_bo *);
	union gbm_bo_handle (*bo_get_handle)(struct gbm_bo *);
	int	 (*bo_get_fd)(struct gbm_bo *);
};

#define GBM_FORMAT_XRGB8888  0x34325258
#define GBM_BO_USE_SCANOUT   (1 << 0)
#define GBM_BO_USE_LINEAR    (1 << 4)

static int
load(struct gbm_provider *p, const char *path)
{
	p->lib = dlopen(path, RTLD_NOW);
	if (p->lib == NULL)
		return (-1);
#define BIND(field, sym) \
	if ((*(void **)&p->field = dlsym(p->lib, sym)) == NULL) { \
		fprintf(stderr, "%s: missing %s\n", path, sym); \
		dlclose(p->lib); return (-1); }
	BIND(create_device, "gbm_create_device");
	BIND(device_destroy, "gbm_device_destroy");
	BIND(device_get_backend_name, "gbm_device_get_backend_name");
	BIND(bo_create, "gbm_bo_create");
	BIND(bo_destroy, "gbm_bo_destroy");
	BIND(bo_get_width, "gbm_bo_get_width");
	BIND(bo_get_height, "gbm_bo_get_height");
	BIND(bo_get_stride, "gbm_bo_get_stride");
	BIND(bo_get_handle, "gbm_bo_get_handle");
	BIND(bo_get_fd, "gbm_bo_get_fd");
#undef BIND
	return (0);
}

int
main(int argc, char **argv)
{
	const char *path = (argc > 1) ? argv[1] : "/dev/dri/card0";
	const char *want = getenv("GBM_PROVIDER");
	struct gbm_provider p = { 0 };

	if (want != NULL && strcmp(want, "kms") == 0) {
		p.name = "libgbm-kms";
		if (load(&p, "libgbm-kms.so") != 0 &&
		    load(&p, "/usr/local/lib/libgbm-kms.so") != 0) {
			fprintf(stderr, "GBM_PROVIDER=kms but libgbm-kms.so "
			    "not found\n");
			return (1);
		}
	} else if (want != NULL && strcmp(want, "mesa") == 0) {
		p.name = "mesa libgbm";
		if (load(&p, "libgbm.so.1") != 0 &&
		    load(&p, "/usr/local/lib/libgbm.so.1") != 0) {
			fprintf(stderr, "GBM_PROVIDER=mesa but libgbm.so.1 "
			    "not found\n");
			return (1);
		}
	} else {
		p.name = "mesa libgbm";
		if (load(&p, "libgbm.so.1") != 0 &&
		    load(&p, "/usr/local/lib/libgbm.so.1") != 0) {
			p.name = "libgbm-kms";
			if (load(&p, "libgbm-kms.so") != 0 &&
			    load(&p, "/usr/local/lib/libgbm-kms.so") != 0) {
				fprintf(stderr,
				    "no GBM provider found (tried mesa, kms)\n");
				return (1);
			}
		}
	}
	printf("picked provider: %s\n", p.name);

	int fd = open(path, O_RDWR);
	if (fd < 0) { perror(path); return (2); }

	struct gbm_device *dev = p.create_device(fd);
	if (dev == NULL) { fprintf(stderr, "create_device failed\n"); return (3); }
	printf("backend reports: %s\n", p.device_get_backend_name(dev));

	struct gbm_bo *bo = p.bo_create(dev, 800, 600, GBM_FORMAT_XRGB8888,
	    GBM_BO_USE_SCANOUT | GBM_BO_USE_LINEAR);
	if (bo == NULL) { fprintf(stderr, "bo_create failed\n"); return (4); }

	printf("bo: %ux%u stride=%u handle=%u\n",
	    p.bo_get_width(bo), p.bo_get_height(bo),
	    p.bo_get_stride(bo), p.bo_get_handle(bo).u32);

	int dmabuf = p.bo_get_fd(bo);
	printf("dmabuf_fd: %d (PRIME via %s)\n", dmabuf, p.name);
	if (dmabuf >= 0) close(dmabuf);

	p.bo_destroy(bo);
	p.device_destroy(dev);
	close(fd);
	printf("OK\n");
	return (0);
}
