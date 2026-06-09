/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Kyle Crenshaw <b1nc0d3x@gmail.com>
 *
 * kms Phase 2 smoke test.  Opens /dev/dri/cardN and exercises the
 * four bootstrap ioctls (VERSION, GET_UNIQUE, SET_VERSION, GET_CAP)
 * against the framework's cdev.  Mirrors what libdrm's drmGetVersion()
 * does on first open.  Exits 0 on round-trip success, non-zero on any
 * mismatch or ioctl failure.
 */

#include <sys/ioctl.h>
#include <sys/types.h>

#include <sys/mman.h>

#include <err.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <drm/drm.h>
#include <drm/drm_mode.h>

static const char	*default_dev = "/dev/dri/card0";

static int
probe_version(int fd)
{
	struct drm_version v;
	char *name, *date, *desc;
	int error;

	memset(&v, 0, sizeof(v));
	if (ioctl(fd, DRM_IOCTL_VERSION, &v) != 0) {
		warn("VERSION probe");
		return (1);
	}
	printf("VERSION probe: driver %d.%d.%d  name_len=%zu date_len=%zu "
	    "desc_len=%zu\n", v.version_major, v.version_minor,
	    v.version_patchlevel, v.name_len, v.date_len, v.desc_len);

	name = calloc(1, v.name_len + 1);
	date = calloc(1, v.date_len + 1);
	desc = calloc(1, v.desc_len + 1);
	if (name == NULL || date == NULL || desc == NULL)
		err(1, "calloc");
	v.name = name;
	v.date = date;
	v.desc = desc;

	error = ioctl(fd, DRM_IOCTL_VERSION, &v);
	if (error != 0) {
		warn("VERSION fill");
		free(name); free(date); free(desc);
		return (1);
	}
	printf("VERSION fill : name=\"%s\" date=\"%s\" desc=\"%s\"\n",
	    name, date, desc);

	free(name); free(date); free(desc);
	return (0);
}

static int
probe_unique(int fd)
{
	struct drm_unique u;
	char *unique;
	int error;

	memset(&u, 0, sizeof(u));
	if (ioctl(fd, DRM_IOCTL_GET_UNIQUE, &u) != 0) {
		warn("GET_UNIQUE probe");
		return (1);
	}
	printf("GET_UNIQUE   : unique_len=%zu\n", u.unique_len);

	unique = calloc(1, u.unique_len + 1);
	if (unique == NULL)
		err(1, "calloc");
	u.unique = unique;

	error = ioctl(fd, DRM_IOCTL_GET_UNIQUE, &u);
	if (error != 0) {
		warn("GET_UNIQUE fill");
		free(unique);
		return (1);
	}
	printf("GET_UNIQUE   : unique=\"%s\"\n", unique);
	free(unique);
	return (0);
}

static int
probe_set_version(int fd)
{
	struct drm_set_version sv;

	sv.drm_di_major = -1;
	sv.drm_di_minor = -1;
	sv.drm_dd_major = -1;
	sv.drm_dd_minor = -1;
	if (ioctl(fd, DRM_IOCTL_SET_VERSION, &sv) != 0) {
		warn("SET_VERSION");
		return (1);
	}
	printf("SET_VERSION  : di=%d.%d  dd=%d.%d\n",
	    sv.drm_di_major, sv.drm_di_minor,
	    sv.drm_dd_major, sv.drm_dd_minor);
	return (0);
}

static int
probe_get_cap(int fd)
{
	struct drm_get_cap c;

	memset(&c, 0, sizeof(c));
	c.capability = 0x1;	/* DRM_CAP_DUMB_BUFFER — should report 0 */
	if (ioctl(fd, DRM_IOCTL_GET_CAP, &c) != 0) {
		warn("GET_CAP(DUMB_BUFFER)");
		return (1);
	}
	printf("GET_CAP      : cap=0x%llx value=0x%llx\n",
	    (unsigned long long)c.capability,
	    (unsigned long long)c.value);
	return (0);
}

static int
probe_mode_getresources(int fd, uint32_t *crtc_id, uint32_t *enc_id,
    uint32_t *conn_id)
{
	struct drm_mode_card_res r;
	uint32_t crtcs[16], encs[16], conns[16], fbs[16];

	memset(&r, 0, sizeof(r));
	if (ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &r) != 0) {
		warn("MODE_GETRESOURCES count probe");
		return (1);
	}
	printf("GETRES count : fb=%u crtc=%u conn=%u enc=%u "
	    "min=%ux%u max=%ux%u\n", r.count_fbs, r.count_crtcs,
	    r.count_connectors, r.count_encoders, r.min_width,
	    r.min_height, r.max_width, r.max_height);

	memset(&r, 0, sizeof(r));
	r.fb_id_ptr = (uintptr_t)fbs;
	r.crtc_id_ptr = (uintptr_t)crtcs;
	r.connector_id_ptr = (uintptr_t)conns;
	r.encoder_id_ptr = (uintptr_t)encs;
	r.count_fbs = 16;
	r.count_crtcs = 16;
	r.count_connectors = 16;
	r.count_encoders = 16;
	if (ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &r) != 0) {
		warn("MODE_GETRESOURCES fill");
		return (1);
	}
	printf("GETRES fill  : fb=%u crtc=%u conn=%u enc=%u\n",
	    r.count_fbs, r.count_crtcs, r.count_connectors,
	    r.count_encoders);
	if (r.count_crtcs > 0)
		printf("  crtc[0]=%u\n", crtcs[0]);
	if (r.count_encoders > 0)
		printf("  enc[0] =%u\n", encs[0]);
	if (r.count_connectors > 0)
		printf("  conn[0]=%u\n", conns[0]);

	*crtc_id = (r.count_crtcs > 0) ? crtcs[0] : 0;
	*enc_id = (r.count_encoders > 0) ? encs[0] : 0;
	*conn_id = (r.count_connectors > 0) ? conns[0] : 0;
	return (0);
}

static int
probe_mode_getcrtc(int fd, uint32_t id)
{
	struct drm_mode_crtc c;

	if (id == 0)
		return (0);
	memset(&c, 0, sizeof(c));
	c.crtc_id = id;
	if (ioctl(fd, DRM_IOCTL_MODE_GETCRTC, &c) != 0) {
		warn("MODE_GETCRTC id=%u", id);
		return (1);
	}
	printf("GETCRTC      : id=%u fb=%u x=%u y=%u mode_valid=%u\n",
	    c.crtc_id, c.fb_id, c.x, c.y, c.mode_valid);
	return (0);
}

static int
probe_mode_getencoder(int fd, uint32_t id)
{
	struct drm_mode_get_encoder e;

	if (id == 0)
		return (0);
	memset(&e, 0, sizeof(e));
	e.encoder_id = id;
	if (ioctl(fd, DRM_IOCTL_MODE_GETENCODER, &e) != 0) {
		warn("MODE_GETENCODER id=%u", id);
		return (1);
	}
	printf("GETENCODER   : id=%u type=%u crtc=%u poss_crtcs=0x%x "
	    "poss_clones=0x%x\n", e.encoder_id, e.encoder_type, e.crtc_id,
	    e.possible_crtcs, e.possible_clones);
	return (0);
}

static int
probe_mode_getconnector(int fd, uint32_t id)
{
	struct drm_mode_get_connector c;
	uint32_t enc_ids[8];
	struct drm_mode_modeinfo modes[16];

	if (id == 0)
		return (0);

	memset(&c, 0, sizeof(c));
	c.connector_id = id;
	if (ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &c) != 0) {
		warn("MODE_GETCONNECTOR count probe id=%u", id);
		return (1);
	}
	printf("GETCONN count: id=%u type=%u/%u status=%u "
	    "enc_count=%u mode_count=%u prop_count=%u\n", c.connector_id,
	    c.connector_type, c.connector_type_id, c.connection,
	    c.count_encoders, c.count_modes, c.count_props);

	memset(&c, 0, sizeof(c));
	c.connector_id = id;
	c.encoders_ptr = (uintptr_t)enc_ids;
	c.count_encoders = 8;
	c.modes_ptr = (uintptr_t)modes;
	c.count_modes = 16;
	if (ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &c) != 0) {
		warn("MODE_GETCONNECTOR fill id=%u", id);
		return (1);
	}
	if (c.count_encoders > 0)
		printf("  enc_ids[0]=%u (of %u)\n", enc_ids[0],
		    c.count_encoders);
	if (c.count_modes > 0) {
		struct drm_mode_modeinfo *m = &modes[0];
		printf("  mode[0]: %s %ux%u@%u clock=%u flags=0x%x type=0x%x\n",
		    m->name, m->hdisplay, m->vdisplay, m->vrefresh,
		    m->clock, m->flags, m->type);
	}
	return (0);
}

static int
probe_mode_getplane_resources(int fd, uint32_t *plane_id)
{
	struct drm_mode_get_plane_res r;
	uint32_t ids[16];

	memset(&r, 0, sizeof(r));
	if (ioctl(fd, DRM_IOCTL_MODE_GETPLANERESOURCES, &r) != 0) {
		warn("MODE_GETPLANERESOURCES count probe");
		return (1);
	}
	printf("GETPLANES cnt: count_planes=%u\n", r.count_planes);

	memset(&r, 0, sizeof(r));
	r.plane_id_ptr = (uintptr_t)ids;
	r.count_planes = 16;
	if (ioctl(fd, DRM_IOCTL_MODE_GETPLANERESOURCES, &r) != 0) {
		warn("MODE_GETPLANERESOURCES fill");
		return (1);
	}
	printf("GETPLANES fil: count_planes=%u\n", r.count_planes);
	if (r.count_planes > 0)
		printf("  plane[0]=%u\n", ids[0]);
	*plane_id = (r.count_planes > 0) ? ids[0] : 0;
	return (0);
}

static int
probe_mode_getplane(int fd, uint32_t id)
{
	struct drm_mode_get_plane p;
	uint32_t formats[16];

	if (id == 0)
		return (0);

	memset(&p, 0, sizeof(p));
	p.plane_id = id;
	if (ioctl(fd, DRM_IOCTL_MODE_GETPLANE, &p) != 0) {
		warn("MODE_GETPLANE count probe id=%u", id);
		return (1);
	}
	printf("GETPLANE cnt : id=%u crtc=%u fb=%u poss_crtcs=0x%x "
	    "count_format_types=%u\n", p.plane_id, p.crtc_id, p.fb_id,
	    p.possible_crtcs, p.count_format_types);

	memset(&p, 0, sizeof(p));
	p.plane_id = id;
	p.format_type_ptr = (uintptr_t)formats;
	p.count_format_types = 16;
	if (ioctl(fd, DRM_IOCTL_MODE_GETPLANE, &p) != 0) {
		warn("MODE_GETPLANE fill id=%u", id);
		return (1);
	}
	if (p.count_format_types > 0) {
		uint32_t f = formats[0];
		printf("  fmt[0]=0x%08x \"%c%c%c%c\" (of %u)\n", f,
		    (char)(f & 0xff), (char)((f >> 8) & 0xff),
		    (char)((f >> 16) & 0xff), (char)((f >> 24) & 0xff),
		    p.count_format_types);
	}
	return (0);
}

static int
probe_dumb_buffer(int fd)
{
	struct drm_mode_create_dumb c;
	struct drm_mode_map_dumb m;
	struct drm_mode_destroy_dumb d;
	uint32_t *map;
	const uint32_t pattern = 0xc0ffee42;
	int error = 0;

	memset(&c, 0, sizeof(c));
	c.width = 128;
	c.height = 64;
	c.bpp = 32;
	if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &c) != 0) {
		warn("CREATE_DUMB");
		return (1);
	}
	printf("CREATE_DUMB  : handle=%u pitch=%u size=%llu\n",
	    c.handle, c.pitch, (unsigned long long)c.size);

	memset(&m, 0, sizeof(m));
	m.handle = c.handle;
	if (ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &m) != 0) {
		warn("MAP_DUMB");
		error = 1;
		goto out;
	}
	printf("MAP_DUMB     : offset=0x%llx\n",
	    (unsigned long long)m.offset);

	map = mmap(NULL, c.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
	    m.offset);
	if (map == MAP_FAILED) {
		warn("mmap");
		error = 1;
		goto out;
	}
	map[0] = pattern;
	map[(c.size / sizeof(*map)) - 1] = ~pattern;
	if (map[0] != pattern || map[(c.size / sizeof(*map)) - 1] != ~pattern) {
		fprintf(stderr, "mmap readback mismatch\n");
		error = 1;
	}
	printf("MMAP         : %u bytes; first=0x%x last=0x%x (write/read OK)\n",
	    (unsigned)c.size, map[0], map[(c.size / sizeof(*map)) - 1]);
	munmap(map, c.size);

out:
	memset(&d, 0, sizeof(d));
	d.handle = c.handle;
	if (ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &d) != 0) {
		warn("DESTROY_DUMB");
		error = 1;
	} else {
		printf("DESTROY_DUMB : handle=%u\n", c.handle);
	}
	return (error);
}

static int
probe_modeset(int fd, uint32_t crtc_id, uint32_t conn_id)
{
	struct drm_mode_create_dumb c1, c2;
	struct drm_mode_fb_cmd2 fbcmd;
	struct drm_mode_crtc setcrtc;
	struct drm_mode_crtc getcrtc;
	struct drm_mode_crtc_page_flip flip;
	struct drm_mode_get_connector conn;
	struct drm_mode_modeinfo modes[1];
	struct drm_mode_destroy_dumb d;
	uint32_t fb1 = 0, fb2 = 0;
	uint32_t conn_array[1];
	int error = 0;

	if (crtc_id == 0 || conn_id == 0) {
		printf("MODESET      : skipped (no crtc/conn)\n");
		return (0);
	}

	/* Pull the first mode off the connector to feed SETCRTC. */
	memset(&conn, 0, sizeof(conn));
	conn.connector_id = conn_id;
	conn.modes_ptr = (uintptr_t)modes;
	conn.count_modes = 1;
	if (ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &conn) != 0 ||
	    conn.count_modes == 0) {
		printf("MODESET      : skipped (no modes)\n");
		return (0);
	}

	/* Buffer #1. */
	memset(&c1, 0, sizeof(c1));
	c1.width = modes[0].hdisplay;
	c1.height = modes[0].vdisplay;
	c1.bpp = 32;
	if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &c1) != 0) {
		warn("CREATE_DUMB fb1");
		return (1);
	}

	memset(&fbcmd, 0, sizeof(fbcmd));
	fbcmd.width = c1.width;
	fbcmd.height = c1.height;
	fbcmd.pixel_format = 0x34325258;	/* DRM_FORMAT_XRGB8888 */
	fbcmd.handles[0] = c1.handle;
	fbcmd.pitches[0] = c1.pitch;
	if (ioctl(fd, DRM_IOCTL_MODE_ADDFB2, &fbcmd) != 0) {
		warn("ADDFB2 fb1");
		error = 1;
		goto cleanup;
	}
	fb1 = fbcmd.fb_id;
	printf("ADDFB2       : fb1=%u (%ux%u XR24 pitch=%u)\n",
	    fb1, fbcmd.width, fbcmd.height, fbcmd.pitches[0]);

	/* Drive SETCRTC. */
	memset(&setcrtc, 0, sizeof(setcrtc));
	conn_array[0] = conn_id;
	setcrtc.set_connectors_ptr = (uintptr_t)conn_array;
	setcrtc.count_connectors = 1;
	setcrtc.crtc_id = crtc_id;
	setcrtc.fb_id = fb1;
	setcrtc.mode_valid = 1;
	setcrtc.mode = modes[0];
	if (ioctl(fd, DRM_IOCTL_MODE_SETCRTC, &setcrtc) != 0) {
		warn("SETCRTC");
		error = 1;
		goto cleanup;
	}
	printf("SETCRTC      : crtc=%u fb=%u mode=%s\n", crtc_id, fb1,
	    modes[0].name);

	memset(&getcrtc, 0, sizeof(getcrtc));
	getcrtc.crtc_id = crtc_id;
	if (ioctl(fd, DRM_IOCTL_MODE_GETCRTC, &getcrtc) != 0) {
		warn("GETCRTC after SETCRTC");
		error = 1;
		goto cleanup;
	}
	printf("  GETCRTC    : fb=%u mode_valid=%u %ux%u\n",
	    getcrtc.fb_id, getcrtc.mode_valid, getcrtc.mode.hdisplay,
	    getcrtc.mode.vdisplay);
	if (getcrtc.fb_id != fb1 || !getcrtc.mode_valid)
		error = 1;

	/* Buffer #2 + PAGE_FLIP. */
	memset(&c2, 0, sizeof(c2));
	c2.width = c1.width;
	c2.height = c1.height;
	c2.bpp = 32;
	if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &c2) != 0) {
		warn("CREATE_DUMB fb2");
		error = 1;
		goto cleanup;
	}
	memset(&fbcmd, 0, sizeof(fbcmd));
	fbcmd.width = c2.width;
	fbcmd.height = c2.height;
	fbcmd.pixel_format = 0x34325258;
	fbcmd.handles[0] = c2.handle;
	fbcmd.pitches[0] = c2.pitch;
	if (ioctl(fd, DRM_IOCTL_MODE_ADDFB2, &fbcmd) != 0) {
		warn("ADDFB2 fb2");
		error = 1;
		goto cleanup_c2;
	}
	fb2 = fbcmd.fb_id;

	memset(&flip, 0, sizeof(flip));
	flip.crtc_id = crtc_id;
	flip.fb_id = fb2;
	if (ioctl(fd, DRM_IOCTL_MODE_PAGE_FLIP, &flip) != 0) {
		warn("PAGE_FLIP");
		error = 1;
	} else {
		printf("PAGE_FLIP    : crtc=%u fb=%u\n", crtc_id, fb2);
	}

	memset(&getcrtc, 0, sizeof(getcrtc));
	getcrtc.crtc_id = crtc_id;
	if (ioctl(fd, DRM_IOCTL_MODE_GETCRTC, &getcrtc) == 0) {
		printf("  GETCRTC    : fb=%u (expect %u)\n", getcrtc.fb_id,
		    fb2);
		if (getcrtc.fb_id != fb2)
			error = 1;
	}

cleanup_c2:
	if (fb2 != 0) {
		if (ioctl(fd, DRM_IOCTL_MODE_RMFB, &fb2) != 0)
			warn("RMFB fb2");
	}
	memset(&d, 0, sizeof(d));
	d.handle = c2.handle;
	(void)ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &d);

cleanup:
	if (fb1 != 0) {
		/* Blank the CRTC first to drop the fb1 ref. */
		memset(&setcrtc, 0, sizeof(setcrtc));
		setcrtc.crtc_id = crtc_id;
		setcrtc.mode_valid = 0;
		(void)ioctl(fd, DRM_IOCTL_MODE_SETCRTC, &setcrtc);
		if (ioctl(fd, DRM_IOCTL_MODE_RMFB, &fb1) != 0)
			warn("RMFB fb1");
	}
	memset(&d, 0, sizeof(d));
	d.handle = c1.handle;
	(void)ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &d);
	return (error);
}

int
main(int argc, char **argv)
{
	const char *path;
	uint32_t crtc_id, enc_id, conn_id, plane_id;
	int fd, rc;

	path = (argc > 1) ? argv[1] : default_dev;
	fd = open(path, O_RDWR);
	if (fd < 0)
		err(1, "open %s", path);
	printf("opened %s fd=%d\n", path, fd);

	rc = 0;
	rc |= probe_version(fd);
	rc |= probe_unique(fd);
	rc |= probe_set_version(fd);
	rc |= probe_get_cap(fd);
	rc |= probe_mode_getresources(fd, &crtc_id, &enc_id, &conn_id);
	rc |= probe_mode_getcrtc(fd, crtc_id);
	rc |= probe_mode_getencoder(fd, enc_id);
	rc |= probe_mode_getconnector(fd, conn_id);
	rc |= probe_mode_getplane_resources(fd, &plane_id);
	rc |= probe_mode_getplane(fd, plane_id);
	rc |= probe_dumb_buffer(fd);
	rc |= probe_modeset(fd, crtc_id, conn_id);

	close(fd);
	return (rc);
}
