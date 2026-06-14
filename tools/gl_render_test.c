/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Offscreen GL render test via EGL surfaceless platform.
 *
 * Proves Mesa + llvmpipe initializes against our kms framework and
 * actually rasterises pixels.  No display, no window system, no GBM
 * loader — pure EGL_MESA_platform_surfaceless + an FBO.
 *
 *   1. eglGetPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA, ...)
 *   2. eglInitialize -> Mesa swrast / llvmpipe attaches
 *   3. eglChooseConfig + eglCreateContext (ES 2 / 3)
 *   4. eglMakeCurrent(EGL_NO_SURFACE, ...)
 *   5. glGenFramebuffers + glRenderbufferStorage(GL_RGBA8, 64, 64)
 *   6. glClearColor + glClear
 *   7. glReadPixels -> verify the 4 corners are the cleared colour
 *   8. Print GL_VENDOR / GL_RENDERER / GL_VERSION
 *
 * Link with: cc -lEGL -lGLESv2
 *
 * Run via:  kms-swrast gl_render_test
 */

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

int
main(void)
{
	EGLDisplay dpy;
	EGLContext ctx;
	EGLConfig cfg;
	EGLint major, minor, n;
	const EGLint cattr[] = {
		EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
		EGL_ALPHA_SIZE, 8,
		EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
		EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
		EGL_NONE };
	const EGLint xattr[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };

	dpy = eglGetPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA,
	    EGL_DEFAULT_DISPLAY, NULL);
	if (dpy == EGL_NO_DISPLAY) {
		fprintf(stderr, "eglGetPlatformDisplay: 0x%x\n", eglGetError());
		return (1);
	}
	if (!eglInitialize(dpy, &major, &minor)) {
		fprintf(stderr, "eglInitialize: 0x%x\n", eglGetError());
		return (2);
	}
	printf("EGL %d.%d  vendor=%s\n", major, minor,
	    eglQueryString(dpy, EGL_VENDOR));
	if (!eglChooseConfig(dpy, cattr, &cfg, 1, &n) || n < 1) {
		fprintf(stderr, "eglChooseConfig: 0x%x  n=%d\n",
		    eglGetError(), n);
		return (3);
	}
	if (!eglBindAPI(EGL_OPENGL_ES_API)) {
		fprintf(stderr, "eglBindAPI: 0x%x\n", eglGetError());
		return (4);
	}
	ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, xattr);
	if (ctx == EGL_NO_CONTEXT) {
		fprintf(stderr, "eglCreateContext: 0x%x\n", eglGetError());
		return (5);
	}
	if (!eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx)) {
		fprintf(stderr, "eglMakeCurrent: 0x%x\n", eglGetError());
		return (6);
	}

	printf("GL_VENDOR   = %s\n", glGetString(GL_VENDOR));
	printf("GL_RENDERER = %s\n", glGetString(GL_RENDERER));
	printf("GL_VERSION  = %s\n", glGetString(GL_VERSION));

	GLuint fbo, rbo;
	glGenFramebuffers(1, &fbo);
	glGenRenderbuffers(1, &rbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glBindRenderbuffer(GL_RENDERBUFFER, rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA4, 64, 64);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
	    GL_RENDERBUFFER, rbo);
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) !=
	    GL_FRAMEBUFFER_COMPLETE) {
		fprintf(stderr, "FBO incomplete: 0x%x\n",
		    glCheckFramebufferStatus(GL_FRAMEBUFFER));
		return (7);
	}

	glViewport(0, 0, 64, 64);
	glClearColor(0.13f, 0.42f, 0.99f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glFinish();

	uint8_t px[4];
	glReadPixels(32, 32, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, px);
	printf("centre pixel = R=%u G=%u B=%u A=%u  (expect ~33 107 252 255)\n",
	    px[0], px[1], px[2], px[3]);
	/* RGBA4 quantises to 4 bits/channel; tolerance must cover ~17. */
	int ok = (px[0] > 15 && px[0] < 55) &&
	    (px[1] > 85 && px[1] < 125) &&
	    (px[2] > 220 && px[2] < 256) &&
	    (px[3] > 200);
	printf("%s\n", ok ? "OK -- llvmpipe rasterised" : "FAIL -- wrong color");

	glDeleteFramebuffers(1, &fbo);
	glDeleteRenderbuffers(1, &rbo);
	eglDestroyContext(dpy, ctx);
	eglTerminate(dpy);
	return (ok ? 0 : 8);
}
