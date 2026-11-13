// Legacy drawing is optional; estimation never needs a graphics context.
#pragma once
#ifdef IPTAM_HEADLESS
#define GL_TEXTURE_2D 0
using GLfloat = float;
using GLuint = unsigned int;
using GLubyte = unsigned char;
using GLshort = short;
using GLushort = unsigned short;
#define GL_BLEND 0
#define GL_CLAMP_TO_EDGE 0
#define GL_COLOR_ARRAY 0
#define GL_COLOR_BUFFER_BIT 0
#define GL_FALSE 0
#define GL_FLOAT 0
#define GL_GENERATE_MIPMAP 0
#define GL_LINEAR 0
#define GL_LINES 0
#define GL_LINE_SMOOTH 0
#define GL_LINE_STRIP 0
#define GL_LUMINANCE 0
#define GL_MODELVIEW 0
#define GL_ONE_MINUS_SRC_ALPHA 0
#define GL_POINTS 0
#define GL_POINT_SMOOTH 0
#define GL_PROJECTION 0
#define GL_SHORT 0
#define GL_SRC_ALPHA 0
#define GL_TEXTURE_ 0
#define GL_TEXTURE_COORD_ARRAY 0
#define GL_TEXTURE_MAG_FILTER 0
#define GL_TEXTURE_MIN_FILTER 0
#define GL_TEXTURE_WRAP_S 0
#define GL_TEXTURE_WRAP_T 0
#define GL_TRIANGLE_STRIP 0
#define GL_UNSIGNED_BYTE 0
#define GL_VERTEX_ARRAY 0
template<class... T> inline void glBegin(T...) {}
template<class... T> inline void glBindTexture(T...) {}
template<class... T> inline void glBlendFunc(T...) {}
template<class... T> inline void glClear(T...) {}
template<class... T> inline void glClearColor(T...) {}
template<class... T> inline void glColor3f(T...) {}
template<class... T> inline void glColor4f(T...) {}
template<class... T> inline void glColorPointer(T...) {}
template<class... T> inline void glDisable(T...) {}
template<class... T> inline void glDisableClientState(T...) {}
template<class... T> inline void glDrawArrays(T...) {}
template<class... T> inline void glDrawPixels(T...) {}
template<class... T> inline void glEnable(T...) {}
template<class... T> inline void glEnableClientState(T...) {}
template<class... T> inline void glEnd(T...) {}
template<class... T> inline void glGenTextures(T...) {}
template<class... T> inline void glLineWidth(T...) {}
template<class... T> inline void glLoadIdentity(T...) {}
template<class... T> inline void glLoadMatrixf(T...) {}
template<class... T> inline void glMatrixMode(T...) {}
template<class... T> inline void glOrthof(T...) {}
template<class... T> inline void glPointSize(T...) {}
template<class... T> inline void glTexCoordPointer(T...) {}
template<class... T> inline void glTexImage2D(T...) {}
template<class... T> inline void glTexParameterf(T...) {}
template<class... T> inline void glTexParameteri(T...) {}
template<class... T> inline void glTexSubImage2D(T...) {}
template<class... T> inline void glVertex(T...) {}
template<class... T> inline void glVertexPointer(T...) {}
#else
#include <OpenGLES/ES1/gl.h>
#include <OpenGLES/ES1/glext.h>
#endif
